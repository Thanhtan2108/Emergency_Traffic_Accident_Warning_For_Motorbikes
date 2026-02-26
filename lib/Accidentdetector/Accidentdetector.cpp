// ============================================================
//  AccidentDetector.cpp
//  Implement Tầng 3 – State Machine 10 state.
//
//  Tài liệu: README.md – Section VII
// ============================================================

#include "AccidentDetector.h"
#include <math.h>


// ============================================================
//  Constructor
// ============================================================
AccidentDetector::AccidentDetector(SignalProcessor& sp)
    : _sp(sp)
    , _state(WARMUP)
    , _state_enter_time(0)
    , _snapshot{}
    , _last_alert{}
    , _new_alert(false)
    , _debounce_timer(0)
    , _warmup_start(0)
    , _sustained_fall_start(0)
{}


// ============================================================
//  begin()
// ============================================================
void AccidentDetector::begin() {
    _state            = WARMUP;
    _state_enter_time = millis();
    _warmup_start     = millis();
    _resetSnapshot();
    _new_alert      = false;
    _debounce_timer = 0;
    _sustained_fall_start = 0;
}


// ============================================================
//  process()
//  Dispatcher – gọi handler của state hiện tại mỗi chu kỳ.
// ============================================================
void AccidentDetector::process(const ProcessedData& data) {
    switch (_state) {
        case WARMUP:          _handleWarmup(data);         break;
        case NORMAL:          _handleNormal(data);         break;
        case EVENT_DETECTED:  _handleEventDetected(data);  break;
        case CLASSIFYING:     _handleClassifying(data);    break;
        case ROAD_SHOCK:      _handleRoadShock(data);      break;
        case IMPACT_PENDING:  _handleImpactPending(data);  break;
        case FALL_PENDING:    _handleFallPending(data);    break;
        case STRONG_IMPACT:   _handleStrongImpact(data);   break;
        case FALL_DETECTED:   _handleFallDetected(data);   break;
        case TIMEOUT_RESET:   _handleTimeoutReset(data);   break;
    }
}


// ============================================================
//  Getters
// ============================================================
DetectorState AccidentDetector::getState() const {
    return _state;
}

uint32_t AccidentDetector::getStateEnterTime() const {
    return _state_enter_time;
}

bool AccidentDetector::hasNewAlert() const {
    return _new_alert;
}

AlertInfo AccidentDetector::getLastAlert() {
    _new_alert = false;   // xóa flag sau khi StateTask lấy
    return _last_alert;
}

bool AccidentDetector::isInDebounce() const {
    return _isDebounceActive();
}

bool AccidentDetector::isWarmingUp() const {
    return _state == WARMUP;
}


// ============================================================
//  forceReset()  –  WatchdogTask gọi khi state kẹt
// ============================================================
void AccidentDetector::forceReset() {
    _resetSnapshot();
    _sp.setBaselineUpdateEnabled(true);
    _sp.resetFilter();
    // KHÔNG reset _debounce_timer – giữ nguyên nếu đang debounce
    // KHÔNG reset baseline của SignalProcessor
    _transitionTo(NORMAL);
}


// ============================================================
//  STATE HANDLERS
// ============================================================

// ------------------------------------------------------------
//  ⚪ WARMUP
//  Chờ baseline ổn định và Complementary Filter hội tụ.
//  Không detect EVENT dù có spike.
//  Thoát → NORMAL khi cả 2 điều kiện thỏa đồng thời.
// ------------------------------------------------------------
void AccidentDetector::_handleWarmup(const ProcessedData& d) {
    bool baseline_ok  = d.baseline_ready;
    bool time_ok      = (millis() - _warmup_start) >= WARMUP_DURATION_MS;

    if (baseline_ok && time_ok) {
        _transitionTo(NORMAL);
    }
    // Không làm gì khác – chờ im lặng
}


// ------------------------------------------------------------
//  🟢 NORMAL
//  Hoạt động bình thường. Kích hoạt EVENT khi có spike raw.
//  Dùng raw (không smoothed) để không bỏ lỡ 50ms đầu của spike.
// ------------------------------------------------------------
void AccidentDetector::_handleNormal(const ProcessedData& d) {
    bool accel_spike = (d.magnitude_a_raw     >= A_EVENT_THRESHOLD);
    bool omega_spike = (d.magnitude_omega_raw >= W_EVENT_THRESHOLD);

    if ((accel_spike || omega_spike)
        && !_isDebounceActive()
        && d.baseline_ready)
    {
        // Khóa baseline ngay khi phát hiện sự kiện
        _sp.setBaselineUpdateEnabled(false);

        // Khởi tạo snapshot
        _snapshot.event_start    = millis();
        _snapshot.roll_at_event  = d.roll;
        _snapshot.pitch_at_event = d.pitch;
        _snapshot.peak_a         = d.magnitude_a_raw;
        _snapshot.peak_omega     = d.magnitude_omega_raw;
        _snapshot.spike_duration = 0;
        _snapshot.roll_delta     = 0.0f;
        _snapshot.pitch_delta    = 0.0f;

        _transitionTo(EVENT_DETECTED);
    }
}


// ------------------------------------------------------------
//  🔵 EVENT_DETECTED
//  Thu thập dữ liệu sự kiện trong cửa sổ T_EVENT_WINDOW (300ms).
//  Cập nhật peak, spike_duration, roll_delta, pitch_delta liên tục.
// ------------------------------------------------------------
void AccidentDetector::_handleEventDetected(const ProcessedData& d) {
    uint32_t now          = millis();
    uint32_t elapsed      = now - _snapshot.event_start;
    uint32_t time_in_state = now - _state_enter_time;

    // --- Cập nhật peak ---
    if (d.magnitude_a_raw > _snapshot.peak_a) {
        _snapshot.peak_a = d.magnitude_a_raw;
    }
    if (d.magnitude_omega_raw > _snapshot.peak_omega) {
        _snapshot.peak_omega = d.magnitude_omega_raw;
    }

    // --- Cộng dồn spike_duration khi raw còn vượt ngưỡng ---
    if (d.magnitude_a_raw >= A_EVENT_THRESHOLD) {
        _snapshot.spike_duration += SENSOR_PERIOD_MS;
    }

    // --- Cập nhật delta góc ---
    _snapshot.roll_delta  = fabsf(d.roll  - _snapshot.roll_at_event);
    _snapshot.pitch_delta = fabsf(d.pitch - _snapshot.pitch_at_event);

    // --- Kiểm tra thoát sớm: spike quá ngắn → nhiễu điện ---
    bool spike_ended = (d.magnitude_a_raw     < A_EVENT_THRESHOLD)
                    && (d.magnitude_omega_raw  < W_EVENT_THRESHOLD);

    if (spike_ended && _snapshot.spike_duration < T_SPIKE_MIN) {
        _transitionTo(TIMEOUT_RESET);
        return;
    }

    // --- Timeout toàn state (bảo vệ bất thường) ---
    if (time_in_state >= T_STATE_TIMEOUT) {
        _transitionTo(TIMEOUT_RESET);
        return;
    }

    // --- Điều kiện bình thường: chờ đủ T_EVENT_WINDOW ---
    if (elapsed >= T_EVENT_WINDOW) {
        _transitionTo(CLASSIFYING);
    }
}


// ------------------------------------------------------------
//  🟡 CLASSIFYING
//  Chạy MỘT LẦN DUY NHẤT. Không chờ, không delay.
//  Đánh giá snapshot theo thứ tự ưu tiên cố định.
// ------------------------------------------------------------
void AccidentDetector::_handleClassifying(const ProcessedData& d) {
    // Ưu tiên 1: ROAD_SHOCK – phổ biến nhất, lọc trước
    if (_snapshot.spike_duration  <  T_SPIKE_MAX
     && _snapshot.peak_a          <  A_SHOCK_MAX
     && _snapshot.roll_delta      <  ROLL_DELTA_MIN
     && _snapshot.pitch_delta     <  PITCH_DELTA_MIN)
    {
        _transitionTo(ROAD_SHOCK);
        return;
    }

    // Ưu tiên 2: Va chạm mạnh + quay nhanh → nguy cơ ngã cao nhất
    if (_snapshot.peak_a     >= A_IMPACT_THRESHOLD
     && _snapshot.peak_omega >= W_FALL_THRESHOLD)
    {
        _transitionTo(FALL_PENDING);
        return;
    }

    // Ưu tiên 3: Va chạm mạnh đơn thuần
    if (_snapshot.peak_a >= A_IMPACT_THRESHOLD) {
        _transitionTo(IMPACT_PENDING);
        return;
    }

    // Ưu tiên 4: Quay nhanh + tư thế thay đổi
    if (_snapshot.peak_omega >= W_FALL_THRESHOLD
     && _snapshot.roll_delta  >  ROLL_DELTA_MIN)
    {
        _transitionTo(FALL_PENDING);
        return;
    }

    // Không thỏa điều kiện nào
    _transitionTo(TIMEOUT_RESET);
}


// ------------------------------------------------------------
//  🔵→⚫ ROAD_SHOCK
//  Ổ gà / đường xấu – không cảnh báo, về NORMAL ngay.
// ------------------------------------------------------------
void AccidentDetector::_handleRoadShock(const ProcessedData& d) {
    // Chuyển về NORMAL ngay lập tức (không cần chờ)
    // Mở lại baseline update vì về NORMAL
    _sp.setBaselineUpdateEnabled(true);
    _sp.resetFilter();
    _resetSnapshot();
    _transitionTo(NORMAL);
}


// ------------------------------------------------------------
//  🔴 IMPACT_PENDING
//  Chờ 500ms xem có thêm dấu hiệu ngã không.
//  Nếu roll vượt ngưỡng → FALL_PENDING.
//  Nếu hết 500ms bình thường → STRONG_IMPACT.
//  Timeout bảo vệ: T_IMPACT_CONFIRM + 100ms = 600ms.
// ------------------------------------------------------------
void AccidentDetector::_handleImpactPending(const ProcessedData& d) {
    uint32_t time_in_state = millis() - _state_enter_time;

    // Roll vượt ngưỡng → nâng cấp lên FALL_PENDING
    if (fabsf(d.roll) > ROLL_FALL_THRESHOLD) {
        _transitionTo(FALL_PENDING);
        return;
    }

    // Hết T_IMPACT_CONFIRM mà không có gì thêm → xác nhận STRONG_IMPACT
    if (time_in_state >= T_IMPACT_CONFIRM) {
        _transitionTo(STRONG_IMPACT);
        return;
    }

    // Timeout bảo vệ
    if (time_in_state >= (uint32_t)(T_IMPACT_CONFIRM + 100)) {
        _transitionTo(TIMEOUT_RESET);
    }
}


// ------------------------------------------------------------
//  🟠 FALL_PENDING
//  Chờ xác nhận xe nằm xuống và giữ nguyên ≥ T_FALL_CONFIRM (2000ms).
//
//  Reset sớm (theo thứ tự ưu tiên):
//    1. Roll về < 40° → người dựng xe → TIMEOUT_RESET
//    2. roll_rate thấp + |ω| thấp → dựng chân chống → TIMEOUT_RESET
//    3. Pitch cực lớn → xe chúi đầu → vẫn là FALL, tiếp tục đếm
//
//  Timeout tổng: T_STATE_MAX_TIMEOUT (5000ms)
// ------------------------------------------------------------
void AccidentDetector::_handleFallPending(const ProcessedData& d) {
    uint32_t now           = millis();
    uint32_t time_in_state = now - _state_enter_time;

    // --- Reset sớm 1: xe đã được dựng lại ---
    if (fabsf(d.roll) < ROLL_NORMAL_MAX) {
        _transitionTo(TIMEOUT_RESET);
        return;
    }

    // --- Reset sớm 2: đang dựng chân chống ---
    // roll_rate thấp + tốc độ góc thấp → góc tăng chậm đều → chân chống
    if (fabsf(d.roll_rate)       < ROLL_CHANGE_RATE_SLOW
     && d.magnitude_omega_raw    < W_STATIC_MAX)
    {
        _transitionTo(TIMEOUT_RESET);
        return;
    }

    // --- Kiểm tra 3: pitch cực lớn → xe chúi đầu, vẫn là FALL ---
    // Không reset, nhưng cũng không cần làm gì thêm ở đây

    // --- Đếm sustained_fall_duration ---
    if (fabsf(d.roll) > ROLL_FALL_THRESHOLD) {
        // Roll vẫn vượt ngưỡng → khởi động / tiếp tục đếm
        if (_sustained_fall_start == 0) {
            _sustained_fall_start = now;
        }

        uint32_t sustained = now - _sustained_fall_start;
        if (sustained >= T_FALL_CONFIRM) {
            _transitionTo(FALL_DETECTED);
            return;
        }
    } else {
        // Roll tạm về dưới ROLL_FALL_THRESHOLD nhưng chưa về ROLL_NORMAL_MAX
        // Reset bộ đếm, tiếp tục chờ
        _sustained_fall_start = 0;
    }

    // --- Timeout tổng ---
    if (time_in_state >= T_STATE_MAX_TIMEOUT) {
        _transitionTo(TIMEOUT_RESET);
    }
}


// ------------------------------------------------------------
//  🚨 STRONG_IMPACT
//  Vào state → phát alert ngay (chỉ phát 1 lần khi enter).
//  Ở lại state cho đến khi debounce xong VÀ xe thẳng lại.
// ------------------------------------------------------------
void AccidentDetector::_handleStrongImpact(const ProcessedData& d) {
    // Điều kiện thoát về NORMAL
    bool debounce_done  = (millis() - _debounce_timer) > T_DEBOUNCE;
    bool posture_normal = fabsf(d.roll) < ROLL_NORMAL_MAX;

    if (debounce_done && posture_normal) {
        _sp.setBaselineUpdateEnabled(true);
        _resetSnapshot();
        _transitionTo(NORMAL);
    }
}


// ------------------------------------------------------------
//  🆘 FALL_DETECTED
//  Vào state → phát alert ngay (chỉ phát 1 lần khi enter).
//  Ở lại state cho đến khi debounce xong VÀ xe đã được dựng lại.
// ------------------------------------------------------------
void AccidentDetector::_handleFallDetected(const ProcessedData& d) {
    bool debounce_done  = (millis() - _debounce_timer) > T_DEBOUNCE;
    bool posture_normal = fabsf(d.roll) < ROLL_NORMAL_MAX;

    if (debounce_done && posture_normal) {
        _sp.setBaselineUpdateEnabled(true);
        _resetSnapshot();
        _transitionTo(NORMAL);
    }
}


// ------------------------------------------------------------
//  ⚫ TIMEOUT_RESET
//  Reset tức thì, không điều kiện, chuyển ngay về NORMAL.
// ------------------------------------------------------------
void AccidentDetector::_handleTimeoutReset(const ProcessedData& d) {
    _resetSnapshot();
    _sp.setBaselineUpdateEnabled(true);
    _sp.resetFilter();
    _transitionTo(NORMAL);
}


// ============================================================
//  PRIVATE HELPERS
// ============================================================

// ------------------------------------------------------------
//  _transitionTo()
//  Đổi state, ghi lại thời điểm vào, reset _sustained_fall_start
//  khi rời FALL_PENDING, thực hiện các "entry action" của state mới.
// ------------------------------------------------------------
void AccidentDetector::_transitionTo(DetectorState new_state) {
    // Reset bộ đếm fall khi rời FALL_PENDING
    if (_state == FALL_PENDING && new_state != FALL_PENDING) {
        _sustained_fall_start = 0;
    }

    _state            = new_state;
    _state_enter_time = millis();

    // --- Entry actions ---
    switch (new_state) {

        case STRONG_IMPACT: {
            // Phát alert một lần duy nhất khi enter
            // Baseline đã bị khóa từ EVENT_DETECTED – không cần khóa lại
            _sp.resetFilter();
            _debounce_timer = millis();

            // Lấy dữ liệu processed hiện tại cho alert
            // (không có tham số d ở đây – dùng snapshot đã có)
            _last_alert.type      = ALERT_IMPACT;
            _last_alert.peak_a    = _snapshot.peak_a;
            _last_alert.peak_omega= _snapshot.peak_omega;
            _last_alert.roll      = _snapshot.roll_at_event;
            _last_alert.pitch     = _snapshot.pitch_at_event;
            _last_alert.timestamp = millis();
            _new_alert = true;
            break;
        }

        case FALL_DETECTED: {
            // Phát alert một lần duy nhất khi enter
            _sp.resetFilter();
            _debounce_timer = millis();

            _last_alert.type      = ALERT_FALL;
            _last_alert.peak_a    = _snapshot.peak_a;
            _last_alert.peak_omega= _snapshot.peak_omega;
            _last_alert.roll      = _snapshot.roll_at_event;
            _last_alert.pitch     = _snapshot.pitch_at_event;
            _last_alert.timestamp = millis();
            _new_alert = true;
            break;
        }

        default:
            break;
    }
}


// ------------------------------------------------------------
//  _buildAlert()  – helper tạo AlertInfo (không dùng trong
//  _transitionTo vì không có ProcessedData, giữ lại để mở rộng)
// ------------------------------------------------------------
AlertInfo AccidentDetector::_buildAlert(AlertType type, const ProcessedData& d) const {
    AlertInfo alert;
    alert.type      = type;
    alert.peak_a    = _snapshot.peak_a;
    alert.peak_omega= _snapshot.peak_omega;
    alert.roll      = d.roll;
    alert.pitch     = d.pitch;
    alert.timestamp = millis();
    return alert;
}


// ------------------------------------------------------------
//  _resetSnapshot()
// ------------------------------------------------------------
void AccidentDetector::_resetSnapshot() {
    _snapshot.peak_a         = 0.0f;
    _snapshot.peak_omega     = 0.0f;
    _snapshot.roll_at_event  = 0.0f;
    _snapshot.pitch_at_event = 0.0f;
    _snapshot.roll_delta     = 0.0f;
    _snapshot.pitch_delta    = 0.0f;
    _snapshot.spike_duration = 0;
    _snapshot.event_start    = 0;
}


// ------------------------------------------------------------
//  _isDebounceActive()
// ------------------------------------------------------------
bool AccidentDetector::_isDebounceActive() const {
    if (_debounce_timer == 0) return false;
    return (millis() - _debounce_timer) <= T_DEBOUNCE;
}
