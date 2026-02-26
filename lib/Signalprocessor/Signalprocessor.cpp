// ============================================================
//  SignalProcessor.cpp
//  Implement Tầng 2 – Pipeline xử lý tín hiệu 6 bước.
//
//  Tài liệu: README.md – Section VI
// ============================================================

#include "SignalProcessor.h"
#include <math.h>


// ============================================================
//  begin()
// ============================================================
void SignalProcessor::begin() {
    // Khởi tạo circular buffer (fill = 0)
    for (int i = 0; i < MA_FILTER_SIZE; i++) {
        _a_buffer[i] = 0.0f;
        _w_buffer[i] = 0.0f;
    }
    _buffer_index = 0;

    // Baseline
    _baseline                = BASELINE_INITIAL;  // 1.0g = trọng lực khi đứng yên
    _baseline_update_enabled = true;              // cho warm-up chạy ngay
    _warmup_sample_count     = 0;
    _baseline_ready          = false;

    // roll_rate
    _roll_prev      = 0.0f;
    _timestamp_prev = 0;
}


// ============================================================
//  process()
//  Pipeline 6 bước – THỨ TỰ KHÔNG ĐƯỢC ĐẢO.
// ============================================================
ProcessedData SignalProcessor::process(const RawSensorData& raw) {
    ProcessedData out;

    // Nếu SensorReader đã đánh dấu invalid → bỏ qua toàn bộ pipeline
    if (!raw.valid) {
        out.timestamp = raw.timestamp;
        // Các field float mặc định = 0.0f, bool = false – đúng với "rỗng"
        return out;
    }

    // ----------------------------------------------------------
    //  Bước 1 – Tính |a_raw| và |ω_raw|
    //  Thực hiện TRƯỚC Bước 0 vì Outlier Rejection cần magnitude.
    // ----------------------------------------------------------
    float a_mag = _step1_magnitude(raw.ax, raw.ay, raw.az);
    float w_mag = _step1_magnitude(raw.gx, raw.gy, raw.gz);

    // ----------------------------------------------------------
    //  Bước 0 – Outlier Rejection
    //  Kiểm tra NGAY SAU khi có magnitude, trước mọi tính toán khác.
    //  1 mẫu nhiễu có thể kéo moving average sai trong 5 chu kỳ (50ms).
    // ----------------------------------------------------------
    if (!_step0_outlierCheck(a_mag, w_mag)) {
        out.timestamp = raw.timestamp;
        return out;  // valid = false
    }

    // ----------------------------------------------------------
    //  Bước 2 – Dynamic Acceleration
    //  PHẢI trước Bước 3 (filter).
    //  Lý do: trừ baseline trên raw signal trước khi smooth,
    //  tránh baseline bị kéo vào trong filter làm mất đặc tính spike.
    // ----------------------------------------------------------
    float a_dynamic_raw = _step2_dynamicAccel(a_mag);

    // ----------------------------------------------------------
    //  Bước 3 – Moving Average Filter (N=5, circular buffer)
    //  Ghi cả 2 buffer tại cùng 1 index, tính trung bình,
    //  rồi mới advance index 1 lần duy nhất.
    //  Không advance trong _step3_movingAverage để tránh advance 2 lần.
    // ----------------------------------------------------------
    float a_smoothed = _step3_movingAverage(_a_buffer, a_dynamic_raw);
    float w_smoothed = _step3_movingAverage(_w_buffer, w_mag);
    // Advance index SAU KHI cả 2 buffer đã ghi xong
    _buffer_index = (_buffer_index + 1) % MA_FILTER_SIZE;

    // ----------------------------------------------------------
    //  Bước 4 – Tính roll_rate (°/giây)
    //  Dùng trong FALL_PENDING để phân biệt ngã thật vs dựng chân chống.
    // ----------------------------------------------------------
    float roll_rate = _step4_rollRate(raw.roll, raw.timestamp);

    // ----------------------------------------------------------
    //  Bước 5 – Cập nhật Baseline EMA
    //  Chỉ chạy khi được StateTask cho phép.
    //  Dùng |a_raw| (chưa trừ baseline) để EMA theo dõi mức nền thực.
    // ----------------------------------------------------------
    _step5_updateBaseline(a_mag);

    // ----------------------------------------------------------
    //  Đóng gói ProcessedData
    // ----------------------------------------------------------
    out.magnitude_a         = a_smoothed;
    out.magnitude_a_raw     = a_dynamic_raw;    // chưa lọc – để detect spike
    out.magnitude_omega     = w_smoothed;
    out.magnitude_omega_raw = w_mag;            // chưa lọc – để detect spike
    out.roll                = raw.roll;
    out.pitch               = raw.pitch;
    out.roll_rate           = roll_rate;
    out.baseline            = _baseline;
    out.baseline_ready      = _baseline_ready;
    out.timestamp           = raw.timestamp;
    // valid mặc định = false trong struct, cần set true ở đây
    // Khai báo ProcessedData không có field valid trong document –
    // AccidentDetector dựa vào baseline_ready và các giá trị để quyết định.

    return out;
}


// ============================================================
//  setBaselineUpdateEnabled()
// ============================================================
void SignalProcessor::setBaselineUpdateEnabled(bool enabled) {
    _baseline_update_enabled = enabled;
}


// ============================================================
//  isBaselineReady()
// ============================================================
bool SignalProcessor::isBaselineReady() const {
    return _baseline_ready;
}


// ============================================================
//  getBaseline()
// ============================================================
float SignalProcessor::getBaseline() const {
    return _baseline;
}


// ============================================================
//  resetFilter()
//  Fill circular buffer về 0 – tránh "dư âm" spike cũ.
//  KHÔNG reset baseline (baseline là giá trị dài hạn).
//  KHÔNG reset warmup_sample_count hay baseline_ready.
// ============================================================
void SignalProcessor::resetFilter() {
    for (int i = 0; i < MA_FILTER_SIZE; i++) {
        _a_buffer[i] = 0.0f;
        _w_buffer[i] = 0.0f;
    }
    _buffer_index = 0;
}


// ============================================================
//  PRIVATE – Các bước pipeline
// ============================================================

// ------------------------------------------------------------
//  Bước 0 – Outlier Rejection
//  Trả về true nếu mẫu HỢP LỆ (không phải outlier).
//
//  Điều kiện reject (tài liệu Section VI, Bước 0):
//    |a_mag| > OUTLIER_A_MAX  → vượt range vật lý ±2g + buffer
//    |a_mag| < OUTLIER_A_MIN  → I2C trả về ~0 khi mất kết nối
//    |w_mag| > OUTLIER_W_MAX  → vượt range vật lý ±500°/s
// ------------------------------------------------------------
bool SignalProcessor::_step0_outlierCheck(float a_mag, float w_mag) const {
    if (a_mag > OUTLIER_A_MAX) return false;
    if (a_mag < OUTLIER_A_MIN) return false;
    if (w_mag > OUTLIER_W_MAX) return false;
    return true;
}


// ------------------------------------------------------------
//  Bước 1 – Tính Magnitude (vector 3D)
//  |v| = sqrt(x² + y² + z²)
//  Dùng cho cả gia tốc và tốc độ góc.
// ------------------------------------------------------------
float SignalProcessor::_step1_magnitude(float x, float y, float z) const {
    return sqrtf(x * x + y * y + z * z);
}


// ------------------------------------------------------------
//  Bước 2 – Dynamic Acceleration
//  |a_dynamic_raw| = | |a_raw| - baseline |
//
//  Dùng fabsf() để xử lý cả trường hợp |a_raw| < baseline
//  (ví dụ: rơi tự do → |a_raw| giảm đột ngột về ~0).
// ------------------------------------------------------------
float SignalProcessor::_step2_dynamicAccel(float a_mag) const {
    return fabsf(a_mag - _baseline);
}


// ------------------------------------------------------------
//  Bước 3 – Moving Average Filter (circular buffer)
//
//  Ghi new_value vào vị trí _buffer_index hiện tại, tính tổng / N.
//  _buffer_index KHÔNG được advance ở đây – advance tập trung
//  trong process() sau khi cả a_buffer và w_buffer đã được ghi.
// ------------------------------------------------------------
float SignalProcessor::_step3_movingAverage(float* buffer, float new_value) {
    // Ghi vào vị trí hiện tại
    buffer[_buffer_index] = new_value;

    // Tính tổng toàn bộ buffer
    float sum = 0.0f;
    for (int i = 0; i < MA_FILTER_SIZE; i++) {
        sum += buffer[i];
    }

    return sum / (float)MA_FILTER_SIZE;
}


// ------------------------------------------------------------
//  Bước 4 – roll_rate (°/giây)
//
//  roll_rate = (roll_current - roll_prev) / delta_t_ms * 1000.0f
//
//  Edge case: chu kỳ đầu tiên (_timestamp_prev == 0) → roll_rate = 0
//  để tránh chia cho delta_t = 0 hoặc giá trị rất lớn.
//
//  Cập nhật _roll_prev và _timestamp_prev sau khi tính.
// ------------------------------------------------------------
float SignalProcessor::_step4_rollRate(float roll_current, uint32_t timestamp_current) {
    float roll_rate = 0.0f;

    if (_timestamp_prev != 0) {
        uint32_t delta_t = timestamp_current - _timestamp_prev;  // ms
        if (delta_t > 0) {
            roll_rate = (roll_current - _roll_prev) / (float)delta_t * 1000.0f;
        }
    }

    _roll_prev      = roll_current;
    _timestamp_prev = timestamp_current;

    return roll_rate;
}


// ------------------------------------------------------------
//  Bước 5 – Cập nhật Baseline EMA
//
//  Có 2 chế độ (tài liệu Section VI, Bước 5):
//
//  Chế độ WARM-UP (warmup_sample_count < BASELINE_WARMUP_SAMPLES):
//    α = BASELINE_ALPHA_WARMUP (0.2) – hội tụ nhanh trong 50 mẫu đầu
//    Chạy bất kể _baseline_update_enabled
//    Sau đủ mẫu: baseline_ready = true
//
//  Chế độ BÌNH THƯỜNG (warmup đã xong):
//    α = BASELINE_ALPHA (0.05) – cập nhật chậm, ổn định
//    CHỈ chạy khi _baseline_update_enabled == true
//    (StateTask khóa baseline khi phát hiện sự kiện)
// ------------------------------------------------------------
void SignalProcessor::_step5_updateBaseline(float a_raw_mag) {
    if (_warmup_sample_count < BASELINE_WARMUP_SAMPLES) {
        // --- Chế độ WARM-UP ---
        _baseline = _baseline * (1.0f - BASELINE_ALPHA_WARMUP)
                  + a_raw_mag * BASELINE_ALPHA_WARMUP;

        _warmup_sample_count++;

        if (_warmup_sample_count >= BASELINE_WARMUP_SAMPLES) {
            _baseline_ready = true;
        }
    }
    else if (_baseline_update_enabled) {
        // --- Chế độ BÌNH THƯỜNG ---
        _baseline = _baseline * (1.0f - BASELINE_ALPHA)
                  + a_raw_mag * BASELINE_ALPHA;
    }
    // Nếu warmup xong nhưng _baseline_update_enabled == false
    // → không làm gì, baseline giữ nguyên (đang trong sự kiện)
}
