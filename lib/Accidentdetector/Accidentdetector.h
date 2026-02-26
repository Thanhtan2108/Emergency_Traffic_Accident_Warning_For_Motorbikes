#pragma once
// ============================================================
//  AccidentDetector.h
//  Tầng 3 – State Machine phát hiện và phân loại tai nạn.
//
//  TRÁCH NHIỆM:
//    Nhận ProcessedData mỗi chu kỳ 10ms từ StateTask,
//    chạy State Machine 10 state, phát AlertInfo khi xác nhận
//    STRONG_IMPACT hoặc FALL_DETECTED.
//
//  KHÔNG BIẾT GÌ VỀ: FreeRTOS, Queue, EventGroup, Serial.
//    → Tất cả giao tiếp ra ngoài qua AlertInfo và các getter.
//    → StateTask chịu trách nhiệm đẩy alert vào Queue/EventGroup.
//
//  Tài liệu: README.md – Section VII
//  Thứ tự implement: file số 5 (sau SignalProcessor)
// ============================================================

#include <Arduino.h>
#include "DataTypes.h"
#include "config.h"
#include "SignalProcessor.h"


class AccidentDetector {
public:

    // --------------------------------------------------------
    //  Constructor
    //  Nhận tham chiếu đến SignalProcessor để gọi
    //  setBaselineUpdateEnabled() và resetFilter() khi cần.
    // --------------------------------------------------------
    explicit AccidentDetector(SignalProcessor& sp);

    // --------------------------------------------------------
    //  begin()
    //  Gọi một lần trong setup(), sau signalProcessor.begin().
    //
    //  Sau khi gọi:
    //    - state           = WARMUP
    //    - warmup_start    = millis()
    //    - EventSnapshot   reset về 0
    //    - debounce_timer  = 0
    //    - _new_alert      = false
    // --------------------------------------------------------
    void begin();

    // --------------------------------------------------------
    //  process()
    //  Gọi mỗi chu kỳ từ StateTask (mỗi 10ms).
    //  Chạy handler của state hiện tại, thực hiện transition
    //  nếu điều kiện thỏa, cập nhật state_enter_time khi đổi state.
    // --------------------------------------------------------
    void process(const ProcessedData& data);

    // --------------------------------------------------------
    //  Getters – StateTask và WatchdogTask dùng
    // --------------------------------------------------------
    DetectorState getState()          const;
    uint32_t      getStateEnterTime() const;  // millis() khi vào state hiện tại

    // --------------------------------------------------------
    //  Alert interface
    //  StateTask gọi hasNewAlert() mỗi chu kỳ.
    //  Nếu true, gọi getLastAlert() để lấy payload rồi
    //  đẩy vào Queue/EventGroup.
    //  Flag _new_alert tự động xóa sau khi getLastAlert() được gọi.
    // --------------------------------------------------------
    bool      hasNewAlert()  const;
    AlertInfo getLastAlert();           // clears _new_alert flag

    // --------------------------------------------------------
    //  Trạng thái hệ thống
    // --------------------------------------------------------
    bool isInDebounce()  const;   // true nếu đang trong T_DEBOUNCE
    bool isWarmingUp()   const;   // true nếu state == WARMUP

    // --------------------------------------------------------
    //  forceReset()
    //  WatchdogTask gọi khi phát hiện state kẹt quá T_STATE_MAX_TIMEOUT.
    //
    //  Thực hiện:
    //    - state → NORMAL
    //    - Reset EventSnapshot về 0
    //    - signalProcessor.setBaselineUpdateEnabled(true)
    //    - signalProcessor.resetFilter()
    //
    //  KHÔNG reset: debounce_timer, baseline
    // --------------------------------------------------------
    void forceReset();

private:
    // Tham chiếu đến SignalProcessor (inject qua constructor)
    SignalProcessor& _sp;

    // State hiện tại và thời điểm vào state
    DetectorState _state;
    uint32_t      _state_enter_time;  // millis() khi transition vào state này

    // EventSnapshot – thu thập trong EVENT_DETECTED, dùng trong CLASSIFYING
    EventSnapshot _snapshot;

    // Alert
    AlertInfo _last_alert;
    bool      _new_alert;

    // Debounce – chống báo lại cùng 1 sự kiện
    uint32_t _debounce_timer;   // millis() khi bắt đầu debounce (0 = không debounce)

    // WARMUP
    uint32_t _warmup_start;     // millis() khi begin() được gọi

    // FALL_PENDING – đếm thời gian roll liên tục vượt ngưỡng
    uint32_t _sustained_fall_start;  // millis() khi roll lần đầu vượt ROLL_FALL_THRESHOLD

    // --------------------------------------------------------
    //  Transition helper – đổi state và ghi lại thời điểm vào
    // --------------------------------------------------------
    void _transitionTo(DetectorState new_state);

    // --------------------------------------------------------
    //  Handler cho từng state
    //  Mỗi hàm kiểm tra điều kiện và gọi _transitionTo() nếu cần.
    // --------------------------------------------------------
    void _handleWarmup        (const ProcessedData& d);
    void _handleNormal        (const ProcessedData& d);
    void _handleEventDetected (const ProcessedData& d);
    void _handleClassifying   (const ProcessedData& d);
    void _handleRoadShock     (const ProcessedData& d);
    void _handleImpactPending (const ProcessedData& d);
    void _handleFallPending   (const ProcessedData& d);
    void _handleStrongImpact  (const ProcessedData& d);
    void _handleFallDetected  (const ProcessedData& d);
    void _handleTimeoutReset  (const ProcessedData& d);

    // --------------------------------------------------------
    //  Alert builder – tạo AlertInfo từ snapshot + processed
    // --------------------------------------------------------
    AlertInfo _buildAlert(AlertType type, const ProcessedData& d) const;

    // --------------------------------------------------------
    //  Helpers
    // --------------------------------------------------------
    void  _resetSnapshot();
    bool  _isDebounceActive() const;
};
