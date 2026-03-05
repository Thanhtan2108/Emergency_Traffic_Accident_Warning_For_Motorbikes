#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <IModule.h>
#include <DataTypes.h>
#include <Config.h>
#include <QueueManager.h>

// ============================================================
//  AccidentDetector.h  —  Khối 4: Phát hiện tai nạn
//
//  Trách nhiệm duy nhất (SRP):
//    - Nhận MotionFeatures từ featuresQueue (Khối 3)
//    - Phân tích theo máy trạng thái: UNKNOWN→NORMAL→SUSPICIOUS→ACCIDENT
//    - Áp dụng debounce: phải vượt ngưỡng N frame liên tiếp mới xác nhận
//    - Phân loại loại tai nạn: CRASH / FALL / SUDDEN_STOP / COMBINED
//    - Gửi AccidentEvent vào accidentQueue (→ Khối 5 AlertManager)
//    - Set EventBit EVENT_ACCIDENT_DETECTED trên SystemEventGroup
//
//  State Machine:
//    UNKNOWN   → NORMAL     : frame đầu tiên hợp lệ
//    NORMAL    → SUSPICIOUS : phát hiện bất thường 1 frame
//    SUSPICIOUS→ NORMAL     : NORMAL_CONFIRM_FRAMES frame bình thường liên tiếp
//    SUSPICIOUS→ ACCIDENT   : ACCIDENT_CONFIRM_FRAMES frame bất thường liên tiếp
//    ACCIDENT  → NORMAL     : tự reset sau ACCIDENT_HOLD_MS (30 giây)
//
//  Ngưỡng phát hiện (từ Config.h):
//    CRASH      : totalAccMag  > ACC_CRASH_THRESHOLD    (2.5g)
//    FALL       : |angleX/Y|   > TILT_ANGLE_THRESHOLD   (60°)
//    SUDDEN_STOP: jerk         > JERK_THRESHOLD          (10 g/s)
//    COMBINED   : từ 2 điều kiện trở lên cùng lúc
// ============================================================

class AccidentDetector : public IModule {
public:
    // --------------------------------------------------------
    // Constructor
    // inQueue      : featuresQueue  — nhận từ SignalProcessor
    // outQueue     : accidentQueue  — gửi sang AlertManager
    // eventGroup   : SystemEventGroup — set event bit khi có tai nạn
    // --------------------------------------------------------
    AccidentDetector(QueueHandle_t      inQueue,
                     QueueHandle_t      outQueue,
                     EventGroupHandle_t eventGroup);

    // --------------------------------------------------------
    // IModule interface
    // --------------------------------------------------------
    bool        begin()         override;
    const char* getModuleName() const override { return "AccidentDetector"; }
    bool        isHealthy()     const override;

    // --------------------------------------------------------
    // Getters để monitor / test
    // --------------------------------------------------------
    VehicleState getCurrentState()    const { return _state; }
    AccidentType getLastAccidentType()const { return _lastAccidentType; }
    uint32_t     getProcessedCount()  const { return _processedCount; }
    uint32_t     getAccidentCount()   const { return _accidentCount; }

private:
    // --------------------------------------------------------
    // FreeRTOS Task
    // --------------------------------------------------------
    static void taskEntry(void* pvParameters);
    void        taskRun();

    // --------------------------------------------------------
    // State machine core
    // --------------------------------------------------------

    // Kiểm tra các ngưỡng → trả về AccidentType (NONE nếu bình thường)
    AccidentType analyzeFeatures(const MotionFeatures& f) const;

    // Xử lý chuyển trạng thái dựa trên kết quả phân tích
    void updateStateMachine(AccidentType detected, const MotionFeatures& f);

    // Tạo và gửi AccidentEvent vào accidentQueue
    void publishEvent(const MotionFeatures& f, bool isActive);

    // --------------------------------------------------------
    // State machine variables
    // --------------------------------------------------------
    VehicleState _state;                // Trạng thái hiện tại
    AccidentType _lastAccidentType;     // Loại tai nạn cuối cùng

    // Debounce counters
    uint8_t  _suspiciousFrameCount;     // Số frame bất thường liên tiếp
    uint8_t  _normalFrameCount;         // Số frame bình thường liên tiếp

    // Snapshot tại thời điểm phát hiện (để gửi vào AccidentEvent)
    MotionFeatures _triggerSnapshot;

    // Thời điểm vào trạng thái ACCIDENT (để tự reset sau 30s)
    uint32_t _accidentStartMs;

    // --------------------------------------------------------
    // Queue & sync handles
    // --------------------------------------------------------
    QueueHandle_t      _inQueue;
    QueueHandle_t      _outQueue;
    EventGroupHandle_t _eventGroup;
    TaskHandle_t       _taskHandle;

    volatile bool _taskRunning;
    uint32_t      _processedCount;
    uint32_t      _accidentCount;
};
