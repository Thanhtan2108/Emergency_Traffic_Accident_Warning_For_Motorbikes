#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "IModule.h"
#include "DataTypes.h"
#include "Config.h"
#include "QueueManager.h"

// ============================================================
//  SystemStateManager.h  —  Khối 7: Bộ não hệ thống
//
//  Trách nhiệm:
//    - Giám sát sức khỏe tất cả các khối (K1-K5)
//    - Lắng nghe EventGroup để phản ứng với sự kiện hệ thống
//    - Điều phối: khi có tai nạn → ra lệnh cho AlertManager
//    - Điều phối: khi tai nạn kết thúc → tắt AlertManager
//    - Duy trì SystemState tổng thể (BOOT/RUNNING/ERROR/...)
//    - Phát hiện module không healthy → chuyển sang ERROR
//
//  Luồng sự kiện chính:
//    EVENT_ACCIDENT_DETECTED  → gửi START_ALERT vào alertQueue
//    EVENT_ACCIDENT_CLEARED   → gửi STOP_ALERT  vào alertQueue
//    EVENT_SENSOR_ERROR       → chuyển SystemState → ERROR
//    EVENT_SYSTEM_READY       → chuyển SystemState → RUNNING
//
//  Khác với các khối khác:
//    - KHÔNG xử lý dữ liệu cảm biến
//    - KHÔNG kế thừa IModule (vì nó tạo và sở hữu các module)
//    - Chạy ở Priority cao nhất (6) để phản ứng nhanh
// ============================================================

// Forward declarations
class MotionSensor;
class DataNormalizer;
class SignalProcessor;
class AccidentDetector;
class AlertManager;

class SystemStateManager {
public:
    // --------------------------------------------------------
    // Constructor — nhận tất cả module đã được khởi tạo
    // --------------------------------------------------------
    SystemStateManager(
        MotionSensor*     motionSensor,
        DataNormalizer*   dataNormalizer,
        SignalProcessor*  signalProcessor,
        AccidentDetector* accidentDetector,
        AlertManager*     alertManager,
        QueueManager&     queueManager
    );

    // --------------------------------------------------------
    // Khởi động task giám sát
    // --------------------------------------------------------
    bool begin();

    // --------------------------------------------------------
    // Getters
    // --------------------------------------------------------
    SystemState getCurrentState()   const { return _systemState; }
    SystemError getLastError()      const { return _lastError; }
    bool        isHealthy()         const { return _taskRunning; }
    uint32_t    getUptimeMs()       const { return millis() - _startMs; }

    const char* getModuleName()     const { return "SystemStateManager"; }

private:
    // --------------------------------------------------------
    // FreeRTOS Task
    // --------------------------------------------------------
    static void taskEntry(void* pvParameters);
    void        taskRun();

    // --------------------------------------------------------
    // Xử lý từng loại sự kiện từ EventGroup
    // --------------------------------------------------------
    void onAccidentDetected();
    void onAccidentCleared();
    void onSensorError();

    // --------------------------------------------------------
    // Kiểm tra sức khỏe tất cả module định kỳ
    // Trả về true nếu tất cả healthy
    // --------------------------------------------------------
    bool checkAllModulesHealth();

    // --------------------------------------------------------
    // Gửi lệnh vào alertQueue (helper)
    // --------------------------------------------------------
    void sendAlertCommand(AlertCommand cmd,
                          AccidentType type = AccidentType::NONE);

    // --------------------------------------------------------
    // Chuyển đổi SystemState có log
    // --------------------------------------------------------
    void transitionTo(SystemState newState);

    // --------------------------------------------------------
    // Members — các module được giám sát
    // --------------------------------------------------------
    MotionSensor*     _motionSensor;
    DataNormalizer*   _dataNormalizer;
    SignalProcessor*  _signalProcessor;
    AccidentDetector* _accidentDetector;
    AlertManager*     _alertManager;
    QueueManager&     _queueManager;

    TaskHandle_t      _taskHandle;
    volatile bool     _taskRunning;

    SystemState       _systemState;
    SystemError       _lastError;

    uint32_t          _startMs;
    uint32_t          _lastHealthCheckMs;

    // Lưu AccidentType gần nhất để gửi kèm START_ALERT
    AccidentType      _lastAccidentType;
};
