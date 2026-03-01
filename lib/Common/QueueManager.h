#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include "DataTypes.h"
#include "Config.h"

// ============================================================
//  QueueManager.h
//  Quản lý tập trung tất cả FreeRTOS Queue, EventGroup,
//  và Semaphore dùng chung giữa các khối.
//
//  Đây là Singleton — chỉ tồn tại một instance duy nhất.
//  SystemStateManager tạo và sở hữu QueueManager.
//  Các module khác chỉ nhận handle thông qua con trỏ/reference.
//
//  Nguyên tắc SOLID:
//  - Single Responsibility: chỉ quản lý giao tiếp inter-module
//  - Dependency Inversion: module nhận QueueHandle_t, không tự tạo
// ============================================================

class QueueManager {
public:
    // --------------------------------------------------------
    // Singleton access
    // --------------------------------------------------------
    static QueueManager& getInstance() {
        static QueueManager instance;
        return instance;
    }

    // Xóa copy/move để đảm bảo singleton
    QueueManager(const QueueManager&)             = delete;
    QueueManager& operator=(const QueueManager&)  = delete;

    // --------------------------------------------------------
    // Khởi tạo tất cả queue và synchronization primitives
    // Phải gọi trước khi bất kỳ task nào bắt đầu
    // Trả về: true nếu tất cả khởi tạo thành công
    // --------------------------------------------------------
    bool begin() {
        // --- Data pipeline queues ---
        _rawDataQueue = xQueueCreate(QUEUE_RAW_DATA_SIZE, sizeof(RawSensorData));
        if (_rawDataQueue == nullptr) return false;

        _normalizedQueue = xQueueCreate(QUEUE_NORMALIZED_SIZE, sizeof(NormalizedData));
        if (_normalizedQueue == nullptr) return false;

        _featuresQueue = xQueueCreate(QUEUE_FEATURES_SIZE, sizeof(MotionFeatures));
        if (_featuresQueue == nullptr) return false;

        _accidentQueue = xQueueCreate(QUEUE_ACCIDENT_SIZE, sizeof(AccidentEvent));
        if (_accidentQueue == nullptr) return false;

        _alertQueue = xQueueCreate(QUEUE_ALERT_SIZE, sizeof(AlertRequest));
        if (_alertQueue == nullptr) return false;

        // --- Watchdog heartbeat queue ---
        _watchdogQueue = xQueueCreate(
            static_cast<uint8_t>(TaskID::COUNT) * 2,
            sizeof(WatchdogHeartbeat)
        );
        if (_watchdogQueue == nullptr) return false;

        // --- Event group: hệ thống-wide events ---
        _systemEventGroup = xEventGroupCreate();
        if (_systemEventGroup == nullptr) return false;

        // --- Mutex bảo vệ truy cập Serial (dùng chung nhiều task) ---
        _serialMutex = xSemaphoreCreateMutex();
        if (_serialMutex == nullptr) return false;

        _initialized = true;
        return true;
    }

    // --------------------------------------------------------
    // Giải phóng tất cả resource (gọi khi shutdown)
    // --------------------------------------------------------
    void teardown() {
        if (_rawDataQueue)      { vQueueDelete(_rawDataQueue);      _rawDataQueue = nullptr; }
        if (_normalizedQueue)   { vQueueDelete(_normalizedQueue);   _normalizedQueue = nullptr; }
        if (_featuresQueue)     { vQueueDelete(_featuresQueue);     _featuresQueue = nullptr; }
        if (_accidentQueue)     { vQueueDelete(_accidentQueue);     _accidentQueue = nullptr; }
        if (_alertQueue)        { vQueueDelete(_alertQueue);        _alertQueue = nullptr; }
        if (_watchdogQueue)     { vQueueDelete(_watchdogQueue);     _watchdogQueue = nullptr; }
        if (_systemEventGroup)  { vEventGroupDelete(_systemEventGroup); _systemEventGroup = nullptr; }
        if (_serialMutex)       { vSemaphoreDelete(_serialMutex);   _serialMutex = nullptr; }
        _initialized = false;
    }

    // --------------------------------------------------------
    // Queue getters - trả về handle để module dùng trực tiếp
    // --------------------------------------------------------
    QueueHandle_t getRawDataQueue()     const { return _rawDataQueue; }
    QueueHandle_t getNormalizedQueue()  const { return _normalizedQueue; }
    QueueHandle_t getFeaturesQueue()    const { return _featuresQueue; }
    QueueHandle_t getAccidentQueue()    const { return _accidentQueue; }
    QueueHandle_t getAlertQueue()       const { return _alertQueue; }
    QueueHandle_t getWatchdogQueue()    const { return _watchdogQueue; }

    // --------------------------------------------------------
    // Synchronization getters
    // --------------------------------------------------------
    EventGroupHandle_t  getSystemEventGroup()   const { return _systemEventGroup; }
    SemaphoreHandle_t   getSerialMutex()        const { return _serialMutex; }

    // --------------------------------------------------------
    // System Event Bits (dùng với EventGroup)
    // Mỗi bit đại diện cho một sự kiện hệ thống quan trọng
    // --------------------------------------------------------
    static constexpr EventBits_t EVENT_CALIBRATION_DONE     = (1 << 0);
    static constexpr EventBits_t EVENT_ACCIDENT_DETECTED    = (1 << 1);
    static constexpr EventBits_t EVENT_ACCIDENT_CLEARED     = (1 << 2);
    static constexpr EventBits_t EVENT_SENSOR_ERROR         = (1 << 3);
    static constexpr EventBits_t EVENT_TASK_TIMEOUT         = (1 << 4);
    static constexpr EventBits_t EVENT_SYSTEM_READY         = (1 << 5);
    static constexpr EventBits_t EVENT_SHUTDOWN_REQUEST     = (1 << 6);

    // --------------------------------------------------------
    // Utility: Gửi heartbeat vào watchdog queue (non-blocking)
    // Các task gọi hàm này thay vì truy cập queue trực tiếp
    // --------------------------------------------------------
    void sendHeartbeat(TaskID taskId) {
        if (!_initialized || _watchdogQueue == nullptr) return;

        WatchdogHeartbeat hb;
        hb.taskId    = taskId;
        hb.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Non-blocking: nếu queue đầy thì bỏ qua heartbeat này
        xQueueSend(_watchdogQueue, &hb, 0);
    }

    // --------------------------------------------------------
    // Utility: Kiểm tra trạng thái các queue (debug)
    // --------------------------------------------------------
    void printQueueStatus() const {
        if (!_initialized) return;
        Serial.println("[QueueManager] Queue status:");
        Serial.printf("  rawData:    %d/%d\n",
            uxQueueMessagesWaiting(_rawDataQueue),    QUEUE_RAW_DATA_SIZE);
        Serial.printf("  normalized: %d/%d\n",
            uxQueueMessagesWaiting(_normalizedQueue), QUEUE_NORMALIZED_SIZE);
        Serial.printf("  features:   %d/%d\n",
            uxQueueMessagesWaiting(_featuresQueue),   QUEUE_FEATURES_SIZE);
        Serial.printf("  accident:   %d/%d\n",
            uxQueueMessagesWaiting(_accidentQueue),   QUEUE_ACCIDENT_SIZE);
        Serial.printf("  alert:      %d/%d\n",
            uxQueueMessagesWaiting(_alertQueue),      QUEUE_ALERT_SIZE);
    }

    bool isInitialized() const { return _initialized; }

private:
    QueueManager() = default;
    ~QueueManager() { teardown(); }

    // Data pipeline queues
    QueueHandle_t _rawDataQueue     = nullptr;
    QueueHandle_t _normalizedQueue  = nullptr;
    QueueHandle_t _featuresQueue    = nullptr;
    QueueHandle_t _accidentQueue    = nullptr;
    QueueHandle_t _alertQueue       = nullptr;

    // Watchdog heartbeat queue
    QueueHandle_t _watchdogQueue    = nullptr;

    // Synchronization
    EventGroupHandle_t  _systemEventGroup   = nullptr;
    SemaphoreHandle_t   _serialMutex        = nullptr;

    bool _initialized = false;
};
