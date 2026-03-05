#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include "IModule.h"
#include "DataTypes.h"
#include "Config.h"
#include "QueueManager.h"

// ============================================================
//  SystemWatchdog.h  —  Khối 6: Giám sát hạ tầng task
//
//  Trách nhiệm duy nhất (SRP):
//    Giám sát xem mỗi task có còn sống không bằng cơ chế
//    heartbeat — mỗi task gửi WatchdogHeartbeat vào
//    watchdogQueue định kỳ, Watchdog kiểm tra timestamp.
//
//  Cơ chế hoạt động:
//    1. Mỗi task đăng ký với Watchdog qua register()
//    2. Mỗi task gọi QueueManager::sendHeartbeat() định kỳ
//    3. Watchdog đọc watchdogQueue, cập nhật lastSeen[taskId]
//    4. Mỗi WATCHDOG_CHECK_PERIOD_MS (100ms), quét tất cả task
//       đã đăng ký — nếu (now - lastSeen) > WATCHDOG_TIMEOUT_MS
//       → task đó bị coi là treo
//    5. Set EVENT_TASK_TIMEOUT vào EventGroup
//    6. K7 SystemStateManager nhận event và xử lý
//
//  Khác biệt với K7:
//    K6 = giám sát hạ tầng (task còn sống không?)
//    K7 = điều phối nghiệp vụ (tai nạn → làm gì?)
//
//  Không phụ thuộc vào bất kỳ module nghiệp vụ nào.
//  Chỉ cần watchdogQueue và systemEventGroup.
// ============================================================

// Thông tin theo dõi mỗi task
struct TaskWatchInfo {
    TaskID      taskId;
    const char* taskName;
    uint32_t    lastSeenMs;     // millis() của heartbeat gần nhất
    bool        registered;     // true = đang được giám sát
    bool        timedOut;       // true = đã phát hiện timeout lần này
};

class SystemWatchdog : public IModule {
public:
    // --------------------------------------------------------
    // Constructor
    //   watchdogQueue    : nhận WatchdogHeartbeat từ các task
    //   systemEventGroup : set EVENT_TASK_TIMEOUT khi phát hiện
    // --------------------------------------------------------
    SystemWatchdog(QueueHandle_t     watchdogQueue,
                   EventGroupHandle_t systemEventGroup);

    // --------------------------------------------------------
    // IModule interface
    // --------------------------------------------------------
    bool        begin()         override;
    const char* getModuleName() const override { return "SystemWatchdog"; }
    bool        isHealthy()     const override;

    // --------------------------------------------------------
    // Đăng ký task để giám sát
    // Phải gọi trước khi task bắt đầu gửi heartbeat
    // --------------------------------------------------------
    void registerTask(TaskID taskId, const char* taskName);

    // --------------------------------------------------------
    // Getters
    // --------------------------------------------------------
    bool     hasTimedOutTask()    const { return _hasTimeout; }
    TaskID   getTimedOutTaskId()  const { return _timedOutTaskId; }
    uint32_t getTimeoutCount()    const { return _timeoutCount; }

    // Lấy tên task bị timeout gần nhất
    const char* getTimedOutTaskName() const;

private:
    static void taskEntry(void* pvParameters);
    void        taskRun();

    // Đọc tất cả heartbeat trong queue, cập nhật lastSeen
    void drainHeartbeatQueue();

    // Quét tất cả task đã đăng ký, kiểm tra timeout
    void checkAllTasks();

    // --------------------------------------------------------
    // Members
    // --------------------------------------------------------
    QueueHandle_t       _watchdogQueue;
    EventGroupHandle_t  _systemEventGroup;
    TaskHandle_t        _taskHandle;

    volatile bool       _taskRunning;
    volatile bool       _hasTimeout;
    TaskID              _timedOutTaskId;
    uint32_t            _timeoutCount;

    // Bảng theo dõi tất cả task
    TaskWatchInfo       _tasks[static_cast<uint8_t>(TaskID::COUNT)];
};
