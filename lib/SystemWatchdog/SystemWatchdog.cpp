#include "SystemWatchdog.h"

// ============================================================
//  Constructor
// ============================================================
SystemWatchdog::SystemWatchdog(QueueHandle_t     watchdogQueue,
                               EventGroupHandle_t systemEventGroup)
    : _watchdogQueue(watchdogQueue)
    , _systemEventGroup(systemEventGroup)
    , _taskHandle(nullptr)
    , _taskRunning(false)
    , _hasTimeout(false)
    , _timedOutTaskId(TaskID::MOTION_SENSOR)
    , _timeoutCount(0)
{
    // Khởi tạo bảng theo dõi — tất cả chưa đăng ký
    for (uint8_t i = 0; i < static_cast<uint8_t>(TaskID::COUNT); i++) {
        _tasks[i].taskId      = static_cast<TaskID>(i);
        _tasks[i].taskName    = taskIdToString(static_cast<TaskID>(i));
        _tasks[i].lastSeenMs  = 0;
        _tasks[i].registered  = false;
        _tasks[i].timedOut    = false;
    }
}

// ============================================================
//  begin
// ============================================================
bool SystemWatchdog::begin() {
    if (_watchdogQueue == nullptr) {
        Logger::error(getModuleName(), "watchdogQueue null");
        return false;
    }
    if (_systemEventGroup == nullptr) {
        Logger::error(getModuleName(), "eventGroup null");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "SystemWatchdog",
        STACK_SYSTEM_WATCHDOG,
        this,
        PRIORITY_SYSTEM_WATCHDOG,
        &_taskHandle,
        CORE_MANAGEMENT_TASKS
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Task creation failed");
        return false;
    }

    _taskRunning = true;
    Logger::info(getModuleName(), "begin() OK");
    return true;
}

bool SystemWatchdog::isHealthy() const {
    return _taskRunning && (_taskHandle != nullptr);
}

// ============================================================
//  registerTask
//  Đăng ký task và seed lastSeen = now để tránh false timeout
//  ngay khi task vừa được đăng ký nhưng chưa kịp gửi heartbeat
// ============================================================
void SystemWatchdog::registerTask(TaskID taskId, const char* taskName) {
    uint8_t idx = static_cast<uint8_t>(taskId);
    if (idx >= static_cast<uint8_t>(TaskID::COUNT)) return;

    _tasks[idx].taskName   = taskName;
    _tasks[idx].lastSeenMs = millis();  // seed thời điểm đăng ký
    _tasks[idx].registered = true;
    _tasks[idx].timedOut   = false;

    Logger::info(getModuleName(), taskName);
}

const char* SystemWatchdog::getTimedOutTaskName() const {
    uint8_t idx = static_cast<uint8_t>(_timedOutTaskId);
    if (idx >= static_cast<uint8_t>(TaskID::COUNT)) return "Unknown";
    return _tasks[idx].taskName;
}

// ============================================================
//  FreeRTOS Task
// ============================================================
void SystemWatchdog::taskEntry(void* pvParameters) {
    static_cast<SystemWatchdog*>(pvParameters)->taskRun();
}

// ============================================================
//  taskRun
//  Vòng lặp nhanh (100ms):
//    1. Drain toàn bộ heartbeat queue → cập nhật lastSeen
//    2. Quét bảng task → kiểm tra timeout
// ============================================================
void SystemWatchdog::taskRun() {
    Logger::info(getModuleName(), "Task started");

    while (true) {
        // Bước 1: Cập nhật lastSeen từ tất cả heartbeat
        drainHeartbeatQueue();

        // Bước 2: Kiểm tra timeout
        checkAllTasks();

        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));
    }
}

// ============================================================
//  drainHeartbeatQueue
//  Đọc hết tất cả heartbeat đang chờ trong queue (non-blocking)
//  Cập nhật lastSeen cho mỗi task
// ============================================================
void SystemWatchdog::drainHeartbeatQueue() {
    WatchdogHeartbeat hb;

    // Đọc hết queue trong 1 lần — không block
    while (xQueueReceive(_watchdogQueue, &hb, 0) == pdTRUE) {
        uint8_t idx = static_cast<uint8_t>(hb.taskId);
        if (idx >= static_cast<uint8_t>(TaskID::COUNT)) continue;

        _tasks[idx].lastSeenMs = hb.timestamp;

        // Nếu task đã timeout trước đó mà nay gửi heartbeat lại
        // → reset trạng thái timeout
        if (_tasks[idx].timedOut) {
            _tasks[idx].timedOut = false;
            Logger::info(getModuleName(), _tasks[idx].taskName);
            Logger::info(getModuleName(), "recovered");
        }
    }
}

// ============================================================
//  checkAllTasks
//  Quét bảng task đã đăng ký
//  Nếu (now - lastSeen) > WATCHDOG_TIMEOUT_MS → timeout
// ============================================================
void SystemWatchdog::checkAllTasks() {
    uint32_t now = millis();

    for (uint8_t i = 0; i < static_cast<uint8_t>(TaskID::COUNT); i++) {
        if (!_tasks[i].registered) continue;
        if (_tasks[i].timedOut)    continue; // đã báo rồi, không báo lại

        uint32_t elapsed = now - _tasks[i].lastSeenMs;

        if (elapsed > WATCHDOG_TIMEOUT_MS) {
            // Task bị treo!
            _tasks[i].timedOut  = true;
            _hasTimeout         = true;
            _timedOutTaskId     = _tasks[i].taskId;
            _timeoutCount++;

            Logger::error(getModuleName(), _tasks[i].taskName);
            Logger::error(getModuleName(), "TIMEOUT — task hung!");

            // Thông báo cho K7 SystemStateManager qua EventGroup
            xEventGroupSetBits(
                _systemEventGroup,
                QueueManager::EVENT_TASK_TIMEOUT
            );
        }
    }

    // Reset cờ nếu không còn task nào timeout
    bool anyTimeout = false;
    for (uint8_t i = 0; i < static_cast<uint8_t>(TaskID::COUNT); i++) {
        if (_tasks[i].timedOut) { anyTimeout = true; break; }
    }
    _hasTimeout = anyTimeout;
}
