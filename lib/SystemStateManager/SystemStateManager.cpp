#include "SystemStateManager.h"
#include "MotionSensor.h"
#include "DataNormalizer.h"
#include "SignalProcessor.h"
#include "AccidentDetector.h"
#include "AlertManager.h"

// Chu kỳ kiểm tra sức khỏe module (ms)
static constexpr uint32_t HEALTH_CHECK_PERIOD_MS = 2000;

// ============================================================
//  Constructor
// ============================================================
SystemStateManager::SystemStateManager(
    MotionSensor*     motionSensor,
    DataNormalizer*   dataNormalizer,
    SignalProcessor*  signalProcessor,
    AccidentDetector* accidentDetector,
    AlertManager*     alertManager,
    QueueManager&     queueManager
)
    : _motionSensor(motionSensor)
    , _dataNormalizer(dataNormalizer)
    , _signalProcessor(signalProcessor)
    , _accidentDetector(accidentDetector)
    , _alertManager(alertManager)
    , _queueManager(queueManager)
    , _taskHandle(nullptr)
    , _taskRunning(false)
    , _systemState(SystemState::BOOT)
    , _lastError(SystemError::NONE)
    , _startMs(millis())
    , _lastHealthCheckMs(0)
    , _lastAccidentType(AccidentType::NONE)
{}

// ============================================================
//  begin
// ============================================================
bool SystemStateManager::begin() {
    transitionTo(SystemState::INIT);

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "StateManager",
        STACK_STATE_MANAGER,
        this,
        PRIORITY_STATE_MANAGER,
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

// ============================================================
//  FreeRTOS Task entry
// ============================================================
void SystemStateManager::taskEntry(void* pvParameters) {
    static_cast<SystemStateManager*>(pvParameters)->taskRun();
}

// ============================================================
//  taskRun — vòng lặp chính
//
//  Chiến lược:
//    1. Chờ EventGroup với timeout HEALTH_CHECK_PERIOD_MS
//    2. Nếu có event → xử lý ngay (phản ứng nhanh)
//    3. Nếu timeout  → kiểm tra sức khỏe định kỳ
// ============================================================
void SystemStateManager::taskRun() {
    Logger::info(getModuleName(), "Task started");
    transitionTo(SystemState::RUNNING);

    EventGroupHandle_t eg = _queueManager.getSystemEventGroup();

    // Các bit sự kiện cần lắng nghe
    const EventBits_t WATCH_BITS =
        QueueManager::EVENT_ACCIDENT_DETECTED |
        QueueManager::EVENT_ACCIDENT_CLEARED  |
        QueueManager::EVENT_SENSOR_ERROR      |
        QueueManager::EVENT_TASK_TIMEOUT;

    while (true) {
        // Chờ bất kỳ bit nào được set, tự động clear sau khi đọc
        EventBits_t bits = xEventGroupWaitBits(
            eg,
            WATCH_BITS,
            pdTRUE,     // clear on exit
            pdFALSE,    // any bit (không cần tất cả)
            pdMS_TO_TICKS(HEALTH_CHECK_PERIOD_MS)
        );

        // --- Xử lý sự kiện tai nạn ---
        if (bits & QueueManager::EVENT_ACCIDENT_DETECTED) {
            onAccidentDetected();
        }

        if (bits & QueueManager::EVENT_ACCIDENT_CLEARED) {
            onAccidentCleared();
        }

        if (bits & QueueManager::EVENT_SENSOR_ERROR) {
            onSensorError();
        }

        if (bits & QueueManager::EVENT_TASK_TIMEOUT) {
            Logger::error(getModuleName(), "EVENT: TASK_TIMEOUT from Watchdog");
            if (_systemState != SystemState::ERROR) {
                transitionTo(SystemState::ERROR);
                _lastError = SystemError::TASK_TIMEOUT;
            }
        }

        // --- Kiểm tra sức khỏe định kỳ (timeout hoặc sau event) ---
        uint32_t now = millis();
        if (now - _lastHealthCheckMs >= HEALTH_CHECK_PERIOD_MS) {
            _lastHealthCheckMs = now;

            if (!checkAllModulesHealth()) {
                if (_systemState != SystemState::ERROR) {
                    transitionTo(SystemState::ERROR);
                    _lastError = SystemError::TASK_TIMEOUT;
                    Logger::error(getModuleName(),
                        "Module health check FAILED");
                }
            } else {
                // Nếu đang ERROR và tất cả module đã hồi phục
                if (_systemState == SystemState::ERROR) {
                    transitionTo(SystemState::RECOVERY);
                    Logger::info(getModuleName(),
                        "All modules healthy — recovering");
                }
            }
        }
        // Gửi heartbeat cho Watchdog mỗi vòng lặp
        _queueManager.sendHeartbeat(TaskID::STATE_MANAGER);
    }
}

// ============================================================
//  onAccidentDetected
//  Đọc AccidentType từ AccidentDetector rồi gửi START_ALERT
// ============================================================
void SystemStateManager::onAccidentDetected() {
    Logger::info(getModuleName(), "EVENT: ACCIDENT_DETECTED");

    // Lấy loại tai nạn từ AccidentDetector
    _lastAccidentType = _accidentDetector->getLastAccidentType();

    Logger::info(getModuleName(),
        accidentTypeToString(_lastAccidentType));

    // Ra lệnh cho AlertManager bật buzzer
    sendAlertCommand(AlertCommand::START_ALERT, _lastAccidentType);
}

// ============================================================
//  onAccidentCleared
//  Gửi STOP_ALERT — buzzer tắt sau khi accident reset
// ============================================================
void SystemStateManager::onAccidentCleared() {
    Logger::info(getModuleName(), "EVENT: ACCIDENT_CLEARED");

    sendAlertCommand(AlertCommand::STOP_ALERT);
    _lastAccidentType = AccidentType::NONE;
}

// ============================================================
//  onSensorError
// ============================================================
void SystemStateManager::onSensorError() {
    Logger::error(getModuleName(), "EVENT: SENSOR_ERROR");
    transitionTo(SystemState::ERROR);
    _lastError = SystemError::SENSOR_DISCONNECTED;
}

// ============================================================
//  checkAllModulesHealth
// ============================================================
bool SystemStateManager::checkAllModulesHealth() {
    bool allOk = true;

    if (!_motionSensor->isHealthy()) {
        Logger::warn(getModuleName(), "K1 MotionSensor unhealthy");
        allOk = false;
    }
    if (!_dataNormalizer->isHealthy()) {
        Logger::warn(getModuleName(), "K2 DataNormalizer unhealthy");
        allOk = false;
    }
    if (!_signalProcessor->isHealthy()) {
        Logger::warn(getModuleName(), "K3 SignalProcessor unhealthy");
        allOk = false;
    }
    if (!_accidentDetector->isHealthy()) {
        Logger::warn(getModuleName(), "K4 AccidentDetector unhealthy");
        allOk = false;
    }
    if (!_alertManager->isHealthy()) {
        Logger::warn(getModuleName(), "K5 AlertManager unhealthy");
        allOk = false;
    }

    return allOk;
}

// ============================================================
//  sendAlertCommand
// ============================================================
void SystemStateManager::sendAlertCommand(AlertCommand cmd,
                                           AccidentType type) {
    AlertRequest req;
    req.command      = cmd;
    req.accidentType = type;
    req.timestamp    = millis();

    QueueHandle_t alertQ = _queueManager.getAlertQueue();
    if (xQueueSend(alertQ, &req,
                   pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS)) != pdTRUE) {
        Logger::warn(getModuleName(), "alertQueue full — command dropped");
    }
}

// ============================================================
//  transitionTo
// ============================================================
void SystemStateManager::transitionTo(SystemState newState) {
    if (_systemState == newState) return;

    Logger::info(getModuleName(), systemStateToString(newState));
    _systemState = newState;
}
