#include "AlertManager.h"

// ============================================================
//  Pattern beep theo loại tai nạn
// ============================================================
static constexpr uint32_t PATTERN_CRASH_ON_MS      = 200;
static constexpr uint32_t PATTERN_CRASH_OFF_MS     = 100;

static constexpr uint32_t PATTERN_FALL_ON_MS       = 500;
static constexpr uint32_t PATTERN_FALL_OFF_MS      = 300;

static constexpr uint32_t PATTERN_SUDDEN_ON_MS     = 300;
static constexpr uint32_t PATTERN_SUDDEN_OFF_MS    = 200;

static constexpr uint32_t PATTERN_COMBINED_ON_MS   = 100;
static constexpr uint32_t PATTERN_COMBINED_OFF_MS  = 100;

// Bước kiểm tra cờ trong interruptibleDelay (ms)
static constexpr uint32_t DELAY_CHECK_STEP_MS      = 10;

// ============================================================
//  Constructor / begin
// ============================================================
AlertManager::AlertManager(QueueHandle_t alertQueue, Buzzer* buzzer)
    : _alertQueue(alertQueue)
    , _buzzer(buzzer)
    , _taskHandle(nullptr)
    , _taskRunning(false)
    , _buzzerEnabled(false)
    , _isAlerting(false)
    , _alertCount(0)
    , _processedCount(0)
{}

bool AlertManager::begin() {
    if (_alertQueue == nullptr) {
        Logger::error(getModuleName(), "alertQueue null");
        return false;
    }
    if (_buzzer == nullptr) {
        Logger::error(getModuleName(), "buzzer null");
        return false;
    }

    // Đảm bảo buzzer tắt khi khởi động
    _buzzer->begin();
    _buzzer->turnOff();
    _buzzerEnabled = false;

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "AlertManager",
        STACK_ALERT_MANAGER,
        this,
        PRIORITY_ALERT_MANAGER,
        &_taskHandle,
        CORE_MANAGEMENT_TASKS
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Task creation failed");
        return false;
    }

    _taskRunning = true;
    Logger::info(getModuleName(), "begin() OK — buzzer OFF");
    return true;
}

bool AlertManager::isHealthy() const {
    return _taskRunning && (_taskHandle != nullptr);
}

// ============================================================
//  setBuzzerEnabled — dùng cho nút nhấn sau này
//  Có thể gọi từ bất kỳ task nào, kể cả ISR (volatile)
// ============================================================
void AlertManager::setBuzzerEnabled(bool enabled) {
    _buzzerEnabled = enabled;
    if (!enabled) {
        // Tắt buzzer ngay lập tức khi cờ bị hạ
        _buzzer->turnOff();
        Logger::info(getModuleName(), "setBuzzerEnabled → OFF");
    } else {
        Logger::info(getModuleName(), "setBuzzerEnabled → ON");
    }
}

// ============================================================
//  FreeRTOS Task
// ============================================================
void AlertManager::taskEntry(void* pvParameters) {
    static_cast<AlertManager*>(pvParameters)->taskRun();
}

void AlertManager::taskRun() {
    Logger::info(getModuleName(), "Task started");

    AlertRequest req;

    while (true) {
        if (xQueueReceive(_alertQueue, &req,
                          pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
        {
            _processedCount++;

            switch (req.command) {
                case AlertCommand::START_ALERT:
                    Logger::info(getModuleName(), "START_ALERT");
                    Logger::info(getModuleName(),
                        accidentTypeToString(req.accidentType));
                    handleStartAlert(req.accidentType);
                    break;

                case AlertCommand::STOP_ALERT:
                    Logger::info(getModuleName(), "STOP_ALERT");
                    handleStopAlert();
                    break;

                case AlertCommand::NONE:
                default:
                    break;
            }
        }
        // Gửi heartbeat mỗi vòng lặp (kể cả timeout)
        QueueManager::getInstance().sendHeartbeat(TaskID::ALERT_MANAGER);
    }
}

// ============================================================
//  handleStartAlert
//  Đặt cờ _buzzerEnabled = true
//  Beep liên tục theo pattern cho đến khi _buzzerEnabled = false
// ============================================================
void AlertManager::handleStartAlert(AccidentType type) {
    _buzzerEnabled = true;
    _isAlerting    = true;
    _alertCount++;

    uint32_t onMs  = 0;
    uint32_t offMs = 0;
    getBeepPattern(type, onMs, offMs);

    while (_buzzerEnabled) {
        // --- Pha ON ---
        _buzzer->turnOn();
        bool cont = interruptibleDelay(onMs);
        if (!cont) {
            _buzzer->turnOff();
            break;
        }

        // --- Pha OFF ---
        _buzzer->turnOff();
        cont = interruptibleDelay(offMs);
        if (!cont) {
            break;
        }

        // Kiểm tra có request mới trong queue không
        // (ví dụ: tai nạn mới với type khác → override pattern)
        AlertRequest newReq;
        if (xQueueReceive(_alertQueue, &newReq, 0) == pdTRUE) {
            _processedCount++;
            if (newReq.command == AlertCommand::STOP_ALERT) {
                handleStopAlert();
                _isAlerting = false;
                return;
            }
            if (newReq.command == AlertCommand::START_ALERT) {
                // Tai nạn mới: đổi pattern ngay
                Logger::info(getModuleName(), "Pattern override");
                Logger::info(getModuleName(),
                    accidentTypeToString(newReq.accidentType));
                _alertCount++;
                getBeepPattern(newReq.accidentType, onMs, offMs);
            }
        }
    }

    _buzzer->turnOff();
    _isAlerting = false;
}

// ============================================================
//  handleStopAlert
//  Hạ cờ _buzzerEnabled → vòng while trong handleStartAlert
//  sẽ thoát trong vòng DELAY_CHECK_STEP_MS ms
// ============================================================
void AlertManager::handleStopAlert() {
    _buzzerEnabled = false;
    _isAlerting    = false;
    _buzzer->turnOff();
    Logger::info(getModuleName(), "Buzzer disabled");
}

// ============================================================
//  interruptibleDelay
//  Chia delay thành bước DELAY_CHECK_STEP_MS ms
//  Kiểm tra _buzzerEnabled sau mỗi bước
//  Trả về false nếu cờ bị hạ → dừng beep ngay
// ============================================================
bool AlertManager::interruptibleDelay(uint32_t durationMs) {
    uint32_t elapsed = 0;

    while (elapsed < durationMs) {
        if (!_buzzerEnabled) {
            return false;
        }

        uint32_t step = (durationMs - elapsed < DELAY_CHECK_STEP_MS)
                        ? (durationMs - elapsed)
                        : DELAY_CHECK_STEP_MS;

        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }

    return _buzzerEnabled;
}

// ============================================================
//  getBeepPattern
// ============================================================
void AlertManager::getBeepPattern(AccidentType type,
                                   uint32_t& outOnMs,
                                   uint32_t& outOffMs) {
    switch (type) {
        case AccidentType::CRASH:
            outOnMs  = PATTERN_CRASH_ON_MS;
            outOffMs = PATTERN_CRASH_OFF_MS;
            break;
        case AccidentType::FALL:
            outOnMs  = PATTERN_FALL_ON_MS;
            outOffMs = PATTERN_FALL_OFF_MS;
            break;
        case AccidentType::COMBINED:
            outOnMs  = PATTERN_COMBINED_ON_MS;
            outOffMs = PATTERN_COMBINED_OFF_MS;
            break;
        case AccidentType::SUDDEN_STOP:
        case AccidentType::NONE:
        default:
            outOnMs  = PATTERN_SUDDEN_ON_MS;
            outOffMs = PATTERN_SUDDEN_OFF_MS;
            break;
    }
}
