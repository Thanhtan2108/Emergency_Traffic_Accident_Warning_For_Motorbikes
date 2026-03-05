#include "AccidentDetector.h"
#include "QueueManager.h"

// ============================================================
//  AccidentDetector.cpp
// ============================================================

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
AccidentDetector::AccidentDetector(QueueHandle_t      inQueue,
                                   QueueHandle_t      outQueue,
                                   EventGroupHandle_t eventGroup)
    : _inQueue(inQueue)
    , _outQueue(outQueue)
    , _eventGroup(eventGroup)
    , _taskHandle(nullptr)
    , _taskRunning(false)
    , _processedCount(0)
    , _accidentCount(0)
    , _state(VehicleState::UNKNOWN)
    , _lastAccidentType(AccidentType::NONE)
    , _suspiciousFrameCount(0)
    , _normalFrameCount(0)
    , _accidentStartMs(0)
{
    _triggerSnapshot = MotionFeatures{};
}

// ------------------------------------------------------------
// begin()
// ------------------------------------------------------------
bool AccidentDetector::begin() {
    Logger::info(getModuleName(), "Initializing...");

    if (_inQueue == nullptr || _outQueue == nullptr) {
        Logger::error(getModuleName(), "Queue handles are null!");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        AccidentDetector::taskEntry,
        "AccidentDetector",
        STACK_ACCIDENT_DETECTOR,
        this,
        PRIORITY_ACCIDENT_DETECTOR,
        &_taskHandle,
        CORE_MANAGEMENT_TASKS           // Core 0 — phía management
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Failed to create FreeRTOS task!");
        return false;
    }

    Logger::info(getModuleName(), "Task created — monitoring for accidents");
    return true;
}

// ------------------------------------------------------------
// isHealthy()
// ------------------------------------------------------------
bool AccidentDetector::isHealthy() const {
    return _taskRunning;
}

// ------------------------------------------------------------
// taskEntry() — static bridge
// ------------------------------------------------------------
void AccidentDetector::taskEntry(void* pvParameters) {
    AccidentDetector* self = static_cast<AccidentDetector*>(pvParameters);
    self->taskRun();
    self->_taskRunning = false;
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------
// taskRun() — vòng lặp chính, event-driven
// ------------------------------------------------------------
void AccidentDetector::taskRun() {
    _taskRunning = true;
    Logger::info(getModuleName(), "Task running — state: UNKNOWN");

    MotionFeatures features;

    while (true) {
        // Block chờ MotionFeatures từ SignalProcessor
        BaseType_t received = xQueueReceive(
            _inQueue,
            &features,
            pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)
        );

        if (received != pdTRUE) {
            // Timeout — kiểm tra xem ACCIDENT có cần tự reset không
            if (_state == VehicleState::ACCIDENT) {
                uint32_t now = millis();
                if ((now - _accidentStartMs) >= ACCIDENT_HOLD_MS) {
                    Logger::info(getModuleName(),
                        "Accident hold expired — resetting to NORMAL");
                    _state               = VehicleState::NORMAL;
                    _lastAccidentType    = AccidentType::NONE;
                    _suspiciousFrameCount = 0;
                    _normalFrameCount     = 0;

                    // Gửi event "cleared"
                    if (_eventGroup) {
                        xEventGroupSetBits(_eventGroup,
                            QueueManager::EVENT_ACCIDENT_CLEARED);
                    }
                }
            }
            // Gửi heartbeat kể cả khi timeout
            QueueManager::getInstance().sendHeartbeat(TaskID::ACCIDENT_DETECTOR);
            continue;
        }

        if (!features.isValid) continue;

        // Frame đầu tiên hợp lệ: chuyển UNKNOWN → NORMAL
        if (_state == VehicleState::UNKNOWN) {
            _state = VehicleState::NORMAL;
            Logger::info(getModuleName(), "State: UNKNOWN → NORMAL");
        }

        // Phân tích tính năng và cập nhật state machine
        AccidentType detected = analyzeFeatures(features);
        updateStateMachine(detected, features);

        _processedCount++;

        // Gửi heartbeat sau mỗi frame xử lý
        QueueManager::getInstance().sendHeartbeat(TaskID::ACCIDENT_DETECTOR);
    }
}

// ------------------------------------------------------------
// analyzeFeatures() — kiểm tra từng ngưỡng
// Trả về loại tai nạn phát hiện, hoặc NONE nếu bình thường
// ------------------------------------------------------------
AccidentType AccidentDetector::analyzeFeatures(const MotionFeatures& f) const {
    bool isCrash      = (f.totalAccMag    > ACC_CRASH_THRESHOLD);
    bool isFall       = (fabsf(f.angleX)  > TILT_ANGLE_THRESHOLD ||
                         fabsf(f.angleY)  > TILT_ANGLE_THRESHOLD);
    bool isSuddenStop = (f.jerk           > JERK_THRESHOLD);

    // Đếm số điều kiện cùng lúc
    uint8_t count = (uint8_t)isCrash + (uint8_t)isFall + (uint8_t)isSuddenStop;

    if (count == 0)  return AccidentType::NONE;
    if (count >= 2)  return AccidentType::COMBINED;
    if (isCrash)     return AccidentType::CRASH;
    if (isFall)      return AccidentType::FALL;
                     return AccidentType::SUDDEN_STOP;
}

// ------------------------------------------------------------
// updateStateMachine()
// Áp dụng debounce và chuyển trạng thái
// ------------------------------------------------------------
void AccidentDetector::updateStateMachine(AccidentType    detected,
                                          const MotionFeatures& f) {
    switch (_state) {

    // --------------------------------------------------------
    case VehicleState::NORMAL:
        if (detected != AccidentType::NONE) {
            // Phát hiện bất thường đầu tiên → SUSPICIOUS
            _state                = VehicleState::SUSPICIOUS;
            _suspiciousFrameCount = 1;
            _normalFrameCount     = 0;
            _lastAccidentType     = detected;
            _triggerSnapshot      = f;

            Logger::info(getModuleName(), "State: NORMAL → SUSPICIOUS");
            Logger::info(getModuleName(), accidentTypeToString(detected));

            // Gửi event SUSPICIOUS (isActive=false, chưa cần alert)
            publishEvent(f, false);
        }
        break;

    // --------------------------------------------------------
    case VehicleState::SUSPICIOUS:
        if (detected != AccidentType::NONE) {
            _suspiciousFrameCount++;
            _normalFrameCount = 0;

            // Cập nhật loại tai nạn nếu phát hiện COMBINED
            if (detected == AccidentType::COMBINED ||
                _lastAccidentType == AccidentType::NONE) {
                _lastAccidentType = detected;
            }

            // Đủ frame liên tiếp → xác nhận ACCIDENT
            if (_suspiciousFrameCount >= ACCIDENT_CONFIRM_FRAMES) {
                _state           = VehicleState::ACCIDENT;
                _accidentStartMs = millis();
                _accidentCount++;

                Logger::info(getModuleName(), "State: SUSPICIOUS → ACCIDENT");
                Logger::info(getModuleName(),
                    accidentTypeToString(_lastAccidentType));
                Logger::infoValue(getModuleName(),
                    "totalAcc: ", f.totalAccMag);
                Logger::infoValue(getModuleName(),
                    "angleX: ",   f.angleX);
                Logger::infoValue(getModuleName(),
                    "angleY: ",   f.angleY);
                Logger::infoValue(getModuleName(),
                    "jerk: ",     f.jerk);

                // Gửi AccidentEvent với isActive=true → AlertManager
                publishEvent(f, true);

                // Set event bit để SystemStateManager biết
                if (_eventGroup) {
                    xEventGroupSetBits(_eventGroup,
                        QueueManager::EVENT_ACCIDENT_DETECTED);
                }
            }
        } else {
            // Frame bình thường
            _normalFrameCount++;
            _suspiciousFrameCount = 0;

            if (_normalFrameCount >= NORMAL_CONFIRM_FRAMES) {
                // Đủ frame bình thường liên tiếp → quay lại NORMAL
                _state            = VehicleState::NORMAL;
                _lastAccidentType = AccidentType::NONE;
                _normalFrameCount = 0;

                Logger::info(getModuleName(), "State: SUSPICIOUS → NORMAL");
            }
        }
        break;

    // --------------------------------------------------------
    case VehicleState::ACCIDENT:
        // Ở trạng thái ACCIDENT: kiểm tra tự reset theo thời gian
        {
            uint32_t now = millis();
            if ((now - _accidentStartMs) >= ACCIDENT_HOLD_MS) {
                _state               = VehicleState::NORMAL;
                _lastAccidentType    = AccidentType::NONE;
                _suspiciousFrameCount = 0;
                _normalFrameCount     = 0;

                Logger::info(getModuleName(),
                    "Accident hold expired — state: ACCIDENT → NORMAL");

                if (_eventGroup) {
                    xEventGroupSetBits(_eventGroup,
                        QueueManager::EVENT_ACCIDENT_CLEARED);
                }
            }
            // Không xử lý thêm trong ACCIDENT — giữ nguyên cho đến khi reset
        }
        break;

    default:
        break;
    }
}

// ------------------------------------------------------------
// publishEvent() — đóng gói và gửi AccidentEvent
// ------------------------------------------------------------
void AccidentDetector::publishEvent(const MotionFeatures& f, bool isActive) {
    AccidentEvent event;
    event.state            = _state;
    event.type             = _lastAccidentType;
    event.totalAccAtEvent  = f.totalAccMag;
    event.angleXAtEvent    = f.angleX;
    event.angleYAtEvent    = f.angleY;
    event.jerkAtEvent      = f.jerk;
    event.eventTimestamp   = f.timestamp;
    event.isActive         = isActive;

    // Non-blocking: nếu queue đầy thì bỏ qua
    // AlertManager sẽ đọc ngay khi có data
    BaseType_t result = xQueueSend(_outQueue, &event, 0);

    if (result != pdTRUE) {
        Logger::warn(getModuleName(), "accidentQueue full — event lost!");
    }
}
