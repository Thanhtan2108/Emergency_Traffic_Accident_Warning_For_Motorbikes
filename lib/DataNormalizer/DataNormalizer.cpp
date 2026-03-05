#include "DataNormalizer.h"
#include "QueueManager.h"

// ============================================================
//  DataNormalizer.cpp
// ============================================================

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
DataNormalizer::DataNormalizer(QueueHandle_t inQueue,
                               QueueHandle_t outQueue,
                               float gyroOffsetX,
                               float gyroOffsetY,
                               float gyroOffsetZ)
    : _inQueue(inQueue)
    , _outQueue(outQueue)
    , _taskHandle(nullptr)
    , _gyroOffsetX(gyroOffsetX)
    , _gyroOffsetY(gyroOffsetY)
    , _gyroOffsetZ(gyroOffsetZ)
    , _taskRunning(false)
    , _processedCount(0)
    , _droppedCount(0)
{
}

// ------------------------------------------------------------
// begin()
// ------------------------------------------------------------
bool DataNormalizer::begin() {
    Logger::info(getModuleName(), "Initializing...");

    if (_inQueue == nullptr || _outQueue == nullptr) {
        Logger::error(getModuleName(), "Queue handles are null!");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        DataNormalizer::taskEntry,
        "DataNormalizer",
        STACK_DATA_NORMALIZER,
        this,
        PRIORITY_DATA_NORMALIZER,
        &_taskHandle,
        CORE_SENSOR_TASKS           // Cùng core với MotionSensor
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Failed to create FreeRTOS task!");
        return false;
    }

    Logger::info(getModuleName(), "Task created — waiting for raw data");
    return true;
}

// ------------------------------------------------------------
// isHealthy()
// ------------------------------------------------------------
bool DataNormalizer::isHealthy() const {
    return _taskRunning;
}

// ------------------------------------------------------------
// setGyroOffsets() — cập nhật offset sau calibration
// ------------------------------------------------------------
void DataNormalizer::setGyroOffsets(float x, float y, float z) {
    _gyroOffsetX = x;
    _gyroOffsetY = y;
    _gyroOffsetZ = z;
    Logger::info(getModuleName(), "Gyro offsets updated");
}

// ------------------------------------------------------------
// taskEntry() — static bridge vào taskRun()
// ------------------------------------------------------------
void DataNormalizer::taskEntry(void* pvParameters) {
    DataNormalizer* self = static_cast<DataNormalizer*>(pvParameters);
    self->taskRun();
    self->_taskRunning = false;
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------
// taskRun() — vòng lặp chính
// Block chờ rawDataQueue → normalize → push normalizedQueue
// Không dùng vTaskDelayUntil vì task này event-driven:
// chạy ngay khi có data, không cần fixed period
// ------------------------------------------------------------
void DataNormalizer::taskRun() {
    _taskRunning = true;
    Logger::info(getModuleName(), "Task running");

    RawSensorData rawData;

    while (true) {
        // Block chờ data từ MotionSensor (tối đa QUEUE_RECV_TIMEOUT_MS)
        // Khi có data → thức dậy xử lý ngay, không delay nhân tạo
        BaseType_t received = xQueueReceive(
            _inQueue,
            &rawData,
            pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)
        );

        if (received != pdTRUE) {
            // Timeout — gửi heartbeat để Watchdog biết task còn sống
            QueueManager::getInstance().sendHeartbeat(TaskID::DATA_NORMALIZER);
            continue;
        }

        // Bỏ qua frame không hợp lệ từ MotionSensor
        if (!rawData.isValid) {
            Logger::warn(getModuleName(), "Received invalid raw frame, skipping");
            continue;
        }

        // Chuẩn hóa dữ liệu
        NormalizedData normalized = normalize(rawData);

        // Đẩy sang SignalProcessor
        pushToQueue(normalized);

        _processedCount++;

        // Gửi heartbeat sau mỗi frame xử lý
        QueueManager::getInstance().sendHeartbeat(TaskID::DATA_NORMALIZER);

        // Log định kỳ mỗi 500 frame (~5 giây)
        if (LOG_MOTION_DATA && (_processedCount % 500 == 0)) {
            Logger::infoValue(getModuleName(), "Processed: ", (int32_t)_processedCount);
            Logger::infoValue(getModuleName(), "Dropped:   ", (int32_t)_droppedCount);
        }
    }
}

// ------------------------------------------------------------
// normalize() — chuyển đổi RawSensorData → NormalizedData
// Hàm thuần toán học: input → output, không có side-effect
// ------------------------------------------------------------
NormalizedData DataNormalizer::normalize(const RawSensorData& raw) const {
    NormalizedData out;

    // --- Gia tốc (g) ---
    out.accX = rawToAccel(raw.rawAccX);
    out.accY = rawToAccel(raw.rawAccY);
    out.accZ = rawToAccel(raw.rawAccZ);

    // --- Vận tốc góc (°/s), đã trừ offset hiệu chỉnh ---
    out.gyroX = rawToGyro(raw.rawGyroX, _gyroOffsetX);
    out.gyroY = rawToGyro(raw.rawGyroY, _gyroOffsetY);
    out.gyroZ = rawToGyro(raw.rawGyroZ, _gyroOffsetZ);

    // --- Góc từ accelerometer (°) ---
    // Chỉ có nghĩa khi xe đứng yên hoặc gia tốc thấp
    // Khối 3 (SignalProcessor) sẽ kết hợp với gyro để tính chính xác hơn
    out.angleAccX = calcAngleX(out.accX, out.accY, out.accZ);
    out.angleAccY = calcAngleY(out.accX, out.accY, out.accZ);

    // --- Nhiệt độ (°C) ---
    out.temperature = rawToTemp(raw.rawTemp);

    // --- Metadata ---
    out.timestamp = raw.timestamp;  // Giữ nguyên timestamp gốc từ Khối 1
    out.isValid   = true;

    return out;
}

// ------------------------------------------------------------
// pushToQueue() — gửi NormalizedData vào normalizedQueue
// ------------------------------------------------------------
void DataNormalizer::pushToQueue(const NormalizedData& data) {
    BaseType_t result = xQueueSend(
        _outQueue,
        &data,
        pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS)
    );

    if (result != pdTRUE) {
        _droppedCount++;
        if (_droppedCount % 10 == 1) {
            Logger::warn(getModuleName(), "normalizedQueue full — frame dropped!");
            Logger::infoValue(getModuleName(),
                "Total dropped: ", (int32_t)_droppedCount);
        }
    }
}
