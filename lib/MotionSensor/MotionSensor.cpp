#include "MotionSensor.h"

// ============================================================
//  MotionSensor.cpp
// ============================================================

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
MotionSensor::MotionSensor(TwoWire& wire, QueueHandle_t rawDataQueue)
    : _mpu(wire, MPU_ACC_COEF, MPU_GYRO_COEF)
    , _rawDataQueue(rawDataQueue)
    , _taskHandle(nullptr)
    , _sensorConnected(false)
    , _taskRunning(false)
    , _sampleCount(0)
    , _droppedFrames(0)
{
}

// ------------------------------------------------------------
// begin() - Khởi tạo sensor và tạo FreeRTOS Task
// ------------------------------------------------------------
bool MotionSensor::begin() {
    Logger::info(getModuleName(), "Initializing...");

    // Bước 1: Kiểm tra MPU6050 có trên I2C bus không
    if (!probeSensor()) {
        Logger::error(getModuleName(), "MPU6050 not found on I2C bus!");
        _sensorConnected = false;
        return false;
    }
    Logger::info(getModuleName(), "MPU6050 detected on I2C bus");

    // Bước 2: Khởi tạo MPU6050 (cấu hình register)
    _mpu.begin();
    Logger::info(getModuleName(), "MPU6050 registers configured");

    // Bước 3: Hiệu chỉnh gyro offset
    // console=true để in tiến trình ra Serial
    // Xe phải đứng yên hoàn toàn trong quá trình này
    Logger::info(getModuleName(), "Calibrating gyro — keep vehicle STILL...");
    _mpu.calcGyroOffsets(
        true,                       // In tiến trình ra Serial
        GYRO_CALIB_DELAY_BEFORE_MS,
        GYRO_CALIB_DELAY_AFTER_MS
    );
    Logger::info(getModuleName(), "Gyro calibration done");

    _sensorConnected = true;

    // Bước 4: Tạo FreeRTOS Task chạy trên Core 1 (sensor core)
    BaseType_t result = xTaskCreatePinnedToCore(
        MotionSensor::taskEntry,    // Hàm entry point (static)
        "MotionSensor",             // Tên task (hiện trong FreeRTOS debug)
        STACK_MOTION_SENSOR,        // Stack size (words)
        this,                       // pvParameters = con trỏ this
        PRIORITY_MOTION_SENSOR,     // Priority
        &_taskHandle,               // Lưu handle để quản lý sau
        CORE_SENSOR_TASKS           // Chạy trên Core 1
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Failed to create FreeRTOS task!");
        _sensorConnected = false;
        return false;
    }

    Logger::info(getModuleName(), "Task created — sampling at 100Hz");
    return true;
}

// ------------------------------------------------------------
// isHealthy() - Kiểm tra sức khỏe module
// ------------------------------------------------------------
bool MotionSensor::isHealthy() const {
    return _sensorConnected && _taskRunning;
}

// ------------------------------------------------------------
// taskEntry() - Static wrapper, bridge vào member function
// FreeRTOS chỉ nhận hàm static, nên dùng pvParameters
// để truyền con trỏ this và gọi lại taskRun()
// ------------------------------------------------------------
void MotionSensor::taskEntry(void* pvParameters) {
    // Cast pvParameters về đúng kiểu
    MotionSensor* self = static_cast<MotionSensor*>(pvParameters);
    self->taskRun();

    // Nếu taskRun() thoát (không nên xảy ra), xóa task an toàn
    self->_taskRunning = false;
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------
// taskRun() - Vòng lặp chính 100Hz
// Mỗi iteration: đọc sensor → đóng gói → đẩy vào queue → delay
// ------------------------------------------------------------
void MotionSensor::taskRun() {
    _taskRunning = true;
    Logger::info(getModuleName(), "Task running");

    // Lưu thời điểm bắt đầu tick để vTaskDelayUntil chạy đúng 100Hz
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTION_SAMPLE_PERIOD_MS); // 10ms

    while (true) {
        // --- Đọc và gửi dữ liệu ---
        if (_sensorConnected) {
            // Gọi update() để thư viện tính interval và lấy data mới
            _mpu.update();

            // Đóng gói raw data
            RawSensorData data = readRawData();

            // Đẩy vào queue sang DataNormalizer
            pushToQueue(data);

            _sampleCount++;

            // Log định kỳ mỗi 500 mẫu (5 giây) nếu bật verbose
            if (LOG_MOTION_DATA && (_sampleCount % 500 == 0)) {
                Logger::infoValue(getModuleName(), "Samples: ", (int32_t)_sampleCount);
                Logger::infoValue(getModuleName(), "Dropped: ", (int32_t)_droppedFrames);
            }
        }

        // vTaskDelayUntil đảm bảo chính xác 100Hz bất kể
        // thời gian xử lý mỗi iteration (không bị drift tích lũy)
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// ------------------------------------------------------------
// probeSensor() - Kiểm tra MPU6050 có phản hồi trên I2C không
// Đọc WHO_AM_I register — MPU6050 trả về 0x68
// ------------------------------------------------------------
bool MotionSensor::probeSensor() {
    // Thử đọc WHO_AM_I register của MPU6050
    // Giá trị mong đợi: 0x68 (MPU6050_ADDR)
    byte whoAmI = _mpu.readMPU6050(MPU6050_WHO_AM_I);

    // MPU6050 trả về địa chỉ của chính nó trong WHO_AM_I
    if (whoAmI != MPU6050_ADDR) {
        Logger::infoValue(getModuleName(),
            "WHO_AM_I unexpected value: ", (int32_t)whoAmI);
        return false;
    }
    return true;
}

// ------------------------------------------------------------
// readRawData() - Đọc raw register và đóng gói RawSensorData
// Gọi sau khi _mpu.update() đã được gọi trong vòng lặp
// ------------------------------------------------------------
RawSensorData MotionSensor::readRawData() {
    RawSensorData data;

    // Lấy raw ADC counts trực tiếp từ thư viện
    // (thư viện đã đọc qua I2C trong update())
    data.rawAccX  = _mpu.getRawAccX();
    data.rawAccY  = _mpu.getRawAccY();
    data.rawAccZ  = _mpu.getRawAccZ();
    data.rawGyroX = _mpu.getRawGyroX();
    data.rawGyroY = _mpu.getRawGyroY();
    data.rawGyroZ = _mpu.getRawGyroZ();
    data.rawTemp  = _mpu.getRawTemp();

    // Timestamp tại thời điểm đọc (ms kể từ boot)
    data.timestamp = millis();
    data.isValid   = true;

    return data;
}

// ------------------------------------------------------------
// pushToQueue() - Gửi data vào rawDataQueue
// Non-blocking với timeout ngắn để không làm chậm 100Hz loop
// Nếu queue đầy → ghi nhận dropped frame, không block task
// ------------------------------------------------------------
void MotionSensor::pushToQueue(const RawSensorData& data) {
    BaseType_t result = xQueueSend(
        _rawDataQueue,
        &data,
        pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS)   // timeout 10ms
    );

    if (result != pdTRUE) {
        // Queue đầy: DataNormalizer đang xử lý chậm hơn 100Hz
        _droppedFrames++;

        if (_droppedFrames % 10 == 1) {
            // Log mỗi 10 dropped frame để tránh spam
            Logger::warn(getModuleName(), "Queue full — frame dropped!");
            Logger::infoValue(getModuleName(),
                "Total dropped: ", (int32_t)_droppedFrames);
        }
    }
}
