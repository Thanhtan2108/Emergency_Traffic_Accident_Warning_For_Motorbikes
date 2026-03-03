#include "SignalProcessor.h"

// ============================================================
//  SignalProcessor.cpp
// ============================================================

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
SignalProcessor::SignalProcessor(QueueHandle_t inQueue, QueueHandle_t outQueue)
    : _inQueue(inQueue)
    , _outQueue(outQueue)
    , _taskHandle(nullptr)
    , _taskRunning(false)
    , _processedCount(0)
    , _droppedCount(0)
    // LPF state khởi tạo 0 — sẽ được seed từ frame đầu tiên
    , _lpfAccX(0.0f),  _lpfAccY(0.0f),  _lpfAccZ(0.0f)
    , _lpfGyroX(0.0f), _lpfGyroY(0.0f), _lpfGyroZ(0.0f)
    // Góc ban đầu = 0 — Complementary Filter sẽ hội tụ sau vài giây
    , _angleX(0.0f), _angleY(0.0f), _angleZ(0.0f)
    , _prevTotalAcc(1.0f)   // Giả sử ban đầu xe đứng yên ~1g
    , _prevTimestamp(0)
    , _isFirstFrame(true)
{
    // Khởi tạo _latest rỗng
    _latest = MotionFeatures{};
}

// ------------------------------------------------------------
// begin()
// ------------------------------------------------------------
bool SignalProcessor::begin() {
    Logger::info(getModuleName(), "Initializing...");

    if (_inQueue == nullptr || _outQueue == nullptr) {
        Logger::error(getModuleName(), "Queue handles are null!");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        SignalProcessor::taskEntry,
        "SignalProcessor",
        STACK_SIGNAL_PROCESSOR,
        this,
        PRIORITY_SIGNAL_PROCESSOR,
        &_taskHandle,
        CORE_SENSOR_TASKS
    );

    if (result != pdPASS) {
        Logger::error(getModuleName(), "Failed to create FreeRTOS task!");
        return false;
    }

    Logger::info(getModuleName(), "Task created — processing pipeline ready");
    return true;
}

// ------------------------------------------------------------
// isHealthy()
// ------------------------------------------------------------
bool SignalProcessor::isHealthy() const {
    return _taskRunning;
}

// ------------------------------------------------------------
// taskEntry() — static bridge
// ------------------------------------------------------------
void SignalProcessor::taskEntry(void* pvParameters) {
    SignalProcessor* self = static_cast<SignalProcessor*>(pvParameters);
    self->taskRun();
    self->_taskRunning = false;
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------
// taskRun() — vòng lặp chính, event-driven
// ------------------------------------------------------------
void SignalProcessor::taskRun() {
    _taskRunning = true;
    Logger::info(getModuleName(), "Task running");

    NormalizedData input;

    while (true) {
        // Block chờ NormalizedData từ DataNormalizer
        BaseType_t received = xQueueReceive(
            _inQueue,
            &input,
            pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)
        );

        if (received != pdTRUE) {
            continue;   // Timeout, chờ tiếp
        }

        if (!input.isValid) {
            Logger::warn(getModuleName(), "Invalid frame, skipping");
            continue;
        }

        // --- Frame đầu tiên: seed LPF và timestamp ---
        if (_isFirstFrame) {
            // Seed LPF bằng giá trị thực tế đầu tiên
            // thay vì bắt đầu từ 0 → tránh transient spike
            _lpfAccX  = input.accX;
            _lpfAccY  = input.accY;
            _lpfAccZ  = input.accZ;
            _lpfGyroX = input.gyroX;
            _lpfGyroY = input.gyroY;
            _lpfGyroZ = input.gyroZ;

            // Seed góc ban đầu từ accelerometer
            _angleX = input.angleAccX;
            _angleY = input.angleAccY;
            _angleZ = 0.0f;

            _prevTotalAcc  = sqrtf(input.accX*input.accX +
                                   input.accY*input.accY +
                                   input.accZ*input.accZ);
            _prevTimestamp = input.timestamp;
            _isFirstFrame  = false;
            continue;   // Bỏ qua frame đầu tiên, không đủ dt để tính
        }

        // Tính delta time (giây) giữa 2 frame liên tiếp
        float dt = calcDeltaTime(input.timestamp, _prevTimestamp);

        // Bảo vệ: dt phải hợp lý (5ms – 50ms cho 100Hz)
        // Nếu dt quá nhỏ hoặc quá lớn → bỏ qua frame tránh spike
        if (dt < 0.005f || dt > 0.050f) {
            _prevTimestamp = input.timestamp;
            continue;
        }

        // ====================================================
        // Pipeline xử lý tín hiệu (theo thứ tự)
        // ====================================================

        // Bước 1: Low-Pass Filter
        applyLPF(input);

        // Bước 2: Complementary Filter → angleX, angleY
        applyComplementaryFilter(input, dt);

        // Bước 3: Tích phân gyroZ → angleZ (yaw)
        integrateYaw(dt);

        // Bước 4: Tính tổng hợp và đóng gói MotionFeatures
        computeFeatures(input.timestamp);

        // Lưu timestamp cho frame tiếp theo
        _prevTimestamp = input.timestamp;

        // Gửi sang AccidentDetector
        pushToQueue(_latest);

        _processedCount++;

        if (LOG_FEATURES && (_processedCount % 500 == 0)) {
            Logger::infoValue(getModuleName(), "Processed: ", (int32_t)_processedCount);
            Logger::infoValue(getModuleName(), "AngleX: ",    _latest.angleX);
            Logger::infoValue(getModuleName(), "AngleY: ",    _latest.angleY);
            Logger::infoValue(getModuleName(), "TotalAcc: ",  _latest.totalAccMag);
        }
    }
}

// ------------------------------------------------------------
// Bước 1: Low-Pass Filter
// Công thức: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
// alpha = LPF_ALPHA_ACC / LPF_ALPHA_GYRO (từ Config.h)
// Mục đích: loại bỏ nhiễu tần số cao (rung động, điện từ)
// ------------------------------------------------------------
void SignalProcessor::applyLPF(const NormalizedData& in) {
    const float aA = LPF_ALPHA_ACC;    // 0.15 — lọc mạnh cho acc
    const float aG = LPF_ALPHA_GYRO;   // 0.10 — lọc mạnh hơn cho gyro

    _lpfAccX  = aA * in.accX  + (1.0f - aA) * _lpfAccX;
    _lpfAccY  = aA * in.accY  + (1.0f - aA) * _lpfAccY;
    _lpfAccZ  = aA * in.accZ  + (1.0f - aA) * _lpfAccZ;

    _lpfGyroX = aG * in.gyroX + (1.0f - aG) * _lpfGyroX;
    _lpfGyroY = aG * in.gyroY + (1.0f - aG) * _lpfGyroY;
    _lpfGyroZ = aG * in.gyroZ + (1.0f - aG) * _lpfGyroZ;
}

// ------------------------------------------------------------
// Bước 2: Complementary Filter
// angle = gyroCoef * (angle + gyro*dt) + accCoef * angleAcc
//
// Lý do dùng Complementary Filter thay vì chỉ dùng 1 nguồn:
//   - Gyro: chính xác ngắn hạn nhưng bị drift theo thời gian
//   - Accel: ổn định dài hạn nhưng nhiễu khi có gia tốc động
//   - Kết hợp 98% gyro + 2% accel → tận dụng ưu điểm cả hai
// ------------------------------------------------------------
void SignalProcessor::applyComplementaryFilter(const NormalizedData& in, float dt) {
    const float gC = MPU_GYRO_COEF;    // 0.98
    const float aC = MPU_ACC_COEF;     // 0.02

    // Dùng gyro đã qua LPF để tính góc dự đoán
    // Dùng angleAcc từ DataNormalizer (tính từ acc thô) làm correction
    _angleX = gC * (_angleX + _lpfGyroX * dt) + aC * in.angleAccX;
    _angleY = gC * (_angleY + _lpfGyroY * dt) + aC * in.angleAccY;
}

// ------------------------------------------------------------
// Bước 3: Tích phân Yaw từ gyroZ
// Accel không cung cấp thông tin yaw (xoay quanh trục đứng)
// → Chỉ có thể tích phân gyroZ, chấp nhận drift dài hạn
// ------------------------------------------------------------
void SignalProcessor::integrateYaw(float dt) {
    _angleZ += _lpfGyroZ * dt;

    // Giới hạn angleZ trong [-360°, 360°] để tránh overflow lâu dài
    if (_angleZ >  360.0f) _angleZ -= 360.0f;
    if (_angleZ < -360.0f) _angleZ += 360.0f;
}

// ------------------------------------------------------------
// Bước 4: Tính đặc trưng và đóng gói MotionFeatures
// ------------------------------------------------------------
void SignalProcessor::computeFeatures(uint32_t timestamp) {
    // Tổng độ lớn gia tốc (đã lọc)
    float totalAcc = sqrtf(_lpfAccX * _lpfAccX +
                           _lpfAccY * _lpfAccY +
                           _lpfAccZ * _lpfAccZ);

    // Tổng độ lớn vận tốc góc (đã lọc)
    float angularVel = sqrtf(_lpfGyroX * _lpfGyroX +
                             _lpfGyroY * _lpfGyroY +
                             _lpfGyroZ * _lpfGyroZ);

    // Jerk: tốc độ thay đổi gia tốc (g/s)
    // dt tính bằng giây từ _prevTimestamp → timestamp
    float dt   = calcDeltaTime(timestamp, _prevTimestamp);
    float jerk = (dt > 0.0f)
                 ? fabsf(totalAcc - _prevTotalAcc) / dt
                 : 0.0f;

    // Cập nhật state cho frame tiếp theo
    _prevTotalAcc = totalAcc;

    // Đóng gói MotionFeatures
    _latest.filteredAccX    = _lpfAccX;
    _latest.filteredAccY    = _lpfAccY;
    _latest.filteredAccZ    = _lpfAccZ;
    _latest.filteredGyroX   = _lpfGyroX;
    _latest.filteredGyroY   = _lpfGyroY;
    _latest.filteredGyroZ   = _lpfGyroZ;
    _latest.angleX          = _angleX;
    _latest.angleY          = _angleY;
    _latest.angleZ          = _angleZ;
    _latest.totalAccMag     = totalAcc;
    _latest.angularVelMag   = angularVel;
    _latest.jerk            = jerk;
    _latest.timestamp       = timestamp;
    _latest.isValid         = true;
}

// ------------------------------------------------------------
// pushToQueue()
// ------------------------------------------------------------
void SignalProcessor::pushToQueue(const MotionFeatures& features) {
    BaseType_t result = xQueueSend(
        _outQueue,
        &features,
        pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS)
    );

    if (result != pdTRUE) {
        _droppedCount++;
        if (_droppedCount % 10 == 1) {
            Logger::warn(getModuleName(), "featuresQueue full — frame dropped!");
            Logger::infoValue(getModuleName(),
                "Total dropped: ", (int32_t)_droppedCount);
        }
    }
}
