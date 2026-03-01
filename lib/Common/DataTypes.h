#pragma once

#include <stdint.h>

// ============================================================
//  DataTypes.h
//  Định nghĩa tất cả struct dữ liệu được truyền giữa các khối
//  thông qua FreeRTOS Queue.
//
//  Nguyên tắc:
//  - Mỗi struct là một "snapshot" tại một thời điểm (timestamp)
//  - Tất cả struct là Plain Old Data (POD) để copy-safe qua Queue
//  - Không chứa con trỏ hay reference
// ============================================================


// ------------------------------------------------------------
// [Khối 1 → Khối 2] Raw Sensor Data
// Dữ liệu thô trực tiếp từ ADC của MPU6050 (chưa xử lý)
// ------------------------------------------------------------
struct RawSensorData {
    int16_t rawAccX;        // Accelerometer X (ADC counts)
    int16_t rawAccY;        // Accelerometer Y (ADC counts)
    int16_t rawAccZ;        // Accelerometer Z (ADC counts)
    int16_t rawGyroX;       // Gyroscope X (ADC counts)
    int16_t rawGyroY;       // Gyroscope Y (ADC counts)
    int16_t rawGyroZ;       // Gyroscope Z (ADC counts)
    int16_t rawTemp;        // Temperature raw (ADC counts)
    uint32_t timestamp;     // millis() tại thời điểm đọc
    bool     isValid;       // false nếu sensor bị lỗi khi đọc
};


// ------------------------------------------------------------
// [Khối 2 → Khối 3] Normalized Sensor Data
// Dữ liệu đã chuyển sang đơn vị vật lý có nghĩa
// ------------------------------------------------------------
struct NormalizedData {
    // Gia tốc tuyến tính (đơn vị: g, 1g = 9.81 m/s²)
    float accX;             // Gia tốc trục X
    float accY;             // Gia tốc trục Y
    float accZ;             // Gia tốc trục Z

    // Vận tốc góc (đơn vị: °/s)
    float gyroX;            // Vận tốc góc trục X (roll rate)
    float gyroY;            // Vận tốc góc trục Y (pitch rate)
    float gyroZ;            // Vận tốc góc trục Z (yaw rate)

    // Góc tính từ accelerometer (đơn vị: độ °)
    float angleAccX;        // Roll angle từ accel
    float angleAccY;        // Pitch angle từ accel

    // Nhiệt độ (°C)
    float temperature;

    uint32_t timestamp;
    bool     isValid;
};


// ------------------------------------------------------------
// [Khối 3 → Khối 4] Motion Features
// Các đặc trưng chuyển động đã lọc nhiễu và tính toán
// ------------------------------------------------------------
struct MotionFeatures {
    // Gia tốc sau khi qua Low-Pass Filter (đơn vị: g)
    float filteredAccX;
    float filteredAccY;
    float filteredAccZ;

    // Vận tốc góc sau khi qua Low-Pass Filter (°/s)
    float filteredGyroX;
    float filteredGyroY;
    float filteredGyroZ;

    // Góc tổng hợp từ Complementary Filter (đơn vị: độ °)
    float angleX;           // Roll  (nghiêng trái/phải)
    float angleY;           // Pitch (nghiêng trước/sau)
    float angleZ;           // Yaw   (xoay ngang - tích phân gyro)

    // Đại lượng tổng hợp quan trọng cho phát hiện tai nạn
    float totalAccMag;      // |a| = sqrt(aX²+aY²+aZ²) (g)
    float angularVelMag;    // |ω| = sqrt(gX²+gY²+gZ²) (°/s)
    float jerk;             // Δ|a|/Δt (g/s) - thay đổi gia tốc đột ngột

    uint32_t timestamp;
    bool     isValid;
};


// ------------------------------------------------------------
// [Khối 4 → Khối 5, 7] Accident Event
// Kết quả phân tích trạng thái xe
// ------------------------------------------------------------

// Các trạng thái chuyển động của xe
enum class VehicleState : uint8_t {
    NORMAL      = 0,    // Hoạt động bình thường
    SUSPICIOUS  = 1,    // Phát hiện bất thường, đang theo dõi
    ACCIDENT    = 2,    // Xác nhận tai nạn
    UNKNOWN     = 3     // Chưa xác định (khởi động)
};

// Loại sự kiện tai nạn
enum class AccidentType : uint8_t {
    NONE        = 0,    // Không có tai nạn
    CRASH       = 1,    // Va chạm mạnh (totalAcc vượt ngưỡng)
    FALL        = 2,    // Xe ngã/lật (góc nghiêng vượt ngưỡng)
    SUDDEN_STOP = 3,    // Phanh gấp / dừng đột ngột (jerk cao)
    COMBINED    = 4     // Kết hợp nhiều dấu hiệu
};

struct AccidentEvent {
    VehicleState state;             // Trạng thái hiện tại
    AccidentType type;              // Loại tai nạn (nếu có)

    // Snapshot các giá trị tại thời điểm phát hiện
    float totalAccAtEvent;          // Tổng gia tốc khi sự kiện xảy ra
    float angleXAtEvent;            // Góc X khi sự kiện xảy ra
    float angleYAtEvent;            // Góc Y khi sự kiện xảy ra
    float jerkAtEvent;              // Jerk khi sự kiện xảy ra

    uint32_t eventTimestamp;        // millis() khi phát hiện
    bool     isActive;              // true = cần cảnh báo ngay
};


// ------------------------------------------------------------
// [Khối 5] Alert Command
// Lệnh điều khiển cảnh báo gửi đến AlertManager
// ------------------------------------------------------------
enum class AlertCommand : uint8_t {
    NONE            = 0,
    START_ALERT     = 1,    // Bắt đầu cảnh báo tai nạn
    STOP_ALERT      = 2,    // Dừng cảnh báo (manual reset)
    HEARTBEAT_BEEP  = 3     // Beep ngắn xác nhận hệ thống sống
};

struct AlertRequest {
    AlertCommand  command;
    AccidentType  accidentType;     // Loại tai nạn để chọn pattern phù hợp
    uint32_t      timestamp;
};


// ------------------------------------------------------------
// [Khối 6] Watchdog Heartbeat
// Mỗi Task gửi heartbeat để Watchdog biết task còn sống
// ------------------------------------------------------------
enum class TaskID : uint8_t {
    MOTION_SENSOR       = 0,
    DATA_NORMALIZER     = 1,
    SIGNAL_PROCESSOR    = 2,
    ACCIDENT_DETECTOR   = 3,
    ALERT_MANAGER       = 4,
    STATE_MANAGER       = 5,
    COUNT               = 6     // Tổng số task được giám sát
};

struct WatchdogHeartbeat {
    TaskID      taskId;
    uint32_t    timestamp;      // millis() khi gửi heartbeat
};


// ------------------------------------------------------------
// [Khối 7] System State
// Trạng thái tổng thể của toàn hệ thống
// ------------------------------------------------------------
enum class SystemState : uint8_t {
    BOOT        = 0,    // Khởi động ban đầu
    INIT        = 1,    // Khởi tạo các khối
    CALIBRATING = 2,    // Đang hiệu chỉnh gyro
    RUNNING     = 3,    // Hoạt động bình thường
    ERROR       = 4,    // Có lỗi nghiêm trọng
    RECOVERY    = 5     // Đang cố phục hồi
};

// Mã lỗi hệ thống
enum class SystemError : uint8_t {
    NONE                = 0,
    SENSOR_INIT_FAILED  = 1,    // Không thể khởi tạo MPU6050
    SENSOR_DISCONNECTED = 2,    // Mất kết nối sensor khi đang chạy
    TASK_TIMEOUT        = 3,    // Một task bị treo (watchdog timeout)
    QUEUE_OVERFLOW      = 4,    // Queue bị đầy liên tục
    LOW_MEMORY          = 5     // Heap memory quá thấp
};

struct SystemStatus {
    SystemState state;
    SystemError lastError;
    uint32_t    uptimeMs;           // Thời gian hoạt động (ms)
    uint32_t    freeHeap;           // Heap còn lại (bytes)
    bool        allTasksHealthy;    // Tất cả task đang hoạt động
};


// ------------------------------------------------------------
// Utility: Chuyển enum sang string (dùng để log)
// ------------------------------------------------------------
inline const char* vehicleStateToString(VehicleState state) {
    switch (state) {
        case VehicleState::NORMAL:     return "NORMAL";
        case VehicleState::SUSPICIOUS: return "SUSPICIOUS";
        case VehicleState::ACCIDENT:   return "ACCIDENT";
        default:                       return "UNKNOWN";
    }
}

inline const char* accidentTypeToString(AccidentType type) {
    switch (type) {
        case AccidentType::NONE:        return "NONE";
        case AccidentType::CRASH:       return "CRASH";
        case AccidentType::FALL:        return "FALL";
        case AccidentType::SUDDEN_STOP: return "SUDDEN_STOP";
        case AccidentType::COMBINED:    return "COMBINED";
        default:                        return "UNKNOWN";
    }
}

inline const char* systemStateToString(SystemState state) {
    switch (state) {
        case SystemState::BOOT:        return "BOOT";
        case SystemState::INIT:        return "INIT";
        case SystemState::CALIBRATING: return "CALIBRATING";
        case SystemState::RUNNING:     return "RUNNING";
        case SystemState::ERROR:       return "ERROR";
        case SystemState::RECOVERY:    return "RECOVERY";
        default:                       return "UNKNOWN";
    }
}

inline const char* taskIdToString(TaskID id) {
    switch (id) {
        case TaskID::MOTION_SENSOR:     return "MotionSensor";
        case TaskID::DATA_NORMALIZER:   return "DataNormalizer";
        case TaskID::SIGNAL_PROCESSOR:  return "SignalProcessor";
        case TaskID::ACCIDENT_DETECTOR: return "AccidentDetector";
        case TaskID::ALERT_MANAGER:     return "AlertManager";
        case TaskID::STATE_MANAGER:     return "StateManager";
        default:                        return "Unknown";
    }
}
