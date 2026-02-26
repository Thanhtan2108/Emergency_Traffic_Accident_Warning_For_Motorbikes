#pragma once
// ============================================================
//  DataTypes.h
//  Định nghĩa tập trung toàn bộ struct và enum cho hệ thống.
//  Đây là điểm tham chiếu DUY NHẤT – mọi file khác include
//  file này và KHÔNG tự định nghĩa lại kiểu dữ liệu.
//
//  Tài liệu: README.md – Section IV
//  Thứ tự implement: file số 1 (nền tảng của toàn bộ project)
// ============================================================

#include <Arduino.h>


// ============================================================
//  PHẦN 1 – ENUM
//  Khai báo DetectorState trước vì SystemHealth phụ thuộc vào.
// ============================================================

// ------------------------------------------------------------
//  DetectorState  (Section IV.6)
//  Tất cả trạng thái có thể của State Machine.
//  Dùng uint8_t để tiết kiệm RAM trong struct SystemHealth.
// ------------------------------------------------------------
enum DetectorState : uint8_t {
    WARMUP         = 0,   // Khởi động – chờ baseline ổn định, KHÔNG detect event
    NORMAL         = 1,   // Hoạt động bình thường, baseline được cập nhật
    EVENT_DETECTED = 2,   // Đang thu thập dữ liệu sự kiện (cửa sổ 300ms)
    CLASSIFYING    = 3,   // Phân loại snapshot – chạy 1 lần, không chờ
    ROAD_SHOCK     = 4,   // Ổ gà / đường xấu – KHÔNG cảnh báo → về NORMAL
    IMPACT_PENDING = 5,   // Nghi va chạm, đang chờ xác nhận 500ms
    FALL_PENDING   = 6,   // Nghi té ngã, đang chờ xác nhận góc 2000ms
    STRONG_IMPACT  = 7,   // Xác nhận va chạm mạnh – phát ALERT_IMPACT
    FALL_DETECTED  = 8,   // Xác nhận té ngã – phát ALERT_FALL
    TIMEOUT_RESET  = 9    // Hết timeout – reset ngay về NORMAL
};

// ------------------------------------------------------------
//  AlertType  (Section IV.4)
//  Phân loại cảnh báo gửi từ StateTask sang OutputTask.
// ------------------------------------------------------------
enum AlertType : uint8_t {
    ALERT_NONE   = 0,
    ALERT_IMPACT = 1,   // Va chạm mạnh – buzzer 3 tiếng
    ALERT_FALL   = 2    // Té ngã – buzzer liên tục
};

// ------------------------------------------------------------
//  EventGroup bit mask  (Section IX.1)
//  Dùng cho xEventGroupSetBits() / xEventGroupWaitBits()
// ------------------------------------------------------------
#define ALERT_IMPACT_BIT  (EventBits_t)(1 << 0)
#define ALERT_FALL_BIT    (EventBits_t)(1 << 1)


// ============================================================
//  PHẦN 2 – STRUCT
//  Thứ tự theo luồng dữ liệu của hệ thống:
//    SensorReader → SignalProcessor → AccidentDetector → OutputTask
//  SystemHealth đứng riêng, dùng bởi lớp FreeRTOS.
// ============================================================

// ------------------------------------------------------------
//  4.1  RawSensorData  (Section IV.1)
//  Đầu ra của SensorReader.
//  Đầu vào của SignalProcessor.
// ------------------------------------------------------------
struct RawSensorData {
    // Gia tốc thô – range ±2g (ACCEL_CONFIG = 0x00, LSB = 16384)
    float    ax;
    float    ay;
    float    az;

    // Tốc độ góc thô – range ±500°/s (GYRO_CONFIG = 0x08, LSB = 65.5)
    float    gx;
    float    gy;
    float    gz;

    // Góc từ Complementary Filter trong MPU6050_tockn
    float    roll;    // getAngleX() – °
    float    pitch;   // getAngleY() – °
    // KHÔNG có yaw: getAngleZ() bị drift tích lũy, không dùng

    uint32_t timestamp;  // millis() lúc đọc sensor
    bool     valid;      // false nếu I2C lỗi, timeout, hoặc outlier bị reject
};

// ------------------------------------------------------------
//  4.2  ProcessedData  (Section IV.2)
//  Đầu ra của SignalProcessor (sau pipeline 6 bước).
//  Đầu vào của AccidentDetector.
// ------------------------------------------------------------
struct ProcessedData {
    // --- Gia tốc động ---
    float    magnitude_a;          // |a_dynamic| qua MA filter (g)
                                   // Dùng để so sánh ngưỡng trong State Machine
    float    magnitude_a_raw;      // |a_dynamic| CHƯA lọc (g)
                                   // Dùng để kích hoạt EVENT tức thì (không có độ trễ)

    // --- Tốc độ góc ---
    float    magnitude_omega;      // |ω| qua MA filter (°/s)
    float    magnitude_omega_raw;  // |ω| CHƯA lọc (°/s)
                                   // Dùng để kích hoạt EVENT tức thì

    // --- Tư thế ---
    float    roll;        // Góc roll hiện tại (°)
    float    pitch;       // Góc pitch hiện tại (°)
    float    roll_rate;   // Tốc độ thay đổi roll (°/giây)
                          // Phân biệt: ngã thật (cao) vs dựng chân chống (thấp, đều)

    // --- Baseline ---
    float    baseline;        // Giá trị baseline EMA hiện tại (g)
    bool     baseline_ready;  // true sau khi đủ BASELINE_WARMUP_SAMPLES mẫu
                              // AccidentDetector KHÔNG được detect event khi false

    uint32_t timestamp;  // millis() lúc xử lý
};

// ------------------------------------------------------------
//  4.3  EventSnapshot  (Section IV.3)
//  "Ảnh chụp" dữ liệu thu thập trong cửa sổ EVENT_DETECTED.
//  AccidentDetector dùng để ra quyết định trong CLASSIFYING.
//  Reset về 0 mỗi lần vào TIMEOUT_RESET hoặc forceReset().
// ------------------------------------------------------------
struct EventSnapshot {
    float    peak_a;         // |a_dynamic_raw| đỉnh trong cửa sổ (g)
    float    peak_omega;     // |ω_raw| đỉnh trong cửa sổ (°/s)

    float    roll_at_event;  // Roll khi EVENT BẮT ĐẦU (°)
    float    pitch_at_event; // Pitch khi EVENT BẮT ĐẦU (°)

    float    roll_delta;     // |roll_now − roll_at_event| (°) – cập nhật liên tục
    float    pitch_delta;    // |pitch_now − pitch_at_event| (°) – cập nhật liên tục

    uint32_t spike_duration; // Tổng ms |a_raw| liên tục vượt A_EVENT_THRESHOLD
    uint32_t event_start;    // millis() khi EVENT_DETECTED bắt đầu
};

// ------------------------------------------------------------
//  4.4  AlertInfo  (Section IV.4)
//  Payload gửi qua alertQueue từ StateTask sang OutputTask.
//  Đồng thời set EventGroup bit để OutputTask unblock ngay.
// ------------------------------------------------------------
struct AlertInfo {
    AlertType type;       // ALERT_IMPACT hoặc ALERT_FALL

    float    peak_a;      // |a| đỉnh tại sự kiện (g)   – dùng để log
    float    peak_omega;  // |ω| đỉnh tại sự kiện (°/s) – dùng để log
    float    roll;        // Góc roll khi phát cảnh báo (°)
    float    pitch;       // Góc pitch khi phát cảnh báo (°)

    uint32_t timestamp;   // millis() lúc phát cảnh báo
};

// ------------------------------------------------------------
//  4.5  SystemHealth  (Section IV.5)
//  Dữ liệu sức khỏe hệ thống cho WatchdogTask giám sát.
//
//  QUY TẮC OWNERSHIP (Section IX.2):
//    • volatile fields → SensorTask ghi, 1 writer duy nhất
//      Không cần Mutex (volatile đủ để tránh compiler optimization)
//    • non-volatile fields → StateTask ghi
//      BẮT BUỘC bảo vệ bởi stateMutex khi đọc/ghi
//    • WatchdogTask: CHỈ ĐỌC
//      Ngoại lệ: reset queue_overflow_count = 0 (race condition nhỏ, chấp nhận được)
// ------------------------------------------------------------
struct SystemHealth {

    // --- SensorTask ghi (volatile – không cần Mutex) ---
    volatile uint32_t last_sensor_tick;    // Timestamp lần đọc sensor gần nhất (ms)
    volatile bool     sensor_task_alive;   // SensorTask còn hoạt động?
    volatile uint16_t sensor_error_count;  // Lỗi đọc liên tiếp – reset về 0 khi có mẫu tốt
    volatile uint16_t queue_overflow_count; // Số lần sensorQueue bị đầy (bỏ mẫu)

    // --- StateTask ghi (BẢO VỆ bởi stateMutex) ---
    uint32_t      last_state_tick;   // Timestamp lần StateTask xử lý gần nhất (ms)
    uint32_t      state_enter_time;  // millis() khi vào state hiện tại
    DetectorState current_state;     // State Machine đang ở đâu
    bool          state_task_alive;  // StateTask còn hoạt động?
};
