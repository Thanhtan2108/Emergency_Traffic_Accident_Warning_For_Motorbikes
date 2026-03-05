#pragma once

// ============================================================
//  Config.h
//  Tập trung toàn bộ hằng số cấu hình hệ thống.
//  Chỉnh sửa tại đây để thay đổi hành vi toàn hệ thống.
// ============================================================

// ------------------------------------------------------------
// Hardware - GPIO Pin Definitions
// ------------------------------------------------------------
#define PIN_SDA             21          // I2C SDA
#define PIN_SCL             22          // I2C SCL
#define PIN_BUZZER          13          // Buzzer (PWM capable)
#define PIN_BUTTON          4           // Button for manual buzzer toggle
#define PIN_LED_STATUS      2           // LED trạng thái hệ thống (built-in)

// ------------------------------------------------------------
// I2C Configuration
// ------------------------------------------------------------
#define I2C_FREQUENCY       400000UL    // 400kHz Fast Mode

// ------------------------------------------------------------
// MPU6050 Complementary Filter Coefficients
// accCoef + gyroCoef = 1.0
// gyroCoef cao → tin gyro nhiều hơn (ít drift nhưng chậm chỉnh)
// accCoef cao  → tin accel nhiều hơn (nhanh chỉnh nhưng nhiễu)
// ------------------------------------------------------------
#define MPU_ACC_COEF        0.02f
#define MPU_GYRO_COEF       0.98f

// ------------------------------------------------------------
// Sampling & Timing
// ------------------------------------------------------------
#define MOTION_SAMPLE_RATE_HZ       100         // Tần suất đọc sensor (100Hz)
#define MOTION_SAMPLE_PERIOD_MS     (1000 / MOTION_SAMPLE_RATE_HZ)  // 10ms

#define GYRO_CALIB_DELAY_BEFORE_MS  1000        // Chờ trước khi hiệu chỉnh
#define GYRO_CALIB_DELAY_AFTER_MS   2000        // Chờ sau khi hiệu chỉnh

// ------------------------------------------------------------
// Signal Processing - Low Pass Filter
// alpha gần 1 → lọc mạnh (phản hồi chậm)
// alpha gần 0 → lọc yếu  (phản hồi nhanh)
// ------------------------------------------------------------
#define LPF_ALPHA_ACC       0.15f       // LPF cho gia tốc
#define LPF_ALPHA_GYRO      0.10f       // LPF cho vận tốc góc

// ------------------------------------------------------------
// Accident Detection Thresholds
// ------------------------------------------------------------

// Ngưỡng tổng gia tốc (g) để kết luận va chạm mạnh
// Normal riding ~1.0g, hard brake ~1.5g, crash > 2.5g
#define ACC_CRASH_THRESHOLD         2.5f

// Ngưỡng góc nghiêng (độ) để kết luận xe bị ngã
#define TILT_ANGLE_THRESHOLD        60.0f

// Ngưỡng jerk (g/s) - thay đổi gia tốc đột ngột
#define JERK_THRESHOLD              10.0f

// Ngưỡng vận tốc góc (°/s) khi té ngã
#define ANGULAR_VEL_THRESHOLD       200.0f

// Số frame liên tiếp phải vượt ngưỡng để xác nhận tai nạn (debounce)
#define ACCIDENT_CONFIRM_FRAMES     5

// Số frame liên tiếp bình thường để thoát trạng thái SUSPICIOUS
#define NORMAL_CONFIRM_FRAMES       10

// Thời gian (ms) giữ trạng thái ACCIDENT trước khi tự reset
#define ACCIDENT_HOLD_MS            30000UL     // 30 giây

// ------------------------------------------------------------
// Alert Configuration
// ------------------------------------------------------------
#define BUZZER_FREQ_HZ              2000        // Tần số buzzer (Hz)
#define BUZZER_BEEP_ON_MS           300         // Thời gian bật mỗi beep
#define BUZZER_BEEP_OFF_MS          200         // Thời gian tắt giữa beep
#define BUZZER_PWM_CHANNEL          0           // LEDC channel (ESP32)
#define BUZZER_PWM_RESOLUTION       8           // 8-bit resolution (0-255)
#define BUZZER_PWM_DUTY             128         // 50% duty cycle

// ------------------------------------------------------------
// FreeRTOS Task Configuration
// ------------------------------------------------------------

// Stack sizes (words = 4 bytes each)
#define STACK_MOTION_SENSOR         2048
#define STACK_DATA_NORMALIZER       2048
#define STACK_SIGNAL_PROCESSOR      3072
#define STACK_ACCIDENT_DETECTOR     2048
#define STACK_ALERT_MANAGER         2048
#define STACK_SYSTEM_WATCHDOG       2048
#define STACK_STATE_MANAGER         3072

// Task priorities (higher number = higher priority)
// Idle = 0, max = configMAX_PRIORITIES-1 (thường là 24 trên ESP32)
#define PRIORITY_STATE_MANAGER      6       // Cao nhất - bộ não hệ thống
#define PRIORITY_MOTION_SENSOR      5       // Cao - đọc sensor thời gian thực
#define PRIORITY_SYSTEM_WATCHDOG    5       // Cao - giám sát an toàn
#define PRIORITY_SIGNAL_PROCESSOR   4       // Trung bình cao
#define PRIORITY_DATA_NORMALIZER    4       // Trung bình cao
#define PRIORITY_ACCIDENT_DETECTOR  3       // Trung bình
#define PRIORITY_ALERT_MANAGER      2       // Thấp hơn - phản hồi output

// Core assignment (ESP32 dual-core)
#define CORE_SENSOR_TASKS           1       // Core 1: sensor pipeline
#define CORE_MANAGEMENT_TASKS       0       // Core 0: watchdog, state, alert

// ------------------------------------------------------------
// FreeRTOS Queue Configuration
// ------------------------------------------------------------
#define QUEUE_RAW_DATA_SIZE         5       // rawDataQueue depth
#define QUEUE_NORMALIZED_SIZE       5       // normalizedQueue depth
#define QUEUE_FEATURES_SIZE         5       // featuresQueue depth
#define QUEUE_ACCIDENT_SIZE         3       // accidentQueue depth
#define QUEUE_ALERT_SIZE            3       // alertQueue depth

#define QUEUE_SEND_TIMEOUT_MS       10      // Timeout khi gửi vào queue (ms)
#define QUEUE_RECV_TIMEOUT_MS       20      // Timeout khi đọc từ queue (ms)

// ------------------------------------------------------------
// System Watchdog Configuration
// ------------------------------------------------------------
#define WATCHDOG_TIMEOUT_MS         500     // Task phải kick trong 500ms
#define WATCHDOG_CHECK_PERIOD_MS    100     // Watchdog check mỗi 100ms
#define MAX_MONITORED_TASKS         8       // Số task tối đa watchdog giám sát

// ------------------------------------------------------------
// Logging / Debug
// ------------------------------------------------------------
#define SERIAL_BAUD_RATE            115200
#define LOG_ENABLED                 true    // Bật/tắt toàn bộ log
#define LOG_MOTION_DATA             false   // Log raw/normalized data (verbose)
#define LOG_FEATURES                false   // Log signal features
#define LOG_ACCIDENT_STATE          true    // Log trạng thái phát hiện tai nạn
#define LOG_SYSTEM_STATE            true    // Log trạng thái hệ thống
