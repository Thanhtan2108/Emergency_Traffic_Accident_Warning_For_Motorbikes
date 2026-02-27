#pragma once
// ============================================================
//  config.h
//  Tập trung toàn bộ hằng số và ngưỡng của hệ thống.
//  Đây là file DUY NHẤT cần chỉnh khi tinh chỉnh thuật toán
//  sau khi test thực tế trên xe.
//
//  Tài liệu: README.md – Section X
//  Thứ tự implement: file số 2 (sau DataTypes.h)
//
//  KHÔNG có magic number ở bất kỳ file nào khác trong project.
// ============================================================


// ============================================================
//  NHÓM 1 – NGƯỠNG GIA TỐC
//  Đơn vị: g  (1g ≈ 9.81 m/s²)
//  Áp dụng cho |a_dynamic| = | |a_raw| - baseline |
//
//  Mức tham chiếu (chỉ để hiểu ngữ cảnh, không phải hằng số):
//    Lái bình thường : |a_dynamic| < 1.5g
//    Ổ gà / đường xấu: 1.5 – 2.5g, thời gian ngắn
// ============================================================

// Mức gia tốc bình thường khi lái xe thông thường
#define A_NORMAL_MAX          1.50f    // g

// Vượt ngưỡng này → kích hoạt EVENT_DETECTED (dùng magnitude_a_raw)
#define A_EVENT_THRESHOLD     1.4f    // g

// Giới hạn trên để phân loại ROAD_SHOCK trong CLASSIFYING
#define A_SHOCK_MAX           2.2f    // g

// Vượt ngưỡng này trong CLASSIFYING → IMPACT_PENDING hoặc FALL_PENDING
#define A_IMPACT_THRESHOLD    2.5f    // g

// Giá trị khởi tạo baseline = 1g (trọng lực khi xe đứng yên, trục Z hướng lên)
#define BASELINE_INITIAL      1.0f    // g

// Hệ số EMA cập nhật baseline – chế độ bình thường (α nhỏ = cập nhật chậm, ổn định)
// baseline = baseline * (1 - α) + |a_raw| * α
#define BASELINE_ALPHA        0.05f   // –

// Hệ số EMA warm-up (α lớn hơn = hội tụ nhanh trong 50 mẫu đầu)
#define BASELINE_ALPHA_WARMUP 0.20f   // –

// Số mẫu cho Moving Average Filter
#define MA_FILTER_SIZE        5       // mẫu  (5 × 10ms = 50ms độ trễ)


// ============================================================
//  NHÓM 2 – NGƯỠNG TỐC ĐỘ GÓC
//  Đơn vị: °/s
//  Áp dụng cho |ω| = sqrt(gx² + gy² + gz²)
//
//  Mức tham chiếu:
//    Lái bình thường: |ω| < 100°/s
//    Lắc mạnh       : 100 – 300°/s
// ============================================================

// Tốc độ góc tối đa khi lái xe bình thường
#define W_NORMAL_MAX          100.0f  // °/s

// Vượt ngưỡng này → kích hoạt EVENT_DETECTED (dùng magnitude_omega_raw)
#define W_EVENT_THRESHOLD     120.0f  // °/s

// Ngưỡng phân loại FALL trong CLASSIFYING
#define W_FALL_THRESHOLD      200.0f  // °/s

// Ngưỡng "đứng yên" trong FALL_PENDING – lọc dựng chân chống
// Kết hợp với ROLL_CHANGE_RATE_SLOW để phát hiện xe được dựng lại từ từ
#define W_STATIC_MAX          30.0f   // °/s


// ============================================================
//  NHÓM 3 – NGƯỠNG GÓC NGHIÊNG
//  Đơn vị: °
//  roll = getAngleX(), pitch = getAngleY()
// ============================================================

// Ngưỡng VÀO FALL_PENDING/FALL_DETECTED (xe đang nằm)
// Hysteresis: vào khi roll > 60°, ra khi roll < 40°  →  tránh oscillation
#define ROLL_FALL_THRESHOLD   45.0f   // °  ← ngưỡng VÀO

// Ngưỡng RA khỏi FALL (xe đã được dựng lại đủ thẳng)
#define ROLL_NORMAL_MAX       40.0f   // °  ← ngưỡng RA  (hysteresis gap = 20°)

// Ngưỡng xác nhận xe chúi đầu trong FALL_PENDING
#define PITCH_FALL_THRESHOLD  70.0f   // °

// Roll tối đa khi cua bình thường (tham chiếu chống báo giả khi cua)
#define ROLL_TURN_MAX         45.0f   // °

// Thay đổi roll tối thiểu để KHÔNG phân loại là ROAD_SHOCK trong CLASSIFYING
#define ROLL_DELTA_MIN        15.0f   // °

// Thay đổi pitch tối thiểu để KHÔNG phân loại là ROAD_SHOCK trong CLASSIFYING
#define PITCH_DELTA_MIN       20.0f   // °


// ============================================================
//  NHÓM 4 – NGƯỠNG TỐC ĐỘ THAY ĐỔI GÓC
//  Đơn vị: °/s  (tính từ delta roll / delta time)
//  Dùng trong FALL_PENDING để phân biệt:
//    ngã thật     → roll tăng nhanh (roll_rate cao)
//    chân chống   → roll tăng chậm, đều (roll_rate thấp)
// ============================================================

// Dưới ngưỡng này + |ω| < W_STATIC_MAX → đang dựng chân chống → TIMEOUT_RESET
#define ROLL_CHANGE_RATE_SLOW 5.0f    // °/s

// Trên ngưỡng này → xe đang ngã thật (tham chiếu)
#define ROLL_CHANGE_RATE_FAST 30.0f   // °/s


// ============================================================
//  NHÓM 5 – NGƯỠNG THỜI GIAN
//  Đơn vị: ms (milliseconds)
// ============================================================

// Spike ngắn hơn T_SPIKE_MIN → nhiễu điện, thoát EVENT_DETECTED sớm → TIMEOUT_RESET
#define T_SPIKE_MIN           10      // ms

// Spike ngắn hơn T_SPIKE_MAX → điều kiện ROAD_SHOCK trong CLASSIFYING
#define T_SPIKE_MAX           80      // ms

// Cửa sổ thu thập EventSnapshot trong EVENT_DETECTED
#define T_EVENT_WINDOW        300     // ms

// Thời gian chờ xác nhận trong IMPACT_PENDING trước khi → STRONG_IMPACT
#define T_IMPACT_CONFIRM      500     // ms

// Thời gian |roll| phải liên tục > ROLL_FALL_THRESHOLD để xác nhận → FALL_DETECTED
#define T_FALL_CONFIRM        800    // ms

// Timeout riêng cho EVENT_DETECTED (phòng trường hợp bất thường)
#define T_STATE_TIMEOUT       600     // ms

// Timeout tổng cho các intermediate state – WatchdogTask force reset nếu vượt
#define T_STATE_MAX_TIMEOUT   5000    // ms

// Thời gian debounce sau khi phát cảnh báo – chặn event mới trong thời gian này
#define T_DEBOUNCE            3000    // ms

// Chu kỳ đọc sensor: 1000ms / 10ms = 100Hz
#define SENSOR_PERIOD_MS      10      // ms

// Thời gian bắt buộc ở WARMUP state sau khi begin()
// Đảm bảo Complementary Filter ổn định và baseline hội tụ
#define WARMUP_DURATION_MS    2000    // ms

// Buffer timeout bảo vệ sau T_IMPACT_CONFIRM trong IMPACT_PENDING
// Tổng timeout IMPACT_PENDING = T_IMPACT_CONFIRM + T_IMPACT_CONFIRM_BUFFER = 600ms
#define T_IMPACT_CONFIRM_BUFFER  100  // ms

// WatchdogTask: chu kỳ kiểm tra sức khỏe hệ thống
#define WATCHDOG_CHECK_INTERVAL_MS  1000  // ms

// WatchdogTask: timeout lấy stateMutex – không chờ lâu tránh block
#define WATCHDOG_MUTEX_TIMEOUT_MS   100   // ms

// StateTask: ngưỡng "chết" – không phản hồi quá thời gian này
#define STATE_DEAD_TIMEOUT_MS       1000  // ms


// ============================================================
//  NHÓM 6 – NGƯỠNG KHỞI TẠO & SENSOR HEALTH
// ============================================================

// Số mẫu warm-up để baseline EMA hội tụ (50 × 10ms = 500ms)
// Sau khi đủ mẫu này, ProcessedData.baseline_ready = true
#define BASELINE_WARMUP_SAMPLES   50  // mẫu

// Số lần đọc sensor lỗi liên tiếp trước khi log WARNING một lần
#define SENSOR_MAX_ERROR_COUNT    10  // lần

// Nếu không nhận được mẫu hợp lệ trong thời gian này → sensor dead
#define SENSOR_DEAD_TIMEOUT_MS    500 // ms

// Outlier Rejection – SignalProcessor Bước 0
// Vượt quá range vật lý MPU6050 config ±2g → nhiễu I2C → reject
#define OUTLIER_A_MAX             4.5f  // g   (> A_IMPACT_THRESHOLD + buffer)
// I2C trả về 0 khi mất kết nối → reject
#define OUTLIER_A_MIN             0.3f  // g
// Vượt quá range vật lý ±500°/s → nhiễu I2C → reject
#define OUTLIER_W_MAX             550.0f // °/s


// ============================================================
//  NHÓM 7 – FREERTOS TASK CONFIGURATION
// ============================================================

// --- Stack size (bytes) ---
#define SENSOR_TASK_STACK     4096    // bytes  – Core 1
#define STATE_TASK_STACK      4096    // bytes  – Core 1
#define OUTPUT_TASK_STACK     3072    // bytes  – Core 0
#define WATCHDOG_TASK_STACK   2048    // bytes  – Core 0

// --- Priority (số lớn = ưu tiên cao hơn trong FreeRTOS) ---
// SensorTask và StateTask cùng mức cao nhất để đảm bảo real-time
#define SENSOR_TASK_PRIORITY   3      // Real-time sampling
#define STATE_TASK_PRIORITY    3      // Real-time state machine
#define WATCHDOG_TASK_PRIORITY 2      // Health monitor
#define OUTPUT_TASK_PRIORITY   1      // I/O không nghiêm ngặt

// --- Core assignment ---
// Core 1: toàn bộ real-time processing
// Core 0: I/O và watchdog (WiFi sẽ thêm vào đây ở phiên bản sau)
#define SENSOR_TASK_CORE      1
#define STATE_TASK_CORE       1
#define OUTPUT_TASK_CORE      0
#define WATCHDOG_TASK_CORE    0

// --- Queue size ---
// sensorQueue: SensorTask → StateTask
// Timeout = 0 phía writer (SensorTask không block, thà bỏ mẫu hơn lệch chu kỳ)
#define SENSOR_QUEUE_SIZE     5       // phần tử  (ProcessedData)

// alertQueue: StateTask → OutputTask
#define ALERT_QUEUE_SIZE      3       // phần tử  (AlertInfo)
