# THIẾT KẾ HỆ THỐNG PHÁT HIỆN TÉ NGÃ / VA CHẠM VỚI FREERTOS

**Dự án:** Accident Detection System – ESP32 + MPU6050  
**Phiên bản:** 2.0  
**Ngày:** 2026-02-25  
**Trạng thái:** Hoàn thiện – sẵn sàng implement  
**Thay đổi so với v1.0:** Bổ sung threshold còn thiếu, pipeline xử lý tín hiệu chi tiết, outlier rejection, warm-up period, interface đầy đủ cho AccidentDetector, làm rõ ownership SystemHealth

---

## MỤC LỤC

1. [Tổng quan hệ thống](#i-tổng-quan-hệ-thống)
2. [Ràng buộc & giả định thiết kế](#ii-ràng-buộc--giả-định-thiết-kế)
3. [Kiến trúc phân tầng](#iii-kiến-trúc-phân-tầng)
4. [Cấu trúc dữ liệu trung tâm](#iv-cấu-trúc-dữ-liệu-trung-tâm)
5. [Tầng 1 – Sensor Reader](#v-tầng-1--sensor-reader)
6. [Tầng 2 – Signal Processor](#vi-tầng-2--signal-processor)
7. [Tầng 3 – Accident Detector (State Machine)](#vii-tầng-3--accident-detector-state-machine)
8. [Tầng 4 – FreeRTOS Task Design](#viii-tầng-4--freertos-task-design)
9. [Cơ chế đồng bộ hóa](#ix-cơ-chế-đồng-bộ-hóa)
10. [Bảng ngưỡng tham số (config.h)](#x-bảng-ngưỡng-tham-số-configh)
11. [Sơ đồ luồng tổng thể](#xi-sơ-đồ-luồng-tổng-thể)
12. [Những quyết định thiết kế quan trọng](#xii-những-quyết-định-thiết-kế-quan-trọng)
13. [Rủi ro & biện pháp xử lý](#xiii-rủi-ro--biện-pháp-xử-lý)
14. [Danh sách file & thứ tự implement](#xiv-danh-sách-file--thứ-tự-implement)

---

## I. TỔNG QUAN HỆ THỐNG

### Mục tiêu

Xây dựng hệ thống nhúng real-time chạy trên **ESP32**, sử dụng cảm biến **MPU6050** để liên tục theo dõi chuyển động của xe máy, phát hiện và phân loại chính xác 3 loại sự kiện:

| Loại sự kiện | Mô tả | Hành động |
| --- | --- | --- |
| `ROAD_SHOCK` | Ổ gà, đường xấu, sốc nhẹ | **Không cảnh báo** |
| `STRONG_IMPACT` | Va chạm mạnh, tai nạn | Cảnh báo mức cao |
| `FALL_DETECTED` | Xe đổ nằm xuống | Cảnh báo nghiêm trọng |

### Tư duy cốt lõi

> **Không bao giờ kết luận chỉ bằng 1 ngưỡng hay 1 mẫu đo duy nhất.**  
> Mọi sự kiện = **Phát hiện → Diễn tiến → Trạng thái sau cùng**  
> Khác nhau ở: **cường độ + thời gian + tư thế sau sự kiện**

### Yêu cầu chức năng

- Đọc cảm biến ở tần số **100Hz** (mỗi 10ms)
- Phân loại sự kiện trong vòng **< 500ms** kể từ khi xảy ra
- Xác nhận té ngã trong vòng **2–3 giây**
- Không báo giả khi đi đường xấu, phanh gấp, hoặc vào cua
- Hệ thống tự phục hồi sau mỗi sự kiện, không cần reset thủ công

### Yêu cầu phi chức năng

- Không sử dụng `delay()` ở bất kỳ đâu trong toàn bộ hệ thống
- Mọi task và mọi state đều có timeout tự phục hồi
- Hệ thống không bị treo dù sensor mất tín hiệu hoàn toàn
- Code OOP, tách biệt rõ từng tầng, có thể test độc lập từng module

---

## II. RÀNG BUỘC & GIẢ ĐỊNH THIẾT KẾ

### Phần cứng & phần mềm

| Thành phần | Chi tiết |
| --- | --- |
| Vi điều khiển | ESP32 – Dual-core Xtensa LX6, 240MHz |
| Cảm biến | MPU6050 giao tiếp I2C |
| Thư viện cảm biến | MPU6050_tockn (đã phân tích kỹ) |
| Ngôn ngữ | C++ – Arduino framework trên ESP32 |
| RTOS | FreeRTOS tích hợp sẵn trong ESP32 Arduino Core |

### Cấu hình MPU6050 (do thư viện MPU6050_tockn thiết lập)

| Thanh ghi | Giá trị | Ý nghĩa |
| --- | --- | --- |
| GYRO_CONFIG | 0x08 | Full scale ±500°/s → LSB = 65.5 |
| ACCEL_CONFIG | 0x00 | Full scale ±2g → LSB = 16384 |
| PWR_MGMT_1 | 0x01 | Clock từ PLL X-gyro |

> **Quan trọng:** Accel range ±2g → giá trị |a| hợp lệ tối đa = 2g. Gyro range ±500°/s → |ω| hợp lệ tối đa = 500°/s. Bất kỳ giá trị nào vượt quá range vật lý này đều là nhiễu I2C và phải bị reject.

### Giả định lắp đặt cảm biến

- MPU6050 gắn cố định trên khung xe, không bị rung lắc tương đối so với xe
- Trục Z của sensor hướng lên trên khi xe đứng thẳng bình thường → accZ ≈ 1g khi đứng yên
- AD0 = LOW → địa chỉ I2C = 0x68
- **Không dùng getAngleZ() (yaw)** vì yaw drift theo thời gian và không có giá trị trong phát hiện té ngã

### Phạm vi phiên bản hiện tại (v2.0)

- Không tích hợp GPS
- Không gửi cảnh báo qua WiFi/SMS (dành cho phiên bản sau)
- Không phân biệt hướng va chạm (trước/sau/bên hông)
- Chỉ hỗ trợ 1 cảm biến MPU6050 duy nhất

---

## III. KIẾN TRÚC PHÂN TẦNG

```text
┌─────────────────────────────────────────────────────────────────┐
│                       TẦNG ỨNG DỤNG                             │
│                    AccidentDetector                             │
│     State Machine + Classification + Warm-up + Debounce         │
│                   [AccidentDetector.h/.cpp]                     │
├─────────────────────────────────────────────────────────────────┤
│                   TẦNG XỬ LÝ TÍN HIỆU                           │
│                     SignalProcessor                             │
│   Outlier Rejection → Magnitude → Baseline → Filter → Output    │
│                   [SignalProcessor.h/.cpp]                      │
├─────────────────────────────────────────────────────────────────┤
│                      TẦNG CẢM BIẾN                              │
│                       SensorReader                              │
│               Wrapper cho MPU6050_tockn                         │
│                   [SensorReader.h/.cpp]                         │
├─────────────────────────────────────────────────────────────────┤
│                      TẦNG FREERTOS                              │
│       SensorTask | StateTask | OutputTask | WatchdogTask        │
│       Queue | EventGroup | Mutex | vTaskDelayUntil              │
│                   [TaskManager.h/.cpp]                          │
└─────────────────────────────────────────────────────────────────┘
```

### Nguyên tắc tách tầng

- Mỗi tầng chỉ biết tầng ngay bên dưới, không biết tầng trên
- Tầng cảm biến không biết gì về State Machine
- Tầng State Machine không biết gì về FreeRTOS Task
- Giao tiếp giữa các tầng hoàn toàn qua các struct trong `DataTypes.h`
- Mỗi tầng có thể được test độc lập với dữ liệu mock

---

## IV. CẤU TRÚC DỮ LIỆU TRUNG TÂM

Toàn bộ struct và enum tập trung trong `DataTypes.h`. Đây là điểm tham chiếu duy nhất cho toàn bộ hệ thống.

### 4.1 – RawSensorData

Dữ liệu thô từ MPU6050. Đầu ra của `SensorReader`, đầu vào của `SignalProcessor`.

```c
RawSensorData {
    float    ax, ay, az        // Gia tốc thô (g) – range ±2g
    float    gx, gy, gz        // Tốc độ góc thô (°/s) – range ±500°/s
    float    roll              // Góc roll từ Complementary Filter (°)
    float    pitch             // Góc pitch từ Complementary Filter (°)
    // Lưu ý: KHÔNG có yaw – không dùng getAngleZ() vì drift
    uint32_t timestamp         // millis() tại thời điểm đọc
    bool     valid             // false nếu I2C timeout, lỗi, hoặc outlier bị reject
}
```

### 4.2 – ProcessedData

Dữ liệu sau pre-processing. Đầu ra của `SignalProcessor`, đầu vào của `AccidentDetector`.

```c
ProcessedData {
    float    magnitude_a         // |a_dynamic| đã qua moving average filter (g)
    float    magnitude_a_raw     // |a_dynamic| chưa lọc – để detect spike tức thì
    float    magnitude_omega     // |ω| đã qua moving average filter (°/s)
    float    magnitude_omega_raw // |ω| chưa lọc – để detect spike tức thì
    float    roll                // Góc roll hiện tại (°)
    float    pitch               // Góc pitch hiện tại (°)
    float    roll_rate           // Tốc độ thay đổi roll (°/giây) – dùng lọc chân chống
    float    baseline            // Giá trị baseline gia tốc hiện tại (g)
    bool     baseline_ready      // true sau khi đủ BASELINE_WARMUP_SAMPLES mẫu
    uint32_t timestamp
}
```

> **Lý do thêm `roll_rate`:** Cần phân biệt "xe đang ngã" (roll_rate cao) vs "người dựng chân chống" (roll_rate thấp và đều). Tính từ delta roll / delta time giữa 2 lần gọi process().
.
> **Lý do thêm `baseline_ready`:** Trong `BASELINE_WARMUP_SAMPLES` mẫu đầu tiên, baseline chưa ổn định. AccidentDetector không được phép chuyển sang EVENT_DETECTED trong thời gian này.

### 4.3 – EventSnapshot

Ảnh chụp toàn bộ thông tin sự kiện thu thập trong cửa sổ `EVENT_DETECTED`.

```c
EventSnapshot {
    float    peak_a            // Giá trị |a| đỉnh cao nhất trong cửa sổ (g)
    float    peak_omega        // Giá trị |ω| đỉnh cao nhất trong cửa sổ (°/s)
    float    roll_at_event     // Roll tại thời điểm bắt đầu sự kiện (°)
    float    pitch_at_event    // Pitch tại thời điểm bắt đầu sự kiện (°)
    float    roll_delta        // Tổng thay đổi roll trong cửa sổ sự kiện (°)
    float    pitch_delta       // Tổng thay đổi pitch trong cửa sổ sự kiện (°)
    uint32_t spike_duration    // Thời gian |a_raw| liên tục vượt ngưỡng (ms)
    uint32_t event_start       // Timestamp bắt đầu sự kiện (millis)
}
```

### 4.4 – AlertInfo

Thông tin cảnh báo gửi từ `StateTask` sang `OutputTask`.

```c
enum AlertType {
    ALERT_NONE   = 0
    ALERT_IMPACT = 1
    ALERT_FALL   = 2
}

AlertInfo {
    AlertType type
    float     peak_a
    float     peak_omega
    float     roll
    float     pitch
    uint32_t  timestamp
}
```

### 4.5 – SystemHealth

Dữ liệu sức khỏe hệ thống. Được ghi bởi SensorTask và StateTask, được đọc bởi WatchdogTask.

```c
SystemHealth {
    // Ghi bởi SensorTask – dùng volatile, không cần Mutex
    volatile uint32_t last_sensor_tick      // Timestamp đọc sensor gần nhất
    volatile bool     sensor_task_alive
    volatile uint16_t sensor_error_count    // Số lần đọc lỗi liên tiếp

    // Ghi bởi StateTask – bảo vệ bởi stateMutex
    uint32_t      last_state_tick           // Timestamp StateTask xử lý gần nhất
    uint32_t      state_enter_time          // Thời điểm vào state hiện tại
    DetectorState current_state
    bool          state_task_alive
    uint16_t      queue_overflow_count      // Số lần sensorQueue bị đầy
}
```

**Quy tắc ownership SystemHealth:**

> - Các field SensorTask ghi (`last_sensor_tick`, `sensor_task_alive`, `sensor_error_count`) được khai báo `volatile` và chỉ có **1 writer duy nhất** (SensorTask) → không cần Mutex, chỉ cần volatile để tránh compiler optimization.
-.-
> - Các field StateTask ghi được bảo vệ bởi `stateMutex` vì WatchdogTask cũng đọc chúng đồng thời.
-.-
> - WatchdogTask chỉ **đọc**, không ghi vào SystemHealth.

### 4.6 – DetectorState (enum)

```c
enum DetectorState {
    WARMUP,          // Trạng thái khởi động, chờ baseline ổn định
    NORMAL,
    EVENT_DETECTED,
    CLASSIFYING,
    ROAD_SHOCK,
    IMPACT_PENDING,
    FALL_PENDING,
    STRONG_IMPACT,
    FALL_DETECTED,
    TIMEOUT_RESET
}
```

> **Thêm `WARMUP`:** State bắt buộc sau `begin()`, giữ trong `WARMUP_DURATION_MS`. Không cho phép chuyển sang EVENT_DETECTED trong thời gian này dù có spike.

---

## V. TẦNG 1 – SENSOR READER

### Trách nhiệm

Đóng gói `MPU6050_tockn`. Chỉ đọc và trả dữ liệu thô. Không xử lý, không lọc, không tính toán gì thêm.

### Interface – Các phương thức public

```c
begin(wire, calcOffset) → bool
    // Khởi tạo I2C, khởi động MPU6050
    // calcOffset=true: gọi calcGyroOffsets(false) – tính offset không in Serial
    // Trả về false nếu không tìm thấy sensor

read() → RawSensorData
    // Gọi mpu.update() rồi đóng gói kết quả
    // Đặt valid=false nếu I2C timeout hoặc không phản hồi

isReady() → bool
    // Kiểm tra sensor có phản hồi ở địa chỉ 0x68 không

getLastError() → String
    // Mô tả lỗi gần nhất để debug
```

### Lưu ý implement

- `begin()` gọi `calcGyroOffsets(false)` – tắt in Serial để không block
- `read()` đặt `valid = false` nếu Wire timeout hoặc dữ liệu không hợp lệ
- `roll` lấy từ `mpu.getAngleX()`, `pitch` từ `mpu.getAngleY()` – đã qua Complementary Filter
- **Không lấy** `mpu.getAngleZ()` (yaw bị drift, không dùng)
- Không giữ state nội bộ nào ngoài con trỏ đến đối tượng `MPU6050_tockn`

---

## VI. TẦNG 2 – SIGNAL PROCESSOR

### .Trách nhiệm

Nhận `RawSensorData`, xử lý tín hiệu qua pipeline 6 bước, trả về `ProcessedData` sạch sẵn sàng cho State Machine.

### Pipeline xử lý tín hiệu (thứ tự bắt buộc)

```text
RawSensorData
     │
     ▼
[Bước 0] Outlier Rejection
     │  Loại bỏ mẫu nhiễu I2C bất hợp lý
     │  NẾU bị reject → trả ProcessedData với valid=false
     ▼
[Bước 1] Tính |a_raw| và |ω_raw|
     │  |a_raw| = sqrt(ax²+ay²+az²)
     │  |ω_raw| = sqrt(gx²+gy²+gz²)
     ▼
[Bước 2] Tính Dynamic Acceleration (trừ baseline)
     │  |a_dynamic_raw| = | |a_raw| - baseline |
     ▼
[Bước 3] Moving Average Filter (N=5)
     │  Áp dụng cho |a_dynamic| và |ω|
     │  Lưu đồng thời cả raw và smoothed
     ▼
[Bước 4] Tính roll_rate
     │  roll_rate = (roll_current - roll_prev) / delta_t * 1000
     │              (°/giây, delta_t tính bằng ms)
     ▼
[Bước 5] Cập nhật Baseline (chỉ khi được phép)
     │  EMA: baseline = baseline * (1-α) + |a_raw| * α
     │  Chỉ chạy khi setBaselineUpdateEnabled(true)
     ▼
ProcessedData
```

> **Tại sao bước 2 (trừ baseline) phải trước bước 3 (filter):**  
> Nếu filter trước rồi mới trừ baseline, sẽ có tình huống baseline bị "smooth" cùng signal, làm mất đặc tính của spike. Trừ baseline trên raw signal trước, sau đó mới smooth kết quả, đảm bảo `magnitude_a` phản ánh đúng mức độ "lệch so với bình thường".

---

### Bước 0 – Outlier Rejection (Chi tiết)

**Mục đích:** Loại bỏ mẫu nhiễu I2C cực đoan trước khi đưa vào bất kỳ tính toán nào. Nếu không có bước này, 1 mẫu nhiễu có thể kéo moving average sai trong 5 chu kỳ tiếp theo (50ms).

**Điều kiện reject:**

```text
NẾU |a_raw| > OUTLIER_A_MAX (4.5g):
    // Vượt quá range vật lý của MPU6050 config ±2g + ngưỡng tai nạn
    // Giá trị này không thể xảy ra trong thực tế → nhiễu I2C
    → valid = false, bỏ qua mẫu

NẾU |ω_raw| > OUTLIER_W_MAX (550 °/s):
    // Vượt quá range vật lý ±500°/s
    → valid = false, bỏ qua mẫu

NẾU |a_raw| < OUTLIER_A_MIN (0.3g):
    // Gia tốc quá nhỏ kể cả khi rơi tự do không thể < 0.3g
    // Thường do I2C trả về 0 khi bị ngắt kết nối
    → valid = false, bỏ qua mẫu
```

> **Lưu ý:** SensorTask đếm số lần `valid=false` liên tiếp. Nếu vượt `SENSOR_MAX_ERROR_COUNT` (10 lần) → log WARNING "Sensor có thể mất kết nối".

---

### Bước 1 – Tính Magnitude (Chi tiết)

```c
|a_raw|   = sqrt(ax² + ay² + az²)
|ω_raw|   = sqrt(gx² + gy² + gz²)
```

---

### Bước 2 – Dynamic Acceleration (Chi tiết)

```c
|a_dynamic_raw| = | |a_raw| - baseline |
```

Khi xe đứng yên: `|a_raw|` ≈ 1.0g (trọng lực), baseline ≈ 1.0g → `|a_dynamic_raw|` ≈ 0.  
Khi có va chạm: `|a_raw|` tăng vọt → `|a_dynamic_raw|` tăng tương ứng.  
Dùng giá trị tuyệt đối để xử lý cả trường hợp `|a_raw|` đột ngột giảm (rơi tự do).

---

### Bước 3 – Moving Average Filter (Chi tiết)

Dùng **circular buffer** để tránh dịch chuyển mảng mỗi chu kỳ:

```c
Buffer: float a_buffer[MA_FILTER_SIZE]   // size = 5
        float w_buffer[MA_FILTER_SIZE]
        int   buffer_index = 0           // con trỏ vị trí ghi tiếp theo

Mỗi chu kỳ:
    a_buffer[buffer_index] = |a_dynamic_raw|
    w_buffer[buffer_index] = |ω_raw|
    buffer_index = (buffer_index + 1) % MA_FILTER_SIZE

    magnitude_a     = sum(a_buffer) / MA_FILTER_SIZE   // smoothed
    magnitude_omega = sum(w_buffer) / MA_FILTER_SIZE   // smoothed

    magnitude_a_raw     = |a_dynamic_raw|   // raw, không qua filter
    magnitude_omega_raw = |ω_raw|           // raw, không qua filter
```

**Khi nào reset buffer:** Gọi `resetFilter()` sau khi xử lý xong sự kiện. Reset bằng cách fill toàn bộ buffer về 0. Điều này tránh "dư âm" của spike từ sự kiện cũ ảnh hưởng sang lần detect tiếp theo.

---

### Bước 4 – Tính roll_rate (Chi tiết)

```c
roll_rate = (roll_current - roll_prev) / (timestamp_current - timestamp_prev) * 1000.0f
// Đơn vị: °/giây
// timestamp tính bằng ms → nhân 1000 để ra °/giây

roll_prev = roll_current   // lưu lại cho lần tiếp theo
```

Dùng để phân biệt trong `FALL_PENDING`:

- `roll_rate` > `ROLL_CHANGE_RATE_FAST` (30°/s) → xe đang ngã thật

- `roll_rate` < `ROLL_CHANGE_RATE_SLOW` (5°/s) → dựng chân chống hoặc xe được dựng lại

---

### Bước 5 – Cập nhật Baseline Động (Chi tiết)

```text
// CHỈ chạy khi baseline_update_enabled == true (do StateTask điều khiển)
// VÀ warmup_sample_count >= BASELINE_WARMUP_SAMPLES

NẾU baseline_update_enabled VÀ warmup_sample_count >= BASELINE_WARMUP_SAMPLES:
    baseline = baseline * (1.0f - BASELINE_ALPHA) + |a_raw| * BASELINE_ALPHA
    // BASELINE_ALPHA = 0.05
KHÔNG THÌ NẾU warmup_sample_count < BASELINE_WARMUP_SAMPLES:
    // Giai đoạn warm-up: cập nhật nhanh hơn để baseline hội tụ nhanh
    baseline = baseline * 0.8f + |a_raw| * 0.2f
    warmup_sample_count++
    // Sau BASELINE_WARMUP_SAMPLES mẫu, baseline_ready = true
```

**Giá trị khởi tạo baseline:** `BASELINE_INITIAL = 1.0g` – tương ứng với trọng lực khi xe đứng yên. Đây là điểm xuất phát tốt nhất trước khi EMA hội tụ về giá trị thực.

**Tại sao dùng α=0.2 trong warm-up:** Cần baseline hội tụ nhanh trong 50 mẫu đầu (0.5 giây). Sau đó chuyển sang α=0.05 để cập nhật chậm và ổn định.

---

### .Interface – Các phương thức public

```c
begin()
    // Khởi tạo circular buffer (fill = 0)
    // Đặt baseline = BASELINE_INITIAL (1.0g)
    // Đặt warmup_sample_count = 0, baseline_ready = false
    // Đặt baseline_update_enabled = true (để warm-up chạy ngay)

process(RawSensorData) → ProcessedData
    // Chạy toàn bộ pipeline 6 bước theo thứ tự
    // Nếu input.valid == false: trả ProcessedData rỗng với valid=false

setBaselineUpdateEnabled(bool enabled)
    // StateTask gọi hàm này để bật/tắt cập nhật baseline
    // Tắt khi vào EVENT_DETECTED, bật lại khi về NORMAL/TIMEOUT_RESET

isBaselineReady() → bool
    // Trả về baseline_ready
    // AccidentDetector dùng để biết khi nào thoát WARMUP state

getBaseline() → float
    // Trả về giá trị baseline hiện tại (để debug/log)

resetFilter()
    // Fill toàn bộ circular buffer về 0
    // Gọi sau khi xử lý xong sự kiện (từ STRONG_IMPACT, FALL_DETECTED, TIMEOUT_RESET)
```

---

## VII. TẦNG 3 – ACCIDENT DETECTOR (STATE MACHINE)

### 7.1 – Sơ đồ chuyển trạng thái đầy đủ

```text
[BEGIN]
   │
   ▼
 WARMUP ──────────────────────────────────────────────────────────────┐
   │ [baseline_ready == true                                          │
   │  VÀ millis()-start > WARMUP_DURATION_MS]                         │
   ▼                                                                  │
 NORMAL ◄─────────────────────────────────────────────────────────────┘
   │    ◄── TIMEOUT_RESET (reset ngay, không điều kiện)
   │    ◄── ROAD_SHOCK (sau khi log)
   │    ◄── STRONG_IMPACT (sau debounce VÀ |roll| < ROLL_NORMAL_MAX)
   │    ◄── FALL_DETECTED (sau debounce VÀ |roll| < ROLL_NORMAL_MAX)
   │
   │ [magnitude_a_raw ≥ A_EVENT_THRESHOLD
   │  HOẶC magnitude_omega_raw ≥ W_EVENT_THRESHOLD
   │  VÀ KHÔNG trong debounce
   │  VÀ baseline_ready == true]
   ▼
 EVENT_DETECTED ──[T_STATE_TIMEOUT 600ms không phân loại được]──► TIMEOUT_RESET
   │
   │ [sau T_EVENT_WINDOW = 300ms]
   │ [HOẶC spike kết thúc sớm VÀ spike_duration < T_SPIKE_MIN]
   ▼
 CLASSIFYING (chạy 1 lần, chuyển ngay)
   │
   ├──[spike_duration < T_SPIKE_MAX
   │   VÀ peak_a < A_SHOCK_MAX
   │   VÀ roll_delta < ROLL_DELTA_MIN]──────────────────► ROAD_SHOCK ──► NORMAL
   │
   ├──[peak_a ≥ A_IMPACT_THRESHOLD
   │   VÀ peak_omega ≥ W_FALL_THRESHOLD]────────────────► FALL_PENDING
   │
   ├──[peak_a ≥ A_IMPACT_THRESHOLD]─────────────────────► IMPACT_PENDING
   │
   ├──[peak_omega ≥ W_FALL_THRESHOLD
   │   VÀ roll_delta > ROLL_DELTA_MIN]──────────────────► FALL_PENDING
   │
   └──[không thỏa điều kiện nào]────────────────────────► TIMEOUT_RESET
                  │
    IMPACT_PENDING├──[|roll|>ROLL_FALL_THRESHOLD]────────► FALL_PENDING
         │        └──[hết T_IMPACT_CONFIRM, không có gì]─► STRONG_IMPACT
         │
         │        [timeout T_IMPACT_CONFIRM+100ms]────────► TIMEOUT_RESET
         │
    FALL_PENDING
         │ [sustained |roll|>ROLL_FALL_THRESHOLD ≥ T_FALL_CONFIRM]
         ▼
    FALL_DETECTED ──[debounce VÀ |roll|<ROLL_NORMAL_MAX]──► NORMAL
         │
         │ [|roll| về < ROLL_NORMAL_MAX trước T_FALL_CONFIRM]──► TIMEOUT_RESET
         │ [roll_rate < ROLL_CHANGE_RATE_SLOW VÀ |ω|<W_STATIC_MAX]──► TIMEOUT_RESET
         │ [timeout T_FALL_CONFIRM + T_FALL_MAX_WAIT = 5000ms]──► TIMEOUT_RESET
```

### 7.2 – Chi tiết từng trạng thái

---

#### ⚪ WARMUP (MỚI – thêm trong v2.0)

**Mục đích:** Chờ Complementary Filter của MPU6050_tockn ổn định góc, và chờ baseline EMA hội tụ về giá trị thực. Tránh báo giả ngay sau khi khởi động.

**Điều kiện vào:** Ngay sau `begin()`.

**Hành động trong state:**

- Cho phép `SignalProcessor` cập nhật baseline (warm-up mode, α=0.2)

- Không xử lý bất kỳ EVENT nào dù có spike

- Đếm thời gian từ lúc bắt đầu

**Điều kiện thoát sang NORMAL:**

- `signalProcessor.isBaselineReady() == true` (đủ `BASELINE_WARMUP_SAMPLES` mẫu)

- VÀ `millis() - warmup_start > WARMUP_DURATION_MS` (2000ms)

- Cả hai điều kiện phải thỏa đồng thời

---

#### 🟢 NORMAL

**Điều kiện duy trì:**

- `magnitude_a_raw` < `A_EVENT_THRESHOLD` (2.0g)

- `magnitude_omega_raw` < `W_EVENT_THRESHOLD` (200°/s)

- `|roll|` < 45° và `|pitch|` < 45°

- Không trong thời gian debounce

**Hành động trong state:**

- Cho phép `SignalProcessor` cập nhật baseline (EMA mode, α=0.05)

- Liên tục theo dõi roll, pitch

**Điều kiện chuyển sang `EVENT_DETECTED`:**

- `magnitude_a_raw` ≥ `A_EVENT_THRESHOLD` **HOẶC** `magnitude_omega_raw` ≥ `W_EVENT_THRESHOLD`

- VÀ `isInDebounce() == false`

- VÀ `processed.baseline_ready == true`

> **Lý do dùng raw để kích hoạt:** Raw phản ứng tức thì với spike. Smoothed có độ trễ ~50ms (5 mẫu × 10ms). Bỏ lỡ 50ms đầu của spike là bỏ lỡ thông tin định danh quan trọng nhất.

---

#### 🔵 EVENT_DETECTED

**Hành động ngay khi vào state:**

- Lưu `snapshot.event_start = millis()`

- Lưu `snapshot.roll_at_event = processed.roll`

- Lưu `snapshot.pitch_at_event = processed.pitch`

- Khóa baseline: `signalProcessor.setBaselineUpdateEnabled(false)`

- Khởi tạo `snapshot.peak_a = magnitude_a_raw`

- Khởi tạo `snapshot.peak_omega = magnitude_omega_raw`

- Khởi tạo `snapshot.spike_duration = 0`

- Lưu `state_enter_time = millis()`

**Hành động liên tục (tối đa `T_EVENT_WINDOW` = 300ms):**

- Cập nhật `snapshot.peak_a` nếu `magnitude_a_raw` lớn hơn hiện tại

- Cập nhật `snapshot.peak_omega` nếu `magnitude_omega_raw` lớn hơn

- Cộng dồn `snapshot.spike_duration` khi `magnitude_a_raw` còn vượt `A_EVENT_THRESHOLD`

- Cập nhật `snapshot.roll_delta = |processed.roll - snapshot.roll_at_event|`

- Cập nhật `snapshot.pitch_delta = |processed.pitch - snapshot.pitch_at_event|`

**Điều kiện chuyển sang `CLASSIFYING`:**

- `millis() - snapshot.event_start` ≥ `T_EVENT_WINDOW` (300ms)

**Điều kiện chuyển sang `TIMEOUT_RESET` sớm:**

- `magnitude_a_raw` và `magnitude_omega_raw` cùng trở về dưới ngưỡng

- VÀ `snapshot.spike_duration` < `T_SPIKE_MIN` (10ms) – quá ngắn, là nhiễu điện

**Timeout toàn state:** `T_STATE_TIMEOUT` = 600ms – nếu quá 600ms mà chưa đủ T_EVENT_WINDOW (trường hợp bất thường) → TIMEOUT_RESET

---

#### 🟡 CLASSIFYING

Chạy **một lần duy nhất**. Không delay, không chờ. Đánh giá `EventSnapshot` và chuyển ngay.

**Logic phân loại theo thứ tự ưu tiên cố định:**

```text
// Ưu tiên 1: ROAD_SHOCK – phổ biến nhất, lọc trước
NẾU snapshot.spike_duration < T_SPIKE_MAX (80ms)
  VÀ snapshot.peak_a < A_SHOCK_MAX (3.0g)
  VÀ snapshot.roll_delta < ROLL_DELTA_MIN (15°)
  VÀ snapshot.pitch_delta < PITCH_DELTA_MIN (20°)
→ ROAD_SHOCK

// Ưu tiên 2: Va chạm mạnh + quay nhanh → nguy cơ ngã cao nhất
NẾU snapshot.peak_a ≥ A_IMPACT_THRESHOLD (4.0g)
  VÀ snapshot.peak_omega ≥ W_FALL_THRESHOLD (350°/s)
→ FALL_PENDING

// Ưu tiên 3: Va chạm mạnh đơn thuần
NẾU snapshot.peak_a ≥ A_IMPACT_THRESHOLD (4.0g)
→ IMPACT_PENDING

// Ưu tiên 4: Quay nhanh + tư thế thay đổi
NẾU snapshot.peak_omega ≥ W_FALL_THRESHOLD (350°/s)
  VÀ snapshot.roll_delta > ROLL_DELTA_MIN (15°)
→ FALL_PENDING

// Không đủ điều kiện
→ TIMEOUT_RESET
```

---

#### 🔴 IMPACT_PENDING

**Mục đích:** Chờ xác nhận va chạm thật, phân biệt với vượt vật cản lớn hay ổ gà sâu.

**Theo dõi trong `T_IMPACT_CONFIRM` = 500ms:**

- Nếu `|processed.roll|` > `ROLL_FALL_THRESHOLD` (60°) → chuyển sang `FALL_PENDING`

- Nếu hết 500ms, không có gì thêm → chuyển sang `STRONG_IMPACT`

**Timeout:** `T_IMPACT_CONFIRM` + 100ms = 600ms. Quá timeout → `TIMEOUT_RESET`.

---

#### 🟠 FALL_PENDING

**Mục đích:** Chờ xác nhận xe thật sự nằm xuống và giữ nguyên.

**Hành động liên tục:**

- Đếm `sustained_fall_duration` = thời gian `|processed.roll|` liên tục > `ROLL_FALL_THRESHOLD` (60°)

- Nếu `sustained_fall_duration` ≥ `T_FALL_CONFIRM` (2000ms) → `FALL_DETECTED`

**Reset sớm (chống báo giả – theo thứ tự kiểm tra):**

```text
// Kiểm tra 1: Xe đã được dựng lại
NẾU |processed.roll| < ROLL_NORMAL_MAX (40°):
    → TIMEOUT_RESET (người đã dựng xe)

// Kiểm tra 2: Đang dựng chân chống (không phải ngã thật)
NẾU processed.roll_rate < ROLL_CHANGE_RATE_SLOW (5 °/s)
  VÀ processed.magnitude_omega_raw < W_STATIC_MAX (30 °/s):
    // Góc tăng rất chậm và đều → dựng chân chống, không phải ngã
    → TIMEOUT_RESET

// Kiểm tra 3: Phân biệt Pitch – xe chúi đầu vs leo dốc đứng
NẾU |processed.pitch| > PITCH_FALL_THRESHOLD (70°):
    // Pitch quá lớn → xe chúi đầu nghiêm trọng, vẫn là FALL
    // Không reset, tiếp tục đếm sustained_fall_duration
```

**Timeout toàn state:** `T_STATE_MAX_TIMEOUT` = 5000ms. Quá 5s không kết luận → `TIMEOUT_RESET`.

---

#### 🚨 STRONG_IMPACT

**Hành động ngay khi vào:**

- Set bit `ALERT_IMPACT_BIT` trong EventGroup

- Gửi `AlertInfo` vào `alertQueue`

- Bật `debounce_timer = millis()`

- Gọi `signalProcessor.resetFilter()`

- *(Baseline đã bị khóa từ EVENT_DETECTED – không cần khóa lại)*

**Điều kiện chuyển về `NORMAL`:**

- `millis() - debounce_timer` > `T_DEBOUNCE` (3000ms)

- VÀ `|processed.roll|` < `ROLL_NORMAL_MAX` (40°)

- Khi chuyển về NORMAL: gọi `signalProcessor.setBaselineUpdateEnabled(true)` để mở khóa baseline

---

#### 🆘 FALL_DETECTED

**Hành động ngay khi vào:**

- Set bit `ALERT_FALL_BIT` trong EventGroup

- Gửi `AlertInfo` vào `alertQueue` với `type = ALERT_FALL`

- Bật `debounce_timer = millis()`

- Gọi `signalProcessor.resetFilter()`

- *(Baseline đã bị khóa từ EVENT_DETECTED – không cần khóa lại)*

**Điều kiện chuyển về `NORMAL`:**

- `|processed.roll|` < `ROLL_NORMAL_MAX` (40°) – xe đã được dựng lại

- VÀ `millis() - debounce_timer` > `T_DEBOUNCE` (3000ms)

- Khi chuyển về NORMAL: gọi `signalProcessor.setBaselineUpdateEnabled(true)` để mở khóa baseline

> **Lý do yêu cầu cả 2 điều kiện:** Tránh reset khi xe vẫn đang nằm nguyên dù đã qua debounce.

---

#### ⚫ TIMEOUT_RESET

**Hành động (tức thì, không điều kiện):**

- Reset toàn bộ `EventSnapshot` về 0

- Gọi `signalProcessor.setBaselineUpdateEnabled(true)`

- Gọi `signalProcessor.resetFilter()`

- Chuyển ngay sang `NORMAL`

---

### 7.3 – Interface đầy đủ của AccidentDetector

```c
// Khởi tạo
begin()
    // Đặt state = WARMUP
    // Lưu warmup_start = millis()
    // Reset EventSnapshot

// Xử lý chính – gọi mỗi chu kỳ từ StateTask
process(ProcessedData) → void
    // Chạy state machine dựa trên state hiện tại
    // Cập nhật state_enter_time khi state thay đổi

// Getters – StateTask và WatchdogTask dùng
getState() → DetectorState
getStateEnterTime() → uint32_t      // Thời điểm vào state hiện tại
getLastAlert() → AlertInfo          // Alert vừa được tạo (gọi ngay sau khi state đổi)
hasNewAlert() → bool                // true nếu vừa có alert mới chưa được lấy

// Trạng thái hệ thống
isInDebounce() → bool               // true nếu đang trong T_DEBOUNCE sau cảnh báo
isWarmingUp() → bool                // true nếu state == WARMUP

// Điều khiển khẩn cấp – WatchdogTask gọi khi phát hiện kẹt
forceReset()
    // Reset state về NORMAL
    // Reset EventSnapshot
    // Gọi signalProcessor.setBaselineUpdateEnabled(true)
    // Gọi signalProcessor.resetFilter()
    // KHÔNG reset debounce_timer (nếu đang debounce thì giữ nguyên)
    // KHÔNG reset baseline của SignalProcessor
```

---

### 7.4 – Lớp Anti False-Positive (Tổng hợp)

Bảng tóm tắt tất cả cơ chế chống báo giả, ở đâu trong code và dựa vào threshold nào:

| Tình huống | Cơ chế lọc | Ở đâu | Threshold dùng |
| --- | --- | --- | --- |
| Nhiễu I2C cực đoan | Outlier Rejection | SignalProcessor Bước 0 | `OUTLIER_A_MAX`, `OUTLIER_W_MAX`, `OUTLIER_A_MIN` |
| Ổ gà / đường xấu | spike_duration ngắn + peak thấp | CLASSIFYING | `T_SPIKE_MAX`, `A_SHOCK_MAX`, `ROLL_DELTA_MIN` |
| Nhiễu điện ngắn | T_SPIKE_MIN | EVENT_DETECTED | `T_SPIKE_MIN` |
| Phanh gấp | roll_delta nhỏ + peak_omega thấp | CLASSIFYING | `ROLL_DELTA_MIN`, `W_FALL_THRESHOLD` |
| Vào cua gấp | roll tăng từ từ, không có spike trước | CLASSIFYING + FALL_PENDING | `roll_delta`, `roll_rate` |
| Dựng chân chống | roll_rate thấp + \|ω\| thấp | FALL_PENDING | `ROLL_CHANGE_RATE_SLOW`, `W_STATIC_MAX` |
| Người dựng xe ngay | roll về < 40° trong 2s | FALL_PENDING | `ROLL_NORMAL_MAX`, `T_FALL_CONFIRM` |
| Báo lại sau tai nạn | Debounce timer | STRONG_IMPACT / FALL_DETECTED | `T_DEBOUNCE` |
| Báo giả lúc khởi động | WARMUP state | AccidentDetector | `WARMUP_DURATION_MS`, `BASELINE_WARMUP_SAMPLES` |
| Hysteresis góc | Ngưỡng vào ≠ ngưỡng ra | FALL_PENDING / FALL_DETECTED | Vào: `ROLL_FALL_THRESHOLD` (60°), Ra: `ROLL_NORMAL_MAX` (40°) |

---

## VIII. TẦNG 4 – FREERTOS TASK DESIGN

### 8.1 – Tổng quan phân bổ Task

```text
┌────────────────────────────────────────────────────────────────────┐
│                       ESP32 – FreeRTOS                             │
│                                                                    │
│       Core 1 (Application CPU)       Core 0 (Protocol CPU)         │
│  ┌──────────────────────────────┐   ┌────────────────────────────┐ │
│  │ SensorTask                   │   │ OutputTask                 │ │
│  │ Priority: 3 | Stack: 4096B   │   │ Priority: 1 | Stack: 3072B │ │
│  │ vTaskDelayUntil 10ms         │   │ Event-driven (EventGroup)  │ │
│  ├──────────────────────────────┤   ├────────────────────────────┤ │
│  │ StateTask                    │   │ WatchdogTask               │ │
│  │ Priority: 3 | Stack: 4096B   │   │ Priority: 2 | Stack: 2048B │ │
│  │ Block chờ sensorQueue        │   │ vTaskDelay 1000ms          │ │
│  └──────────────────────────────┘   └────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

### 8.2 – SensorTask (Core 1, Priority 3)

**Trách nhiệm:** Đọc MPU6050, pre-process, đẩy `ProcessedData` vào Queue đúng chu kỳ 100Hz.

**Luồng xử lý:**

```c
[Khởi động]
    xLastWakeTime = xTaskGetTickCount()
    error_count = 0

[Vòng lặp]:
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS))

    raw = sensorReader.read()

    // Cập nhật health (volatile – không cần mutex)
    health.last_sensor_tick = millis()
    health.sensor_task_alive = true

    NẾU raw.valid == false:
        health.sensor_error_count++
        NẾU health.sensor_error_count > SENSOR_MAX_ERROR_COUNT:
            // Log một lần duy nhất, không spam
            Serial.println("WARNING: Sensor errors > threshold")
        TIẾP TỤC vòng lặp (bỏ qua mẫu lỗi)
    KHÔNG THÌ:
        health.sensor_error_count = 0  // reset counter khi có mẫu tốt

    processed = signalProcessor.process(raw)

    result = xQueueSend(sensorQueue, &processed, 0)
    NẾU result == errQUEUE_FULL:
        // Không dùng mutex – chỉ increment volatile counter
        health.queue_overflow_count++
```

> **Tại sao `vTaskDelayUntil()` thay vì `vTaskDelay()`:** `vTaskDelay(10)` delay 10ms kể từ lúc kết thúc xử lý → chu kỳ thực = 10ms + thời gian xử lý (trôi dần theo thời gian). `vTaskDelayUntil()` delay đến mốc thời gian tuyệt đối → chu kỳ luôn chính xác 10ms bất kể thời gian xử lý.
-.-
> **Tại sao Queue timeout = 0:** SensorTask không được phép bị block bởi Queue. Thà bỏ 1 mẫu còn hơn làm lệch chu kỳ 100Hz.

---

### 8.3 – StateTask (Core 1, Priority 3)

**Trách nhiệm:** Nhận `ProcessedData` từ Queue, điều phối `SignalProcessor` và `AccidentDetector`, ra quyết định cảnh báo.

**Luồng xử lý:**

```c
[Vòng lặp]:
    xQueueReceive(sensorQueue, &data, portMAX_DELAY)
    // Block vô thời hạn chờ dữ liệu – không tốn CPU khi chờ

    // Điều phối baseline update
    signalProcessor.setBaselineUpdateEnabled(
        accidentDetector.getState() == NORMAL ||
        accidentDetector.getState() == WARMUP
    )

    // Chạy State Machine
    accidentDetector.process(data)

    // Kiểm tra alert mới
    NẾU accidentDetector.hasNewAlert():
        alert = accidentDetector.getLastAlert()
        xQueueSend(alertQueue, &alert, 0)

        NẾU alert.type == ALERT_FALL:
            xEventGroupSetBits(alertGroup, ALERT_FALL_BIT)
        NẾU alert.type == ALERT_IMPACT:
            xEventGroupSetBits(alertGroup, ALERT_IMPACT_BIT)

    // Cập nhật health (cần mutex vì WatchdogTask đọc đồng thời)
    xSemaphoreTake(stateMutex, portMAX_DELAY)
    health.last_state_tick = millis()
    health.state_task_alive = true
    health.current_state = accidentDetector.getState()
    health.state_enter_time = accidentDetector.getStateEnterTime()
    xSemaphoreGive(stateMutex)
```

---

### 8.4 – OutputTask (Core 0, Priority 1)

**Trách nhiệm:** Xử lý toàn bộ đầu ra khi có cảnh báo. Không ảnh hưởng real-time Core 1.

**Luồng xử lý:**

```c
[Vòng lặp]:
    bits = xEventGroupWaitBits(
        alertGroup,
        ALERT_FALL_BIT | ALERT_IMPACT_BIT,
        pdTRUE,          // Tự clear bit sau khi nhận
        pdFALSE,         // Chờ bất kỳ bit nào (OR logic)
        portMAX_DELAY
    )

    xQueueReceive(alertQueue, &alert, pdMS_TO_TICKS(10))

    NẾU bits & ALERT_FALL_BIT:
        buzzer.alarmContinuous()
        led.setRed()
        Serial.printf("[FALL] t=%u peak_a=%.2f peak_w=%.2f roll=%.1f pitch=%.1f\n",
                       alert.timestamp, alert.peak_a, alert.peak_omega,
                       alert.roll, alert.pitch)

    NẾU bits & ALERT_IMPACT_BIT:
        buzzer.beep(3)
        led.setYellow()
        Serial.printf("[IMPACT] t=%u peak_a=%.2f peak_w=%.2f roll=%.1f pitch=%.1f\n",
                       alert.timestamp, alert.peak_a, alert.peak_omega,
                       alert.roll, alert.pitch)
```

---

### 8.5 – WatchdogTask (Core 0, Priority 2)

**Trách nhiệm:** Giám sát sức khỏe hệ thống, phát hiện task bị treo, force reset State Machine khi kẹt.

**Luồng xử lý:**

```c
[Vòng lặp]:
    vTaskDelay(pdMS_TO_TICKS(1000))

    // Đọc health fields của StateTask (cần mutex)
    xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100))  // timeout 100ms, không chờ lâu
    snapshot_state_tick = health.last_state_tick
    snapshot_state      = health.current_state
    snapshot_enter_time = health.state_enter_time
    xSemaphoreGive(stateMutex)

    // Đọc health fields của SensorTask (volatile – không cần mutex)
    snapshot_sensor_tick  = health.last_sensor_tick
    snapshot_alive        = health.sensor_task_alive
    snapshot_error_count  = health.sensor_error_count
    snapshot_q_overflow   = health.queue_overflow_count

    // === Kiểm tra SensorTask ===
    NẾU millis() - snapshot_sensor_tick > SENSOR_DEAD_TIMEOUT_MS (500):
        Serial.println("ERROR: SensorTask không phản hồi > 500ms")

    // === Kiểm tra StateTask ===
    NẾU millis() - snapshot_state_tick > 1000:
        Serial.println("ERROR: StateTask không phản hồi > 1000ms")

    // === Kiểm tra State Machine kẹt ===
    is_intermediate_state = (snapshot_state != NORMAL &&
                             snapshot_state != WARMUP &&
                             snapshot_state != STRONG_IMPACT &&
                             snapshot_state != FALL_DETECTED &&
                             snapshot_state != ROAD_SHOCK)

    NẾU is_intermediate_state:
        time_in_state = millis() - snapshot_enter_time
        NẾU time_in_state > T_STATE_MAX_TIMEOUT (5000):
            Serial.printf("ERROR: State %d kẹt %ums – force reset\n",
                          snapshot_state, time_in_state)
            xSemaphoreTake(stateMutex, portMAX_DELAY)
            accidentDetector.forceReset()
            xSemaphoreGive(stateMutex)

    // === Stack health monitor (debug mode) ===
    // Chỉ bật trong giai đoạn debug
    #ifdef DEBUG_STACK
    Serial.printf("Stack free – Sensor:%u State:%u Output:%u Watchdog:%u\n",
        uxTaskGetStackHighWaterMark(sensorTaskHandle),
        uxTaskGetStackHighWaterMark(stateTaskHandle),
        uxTaskGetStackHighWaterMark(outputTaskHandle),
        uxTaskGetStackHighWaterMark(NULL))
    #endif

    // === Queue overflow monitor ===
    NẾU snapshot_q_overflow > 0:
        Serial.printf("WARNING: sensorQueue overflow %u lần\n", snapshot_q_overflow)
        health.queue_overflow_count = 0  // reset (volatile, chấp nhận race condition nhỏ)

    // === Feed hardware watchdog ===
    esp_task_wdt_reset()
```

---

## IX. CƠ CHẾ ĐỒNG BỘ HÓA

### 9.1 – Bảng tổng hợp tài nguyên dùng chung

| Tài nguyên | Loại | Size | Producer | Consumer | Bảo vệ bởi |
| --- | --- | --- | --- | --- | --- |
| `sensorQueue` | Queue | 5 phần tử | SensorTask | StateTask | Nội bộ Queue |
| `alertQueue` | Queue | 3 phần tử | StateTask | OutputTask | Nội bộ Queue |
| `alertGroup` | EventGroup | 2 bit | StateTask | OutputTask | Nội bộ EventGroup |
| `stateMutex` | Mutex | – | StateTask | WatchdogTask | stateMutex |
| `SystemHealth` (field SensorTask) | volatile struct | – | SensorTask | WatchdogTask | volatile keyword |
| `SystemHealth` (field StateTask) | struct | – | StateTask | WatchdogTask | stateMutex |

### 9.2 – Quy tắc ownership SystemHealth (làm rõ trong v2.0)

```c
SensorTask ghi:
    health.last_sensor_tick     → volatile uint32_t, single writer, không cần mutex
    health.sensor_task_alive    → volatile bool, single writer
    health.sensor_error_count   → volatile uint16_t, single writer
    health.queue_overflow_count → volatile uint16_t, SensorTask ghi, WatchdogTask reset
    // Chỉ cần volatile để tránh compiler optimization
    // Không cần mutex vì chỉ có 1 writer duy nhất

StateTask ghi (cần mutex vì WatchdogTask đọc đồng thời):
    health.last_state_tick      → bảo vệ bởi stateMutex
    health.state_task_alive     → bảo vệ bởi stateMutex
    health.current_state        → bảo vệ bởi stateMutex
    health.state_enter_time     → bảo vệ bởi stateMutex

WatchdogTask:
    Chỉ ĐỌC health, không ghi
    Ngoại lệ: reset health.queue_overflow_count = 0 (chấp nhận race condition nhỏ, không critical)
```

### 9.3 – Quy tắc sử dụng Mutex

- Giữ Mutex càng ngắn càng tốt – chỉ để copy struct, không làm gì phức tạp bên trong
- Không gọi I2C, Serial, hoặc bất kỳ blocking call nào khi đang giữ Mutex
- Dùng `xSemaphoreCreateMutex()` – có priority inheritance để tránh priority inversion
- WatchdogTask dùng timeout 100ms khi lấy Mutex – không chờ vô hạn

### 9.4 – Tại sao dùng cả Queue lẫn EventGroup cho cảnh báo

- **EventGroup** → OutputTask biết **ngay lập tức** có cảnh báo, không phải polling
- **Queue** → OutputTask lấy được **thông tin chi tiết** (peak_a, timestamp...) để log
- Hai cơ chế bổ sung cho nhau

---

## X. BẢNG NGƯỠNG THAM SỐ (config.h)

Toàn bộ tham số tập trung trong một file `config.h`. Khi test thực tế chỉ cần chỉnh 1 file duy nhất.

### Ngưỡng gia tốc

> **Mức tham chiếu (không phải hằng số, chỉ để hiểu ngữ cảnh):**  
> Bình thường khi lái: `|a_dynamic|` < 1.5g — Ổ gà / đường xấu: 1.5–2.5g (thời gian ngắn, không báo)

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `A_NORMAL_MAX` | 1.5 | g | Mức gia tốc bình thường khi lái xe thông thường |
| `A_EVENT_THRESHOLD` | 2.0 | g | Kích hoạt EVENT_DETECTED |
| `A_SHOCK_MAX` | 3.0 | g | Giới hạn trên phân loại ROAD_SHOCK |
| `A_IMPACT_THRESHOLD` | 4.0 | g | Ngưỡng xác định STRONG_IMPACT |
| `BASELINE_INITIAL` | 1.0 | g | **[MỚI]** Giá trị baseline khởi tạo = 1g (trọng lực) |
| `BASELINE_ALPHA` | 0.05 | – | Hệ số EMA cập nhật baseline (chế độ bình thường) |
| `BASELINE_ALPHA_WARMUP` | 0.20 | – | **[MỚI]** Hệ số EMA trong warm-up (hội tụ nhanh hơn) |
| `MA_FILTER_SIZE` | 5 | mẫu | Số mẫu moving average |

### Ngưỡng tốc độ góc

> **Mức tham chiếu:** Lái bình thường: `|ω|` < 100°/s — Lắc mạnh: 100–300°/s

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `W_NORMAL_MAX` | 100 | °/s | Tốc độ góc tối đa khi lái xe bình thường |
| `W_EVENT_THRESHOLD` | 200 | °/s | Kích hoạt EVENT_DETECTED |
| `W_FALL_THRESHOLD` | 350 | °/s | Ngưỡng phân loại FALL trong CLASSIFYING |
| `W_STATIC_MAX` | 30 | °/s | **[MỚI]** Ngưỡng "đứng yên" để lọc dựng chân chống |

### Ngưỡng góc nghiêng

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `ROLL_FALL_THRESHOLD` | 60 | ° | Xe đang nằm – ngưỡng VÀO FALL_PENDING |
| `PITCH_FALL_THRESHOLD` | 70 | ° | Xe chúi đầu |
| `ROLL_NORMAL_MAX` | 40 | ° | Ngưỡng RA khỏi FALL (hysteresis: 60° vào, 40° ra) |
| `ROLL_TURN_MAX` | 45 | ° | Roll tối đa khi cua bình thường |
| `ROLL_DELTA_MIN` | 15 | ° | Thay đổi roll tối thiểu để không phải ROAD_SHOCK |
| `PITCH_DELTA_MIN` | 20 | ° | **[MỚI]** Thay đổi pitch tối thiểu để không phải ROAD_SHOCK |

### Ngưỡng tốc độ thay đổi góc (MỚI trong v2.0)

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `ROLL_CHANGE_RATE_SLOW` | 5 | °/s | Dưới ngưỡng này → dựng chân chống, không phải ngã |
| `ROLL_CHANGE_RATE_FAST` | 30 | °/s | Trên ngưỡng này → xe đang ngã thật |

### Ngưỡng thời gian

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `T_SPIKE_MIN` | 10 | ms | **[MỚI]** Spike ngắn hơn → nhiễu điện, bỏ qua |
| `T_SPIKE_MAX` | 80 | ms | Spike ngắn hơn ngưỡng này → ROAD_SHOCK |
| `T_EVENT_WINDOW` | 300 | ms | Cửa sổ thu thập dữ liệu sự kiện |
| `T_IMPACT_CONFIRM` | 500 | ms | Thời gian chờ xác nhận STRONG_IMPACT |
| `T_FALL_CONFIRM` | 2000 | ms | Thời gian giữ góc để xác nhận FALL_DETECTED |
| `T_STATE_TIMEOUT` | 600 | ms | Timeout riêng cho EVENT_DETECTED |
| `T_STATE_MAX_TIMEOUT` | 5000 | ms | Timeout tổng – WatchdogTask force reset |
| `T_DEBOUNCE` | 3000 | ms | Chặn sự kiện lặp sau khi đã phát cảnh báo |
| `SENSOR_PERIOD_MS` | 10 | ms | Chu kỳ đọc sensor (100Hz) |
| `WARMUP_DURATION_MS` | 2000 | ms | **[MỚI]** Thời gian bắt buộc giữ WARMUP state |

### Ngưỡng khởi tạo & sensor health (MỚI trong v2.0)

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `BASELINE_WARMUP_SAMPLES` | 50 | mẫu | Số mẫu để baseline hội tụ (50 × 10ms = 0.5s) |
| `SENSOR_MAX_ERROR_COUNT` | 10 | lần | Số lần đọc lỗi liên tiếp trước khi log WARNING |
| `SENSOR_DEAD_TIMEOUT_MS` | 500 | ms | Không có mẫu hợp lệ trong thời gian này → sensor dead |
| `OUTLIER_A_MAX` | 4.5 | g | **[MỚI]** Gia tốc vượt ngưỡng → reject là nhiễu I2C |
| `OUTLIER_A_MIN` | 0.3 | g | **[MỚI]** Gia tốc dưới ngưỡng → reject (I2C trả về 0) |
| `OUTLIER_W_MAX` | 550 | °/s | **[MỚI]** Tốc độ góc vượt range vật lý → reject |

### Ngưỡng FreeRTOS

| Tên hằng số | Giá trị | Đơn vị | Ý nghĩa |
| --- | --- | --- | --- |
| `SENSOR_TASK_STACK` | 4096 | bytes | Stack cho SensorTask |
| `STATE_TASK_STACK` | 4096 | bytes | Stack cho StateTask |
| `OUTPUT_TASK_STACK` | 3072 | bytes | Stack cho OutputTask |
| `WATCHDOG_TASK_STACK` | 2048 | bytes | Stack cho WatchdogTask |
| `SENSOR_TASK_PRIORITY` | 3 | – | Priority SensorTask |
| `STATE_TASK_PRIORITY` | 3 | – | Priority StateTask |
| `WATCHDOG_TASK_PRIORITY` | 2 | – | Priority WatchdogTask |
| `OUTPUT_TASK_PRIORITY` | 1 | – | Priority OutputTask |
| `SENSOR_QUEUE_SIZE` | 5 | phần tử | Số phần tử tối đa trong sensorQueue |
| `ALERT_QUEUE_SIZE` | 3 | phần tử | Số phần tử tối đa trong alertQueue |
| `SENSOR_TASK_CORE` | 1 | – | Core cho SensorTask |
| `STATE_TASK_CORE` | 1 | – | Core cho StateTask |
| `OUTPUT_TASK_CORE` | 0 | – | Core cho OutputTask |
| `WATCHDOG_TASK_CORE` | 0 | – | Core cho WatchdogTask |

---

## XI. SƠ ĐỒ LUỒNG TỔNG THỂ

### Luồng khởi động (Boot)

```c
setup() {
    Serial.begin(115200)
    Wire.begin()

    sensorReader.begin(Wire, true)      // Init MPU6050, tính gyro offset
    signalProcessor.begin()             // Init buffer, baseline = 1.0g
    accidentDetector.begin()            // Init State Machine → WARMUP

    sensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(ProcessedData))
    alertQueue  = xQueueCreate(ALERT_QUEUE_SIZE,  sizeof(AlertInfo))
    alertGroup  = xEventGroupCreate()
    stateMutex  = xSemaphoreCreateMutex()

    xTaskCreatePinnedToCore(SensorTask,   "Sensor",   SENSOR_TASK_STACK,   NULL, SENSOR_TASK_PRIORITY,   &sensorTaskHandle,   SENSOR_TASK_CORE)
    xTaskCreatePinnedToCore(StateTask,    "State",    STATE_TASK_STACK,    NULL, STATE_TASK_PRIORITY,    &stateTaskHandle,    STATE_TASK_CORE)
    xTaskCreatePinnedToCore(WatchdogTask, "Watchdog", WATCHDOG_TASK_STACK, NULL, WATCHDOG_TASK_PRIORITY, &watchdogTaskHandle, WATCHDOG_TASK_CORE)
    xTaskCreatePinnedToCore(OutputTask,   "Output",   OUTPUT_TASK_STACK,   NULL, OUTPUT_TASK_PRIORITY,   &outputTaskHandle,   OUTPUT_TASK_CORE)
}

loop() {
    vTaskDelay(portMAX_DELAY)   // Nhường hoàn toàn cho FreeRTOS scheduler
}
```

### Luồng runtime

```text
──────────── Mỗi 10ms (SensorTask, Core 1) ────────────
    vTaskDelayUntil(10ms)
    raw      ← sensorReader.read()
    processed ← signalProcessor.process(raw)
              [Outlier → |a|,|ω| → dynamic → filter → roll_rate → baseline]
    sensorQueue ← processed
                    │
                    ▼
──────────── Khi Queue có dữ liệu (StateTask, Core 1) ────────────
    processed ← sensorQueue
    signalProcessor.setBaselineUpdateEnabled(state == NORMAL || WARMUP)
    accidentDetector.process(processed)
    [WARMUP → NORMAL → EVENT_DETECTED → CLASSIFYING → ... → cảnh báo]
    NẾU có alert mới:
        alertQueue ← AlertInfo
        alertGroup ← set bit
                        │
                        ▼
──────────── Khi EventGroup được set (OutputTask, Core 0) ────────────
    alert ← alertQueue
    Kích hoạt buzzer / LED / Serial log

──────────── Mỗi 1000ms (WatchdogTask, Core 0) ────────────
    Đọc health (volatile + mutex)
    Kiểm tra sensor còn sống
    Kiểm tra state không bị kẹt
    Force reset nếu kẹt quá T_STATE_MAX_TIMEOUT
    esp_task_wdt_reset()
```

---

## XII. NHỮNG QUYẾT ĐỊNH THIẾT KẾ QUAN TRỌNG

**1. Baseline chỉ cập nhật khi NORMAL hoặc WARMUP.**  
Khi đang trong sự kiện, baseline bị khóa. Nếu không khóa, spike gia tốc sẽ trở thành "bình thường mới" và các sự kiện tiếp theo bị bỏ lỡ.

**2. Kích hoạt EVENT bằng raw, so sánh phân loại bằng peak (cũng là raw đỉnh).**  
Raw phản ứng tức thì. Smoothed có độ trễ 50ms. Bỏ lỡ 50ms đầu spike = bỏ lỡ thông tin định danh quan trọng nhất.

**3. Pipeline xử lý tín hiệu có thứ tự cố định và không đảo được.**  
Outlier → Magnitude → Dynamic → Filter → roll_rate → Baseline. Đảo bất kỳ bước nào sẽ cho kết quả sai.

**4. CLASSIFYING chạy một lần duy nhất, không có timeout.**  
Toàn bộ thông tin đã được thu thập trong EVENT_DETECTED. CLASSIFYING chỉ ra quyết định, không chờ.

**5. FALL luôn ưu tiên hơn IMPACT khi cả hai điều kiện thỏa.**  
Té ngã nguy hiểm hơn, cần cảnh báo mạnh hơn. Va chạm mạnh + quay nhanh = khả năng cao đã đổ xe.

**6. ROAD_SHOCK được lọc đầu tiên trong CLASSIFYING.**  
Phổ biến nhất trên đường thực tế. Lọc sớm tránh xử lý không cần thiết.

**7. Hysteresis trên ngưỡng góc là cố ý, không phải bug.**  
Vào FALL khi `roll > 60°`, thoát khi `roll < 40°`. Khoảng 20° hysteresis tránh oscillation ở biên ngưỡng.

**8. Debounce 3 giây sau mỗi cảnh báo.**  
Tránh spam khi xe đang lăn sau tai nạn. 3 giây đủ để phân biệt "cùng 1 sự kiện" vs "sự kiện mới".

**9. SensorTask và StateTask ở Core 1, OutputTask và WatchdogTask ở Core 0.**  
Core 1 dành hoàn toàn cho real-time. Core 0 cho I/O và WiFi sau này.

**10. SystemHealth dùng volatile cho SensorTask fields, Mutex cho StateTask fields.**  
SensorTask có single writer → volatile đủ. StateTask fields có WatchdogTask đọc đồng thời → cần Mutex. Không dùng Mutex cho volatile fields để tránh SensorTask bị block.

---

## XIII. RỦI RO & BIỆN PHÁP XỬ LÝ

| Rủi ro | Mức độ | Biện pháp xử lý |
| --- | --- | --- |
| Stack overflow task | **Cao** | Bật `configCHECK_FOR_STACK_OVERFLOW=2` khi debug; `uxTaskGetStackHighWaterMark()` trong WatchdogTask (bật bằng `#define DEBUG_STACK`) |
| Baseline khởi động sai | **Cao** | WARMUP state bắt buộc 2s; `BASELINE_INITIAL=1.0g`; EMA nhanh α=0.2 trong warm-up |
| State Machine bị kẹt | **Trung bình** | WatchdogTask force reset sau `T_STATE_MAX_TIMEOUT` (5000ms) |
| Nhiễu I2C cực đoan | **Trung bình** | Outlier Rejection trong SignalProcessor Bước 0 |
| Queue overflow liên tục | **Trung bình** | Monitor `queue_overflow_count`; xem xét tăng `STATE_TASK_STACK` nếu StateTask xử lý chậm |
| MPU6050 mất kết nối | **Trung bình** | `valid=false` → SensorTask đếm error; WatchdogTask phát hiện sau `SENSOR_DEAD_TIMEOUT_MS` |
| Báo giả lúc khởi động | **Trung bình** | WARMUP state chặn toàn bộ EVENT detection |
| Priority inversion | **Thấp** | `xSemaphoreCreateMutex()` – có priority inheritance |
| Floating point đồng thời | **Thấp** | ESP32 FPU riêng mỗi core; SensorTask + StateTask cùng Core 1 → không conflict |
| Báo giả khi phanh gấp | **Thấp** | `roll_delta` nhỏ + `peak_omega` thấp → ROAD_SHOCK trong CLASSIFYING |
| Báo giả khi cua gấp | **Thấp** | `roll_rate` kiểm tra trong FALL_PENDING; `roll_delta` trong CLASSIFYING |
| Baseline lệch dài hạn | **Thấp** | α=0.05 cập nhật chậm; chỉ cập nhật khi NORMAL; forceReset() không reset baseline |

---

## XIV. DANH SÁCH FILE & THỨ TỰ IMPLEMENT

### Cấu trúc project

```text
AccidentDetection/
│
├── config.h                    // Tất cả hằng số và ngưỡng
│                               // (chỉnh tại đây khi test thực tế)
│
├── DataTypes.h                 // Tất cả struct và enum:
│                               //   RawSensorData, ProcessedData,
│                               //   EventSnapshot, AlertInfo,
│                               //   SystemHealth, DetectorState, AlertType
│
├── SensorReader.h              // Interface tầng cảm biến
├── SensorReader.cpp            // Wrapper MPU6050_tockn
│                               // Không xử lý gì thêm
│
├── SignalProcessor.h           // Interface tầng xử lý tín hiệu
├── SignalProcessor.cpp         // Pipeline 6 bước:
│                               //   Outlier → |a|,|ω| → Dynamic
│                               //   → Filter → roll_rate → Baseline
│
├── AccidentDetector.h          // Interface State Machine
├── AccidentDetector.cpp        // 10 states + Classification + Anti-FP
│
├── TaskManager.h               // Khai báo 4 task functions + handles
├── TaskManager.cpp             // 4 FreeRTOS Task + Queue/EventGroup/Mutex
│
└── main.ino                    // setup(): init + tạo task
                                // loop(): vTaskDelay(portMAX_DELAY)
```

### Thứ tự implement đề xuất

| Bước | File | Mục tiêu kiểm tra |
| --- | --- | --- |
| 1 | `DataTypes.h` | Compile không lỗi, tất cả struct có đủ field |
| 2 | `config.h` | Tất cả threshold có tên hằng số, không có magic number |
| 3 | `SensorReader` | Serial print raw data, kiểm tra roll/pitch ổn định |
| 4 | `SignalProcessor` | Serial print magnitude_a, baseline, kiểm tra warm-up |
| 5 | `AccidentDetector` | Test với dữ liệu mock, kiểm tra từng state transition |
| 6 | `TaskManager` | Ghép FreeRTOS, kiểm tra chu kỳ 10ms ổn định |
| 7 | `main.ino` | Test tích hợp đầu cuối, kiểm tra không có delay() |
| 8 | Tinh chỉnh | Chỉnh threshold trong `config.h` dựa trên log thực tế |

### Checklist trước khi code

- [ ] Tất cả threshold đã có trong bảng Section X
- [ ] Tất cả state transition đã có timeout
- [ ] Pipeline SignalProcessor đã đúng thứ tự (Section VI)
- [ ] Interface AccidentDetector đã đủ method (Section VII.3)
- [ ] Ownership SystemHealth đã rõ ràng (Section IX.2)
- [ ] Hysteresis góc được hiểu là cố ý (60° vào, 40° ra)
- [ ] WARMUP state được implement trước NORMAL

---

*Bản thiết kế v2.0 – Đã bổ sung đầy đủ: threshold còn thiếu, pipeline xử lý tín hiệu chi tiết với thứ tự bắt buộc, outlier rejection, warm-up period, roll_rate, interface đầy đủ cho AccidentDetector, làm rõ ownership SystemHealth, bảng tổng hợp Anti False-Positive, và checklist trước khi code.*
