# Phân tích cảm biến MPU6050

[Xem datasheet MPU6050 tại đây](../../Datasheet/MPU-6000.pdf)

## Quy trình code

- Reste thanh ghi 6B về 0

- Đọc dữ liệu 6 thanh ghi từ 3B - 40 lấy dữ liệu gia tốc

- Đọc dữ liệu 2 thanh ghi từ 41 - 42 để lấy dữ liệu nhiệt độ

- Đọc dữ liệu 6 thanh ghi từ 43 - 48 lấy dữ liệu góc xoay (vận tốc góc)

- Ghi 3 thanh ghi 1A 1B 1C:

  - 1A để set lọc tần số thấp

  - 1B để set gyro_config

  - 1C để set gia tốc góc

## Wiring

| MPU6050 | ESP32 |
| --- | --- |
| VCC | 3.3V / 5V |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

---

## Phát triển code cho MPU6050 dùng thư viện `MPU6050_tockn`

### Phân tích thư viện `MPU6050_tockn`

#### Tổng quan

Thư viện này là wrapper cho cảm biến MPU6050 (gia tốc kế + con quay hồi chuyển 6 trục) trên Arduino, giao tiếp qua I2C (Wire). Thư viện cung cấp đọc dữ liệu thô, tính góc từ accelerometer, tích phân góc từ gyroscope, và kết hợp 2 nguồn bằng Complementary Filter.

#### File Header – MPU6050_tockn.h

##### Định nghĩa địa chỉ & thanh ghi

```cpp
#define MPU6050_ADDR         0x68   // Địa chỉ I2C mặc định (AD0 = LOW)
#define MPU6050_SMPLRT_DIV   0x19   // Sample Rate Divider
#define MPU6050_CONFIG       0x1a   // Cấu hình DLPF (lọc nhiễu)
#define MPU6050_GYRO_CONFIG  0x1b   // Cấu hình độ nhạy gyro
#define MPU6050_ACCEL_CONFIG 0x1c   // Cấu hình độ nhạy accel
#define MPU6050_PWR_MGMT_1   0x6b   // Quản lý nguồn
```

##### Thuộc tính private

| Thuộc tính | Kiểu | Ý nghĩa |
| --- | --- | --- |
| wire | TwoWire* | Con trỏ đến bus I2C |
| rawAccX/Y/Z | int16_t | Giá trị thô accelerometer |
| rawGyroX/Y/Z | int16_t | Giá trị thô gyroscope |
| rawTemp | int16_t | Giá trị thô nhiệt độ |
| gyroXoffset/Yoffset/Zoffset | float | Offset bù sai số gyro |
| accX/Y/Z | float | Gia tốc đã quy đổi (đơn vị g) |
| gyroX/Y/Z | float | Tốc độ góc đã quy đổi (°/s) |
| angleAccX/Y | float | Góc tính từ accelerometer (°) |
| angleGyroX/Y/Z | float | Góc tích phân từ gyroscope (°) |
| angleX/Y/Z | float | Góc cuối sau Complementary Filter |
| interval | float | Khoảng thời gian giữa 2 lần update (giây) |
| preInterval | long | Thời điểm update trước (ms) |
| accCoef / gyroCoef | float | Hệ số filter (mặc định 0.02 / 0.98) |

#### File Source – MPU6050_tockn.cpp

##### 1. Constructors

```cpp
MPU6050(TwoWire &w)
// Khởi tạo với hệ số mặc định: accCoef = 0.02, gyroCoef = 0.98

MPU6050(TwoWire &w, float aC, float gC)
// Khởi tạo với hệ số tùy chỉnh
// aC + gC nên = 1.0 để filter hoạt động đúng
```

##### Ví dụ sử dụng

```cpp
MPU6050 mpu(Wire);               // dùng hệ số mặc định
MPU6050 mpu(Wire, 0.05, 0.95);   // tin gyro nhiều hơn
```

##### 2. `begin()`

```cpp
void MPU6050::begin()
```

Cấu hình các thanh ghi khởi động:

| Thanh ghi | Giá trị | Ý nghĩa |
| --- | --- | --- |
| SMPLRT_DIV | 0x00 | Sample rate = Gyro rate / 1 (tối đa) |
| CONFIG | 0x00 | Tắt DLPF |
| GYRO_CONFIG | 0x08 | Full scale ±500°/s → LSB = 65.5 |
| ACCEL_CONFIG | 0x00 | Full scale ±2g → LSB = 16384 |
| PWR_MGMT_1 | 0x01 | Dùng PLL từ X-gyro làm clock |

Sau đó gọi `update()` một lần, đặt `angleGyroX/Y = 0`, lấy góc accelerometer làm góc ban đầu, lưu timestamp.

##### 3. `writeMPU6050()` / `readMPU6050()`

```cpp
void writeMPU6050(byte reg, byte data)  // Ghi 1 byte vào thanh ghi
byte readMPU6050(byte reg)              // Đọc 1 byte từ thanh ghi
```

Giao tiếp I2C cơ bản, dùng nội bộ hoặc khi cần đọc/ghi thanh ghi tùy chỉnh.

##### 4. `setGyroOffsets()` và `calcGyroOffsets()`

```cpp
void setGyroOffsets(float x, float y, float z)
// Đặt thủ công offset cho gyro (đã biết trước)
```

```cpp
void calcGyroOffsets(bool console = false,
                     uint16_t delayBefore = 1000,
                     uint16_t delayAfter = 3000)
// Tự động đo offset bằng cách lấy trung bình 3000 mẫu
// ⚠️ Phải để sensor hoàn toàn bất động khi gọi hàm này
// console = true → in kết quả ra Serial
```

**Cách hoạt động:** Đọc 3000 mẫu gyro thô → tính trung bình → lưu vào `gyroXoffset/Y/Z`. Các lần `update()` sau sẽ trừ đi offset này.

##### 5. `update()` – Hàm quan trọng nhất

Phải gọi liên tục trong `loop()`. Thực hiện theo trình tự:

**Bước 1 – Đọc 14 byte thô từ thanh ghi 0x3B:**

```cpp
AccX(2) + AccY(2) + AccZ(2) + Temp(2) + GyroX(2) + GyroY(2) + GyroZ(2)
```

**Bước 2 – Quy đổi đơn vị:**

```cpp
temp = (rawTemp + 12412.0) / 340.0          // → °C
accX = rawAccX / 16384.0                    // → g  (±2g range)
gyroX = rawGyroX / 65.5 - gyroXoffset       // → °/s (±500°/s range)
```

**Bước 3 – Tính góc từ accelerometer:**

```cpp
angleAccX = atan2(accY, accZ + |accX|) * 360 / 2π    // góc roll (°)
angleAccY = atan2(accX, accZ + |accY|) * 360 / -2π   // góc pitch (°)
```

**Bước 4 – Tích phân góc gyro:**

```cpp
interval = (millis() - preInterval) * 0.001   // thời gian (giây)
angleGyroX += gyroX * interval                // tích phân → góc
```

**Bước 5 – Complementary Filter:**

```cpp
angleX = (0.98 * (angleX + gyroX * interval)) + (0.02 * angleAccX)
//         ↑ tin gyro 98% vì nhanh, ít nhiễu ngắn hạn
//                                    ↑ tin accel 2% để chống drift dài hạn
```

>angleZ không dùng accel (không tính được từ accel) nên chỉ dùng `angleGyroZ`, có nghĩa là yaw bị drift theo thời gian.

##### 6. Các getter

| Nhóm | Phương thức | Trả về | Mô tả |
| --- | --- | --- | --- |
| Thô | getRawAccX/Y/Z() | int16_t | Giá trị thô ADC accelerometer |
| Thô | getRawGyroX/Y/Z() | int16_t | Giá trị thô ADC gyroscope |
| Đã quy đổi | getAccX/Y/Z() | float | Gia tốc quy đổi (đơn vị g) |
| Đã quy đổi | getGyroX/Y/Z() | float | Tốc độ góc quy đổi (°/s) |
| Nhiệt độ | getTemp() | float | Nhiệt độ (°C) |
| Góc accel | getAccAngleX/Y() | float | Góc từ accelerometer (°) |
| Góc gyro | getGyroAngleX/Y/Z() | float | Góc từ gyroscope (°) |
| Góc tổn hợp | getAngleX/Y() | float | Góc cuối sau filter (°) |
| Góc Z | getAngleZ() | float | Góc Z từ gyro (có drift) |

#### Luồng sử dụng điển hình

```cpp
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.calcGyroOffsets(true);  // Hiệu chỉnh offset, giữ yên sensor ~4 giây
  mpu.begin();
}

void loop() {
  mpu.update();               // Luôn gọi đầu tiên trong loop
  
  Serial.print("AngleX: "); Serial.println(mpu.getAngleX());
  Serial.print("AngleY: "); Serial.println(mpu.getAngleY());
  Serial.print("Temp: ");   Serial.println(mpu.getTemp());
}
```

#### Lưu ý quan trọng

- `accCoef + gyroCoef` phải bằng 1.0 để Complementary Filter đúng nghĩa

- `Yaw (angleZ)` bị `drift` vì không có `nguồn tham chiếu tuyệt đối` — cần thêm `magnetometer` để khắc phục

- Gọi `update()` càng thường xuyên càng tốt để `interval` nhỏ, tích phân gyro chính xác hơn

- Nếu AD0 = HIGH thì địa chỉ I2C là `0x69`, cần sửa `MPU6050_ADDR`

---

## Phân tích 2 file Example

### File 1: GetAngle.ino

#### Mục đích

Đây là ví dụ đơn giản nhất — chỉ đọc và in góc nghiêng theo 3 trục liên tục.

#### Phân tích từng dòng

```cpp
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);  // Khởi tạo đối tượng, dùng bus I2C mặc định
```

```cpp
void setup() {
  Serial.begin(9600);       // Mở Serial Monitor ở baudrate 9600
  Wire.begin();             // Khởi động bus I2C
  mpu6050.begin();          // Cấu hình các thanh ghi MPU6050
  mpu6050.calcGyroOffsets(true); // Tự động đo offset gyro
                                 // true → in tiến trình ra Serial
                                 // ⚠️ Phải giữ yên sensor ~4 giây
}
```

```cpp
void loop() {
  mpu6050.update();   // Đọc dữ liệu mới từ sensor, tính toán góc

  // In 3 góc ra Serial Monitor, cách nhau bằng tab
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());   // Roll – nghiêng trái/phải
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());   // Pitch – nghiêng trước/sau
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ()); // Yaw – xoay ngang (có drift)
}
```

#### Kết quả in ra Serial Monitor

```bash
angleX : 2.34    angleY : -1.12    angleZ : 0.05
angleX : 2.35    angleY : -1.13    angleZ : 0.08
...
```

#### Lưu ý

Vì không có giới hạn thời gian trong `loop()`, dữ liệu được in liên tục với tốc độ tối đa → Serial Monitor cuộn rất nhanh. Nếu chỉ muốn quan sát, nên thêm `delay(100)` hoặc dùng kỹ thuật timer như file 2.

### File 2: GetAllData.ino

#### .Mục đích

Ví dụ đầy đủ hơn — in toàn bộ dữ liệu sensor theo từng nhóm, nhưng chỉ mỗi 1 giây một lần thay vì liên tục.

#### Điểm khác biệt quan trọng: Kỹ thuật Timer

```cpp
long timer = 0;  // Biến lưu thời điểm lần in trước
```

```cpp
void loop() {
  mpu6050.update();  // Vẫn gọi LIÊN TỤC — không bị chặn bởi timer
                     // ✅ Quan trọng: update() phải chạy thường xuyên
                     //    để tích phân góc gyro chính xác

  if(millis() - timer > 1000) {  // Chỉ in khi đã qua 1 giây
    // ... in dữ liệu ...
    timer = millis();  // Reset mốc thời gian
  }
}
```

>Đây là pattern non-blocking timer — khác với `delay(1000)` vì `delay()` sẽ làm dừng cả `update()`, khiến tích phân gyro bị sai. Với cách này, `update()` vẫn chạy hàng trăm lần/giây trong khi in dữ liệu chỉ xảy ra mỗi giây.

#### Các nhóm dữ liệu được in

```cpp
// Nhóm 1: Nhiệt độ chip
Serial.print("temp : "); Serial.println(mpu6050.getTemp());
// Ví dụ: temp : 28.50  (°C)

// Nhóm 2: Gia tốc (đơn vị g)
mpu6050.getAccX() / getAccY() / getAccZ()
// Ví dụ: accX : 0.01   accY : -0.02   accZ : 1.00
// Khi nằm phẳng: accZ ≈ 1.0g (trọng lực), accX/Y ≈ 0

// Nhóm 3: Tốc độ góc (đơn vị °/s)
mpu6050.getGyroX() / getGyroY() / getGyroZ()
// Ví dụ: gyroX : 0.12   gyroY : -0.08   gyroZ : 0.03
// Khi đứng yên: tất cả ≈ 0 (sau khi đã trừ offset)

// Nhóm 4: Góc tính RIÊNG từ accelerometer
mpu6050.getAccAngleX() / getAccAngleY()
// Ổn định dài hạn nhưng nhiễu khi rung động

// Nhóm 5: Góc tích phân RIÊNG từ gyroscope
mpu6050.getGyroAngleX() / getGyroAngleY() / getGyroAngleZ()
// Mượt, nhanh nhạy nhưng bị drift theo thời gian

// Nhóm 6: Góc SAU Complementary Filter ← dùng cái này trong thực tế
mpu6050.getAngleX() / getAngleY() / getAngleZ()
// Kết hợp ưu điểm của cả 2 nhóm trên
```

#### Kết quả in ra Serial Monitor (mỗi 1 giây)

```bash
=======================================================
temp : 28.50
accX : 0.01       accY : -0.02      accZ : 1.00
gyroX : 0.12      gyroY : -0.08     gyroZ : 0.03
accAngleX : 1.15  accAngleY : -1.00
gyroAngleX : 1.20 gyroAngleY : -0.95  gyroAngleZ : 0.30
angleX : 1.19     angleY : -0.97    angleZ : 0.30
=======================================================
```

#### Điều cần nhớ khi dùng 2 file này làm tham khảo

`mpu6050.update()` luôn phải ở đầu `loop()`, không được đặt bên trong `if` hay sau `delay()`. Đây là lỗi phổ biến nhất khi mới dùng thư viện — nếu `update()` chạy không đều, góc Z đặc biệt sẽ bị sai nhiều vì nó hoàn toàn phụ thuộc vào tích phân thời gian.
