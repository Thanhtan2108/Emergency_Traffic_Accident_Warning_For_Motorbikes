// ============================================================
//  SensorReader.cpp
//  Implement Tầng 1 – Wrapper cho MPU6050_tockn.
//
//  Tài liệu: README.md – Section V
// ============================================================

#include "SensorReader.h"


// ============================================================
//  begin()
// ============================================================
bool SensorReader::begin(TwoWire& wire, bool calcOffset) {
    _wire = &wire;
    _lastError = "";

    // Kiểm tra sensor có phản hồi trên bus I2C trước khi khởi tạo
    if (!_pingI2C()) {
        _lastError = "MPU6050 not found at 0x68 – check wiring";
        return false;
    }

    // Khởi tạo đối tượng MPU6050_tockn
    // MPU6050_tockn nhận TwoWire& và địa chỉ I2C
    _mpu = new MPU6050(wire);
    _mpu->begin();

    // Tính gyro offset nếu yêu cầu.
    // calcGyroOffsets(false) → tắt in Serial, không block.
    // Sensor phải đặt BẰNG PHẲNG và KHÔNG RUNG khi gọi hàm này.
    if (calcOffset) {
        _mpu->calcGyroOffsets(false);
    }

    _initialized = true;
    return true;
}


// ============================================================
//  read()
// ============================================================
RawSensorData SensorReader::read() {
    RawSensorData data;
    data.timestamp = millis();
    data.valid     = false;  // Giả định lỗi, xác nhận sau

    // Guard: chưa gọi begin() thành công
    if (!_initialized || _mpu == nullptr) {
        _lastError = "SensorReader not initialized – call begin() first";
        return data;
    }

    // Gọi update() để MPU6050_tockn tính lại Complementary Filter
    // Hàm này thực hiện I2C read bên trong
    _mpu->update();

    // Đọc gia tốc thô (đã chia cho LSB, đơn vị g)
    // ACCEL_CONFIG = 0x00 → Full scale ±2g → LSB = 16384
    data.ax = _mpu->getAccX();
    data.ay = _mpu->getAccY();
    data.az = _mpu->getAccZ();

    // Đọc tốc độ góc thô (đơn vị °/s)
    // GYRO_CONFIG = 0x08 → Full scale ±500°/s → LSB = 65.5
    data.gx = _mpu->getGyroX();
    data.gy = _mpu->getGyroY();
    data.gz = _mpu->getGyroZ();

    // Đọc góc từ Complementary Filter (đơn vị °)
    // getAngleX() = roll, getAngleY() = pitch
    data.roll  = _mpu->getAngleX();
    data.pitch = _mpu->getAngleY();
    // KHÔNG lấy getAngleZ() – yaw bị drift tích lũy, không có giá trị
    // trong việc phát hiện té ngã (Section II)

    // Kiểm tra cơ bản: nếu toàn bộ accel = 0.0 → I2C trả về 0 khi mất kết nối
    // Outlier reject chi tiết hơn sẽ được thực hiện ở SignalProcessor Bước 0
    if (data.ax == 0.0f && data.ay == 0.0f && data.az == 0.0f) {
        _lastError = "All accel values zero – possible I2C disconnection";
        return data;  // valid = false
    }

    // Dữ liệu hợp lệ
    data.valid    = true;
    _lastError    = "";
    return data;
}


// ============================================================
//  isReady()
// ============================================================
bool SensorReader::isReady() const {
    if (!_initialized) return false;
    return _pingI2C();
}


// ============================================================
//  getLastError()
// ============================================================
String SensorReader::getLastError() const {
    return _lastError;
}


// ============================================================
//  _pingI2C()  (private)
//  Thực hiện I2C scan tại địa chỉ I2C_ADDRESS (0x68).
//  Gửi beginTransmission + endTransmission và kiểm tra ACK.
//  Trả về true nếu nhận được ACK (thiết bị phản hồi).
// ============================================================
bool SensorReader::_pingI2C() const {
    if (_wire == nullptr) return false;

    _wire->beginTransmission(I2C_ADDRESS);
    uint8_t error = _wire->endTransmission();

    // endTransmission trả về 0 nếu ACK nhận được (thiết bị tồn tại)
    return (error == 0);
}
