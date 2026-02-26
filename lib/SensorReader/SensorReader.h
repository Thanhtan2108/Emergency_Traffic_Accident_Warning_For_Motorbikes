#pragma once
// ============================================================
//  SensorReader.h
//  Tầng 1 – Wrapper cho thư viện MPU6050_tockn.
//
//  TRÁCH NHIỆM:
//    Chỉ đọc dữ liệu thô từ MPU6050 và đóng gói vào RawSensorData.
//    Không xử lý, không lọc, không tính toán thêm bất cứ điều gì.
//
//  KHÔNG BIẾT GÌ VỀ: SignalProcessor, AccidentDetector, FreeRTOS.
//
//  Tài liệu: README.md – Section V
//  Thứ tự implement: file số 3 (sau DataTypes.h và config.h)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "DataTypes.h"


class SensorReader {
public:

    // --------------------------------------------------------
    //  begin()
    //  Khởi tạo I2C và MPU6050. Gọi một lần trong setup().
    //
    //  Tham số:
    //    wire        – tham chiếu đến đối tượng TwoWire (thường là Wire)
    //    calcOffset  – true: gọi calcGyroOffsets(false) để tính gyro offset
    //                  false: bỏ qua (dùng khi cần khởi động nhanh/test)
    //
    //  Trả về:
    //    true  – sensor phản hồi ở địa chỉ 0x68, sẵn sàng đọc
    //    false – không tìm thấy sensor (I2C lỗi hoặc chưa nối dây)
    //
    //  Lưu ý: calcGyroOffsets(false) tắt in Serial → không block
    // --------------------------------------------------------
    bool begin(TwoWire& wire, bool calcOffset = true);

    // --------------------------------------------------------
    //  read()
    //  Gọi mpu.update() rồi đóng gói toàn bộ kết quả.
    //
    //  Trả về RawSensorData với:
    //    valid = true  – dữ liệu hợp lệ
    //    valid = false – Wire timeout hoặc sensor không phản hồi
    //
    //  Lưu ý quan trọng:
    //    - roll  = mpu.getAngleX()  (Complementary Filter)
    //    - pitch = mpu.getAngleY()  (Complementary Filter)
    //    - KHÔNG lấy getAngleZ() – yaw bị drift, không dùng
    // --------------------------------------------------------
    RawSensorData read();

    // --------------------------------------------------------
    //  isReady()
    //  Kiểm tra sensor có phản hồi trên bus I2C không.
    //  Dùng để WatchdogTask hoặc setup() kiểm tra trạng thái.
    //
    //  Trả về:
    //    true  – sensor phản hồi ở địa chỉ I2C_ADDRESS (0x68)
    //    false – không có phản hồi
    // --------------------------------------------------------
    bool isReady() const;

    // --------------------------------------------------------
    //  getLastError()
    //  Mô tả ngắn gọn lỗi gần nhất (tiếng Anh để dễ log Serial).
    //  Trả về chuỗi rỗng "" nếu không có lỗi.
    // --------------------------------------------------------
    String getLastError() const;

private:
    // Đối tượng thư viện MPU6050_tockn
    // Khai báo con trỏ để trì hoãn khởi tạo đến lúc begin() được gọi
    MPU6050* _mpu = nullptr;

    // Tham chiếu đến Wire object (được lưu lại từ begin())
    TwoWire* _wire = nullptr;

    // Trạng thái khởi tạo
    bool _initialized = false;

    // Lỗi gần nhất
    String _lastError = "";

    // Địa chỉ I2C mặc định khi AD0 = LOW
    static constexpr uint8_t I2C_ADDRESS = 0x68;

    // --------------------------------------------------------
    //  _pingI2C()  (private helper)
    //  Thực hiện I2C scan để kiểm tra địa chỉ 0x68 có phản hồi.
    //  Trả về true nếu nhận ACK.
    // --------------------------------------------------------
    bool _pingI2C() const;
};
