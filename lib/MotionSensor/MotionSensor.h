#pragma once

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IModule.h>
#include <DataTypes.h>
#include <Config.h>
#include <MPU6050_tockn.h>

// ============================================================
//  MotionSensor.h  —  Khối 1: Cảm biến chuyển động
//
//  Trách nhiệm duy nhất (SRP):
//    - Giao tiếp trực tiếp với phần cứng MPU6050 qua I2C
//    - Đọc raw data ở tần suất cố định 100Hz
//    - Đóng gói thành RawSensorData và đẩy vào rawDataQueue
//    - Báo cáo trạng thái kết nối sensor cho hệ thống
//
//  KHÔNG xử lý, KHÔNG tính toán, KHÔNG lọc —
//  chỉ thu thập và chuyển tiếp dữ liệu thô.
// ============================================================

class MotionSensor : public IModule {
public:
    // --------------------------------------------------------
    // Constructor
    // wire        : tham chiếu I2C bus (Wire hoặc Wire1)
    // rawDataQueue: handle queue nhận RawSensorData (từ QueueManager)
    // --------------------------------------------------------
    MotionSensor(TwoWire& wire, QueueHandle_t rawDataQueue);

    // --------------------------------------------------------
    // IModule interface
    // --------------------------------------------------------

    // Khởi tạo MPU6050, hiệu chỉnh gyro, tạo FreeRTOS Task
    // Trả về false nếu MPU6050 không phản hồi trên I2C
    bool        begin()           override;

    const char* getModuleName()   const override { return "MotionSensor"; }

    // true nếu sensor đang kết nối và task đang chạy bình thường
    bool        isHealthy()       const override;

    // --------------------------------------------------------
    // Public getters — dùng để debug hoặc SystemWatchdog query
    // --------------------------------------------------------
    bool        isSensorConnected()  const { return _sensorConnected; }
    uint32_t    getSampleCount()     const { return _sampleCount; }
    uint32_t    getDroppedFrames()   const { return _droppedFrames; }

private:
    // --------------------------------------------------------
    // FreeRTOS Task entry point
    // Phải là static vì FreeRTOS không hiểu con trỏ this
    // Dùng pvParameters để truyền con trỏ this vào
    // --------------------------------------------------------
    static void taskEntry(void* pvParameters);

    // Hàm thực thi vòng lặp chính của task (member function)
    void taskRun();

    // --------------------------------------------------------
    // Internal helpers
    // --------------------------------------------------------

    // Kiểm tra MPU6050 có phản hồi trên I2C không (WHO_AM_I register)
    bool probeSensor();

    // Đọc toàn bộ raw register và đóng gói thành RawSensorData
    RawSensorData readRawData();

    // Gửi data vào queue, ghi nhận dropped frame nếu queue đầy
    void pushToQueue(const RawSensorData& data);

    // --------------------------------------------------------
    // Member variables
    // --------------------------------------------------------
    MPU6050         _mpu;               // Thư viện MPU6050_tockn
    QueueHandle_t   _rawDataQueue;      // Queue gửi data sang DataNormalizer

    TaskHandle_t    _taskHandle;        // Handle của FreeRTOS task

    volatile bool   _sensorConnected;   // true = sensor đang online
    volatile bool   _taskRunning;       // true = task đang chạy

    uint32_t        _sampleCount;       // Tổng số lần đọc thành công
    uint32_t        _droppedFrames;     // Số frame bị drop do queue đầy
};
