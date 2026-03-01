#pragma once

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IModule.h>
#include <DataTypes.h>
#include <Config.h>

// ============================================================
//  DataNormalizer.h  —  Khối 2: Thu thập & Chuẩn hóa dữ liệu
//
//  Trách nhiệm duy nhất (SRP):
//    - Nhận RawSensorData từ rawDataQueue (Khối 1)
//    - Chuyển đổi ADC counts → đơn vị vật lý có nghĩa
//    - Trừ gyro offset đã hiệu chỉnh
//    - Tính góc nghiêng sơ bộ từ accelerometer (arctan)
//    - Đẩy NormalizedData vào normalizedQueue (→ Khối 3)
//
//  Công thức chuẩn hóa (theo datasheet MPU6050):
//    Accel  : value(g)   = raw / 16384.0   (range ±2g)
//    Gyro   : value(°/s) = raw / 65.5      (range ±500°/s)
//    Temp   : value(°C)  = (raw + 12412.0) / 340.0
//    AngleX : arctan2(accY, accZ + |accX|) * (180/π)
//    AngleY : arctan2(accX, accZ + |accY|) * (-180/π)
//
//  KHÔNG lọc nhiễu, KHÔNG tính góc tổng hợp —
//  chỉ chuyển đổi đơn vị và trừ offset.
// ============================================================

class DataNormalizer : public IModule {
public:
    // --------------------------------------------------------
    // Constructor
    // inQueue  : rawDataQueue   — nhận từ MotionSensor
    // outQueue : normalizedQueue — gửi sang SignalProcessor
    // gyroOffsetX/Y/Z : offset đã tính từ MotionSensor calibration
    //   (truyền vào để DataNormalizer chủ động trừ offset trên raw)
    // --------------------------------------------------------
    DataNormalizer(QueueHandle_t inQueue,
                   QueueHandle_t outQueue,
                   float gyroOffsetX = 0.0f,
                   float gyroOffsetY = 0.0f,
                   float gyroOffsetZ = 0.0f);

    // --------------------------------------------------------
    // IModule interface
    // --------------------------------------------------------
    bool        begin()         override;
    const char* getModuleName() const override { return "DataNormalizer"; }
    bool        isHealthy()     const override;

    // --------------------------------------------------------
    // Cập nhật gyro offset sau khi calibration xong
    // (dùng khi SystemStateManager truyền offset từ MotionSensor)
    // --------------------------------------------------------
    void setGyroOffsets(float x, float y, float z);

    // --------------------------------------------------------
    // Getters để monitor / test
    // --------------------------------------------------------
    uint32_t getProcessedCount() const { return _processedCount; }
    uint32_t getDroppedCount()   const { return _droppedCount; }

private:
    // --------------------------------------------------------
    // FreeRTOS Task
    // --------------------------------------------------------
    static void taskEntry(void* pvParameters);
    void        taskRun();

    // --------------------------------------------------------
    // Normalización core — thuần toán học, không có side-effect
    // --------------------------------------------------------

    // Chuyển đổi toàn bộ raw → normalized
    NormalizedData normalize(const RawSensorData& raw) const;

    // Chuyển ADC counts → g (±2g range: LSB = 16384)
    float rawToAccel(int16_t raw)  const { return raw / 16384.0f; }

    // Chuyển ADC counts → °/s (±500°/s range: LSB = 65.5), trừ offset
    float rawToGyro(int16_t raw, float offset) const {
        return (raw / 65.5f) - offset;
    }

    // Chuyển raw → °C
    float rawToTemp(int16_t raw) const {
        return (raw + 12412.0f) / 340.0f;
    }

    // Tính roll angle từ accelerometer (độ °)
    // Công thức: arctan2(accY, accZ + |accX|) — giảm singularity
    float calcAngleX(float accX, float accY, float accZ) const {
        return atan2f(accY, accZ + fabsf(accX)) * (180.0f / M_PI);
    }

    // Tính pitch angle từ accelerometer (độ °)
    // Âm để pitch dương = nghiêng về phía trước
    float calcAngleY(float accX, float accY, float accZ) const {
        return atan2f(accX, accZ + fabsf(accY)) * (-180.0f / M_PI);
    }

    // Gửi vào normalizedQueue, ghi nhận drop nếu đầy
    void pushToQueue(const NormalizedData& data);

    // --------------------------------------------------------
    // Member variables
    // --------------------------------------------------------
    QueueHandle_t   _inQueue;           // rawDataQueue (input)
    QueueHandle_t   _outQueue;          // normalizedQueue (output)

    TaskHandle_t    _taskHandle;

    // Gyro offset (°/s) — trừ vào kết quả chuyển đổi
    float           _gyroOffsetX;
    float           _gyroOffsetY;
    float           _gyroOffsetZ;

    volatile bool   _taskRunning;

    uint32_t        _processedCount;    // Số frame đã chuẩn hóa thành công
    uint32_t        _droppedCount;      // Số frame bị drop do outQueue đầy
};
