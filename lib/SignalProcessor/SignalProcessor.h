#pragma once

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IModule.h>
#include <DataTypes.h>
#include <Config.h>

// ============================================================
//  SignalProcessor.h  —  Khối 3: Xử lý tín hiệu
//
//  Trách nhiệm duy nhất (SRP):
//    - Nhận NormalizedData từ normalizedQueue (Khối 2)
//    - Lọc nhiễu bằng Low-Pass Filter (LPF) cho acc và gyro
//    - Tính góc tổng hợp bằng Complementary Filter (X, Y, Z)
//    - Tính các đặc trưng chuyển động: totalAccMag, angularVelMag, jerk
//    - Đẩy MotionFeatures vào featuresQueue (→ Khối 4)
//
//  Thuật toán:
//    LPF      : y = alpha*x + (1-alpha)*y_prev
//    CompFilter: angle = gyroCoef*(angle + gyro*dt) + accCoef*angleAcc
//    Jerk     : d|acc|/dt = (totalAcc_now - totalAcc_prev) / dt
//
//  KHÔNG phát hiện tai nạn — chỉ xử lý và trích xuất đặc trưng.
// ============================================================

class SignalProcessor : public IModule {
public:
    // --------------------------------------------------------
    // Constructor
    // inQueue  : normalizedQueue — nhận từ DataNormalizer
    // outQueue : featuresQueue   — gửi sang AccidentDetector
    // --------------------------------------------------------
    SignalProcessor(QueueHandle_t inQueue, QueueHandle_t outQueue);

    // --------------------------------------------------------
    // IModule interface
    // --------------------------------------------------------
    bool        begin()         override;
    const char* getModuleName() const override { return "SignalProcessor"; }
    bool        isHealthy()     const override;

    // --------------------------------------------------------
    // Getters để monitor / test
    // --------------------------------------------------------
    uint32_t getProcessedCount() const { return _processedCount; }
    uint32_t getDroppedCount()   const { return _droppedCount; }

    // Truy cập features mới nhất (dùng để debug, không dùng thay queue)
    MotionFeatures getLatestFeatures() const { return _latest; }

private:
    // --------------------------------------------------------
    // FreeRTOS Task
    // --------------------------------------------------------
    static void taskEntry(void* pvParameters);
    void        taskRun();

    // --------------------------------------------------------
    // Signal processing pipeline — theo thứ tự xử lý
    // --------------------------------------------------------

    // Bước 1: Low-Pass Filter cho acc và gyro
    // Công thức: out = alpha*in + (1-alpha)*prev
    // alpha nhỏ → lọc mạnh, alpha lớn → giữ nhiều tín hiệu gốc
    void applyLPF(const NormalizedData& in);

    // Bước 2: Complementary Filter — tính góc X, Y tổng hợp
    // Kết hợp gyro (nhanh, ít drift ngắn hạn) và accel (chậm, ổn định dài hạn)
    void applyComplementaryFilter(const NormalizedData& in, float dt);

    // Bước 3: Tích phân gyroZ → angleZ (yaw)
    // Accel không tính được yaw nên dùng gyro thuần túy
    void integrateYaw(float dt);

    // Bước 4: Tính các đại lượng tổng hợp
    void computeFeatures(uint32_t timestamp);

    // --------------------------------------------------------
    // Helpers
    // --------------------------------------------------------
    // Tính delta time (giây) từ 2 timestamp (ms)
    float calcDeltaTime(uint32_t nowMs, uint32_t prevMs) const {
        return (nowMs - prevMs) * 0.001f;
    }

    void pushToQueue(const MotionFeatures& features);

    // --------------------------------------------------------
    // State — lưu giá trị từ frame trước cho LPF và CompFilter
    // --------------------------------------------------------

    // LPF state (giá trị đã lọc của frame trước)
    float _lpfAccX,  _lpfAccY,  _lpfAccZ;
    float _lpfGyroX, _lpfGyroY, _lpfGyroZ;

    // Complementary Filter state
    float _angleX;      // Roll  tổng hợp (°)
    float _angleY;      // Pitch tổng hợp (°)
    float _angleZ;      // Yaw   tích phân từ gyro (°)

    // Jerk calculation state
    float    _prevTotalAcc;     // |acc| của frame trước (g)
    uint32_t _prevTimestamp;    // Timestamp frame trước (ms)

    // Frame tracking
    bool     _isFirstFrame;     // true = chưa có frame trước → bỏ qua LPF init

    // Output
    MotionFeatures _latest;     // Features mới nhất (cho getter debug)

    // --------------------------------------------------------
    // Queue handles
    // --------------------------------------------------------
    QueueHandle_t   _inQueue;
    QueueHandle_t   _outQueue;
    TaskHandle_t    _taskHandle;

    volatile bool   _taskRunning;
    uint32_t        _processedCount;
    uint32_t        _droppedCount;
};
