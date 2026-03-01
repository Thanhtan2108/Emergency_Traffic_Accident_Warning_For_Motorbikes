// ============================================================
//  main.cpp  —  Test Bước 3: Pipeline MotionSensor → DataNormalizer
//
//  Mục tiêu test:
//    [1] MotionSensor (Khối 1) khởi tạo thành công
//    [2] DataNormalizer (Khối 2) khởi tạo thành công
//    [3] Pipeline Khối1→Khối2 thông suốt (không drop ở rawDataQueue)
//    [4] Giá trị accX/Y/Z hợp lý (tổng magnitude ≈ 1g)
//    [5] Gyro offset được trừ đúng (gyroX/Y/Z ≈ 0 khi đứng yên)
//    [6] angleAccX/Y tính được và hợp lý
//    [7] Nhiệt độ trong khoảng hợp lý (20°C – 50°C)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Config.h"
#include "DataTypes.h"
#include "IModule.h"
#include "QueueManager.h"
#include "MotionSensor.h"
#include "DataNormalizer.h"

// ============================================================
//  Forward declarations
// ============================================================
static void printNormalizedData(const NormalizedData& data);
static void printSeparator();
static void monitorTask(void* pvParameters);

// ============================================================
//  Globals
// ============================================================
static MotionSensor*    g_motionSensor    = nullptr;
static DataNormalizer*  g_dataNormalizer  = nullptr;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  TEST: Pipeline Khoi 1 + Khoi 2");
    Serial.println("  MotionSensor -> DataNormalizer");
    printSeparator();

    // --- I2C ---
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("[SETUP] I2C initialized");

    // --- QueueManager: tạo tất cả queues ---
    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager init failed!");
        while (true) { delay(1000); }
    }
    Serial.println("[SETUP] QueueManager OK");

    // --- Khối 1: MotionSensor ---
    // Gyro offset sẽ được lấy sau calibration và truyền sang DataNormalizer
    Serial.println("[SETUP] >> Giu yen sensor trong qua trinh calibration! <<");
    g_motionSensor = new MotionSensor(Wire, qm.getRawDataQueue());
    if (!g_motionSensor->begin()) {
        Serial.println("[FATAL] MotionSensor init failed!");
        while (true) { delay(1000); }
    }
    Serial.printf("[TEST 1] MotionSensor init : PASS (healthy=%s)\n",
                  g_motionSensor->isHealthy() ? "true" : "false");

    // --- Lấy gyro offset thực từ MotionSensor sau calibration ---
    // getGyroXoffset() đọc trực tiếp từ MPU6050_tockn sau calcGyroOffsets()
    const float offsetX = g_motionSensor->getGyroOffsetX();
    const float offsetY = g_motionSensor->getGyroOffsetY();
    const float offsetZ = g_motionSensor->getGyroOffsetZ();
    Serial.printf("[SETUP] Gyro offsets: X=%.3f  Y=%.3f  Z=%.3f (deg/s)\n",
                  offsetX, offsetY, offsetZ);

    // --- Khối 2: DataNormalizer ---
    g_dataNormalizer = new DataNormalizer(
        qm.getRawDataQueue(),
        qm.getNormalizedQueue(),
        offsetX, offsetY, offsetZ
    );
    if (!g_dataNormalizer->begin()) {
        Serial.println("[FATAL] DataNormalizer init failed!");
        while (true) { delay(1000); }
    }
    Serial.printf("[TEST 2] DataNormalizer init: PASS (healthy=%s)\n",
                  g_dataNormalizer->isHealthy() ? "true" : "false");

    // --- Monitor Task ---
    xTaskCreatePinnedToCore(
        monitorTask, "MonitorTask", 4096,
        nullptr, 1, nullptr, CORE_MANAGEMENT_TASKS
    );

    printSeparator();
    Serial.println("[SETUP] Pipeline running. Monitoring normalizedQueue...");
    printSeparator();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ============================================================
//  monitorTask — Đọc normalizedQueue, in báo cáo mỗi 1 giây
// ============================================================
static void monitorTask(void* pvParameters) {
    QueueManager& qm    = QueueManager::getInstance();
    QueueHandle_t queue = qm.getNormalizedQueue();

    NormalizedData latest;
    bool hasData = false;

    uint32_t receivedCount      = 0;
    uint32_t lastReportMs       = millis();
    uint32_t lastProcessedSnap  = 0;

    const uint32_t REPORT_INTERVAL_MS = 1000;

    while (true) {
        // Drain normalizedQueue — lấy frame mới nhất
        while (xQueueReceive(queue, &latest, pdMS_TO_TICKS(5)) == pdTRUE) {
            receivedCount++;
            hasData = true;
        }

        uint32_t now = millis();
        if (now - lastReportMs >= REPORT_INTERVAL_MS) {
            uint32_t currentProcessed   = g_dataNormalizer->getProcessedCount();
            uint32_t processedInPeriod  = currentProcessed - lastProcessedSnap;

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 1+2] Health check cả 2 khối
            Serial.printf("[TEST 1] MotionSensor healthy  : %s\n",
                g_motionSensor->isHealthy()   ? "PASS" : "FAIL");
            Serial.printf("[TEST 2] DataNormalizer healthy : %s\n",
                g_dataNormalizer->isHealthy() ? "PASS" : "FAIL");

            // [TEST 3] Pipeline thông suốt — MotionSensor không bị drop nữa
            Serial.printf("[TEST 3] MotionSensor drops    : %lu  %s\n",
                g_motionSensor->getDroppedFrames(),
                g_motionSensor->getDroppedFrames() == 0 ? "PASS" : "WARN");

            // DataNormalizer throughput
            Serial.printf("         DataNorm processed    : %lu frames (this sec: %lu)\n",
                currentProcessed, processedInPeriod);
            Serial.printf("         DataNorm drops        : %lu  %s\n",
                g_dataNormalizer->getDroppedCount(),
                g_dataNormalizer->getDroppedCount() == 0 ? "PASS" : "WARN");

            if (hasData) {
                Serial.println();
                Serial.println("[TEST 4-7] Latest NormalizedData:");
                printNormalizedData(latest);
            }

            lastReportMs      = now;
            lastProcessedSnap = currentProcessed;
            receivedCount     = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  printNormalizedData() — In và đánh giá NormalizedData
// ============================================================
static void printNormalizedData(const NormalizedData& d) {
    // --- Accel ---
    Serial.printf("  Accel (g) : X=%7.4f  Y=%7.4f  Z=%7.4f\n",
        d.accX, d.accY, d.accZ);

    // [TEST 4] Tổng độ lớn gia tốc phải ≈ 1g khi đứng yên
    float accMag = sqrtf(d.accX*d.accX + d.accY*d.accY + d.accZ*d.accZ);
    bool  accOk  = (accMag >= 0.85f && accMag <= 1.15f);
    Serial.printf("  |Acc| = %.4f g  → [TEST 4] %s  (expect 0.85~1.15g)\n",
        accMag, accOk ? "PASS" : "WARN");

    // --- Gyro ---
    Serial.printf("  Gyro(d/s) : X=%7.3f  Y=%7.3f  Z=%7.3f\n",
        d.gyroX, d.gyroY, d.gyroZ);

    // [TEST 5] Gyro gần 0 khi đứng yên (sau trừ offset)
    float gyroMag = sqrtf(d.gyroX*d.gyroX + d.gyroY*d.gyroY + d.gyroZ*d.gyroZ);
    bool  gyroOk  = (gyroMag < 2.0f);  // < 2°/s khi yên
    Serial.printf("  |Gyro| = %.3f d/s → [TEST 5] %s  (expect <2.0 when still)\n",
        gyroMag, gyroOk ? "PASS" : "WARN");

    // --- Angle từ Accel ---
    Serial.printf("  AngleAcc  : X=%7.2f°  Y=%7.2f°\n",
        d.angleAccX, d.angleAccY);

    // [TEST 6] Góc trong khoảng hợp lý [-180°, 180°]
    bool angleOk = (d.angleAccX >= -180.0f && d.angleAccX <= 180.0f &&
                    d.angleAccY >= -180.0f && d.angleAccY <= 180.0f);
    Serial.printf("  Angle range valid → [TEST 6] %s\n",
        angleOk ? "PASS" : "FAIL");

    // --- Nhiệt độ ---
    bool tempOk = (d.temperature >= 20.0f && d.temperature <= 50.0f);
    Serial.printf("  Temp      : %.1f°C   → [TEST 7] %s  (expect 20~50C)\n",
        d.temperature, tempOk ? "PASS" : "WARN");

    // --- Metadata ---
    Serial.printf("  Timestamp : %lu ms   isValid=%s\n",
        d.timestamp, d.isValid ? "true" : "false");
}

static void printSeparator() {
    Serial.println("============================================");
}
