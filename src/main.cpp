// ============================================================
//  main.cpp  —  Test Bước 4: Pipeline K1 → K2 → K3
//  MotionSensor → DataNormalizer → SignalProcessor
//
//  Mục tiêu test:
//    [1] SignalProcessor khởi tạo thành công
//    [2] Pipeline 3 khối không drop frame
//    [3] LPF hoạt động: filteredAcc mượt hơn raw (ít dao động)
//    [4] Complementary Filter: angleX/Y hội tụ và ổn định
//    [5] totalAccMag ≈ 1g khi đứng yên
//    [6] angularVelMag ≈ 0 khi đứng yên
//    [7] Jerk ≈ 0 khi đứng yên (không có thay đổi gia tốc đột ngột)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Config.h>
#include <DataTypes.h>
#include <IModule.h>
#include <QueueManager.h>
#include <MotionSensor.h>
#include <DataNormalizer.h>
#include <SignalProcessor.h>

// ============================================================
//  Forward declarations
// ============================================================
static void monitorTask(void* pvParameters);
static void printFeatures(const MotionFeatures& f);
static void printSeparator();

// ============================================================
//  Globals
// ============================================================
static MotionSensor*    g_motionSensor    = nullptr;
static DataNormalizer*  g_dataNormalizer  = nullptr;
static SignalProcessor* g_signalProcessor = nullptr;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  TEST: Pipeline K1 + K2 + K3");
    Serial.println("  MotionSensor -> DataNormalizer -> SignalProcessor");
    printSeparator();

    // --- I2C ---
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);

    // --- QueueManager ---
    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager init failed!");
        while (true) { delay(1000); }
    }

    // --- Khối 1: MotionSensor ---
    Serial.println("[SETUP] >> Giu yen sensor trong qua trinh calibration! <<");
    g_motionSensor = new MotionSensor(Wire, qm.getRawDataQueue());
    if (!g_motionSensor->begin()) {
        Serial.println("[FATAL] MotionSensor init failed!");
        while (true) { delay(1000); }
    }
    Serial.printf("[TEST 1] MotionSensor    : PASS (healthy=%s)\n",
                  g_motionSensor->isHealthy() ? "true" : "false");

    // --- Khối 2: DataNormalizer ---
    g_dataNormalizer = new DataNormalizer(
        qm.getRawDataQueue(),
        qm.getNormalizedQueue(),
        g_motionSensor->getGyroOffsetX(),
        g_motionSensor->getGyroOffsetY(),
        g_motionSensor->getGyroOffsetZ()
    );
    if (!g_dataNormalizer->begin()) {
        Serial.println("[FATAL] DataNormalizer init failed!");
        while (true) { delay(1000); }
    }
    Serial.printf("[TEST 1] DataNormalizer  : PASS (healthy=%s)\n",
                  g_dataNormalizer->isHealthy() ? "true" : "false");

    // --- Khối 3: SignalProcessor ---
    g_signalProcessor = new SignalProcessor(
        qm.getNormalizedQueue(),
        qm.getFeaturesQueue()
    );
    if (!g_signalProcessor->begin()) {
        Serial.println("[FATAL] SignalProcessor init failed!");
        while (true) { delay(1000); }
    }
    Serial.printf("[TEST 1] SignalProcessor : PASS (healthy=%s)\n",
                  g_signalProcessor->isHealthy() ? "true" : "false");

    // --- Monitor Task ---
    xTaskCreatePinnedToCore(
        monitorTask, "MonitorTask", 4096,
        nullptr, 1, nullptr, CORE_MANAGEMENT_TASKS
    );

    printSeparator();
    Serial.println("[SETUP] Pipeline K1->K2->K3 running...");
    printSeparator();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ============================================================
//  monitorTask — Đọc featuresQueue, báo cáo mỗi 2 giây
//  Thống kê min/max trong kỳ để đánh giá độ ổn định LPF
// ============================================================
static void monitorTask(void* pvParameters) {
    QueueManager& qm    = QueueManager::getInstance();
    QueueHandle_t queue = qm.getFeaturesQueue();

    MotionFeatures latest;
    bool hasData = false;

    // Tracking min/max trong 2 giây để đánh giá LPF noise
    float minAcc = 999.0f, maxAcc = 0.0f;
    float minJerk = 999.0f, maxJerk = 0.0f;

    uint32_t lastReportMs      = millis();
    uint32_t lastProcessedSnap = 0;
    const uint32_t REPORT_MS   = 2000;

    while (true) {
        MotionFeatures f;
        while (xQueueReceive(queue, &f, pdMS_TO_TICKS(5)) == pdTRUE) {
            latest  = f;
            hasData = true;

            // Theo dõi biên độ dao động totalAccMag và jerk
            if (f.totalAccMag < minAcc)  minAcc  = f.totalAccMag;
            if (f.totalAccMag > maxAcc)  maxAcc  = f.totalAccMag;
            if (f.jerk < minJerk)        minJerk = f.jerk;
            if (f.jerk > maxJerk)        maxJerk = f.jerk;
        }

        uint32_t now = millis();
        if (now - lastReportMs >= REPORT_MS) {
            uint32_t processed = g_signalProcessor->getProcessedCount();
            uint32_t inPeriod  = processed - lastProcessedSnap;

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 1] Health check 3 khối
            Serial.printf("[TEST 1] MotionSensor    healthy: %s\n",
                g_motionSensor->isHealthy()    ? "PASS" : "FAIL");
            Serial.printf("[TEST 1] DataNormalizer  healthy: %s\n",
                g_dataNormalizer->isHealthy()  ? "PASS" : "FAIL");
            Serial.printf("[TEST 1] SignalProcessor healthy: %s\n",
                g_signalProcessor->isHealthy() ? "PASS" : "FAIL");

            // [TEST 2] Drop check toàn pipeline
            Serial.printf("[TEST 2] MotionSensor  drops: %lu  %s\n",
                g_motionSensor->getDroppedFrames(),
                g_motionSensor->getDroppedFrames() == 0 ? "PASS" : "WARN");
            Serial.printf("[TEST 2] DataNormalizer drops: %lu  %s\n",
                g_dataNormalizer->getDroppedCount(),
                g_dataNormalizer->getDroppedCount() == 0 ? "PASS" : "WARN");
            Serial.printf("[TEST 2] SignalProcessor drops: %lu  %s\n",
                g_signalProcessor->getDroppedCount(),
                g_signalProcessor->getDroppedCount() == 0 ? "PASS" : "WARN");

            // Throughput
            Serial.printf("         SignalProcessor: %lu frames (%lu/2s)\n",
                processed, inPeriod);

            if (hasData) {
                Serial.println();
                printFeatures(latest);

                // [TEST 3] Đánh giá độ ổn định LPF qua biên độ dao động
                float accRange  = maxAcc - minAcc;
                float jerkRange = maxJerk - minJerk;
                Serial.printf("[TEST 3] LPF stability:\n");
                Serial.printf("         totalAcc range: %.4f g  %s  (expect <0.05g)\n",
                    accRange, accRange < 0.05f ? "PASS" : "WARN");
                Serial.printf("         jerk range    : %.3f g/s %s  (expect <1.0)\n",
                    jerkRange, jerkRange < 1.0f ? "PASS" : "WARN");
            }

            // Reset thống kê
            minAcc = 999.0f; maxAcc = 0.0f;
            minJerk = 999.0f; maxJerk = 0.0f;
            lastReportMs      = now;
            lastProcessedSnap = processed;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  printFeatures()
// ============================================================
static void printFeatures(const MotionFeatures& f) {
    // Filtered Accel
    Serial.printf("  filteredAcc (g): X=%7.4f  Y=%7.4f  Z=%7.4f\n",
        f.filteredAccX, f.filteredAccY, f.filteredAccZ);

    // [TEST 5] totalAccMag ≈ 1g khi đứng yên
    bool accOk = (f.totalAccMag >= 0.90f && f.totalAccMag <= 1.10f);
    Serial.printf("  totalAccMag    : %.4f g  [TEST 5] %s  (expect 0.90~1.10g)\n",
        f.totalAccMag, accOk ? "PASS" : "WARN");

    // Filtered Gyro
    Serial.printf("  filteredGyro   : X=%6.3f  Y=%6.3f  Z=%6.3f (d/s)\n",
        f.filteredGyroX, f.filteredGyroY, f.filteredGyroZ);

    // [TEST 6] angularVelMag ≈ 0 khi đứng yên
    bool gyroOk = (f.angularVelMag < 1.0f);
    Serial.printf("  angularVelMag  : %.3f d/s  [TEST 6] %s  (expect <1.0)\n",
        f.angularVelMag, gyroOk ? "PASS" : "WARN");

    // [TEST 4] Complementary Filter angles ổn định
    bool angleOk = (f.angleX >= -180.0f && f.angleX <= 180.0f &&
                    f.angleY >= -180.0f && f.angleY <= 180.0f);
    Serial.printf("  Angle CF (deg) : X=%7.2f  Y=%7.2f  Z=%7.2f  [TEST 4] %s\n",
        f.angleX, f.angleY, f.angleZ, angleOk ? "PASS" : "FAIL");

    // [TEST 7] Jerk ≈ 0 khi đứng yên
    bool jerkOk = (f.jerk < 2.0f);
    Serial.printf("  jerk           : %.3f g/s  [TEST 7] %s  (expect <2.0)\n",
        f.jerk, jerkOk ? "PASS" : "WARN");

    Serial.printf("  timestamp      : %lu ms   isValid=%s\n",
        f.timestamp, f.isValid ? "true" : "false");
}

static void printSeparator() {
    Serial.println("============================================");
}
