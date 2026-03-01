// ============================================================
//  main.cpp  —  Test Bước 2: MotionSensor
//
//  Mục tiêu test:
//    [1] MPU6050 kết nối I2C thành công (WHO_AM_I)
//    [2] Gyro calibration chạy đúng
//    [3] Task 100Hz tạo thành công và đang chạy
//    [4] RawSensorData được đẩy vào queue liên tục
//    [5] Data đọc ra queue hợp lệ (không phải 0 hết)
//    [6] isHealthy() phản ánh đúng trạng thái
//    [7] Thống kê sampleCount và droppedFrames
// ============================================================

#include <Arduino.h>
#include <Wire.h>

#include <Config.h>
#include <DataTypes.h>
#include <IModule.h>
#include <QueueManager.h>
#include <MotionSensor.h>

// ============================================================
//  Forward declarations
// ============================================================
static void printRawData(const RawSensorData& data);
static void printSeparator();
static void monitorTask(void* pvParameters);

// ============================================================
//  Global objects
// ============================================================
static MotionSensor* g_motionSensor = nullptr;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  TEST: MotionSensor Module");
    Serial.println("  Khoi 1 - Cam bien chuyen dong");
    printSeparator();

    // ----------------------------------------------------------
    // [1] Khởi tạo I2C
    // ----------------------------------------------------------
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("[SETUP] I2C initialized (SDA=" + String(PIN_SDA)
                   + ", SCL=" + String(PIN_SCL)
                   + ", " + String(I2C_FREQUENCY / 1000) + "kHz)");

    // ----------------------------------------------------------
    // [2] Khởi tạo QueueManager — tạo rawDataQueue
    // ----------------------------------------------------------
    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager init failed! Halting.");
        while (true) { delay(1000); }
    }
    Serial.println("[SETUP] QueueManager OK — rawDataQueue ready");

    // ----------------------------------------------------------
    // [3] Tạo và khởi tạo MotionSensor
    // ----------------------------------------------------------
    Serial.println("[SETUP] Creating MotionSensor...");
    g_motionSensor = new MotionSensor(Wire, qm.getRawDataQueue());

    Serial.println("[SETUP] Calling MotionSensor::begin()...");
    Serial.println("        >> Giu yen xe/sensor trong qua trinh calibration! <<");

    bool initOk = g_motionSensor->begin();

    if (!initOk) {
        Serial.println("[FATAL] MotionSensor init FAILED!");
        Serial.println("        Kiem tra:");
        Serial.println("        - Day I2C (SDA/SCL) da ket noi dung chua?");
        Serial.println("        - MPU6050 duoc cap nguon 3.3V chua?");
        Serial.println("        - Dia chi I2C co phai 0x68 khong? (AD0=GND)");
        while (true) { delay(1000); }
    }

    Serial.println("[SETUP] MotionSensor init OK!");
    Serial.printf("[SETUP] isHealthy = %s\n",
                  g_motionSensor->isHealthy() ? "TRUE" : "FALSE");

    // ----------------------------------------------------------
    // [4] Tạo Monitor Task — đọc queue và in ra Serial
    // ----------------------------------------------------------
    xTaskCreatePinnedToCore(
        monitorTask,            // Hàm task
        "MonitorTask",          // Tên
        4096,                   // Stack
        nullptr,                // Params
        1,                      // Priority thấp (chỉ để monitor)
        nullptr,                // Không cần handle
        CORE_MANAGEMENT_TASKS   // Core 0 (tách khỏi sensor core)
    );

    printSeparator();
    Serial.println("[SETUP] All tasks started. Monitoring queue output...");
    printSeparator();
}

// ============================================================
//  loop() — Trống, mọi thứ chạy trong FreeRTOS tasks
// ============================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ============================================================
//  monitorTask — Đọc rawDataQueue và in báo cáo mỗi 500ms
//
//  Thay vì in 100 dòng/giây (100Hz), task này gom data
//  và in báo cáo định kỳ để dễ đọc trên Serial Monitor.
// ============================================================
static void monitorTask(void* pvParameters) {
    QueueManager& qm    = QueueManager::getInstance();
    QueueHandle_t queue = qm.getRawDataQueue();

    RawSensorData latestData;
    bool          hasData = false;

    uint32_t receivedCount     = 0;
    uint32_t lastReportMs      = millis();
    uint32_t lastSampleSnapshot = 0;

    const uint32_t REPORT_INTERVAL_MS = 500;

    while (true) {
        // Drain toàn bộ data trong queue, giữ lại frame mới nhất
        while (xQueueReceive(queue, &latestData, pdMS_TO_TICKS(5)) == pdTRUE) {
            receivedCount++;
            hasData = true;
        }

        // In báo cáo định kỳ mỗi 500ms
        uint32_t now = millis();
        if (now - lastReportMs >= REPORT_INTERVAL_MS) {
            uint32_t elapsed         = now - lastReportMs;
            uint32_t currentSamples  = g_motionSensor->getSampleCount();
            uint32_t samplesInPeriod = currentSamples - lastSampleSnapshot;
            float    actualHz        = samplesInPeriod * 1000.0f / elapsed;

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 6] isHealthy
            Serial.printf("[TEST 6] isHealthy()      : %s\n",
                g_motionSensor->isHealthy() ? "PASS (true)" : "FAIL (false)");

            // [TEST 3] Task đang chạy
            Serial.printf("[TEST 3] Task running     : %s  (total samples=%lu)\n",
                samplesInPeriod > 0 ? "PASS" : "FAIL", currentSamples);

            // [TEST 7] Sample rate thực tế
            Serial.printf("[TEST 7] Sample rate      : %.1f Hz  (target=100Hz) %s\n",
                actualHz,
                (actualHz >= 90.0f && actualHz <= 110.0f) ? "PASS" : "WARN");

            // [TEST 4] Queue nhận data
            Serial.printf("[TEST 4] Queue receiving  : %s  (frames drained=%lu)\n",
                hasData ? "PASS" : "FAIL", receivedCount);

            // [TEST 7] Dropped frames
            uint32_t dropped = g_motionSensor->getDroppedFrames();
            Serial.printf("[TEST 7] Dropped frames   : %lu  %s\n",
                dropped, dropped == 0 ? "PASS" : "WARN (consumer too slow?)");

            // [TEST 5] Raw data hợp lệ
            if (hasData) {
                Serial.println();
                Serial.println("[TEST 5] Latest RawSensorData from queue:");
                printRawData(latestData);

                bool allZero = (latestData.rawAccX  == 0 &&
                                latestData.rawAccY  == 0 &&
                                latestData.rawAccZ  == 0 &&
                                latestData.rawGyroX == 0 &&
                                latestData.rawGyroY == 0 &&
                                latestData.rawGyroZ == 0);
                Serial.printf("         Data not all zero : %s\n",
                    !allZero ? "PASS" : "FAIL (all zeros — sensor loi?)");

                // AccZ khi đặt nằm ngang phải gần 16384 (~1g)
                bool accZplausible = abs(latestData.rawAccZ) > 8000;
                Serial.printf("         AccZ plausible    : %s  (rawAccZ=%d)\n",
                    accZplausible ? "PASS" : "WARN", latestData.rawAccZ);

                Serial.printf("         isValid flag      : %s\n",
                    latestData.isValid ? "PASS" : "FAIL");
            }

            // Reset cho kỳ tiếp
            lastReportMs       = now;
            lastSampleSnapshot = currentSamples;
            receivedCount      = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  printRawData()
// ============================================================
static void printRawData(const RawSensorData& data) {
    Serial.printf("         Timestamp : %lu ms\n",  data.timestamp);
    Serial.printf("         AccX      : %6d  (%.4f g)\n",
        data.rawAccX,  data.rawAccX  / 16384.0f);
    Serial.printf("         AccY      : %6d  (%.4f g)\n",
        data.rawAccY,  data.rawAccY  / 16384.0f);
    Serial.printf("         AccZ      : %6d  (%.4f g)\n",
        data.rawAccZ,  data.rawAccZ  / 16384.0f);
    Serial.printf("         GyroX     : %6d  (%.2f deg/s)\n",
        data.rawGyroX, data.rawGyroX / 65.5f);
    Serial.printf("         GyroY     : %6d  (%.2f deg/s)\n",
        data.rawGyroY, data.rawGyroY / 65.5f);
    Serial.printf("         GyroZ     : %6d  (%.2f deg/s)\n",
        data.rawGyroZ, data.rawGyroZ / 65.5f);
    Serial.printf("         RawTemp   : %6d  (%.1f C)\n",
        data.rawTemp, (data.rawTemp + 12412.0f) / 340.0f);
}

// ============================================================
//  printSeparator()
// ============================================================
static void printSeparator() {
    Serial.println("============================================");
}
