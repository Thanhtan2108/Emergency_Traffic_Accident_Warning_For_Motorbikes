#include <Arduino.h>
// ============================================================
//  main.ino
//  Entry point – Accident Detection System
//  ESP32 + MPU6050 + FreeRTOS
//
//  Trách nhiệm:
//    setup() – Khởi tạo toàn bộ hệ thống theo đúng thứ tự,
//              tạo 4 FreeRTOS task, sau đó trả quyền điều phối
//              hoàn toàn cho FreeRTOS scheduler.
//    loop()  – Nhường CPU vô thời hạn. KHÔNG có logic ở đây.
//
//  Thứ tự khởi tạo bắt buộc (Section XI):
//    1. Serial + Wire
//    2. sensorReader.begin()     – kết nối MPU6050, tính gyro offset
//    3. signalProcessor.begin()  – reset buffer, baseline = 1.0g
//    4. accidentDetector.begin() – state = WARMUP
//    5. initTasks()              – tạo Queue/EventGroup/Mutex + 4 task
//
//  Tài liệu: README.md – Section XI, XIV
// ============================================================

#include <Wire.h>
#include "config.h"
#include "DataTypes.h"
#include "SensorReader.h"
#include "SignalProcessor.h"
#include "AccidentDetector.h"
#include "TaskManager.h"


// ============================================================
//  OBJECT DECLARATIONS
//  Thứ tự khởi tạo: sp trước ad (ad cần &sp trong constructor)
// ============================================================
static SensorReader     sensorReader;
static SignalProcessor  signalProcessor;
static AccidentDetector accidentDetector(signalProcessor);


// ============================================================
//  setup()
//  Chạy một lần trên Core 1 khi ESP32 khởi động.
//  Sau khi initTasks() trả về, FreeRTOS scheduler tiếp quản.
// ============================================================
void setup() {
    // ----------------------------------------------------------
    //  Bước 1 – Serial và I2C
    // ----------------------------------------------------------
    Serial.begin(115200);
    while (!Serial) { /* chờ Serial sẵn sàng trên một số board */ }

    Serial.println("========================================");
    Serial.println("  Accident Detection System  v2.0");
    Serial.println("  ESP32 + MPU6050 + FreeRTOS");
    Serial.println("========================================");

    Wire.begin();
    Serial.println("[INIT] I2C initialized");

    // ----------------------------------------------------------
    //  Bước 2 – SensorReader: kết nối MPU6050, tính gyro offset
    //  begin(wire, calcOffset=true):
    //    - Ping địa chỉ 0x68, khởi tạo MPU6050_tockn
    //    - calcGyroOffsets(true) → tính offset VÀ in ra Serial
    //    ⚠ Đặt sensor BẰNG PHẲNG, KHÔNG RUNG trong ~4 giây
    // ----------------------------------------------------------
    Serial.println("[INIT] Initializing MPU6050...");
    Serial.println("[INIT] >>> Keep sensor FLAT and STILL for ~4 seconds <<<");

    if (!sensorReader.begin(Wire, true)) {
        Serial.print("[FATAL] MPU6050 not found: ");
        Serial.println(sensorReader.getLastError());
        Serial.println("[FATAL] System halted. Check wiring and reset.");
        // Treo hệ thống – không tiếp tục nếu không có sensor
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    Serial.println("[INIT] MPU6050 OK – gyro offsets calculated");

    // ----------------------------------------------------------
    //  Bước 3 – SignalProcessor: reset buffer, baseline = 1.0g
    // ----------------------------------------------------------
    signalProcessor.begin();
    Serial.println("[INIT] SignalProcessor OK – baseline = 1.0g, buffer cleared");

    // ----------------------------------------------------------
    //  Bước 4 – AccidentDetector: state = WARMUP
    // ----------------------------------------------------------
    accidentDetector.begin();
    Serial.println("[INIT] AccidentDetector OK – state = WARMUP");

    // ----------------------------------------------------------
    //  Bước 5 – FreeRTOS: Queue, EventGroup, Mutex, 4 Tasks
    //  initTasks() truyền địa chỉ objects cho TaskManager
    //  để các task truy cập qua global pointer.
    // ----------------------------------------------------------
    initTasks(&sensorReader, &signalProcessor, &accidentDetector);

    Serial.println("[INIT] FreeRTOS tasks created:");
    Serial.printf ("  SensorTask   – Core %d, Priority %d, Stack %d bytes\n",
                   SENSOR_TASK_CORE,   SENSOR_TASK_PRIORITY,   SENSOR_TASK_STACK);
    Serial.printf ("  StateTask    – Core %d, Priority %d, Stack %d bytes\n",
                   STATE_TASK_CORE,    STATE_TASK_PRIORITY,    STATE_TASK_STACK);
    Serial.printf ("  WatchdogTask – Core %d, Priority %d, Stack %d bytes\n",
                   WATCHDOG_TASK_CORE, WATCHDOG_TASK_PRIORITY, WATCHDOG_TASK_STACK);
    Serial.printf ("  OutputTask   – Core %d, Priority %d, Stack %d bytes\n",
                   OUTPUT_TASK_CORE,   OUTPUT_TASK_PRIORITY,   OUTPUT_TASK_STACK);

    Serial.println("========================================");
    Serial.println("[INIT] System running – WARMUP phase (~2s)");
    Serial.println("       Waiting for baseline to stabilize...");
    Serial.println("========================================");

    // setup() kết thúc → FreeRTOS scheduler tiếp quản
}


// ============================================================
//  loop()
//  KHÔNG có logic ở đây.
//  Nhường CPU hoàn toàn cho FreeRTOS scheduler.
//  vTaskDelay(portMAX_DELAY) block vô thời hạn,
//  cho phép idle task và các task khác chạy.
// ============================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}
