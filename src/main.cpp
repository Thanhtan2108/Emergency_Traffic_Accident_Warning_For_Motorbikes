// ============================================================
//  main.cpp  —  Entry Point
//
//  Chỉ có nhiệm vụ:
//    1. Khởi tạo Serial & I2C
//    2. Khởi tạo QueueManager (tạo tất cả queue/semaphore)
//    3. Khởi tạo SystemStateManager (tạo tất cả task)
//    4. Giao quyền cho FreeRTOS scheduler
//
//  Toàn bộ logic hệ thống nằm trong các module ở lib/
//  main.cpp càng đơn giản càng tốt (thin entry point)
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// Common layer (Bước 1 - đã hoàn thành)
#include <Config.h>
#include <DataTypes.h>
#include <IModule.h>
#include <QueueManager.h>

// Các module sẽ được include sau khi từng Bước hoàn thành:
// #include "SystemStateManager/SystemStateManager.h"  // Bước 8

// ============================================================
//  setup() - Chạy một lần khi board khởi động
// ============================================================
void setup() {
    // --- Serial ---
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);  // Chờ Serial ổn định
    Serial.println("\n\n========================================");
    Serial.println("  Vehicle Sensor Block - Starting up");
    Serial.println("========================================");

    // --- I2C ---
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("[main] I2C initialized");

    // --- QueueManager ---
    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager init failed! Halting.");
        while (true) { delay(1000); }  // Halt
    }
    Serial.println("[main] QueueManager initialized - all queues ready");

    // --- SystemStateManager sẽ được khởi tạo ở Bước 8 ---
    // SystemStateManager& sm = SystemStateManager::getInstance();
    // sm.begin(qm);

    Serial.println("[main] Common layer (Step 1) complete.");
    Serial.println("[main] Waiting for subsequent modules...");

    // FreeRTOS scheduler tự chạy sau setup() trên Arduino framework
    // Nếu không có task nào thì Idle task chạy
}

// ============================================================
//  loop() - Trống hoàn toàn
//  Toàn bộ logic chạy trong FreeRTOS Tasks
//  loop() = Idle task của Arduino, không dùng
// ============================================================
void loop() {
    // Không làm gì ở đây.
    // Tất cả xử lý nằm trong FreeRTOS tasks của từng module.
    vTaskDelay(portMAX_DELAY);  // Yield vĩnh viễn
}
