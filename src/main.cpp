#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "ReadDataFromSensor.h"
#include "AccidentDetector.h"

// ======= Global objects =========
ReadData sensor;
AccidentDetector detector;

struct SensorData {
    float accelTotal;
    float angleX;
    float angleY;
    uint32_t timestamp;
};

struct AlertData {
    bool accident;
    uint32_t timestamp;
};

// Queues & Mutex
static QueueHandle_t sensorQueue = nullptr;
static QueueHandle_t alertQueue  = nullptr;
static SemaphoreHandle_t serialMutex = nullptr;

// Pins
static const int BUZZER_PIN = 13;

// Task prototypes
void TaskSensor(void* pvParameters);
void TaskDetect(void* pvParameters);
void TaskSendSOS(void* pvParameters);

// helper for safe printing
static void safePrintln(const String &s) {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.println(s);
        xSemaphoreGive(serialMutex);
    }
}

void setup() {
    Serial.begin(115200);
    delay(50);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // init sensor
    sensor.begin(); // this will calibrate gyro (blocking inside)

    // create queues
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    alertQueue  = xQueueCreate(4, sizeof(AlertData));

    // create mutex
    serialMutex = xSemaphoreCreateMutex();

    // create tasks pinned to cores (ESP32)
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskDetect, "TaskDetect", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskSendSOS, "TaskSendSOS", 4096, NULL, 1, NULL, 0);

    safePrintln("System started. Tasks created.");
}

void loop() {
    // empty - all work handBuzzer by tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------- TaskSensor ----------------
void TaskSensor(void* pvParameters) {
    SensorData data;
    const TickType_t delayTicks = pdMS_TO_TICKS(50); // 20Hz
    while(true) {
        sensor.update();
        data.accelTotal = sensor.getAccelTotal();
        data.angleX = sensor.getAngleX();
        data.angleY = sensor.getAngleY();
        data.timestamp = millis();

        // send (don't block long)
        xQueueSend(sensorQueue, &data, 0);

        // debug print
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50))) {
            Serial.print("Sensor updated | A: ");
            Serial.print(data.accelTotal, 3);
            Serial.print(" | angleX: ");
            Serial.print(data.angleX, 2);
            Serial.print(" | angleY: ");
            Serial.println(data.angleY, 2);
            xSemaphoreGive(serialMutex);
        }

        vTaskDelay(delayTicks);
    }
}

// ---------------- TaskDetect ----------------
void TaskDetect(void* pvParameters) {
    SensorData data;
    AlertData alert;
    while(true) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE) {
            bool fireNow = detector.processSample(data.accelTotal, data.angleX, data.angleY);
            if (fireNow) {
                alert.accident = true;
                alert.timestamp = millis();
                xQueueSend(alertQueue, &alert, 0);
            }
            // optional: when not firing we could send false to alertQueue to clear, but TaskSendSOS can maintain its own state
        }
    }
}

// ---------------- TaskSendSOS ----------------
void TaskSendSOS(void* pvParameters) {
    AlertData alert;
    bool BuzzerOn = false;
    while(true) {
        if (xQueueReceive(alertQueue, &alert, portMAX_DELAY) == pdTRUE) {
            if (alert.accident) {
                BuzzerOn = true;
                digitalWrite(BUZZER_PIN, HIGH);

                // Debug print
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                    Serial.println(">> ALERT: ACCIDENT DETECTED! (Buzzer ON)");
                    Serial.print("   at "); Serial.println(alert.timestamp);
                    xSemaphoreGive(serialMutex);
                }

                // TODO: Place SMS/GSM/WiFi send code here.
                // e.g. queue a network task, or call a non-blocking function to request message + location.
            }
        }
        // remain responsive but avoid busy spin
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
