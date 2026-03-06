#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <PinSetup.h>
#include "ReadDataFromSensor.h"
#include "AccidentDetector.h"
#include "AlertBuzzer.h"

// ==============================
// Objects
// ==============================

ReadData sensor;
AccidentDetector detector;
AlertBuzzer buzzer(BUZZER_PIN);


// ==============================
// Data structures
// ==============================

struct SensorData {
    float accelTotal;
    float angleX;
    float angleY;
};

struct AlertEvent {
    bool accident;
};


// ==============================
// FreeRTOS objects
// ==============================

QueueHandle_t sensorQueue;
QueueHandle_t alertQueue;


// ==============================
// Tasks
// ==============================

void TaskSensor(void *pvParameters) {

    SensorData data;

    while (true) {

        sensor.update();

        data.accelTotal = sensor.getAccelTotal();
        data.angleX = sensor.getAngleX();
        data.angleY = sensor.getAngleY();

        // Gửi dữ liệu qua Queue
        xQueueSend(sensorQueue, &data, portMAX_DELAY);

        // ===== LOG SENSOR =====
        Serial.print("A: ");
        Serial.print(data.accelTotal, 3);

        Serial.print(" | angleX: ");
        Serial.print(data.angleX, 2);

        Serial.print(" | angleY: ");
        Serial.println(data.angleY, 2);

        vTaskDelay(pdMS_TO_TICKS(1000)); // 20Hz
    }
}



void TaskDetectAccident(void *pvParameters) {

    SensorData data;
    AlertEvent alert;

    while (true) {

        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {

            bool accident =
                detector.processSample(
                    data.accelTotal,
                    data.angleX,
                    data.angleY);

            // ===== LOG DETECTOR =====
            Serial.print("Detector state: ");
            Serial.println(accident ? "ACCIDENT" : "NORMAL");

            if (accident) {

                Serial.println(">>> ACCIDENT EVENT QUEUED");

                alert.accident = true;

                xQueueSend(alertQueue, &alert, portMAX_DELAY);
            }
        }
    }
}



void TaskSendSOS(void *pvParameters) {

    AlertEvent alert;

    while (true) {

        if (xQueueReceive(alertQueue, &alert, portMAX_DELAY)) {

            if (alert.accident) {

                Serial.println("=================================");
                Serial.println("   ACCIDENT DETECTED !!!");
                Serial.println("=================================");

                buzzer.on();

                vTaskDelay(pdMS_TO_TICKS(2000));

                buzzer.off();

                Serial.println("Buzzer OFF");
            }
        }
    }
}


// ==============================
// Arduino setup
// ==============================

void setup() {

    Serial.begin(115200);

    Serial.println("=================================");
    Serial.println(" Accident Detection System Start ");
    Serial.println("=================================");

    sensor.begin();
    buzzer.begin();

    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    alertQueue  = xQueueCreate(5, sizeof(AlertEvent));

    Serial.println("Queues created");

    xTaskCreatePinnedToCore(
        TaskSensor,
        "TaskSensor",
        4096,
        NULL,
        3,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        TaskDetectAccident,
        "TaskDetectAccident",
        4096,
        NULL,
        2,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        TaskSendSOS,
        "TaskSendSOS",
        4096,
        NULL,
        1,
        NULL,
        0);

    Serial.println("Tasks started");
}


void loop() {

}
