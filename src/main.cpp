#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <ReadDataFromSensor.h>

ReadData ReadDataSensor;

// Task doc cam bien
void TaskReadSensor(void *pvParameters) {
    while (true) {
        ReadDataSensor.update();

        float A = ReadDataSensor.getAccelTotal();
        float angleX = ReadDataSensor.getAngleX();
        float angleY = ReadDataSensor.getAngleY();

        Serial.print("AccelTotal: ");
        Serial.println(A);

        Serial.print("AngleX: ");
        Serial.println(angleX);

        Serial.print("AngleY: ");
        Serial.println(angleY);

        Serial.println("========================================");

        // delay 1000 ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    Serial.begin(115200);

    ReadDataSensor.begin();

    // Tao task FreeRTOS
    xTaskCreate(
        TaskReadSensor,     // function task
        "ReadSensorTask",   // ten task
        4096,               // stack size
        NULL,               // tham so
        1,                  // priority
        NULL                // task handle
    );
}

void loop() {
    // khong can dung loop khi dung FreeRTOS
}
