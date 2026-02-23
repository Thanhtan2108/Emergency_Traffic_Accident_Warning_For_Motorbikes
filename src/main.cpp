#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Common.h>
#include "Button.h"
#include "IBuzzer.h"
#include "Buzzer_Active.h"

/* ================== OBJECT ================== */
Button myButton(BUTTON_PIN);
IBuzzer* myBuzzer = new BuzzerActive(BUZZER_PIN);

/* ================== RTOS ================== */
QueueHandle_t buzzerQueue;

/* ================== TASK BUTTON ================== */
void TaskButton(void *pvParameters) {
  while (1) {
    if (myButton.isPressed()) {
      Serial.println("Button toggle!");

      bool cmd = true;
      xQueueSend(buzzerQueue, &cmd, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================== TASK BUZZER ================== */
void TaskBuzzer(void *pvParameters) {
  bool cmd;
  bool buzzerOn = false;

  while (1) {
    if (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY) == pdTRUE) {

      buzzerOn = !buzzerOn;

      if (buzzerOn) {
        Serial.println("Buzzer ON");
        myBuzzer->turnOn();
      } else {
        Serial.println("Buzzer OFF");
        myBuzzer->turnOff();
      }
    }
  }
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  myButton.begin();
  myBuzzer->begin();

  buzzerQueue = xQueueCreate(5, sizeof(bool));

  xTaskCreate(
    TaskButton,
    "TaskButton",
    2048,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    TaskBuzzer,
    "TaskBuzzer",
    2048,
    NULL,
    1,
    NULL
  );
}

/* ================== LOOP ================== */
void loop() {

}
