#include <Arduino.h>
#include "Button.h"
#include "IBuzzer.h"
#include "Buzzer_Active.h"

Button myButton(4);
IBuzzer* myBuzzer = new BuzzerActive(13);

unsigned long beepStartTime = 0;
const unsigned long beepDuration = 200;  // Beep 200ms
bool isBeeping = false;

void setup() {
  Serial.begin(115200);
  myButton.begin();
  myBuzzer->begin();
}

void loop() {
  if (myButton.isPressed()) {
    Serial.println("Button pressed!");  // Thêm log này
    myBuzzer->turnOn();
    beepStartTime = millis();
    isBeeping = true;
  }

  if (isBeeping && (millis() - beepStartTime >= beepDuration)) {
    Serial.println("Beep off");  // Thêm log này để check off
    myBuzzer->turnOff();
    isBeeping = false;
  }
}
