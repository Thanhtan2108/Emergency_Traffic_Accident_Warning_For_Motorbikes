#include "Buzzer_Active.h"

BuzzerActive::BuzzerActive(uint8_t pinBuzzer) {
    this->pinBuzzer = pinBuzzer;
}

void BuzzerActive::begin() {
    pinMode(pinBuzzer, OUTPUT);
    digitalWrite(pinBuzzer, LOW);  // Off ban đầu
}

void BuzzerActive::turnOn() {
    digitalWrite(pinBuzzer, HIGH);
}

void BuzzerActive::turnOff() {
    digitalWrite(pinBuzzer, LOW);
}
