#include "Buzzer_Active.h"

BuzzerActive::BuzzerActive(uint8_t pinBuzzer)
    : _pinBuzzer(pinBuzzer), _isOn(false) {
}

void BuzzerActive::begin() {
    pinMode(_pinBuzzer, OUTPUT);
    digitalWrite(_pinBuzzer, LOW);
    _isOn = false;
}

void BuzzerActive::turnOn() {
    digitalWrite(_pinBuzzer, HIGH);
    _isOn = true;
}

void BuzzerActive::turnOff() {
    digitalWrite(_pinBuzzer, LOW);
    _isOn = false;
}

bool BuzzerActive::isOn() {
    return _isOn;
}
