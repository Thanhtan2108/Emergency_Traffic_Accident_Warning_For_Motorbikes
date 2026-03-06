#include "AlertBuzzer.h"

AlertBuzzer::AlertBuzzer(uint8_t pin) : _pin(pin) {

}

void AlertBuzzer::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void AlertBuzzer::on() {
    digitalWrite(_pin, HIGH);
}

void AlertBuzzer::off() {
    digitalWrite(_pin, LOW);
}

void AlertBuzzer::beep(uint16_t duration) {
    on();
    delay(duration);
    off();
}
