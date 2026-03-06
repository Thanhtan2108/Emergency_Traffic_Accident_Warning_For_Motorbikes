#pragma once

#include <Arduino.h>

class AlertBuzzer {
public:
    AlertBuzzer(uint8_t pin);

    void begin();

    void on();
    void off();

    void beep(uint16_t duration);

private:
    uint8_t _pin;
};
