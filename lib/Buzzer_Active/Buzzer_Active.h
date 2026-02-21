#ifndef BUZZER_ACTIVE_H
#define BUZZER_ACTIVE_H

#include <Arduino.h>
#include "IBuzzer.h"  // Implement interface

class BuzzerActive : public IBuzzer {
private:
    uint8_t pinBuzzer;

public:
    BuzzerActive(uint8_t pinBuzzer);

    void begin() override;
    void turnOn() override;
    void turnOff() override;
};

#endif
