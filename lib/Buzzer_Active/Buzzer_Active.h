#ifndef BUZZER_ACTIVE_H
#define BUZZER_ACTIVE_H

#include <Arduino.h>
#include "Buzzer.h"  // Implement interface

class BuzzerActive : public Buzzer {
private:
    uint8_t pinBuzzer;

public:
    BuzzerActive(uint8_t pinBuzzer);

    void begin() override;
    void turnOn() override;
    void turnOff() override;
};

#endif
