#ifndef BUZZER_ACTIVE_H
#define BUZZER_ACTIVE_H

#include <Arduino.h>
#include "Buzzer.h"

class BuzzerActive : public Buzzer {
public:
    BuzzerActive(uint8_t pinBuzzer);
    void begin()    override;
    void turnOn()   override;
    void turnOff()  override;
    bool isOn()     override;

private:
    uint8_t _pinBuzzer;
    bool    _isOn;
};

#endif
