#ifndef BUZZER_H
#define BUZZER_H

class Buzzer {
public:
    virtual ~Buzzer() {}
    virtual void begin()    = 0;
    virtual void turnOn()   = 0;
    virtual void turnOff()  = 0;
    virtual bool isOn()     = 0;
};

#endif
