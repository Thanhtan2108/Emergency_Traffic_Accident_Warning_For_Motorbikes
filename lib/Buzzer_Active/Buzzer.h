#ifndef BUZZER_H
#define BUZZER_H

class Buzzer {
public:
    virtual ~Buzzer() {}  // Destructor ảo cho polymorphism
    virtual void begin() = 0;
    virtual void turnOn() = 0;
    virtual void turnOff() = 0;
};

#endif
