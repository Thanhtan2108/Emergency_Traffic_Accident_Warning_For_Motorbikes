#ifndef IBUZZER_H
#define IBUZZER_H

class IBuzzer {
public:
    virtual ~IBuzzer() {}  // Destructor ảo cho polymorphism
    virtual void begin() = 0;
    virtual void turnOn() = 0;
    virtual void turnOff() = 0;
};

#endif
