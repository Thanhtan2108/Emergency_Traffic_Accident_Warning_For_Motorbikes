#pragma once
#include <Arduino.h>

class AccidentDetector {

public:

    enum State {
        NORMAL,
        WAIT_ORIENTATION,
        WAIT_STILLNESS,
        ACCIDENT
    };

    AccidentDetector();

    bool processSample(float A, float angleX, float angleY);

private:

    State currentState;

    uint32_t impactTime;
    uint32_t stillStart;

    const float impactThreshold = 3.0;
    const float tiltThreshold = 60.0;

    const float stillAccelMin = 0.85;
    const float stillAccelMax = 1.15;

    const uint32_t stillTimeRequired = 3000;
};
