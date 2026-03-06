#include "AccidentDetector.h"

AccidentDetector::AccidentDetector() {
    currentState = NORMAL;
}

bool AccidentDetector::processSample(float A, float angleX, float angleY)
{
    uint32_t now = millis();

    switch(currentState)
    {

    case NORMAL:

        if(A > impactThreshold)
        {
            impactTime = now;
            currentState = WAIT_ORIENTATION;
        }

    break;

    case WAIT_ORIENTATION:

        if(abs(angleX) > tiltThreshold || abs(angleY) > tiltThreshold)
        {
            stillStart = now;
            currentState = WAIT_STILLNESS;
        }

        if(now - impactTime > 2000)
        {
            currentState = NORMAL;
        }

    break;

    case WAIT_STILLNESS:

        if(A > stillAccelMin && A < stillAccelMax)
        {
            if(now - stillStart > stillTimeRequired)
            {
                currentState = ACCIDENT;
                return true;
            }
        }
        else
        {
            stillStart = now;
        }

    break;

    case ACCIDENT:

        return true;

    break;
    }

    return false;
}
