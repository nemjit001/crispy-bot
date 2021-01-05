#include "engine_module.h"

float engineModule::getSpeedLeft()
{
    float speedl, speedr;
    mTimer_GetSpeed(&speedl, &speedr);
    return speedl;
}

float engineModule::getSpeedRight()
{
    float speedl, speedr;
    mTimer_GetSpeed(&speedl, &speedr);
    return speedr;
}

void engineModule::setSpeed(float speedL, float speedR)
{
    if (speedL > 1.0)
        speedL = 1.0;
    if (speedL < - 1.0)
        speedL = -1.0;

    if (speedR > 1.0)
        speedR = 1.0;
    if (speedR < - 1.0)
        speedR = -1.0;

    mTimer_SetMotorDuty(speedL, speedR);
}
