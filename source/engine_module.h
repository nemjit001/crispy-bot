#ifndef __ENGINE_MODULE_H__
#define __ENGINE_MODULE_H__

extern "C"
{
#include "Modules/mTimer.h"
}

class engineModule
{
public:
    float getSpeedLeft();
    float getSpeedRight();
    void setSpeed(float speedL, float speedR);
};

#endif // __ENGINE_MODULE_H__
