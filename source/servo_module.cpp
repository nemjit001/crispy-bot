#include "servo_module.h"

void servoModule::setRotation(float in)
{
    if (in > 1)
        in = 1.0;
    else if (in < -1)
        in = -1.0;

    switch (this->servo_port)
    {
    case servo1:
        mTimer_SetServoDuty(0, in);
        break;
    case servo2:
        mTimer_SetServoDuty(1, in);
    }
}
