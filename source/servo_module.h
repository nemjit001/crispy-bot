#ifndef __SERVO_MODULE_H__
#define __SERVO_MODULE_H__

extern "C"
{
#include "Modules/mTimer.h"
}

typedef enum {
    servo1,
    servo2
} servo_port_num;

class servoModule
{
private:
    servo_port_num servo_port;
public:
    servoModule(servo_port_num servo_port)
    {
        this->servo_port = servo_port;
        mTimer_SetServoDuty(this->servo_port, 0);
    }

    void setRotation(float in);
};

#endif
