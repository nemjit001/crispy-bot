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
    volatile float duty;
public:
    servoModule(servo_port_num servo_port)
    {
        this->servo_port = servo_port;
        this->duty = 0.0;
        mTimer_SetServoDuty(this->servo_port, this->duty);
    }

    void setRotation(float in);
    float getRotation(void);
};

#endif
