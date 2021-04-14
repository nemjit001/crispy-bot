#ifndef __SERVO_MODULE_H__
#define __SERVO_MODULE_H__

extern "C"
{
#include "Modules/mTimer.h"
// #include "Utils/util.h"
}

#define SERVO_RANGE 0.7
#define SERVO_CENTER -0.14
#define STEERING_RANGE 0.735

typedef enum
{
    servoPort1,
    servoPort2
} servo_port_num;

class servoModule
{
private:
    servo_port_num servo_port;
    float duty;
public:
    servoModule(servo_port_num servo_port)
    {
        this->servo_port = servo_port;
        this->duty = 0.0;
        mTimer_SetServoDuty(this->servo_port, this->duty);
    }

    void setRotation(float in);
    void setServo(float angle);
    float getRotation(void);
};

#endif
