#include "servo.h"

void servoModule::setRotation(float in)
{
    if (in > 1)
        in = 1.0;
    else if (in < -1)
        in = -1.0;

    this->duty = in;

    switch (this->servo_port)
    {
    case servoPort1:
        mTimer_SetServoDuty(0, this->duty);
        break;
    case servoPort2:
        mTimer_SetServoDuty(1, this->duty);
    }
}

float servoModule::getRotation()
{
    return this->duty;
}

void servoModule::setServo(float angle) {
    float offset = angle / STEERING_RANGE * SERVO_RANGE;

    offset += SERVO_CENTER;
    if (offset > SERVO_CENTER + STEERING_RANGE) offset = SERVO_CENTER + STEERING_RANGE;
    else if (offset < SERVO_CENTER - STEERING_RANGE) offset = SERVO_CENTER - STEERING_RANGE;

    setRotation(offset);
}