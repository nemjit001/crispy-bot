#include "Motor.h"
#include "pins.h"

#define TPM0_FREQ (1.0 / 4166)

/* Private */

DigitalOut Motor::hBridgeEn(H_BRIDGE_PIN);

void Motor::enableHBridge()
{
    this->hBridgeEn = true;
}

void Motor::disableHBridge()
{
    this->hBridgeEn = false;
}

/* Public */

Motor::Motor(int _motorIn1, int _motorIn2)
{
    this->disableHBridge();

    this->motorIn1 = new PwmOut(_motorIn1);
    this->motorIn2 = new PwmOut(_motorIn2);

    this->initMotorInput(this->motorIn1);
    this->initMotorInput(this->motorIn2);

    this->enableHBridge();
}

Motor::~Motor()
{
    this->disableHBridge();
    delete this->motorIn1;
    delete this->motorIn2;
}

void Motor::initMotorInput(PwmOut *motorIn)
{
    motorIn->period(TPM0_FREQ);
    motorIn->write(0.0);
}

void Motor::setSpeed(float speed)
{
    if (speed > 1.0)
        speed = 1.0;
    if (speed < -1.0)
        speed = -1.0;
    
    if (speed >= 0.0)
    {
        *(this->motorIn1) = speed;
        *(this->motorIn2) = 0.0;
    }
    else
    {
        *(this->motorIn1) = 0.0;
        *(this->motorIn2) = -speed;
    }
}

float Motor::getSpeed()
{
    return this->motorIn1->read() = this->motorIn2->read();
}

Motor& Motor::operator= (float speed)
{
    this->setSpeed(speed);
    return *this;
}

Motor& Motor::operator= (Motor &rhs)
{
    this->setSpeed(rhs.getSpeed());
    return *this;
}

Motor::operator float()
{
    return this->getSpeed();
}
