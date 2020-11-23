#include "gpio.h"

/* PWM Out */

PwmOut::PwmOut(PinName _pin)
{
    this->pin = _pin;
}

PwmOut::~PwmOut()
{
    //
}

void PwmOut::period(float period)
{
    // write period to GPIO pin
}

void PwmOut::write(float value)
{
    // write value to GPIO pin register
}

float PwmOut::read()
{
    // read from GPIO pin register
    return 0.0f;
}

PwmOut& PwmOut::operator= (float value)
{
    this->write(value);
    return *this;
}

/* Digital Out */

DigitalOut::DigitalOut(PinName _pin)
{
    this->pin = _pin;
}

DigitalOut::~DigitalOut()
{
    //
}

void DigitalOut::write(int value)
{
    // write value to GPIO pin register
}

int DigitalOut::read()
{
    // read value from GPIO pin register
    return 0;
}

DigitalOut& DigitalOut::operator= (int value)
{
    this->write(value);
    return *this;
}
