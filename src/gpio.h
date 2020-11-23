#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdio.h>

typedef enum 
{
    PTA01,
    PTA02,
    PTA03,
    PTA04,
    PTA05,
    PTA06,
    PTA07,
    PTA08,
    PTA09,
    PTA10,
    PTA11,
    PTA12
} PinName;

class PwmOut
{
private:
    PinName pin;
public:
    PwmOut(PinName pin);
    ~PwmOut();
    void period(float period);
    void write(float value);
    float read();
    PwmOut& operator= (float value);
};

class DigitalOut
{
private:
    PinName pin;
public:
    DigitalOut(PinName pin);
    ~DigitalOut();
    void write(int value);
    int read();
    DigitalOut& operator= (int value);
};

#endif // (__GPIO_H__)
