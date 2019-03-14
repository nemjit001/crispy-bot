#ifndef _MW_SERVO_H
#define _MW_SERVO_H

#include "mbed.h"

class MW_Servo
{

public:
    // Constructor and Destructor
    ~MW_Servo();
    MW_Servo(uint8_t ServoNumber, float minAngle = -30, float maxAngle = 30);

    // initialize pwm signals to 240us period and 0% duty cycle
    void initServo(PwmOut* Servo);

    // accessor methods
    void  setMinAngle(float minAngleInDeg); // Set Minimum Angle of Servo
    void  setMaxAngle(float maxAngleInDeg); // Set Maximum Angle of Servo
    void  setPosition(float position);      // Set position of Servo Relative to Range (-1 to 1)
    void  setRange(float minAngleInDeg, float maxAngleInDeg); // Set Range
    void  setAngle(float angleInDeg);       // Set position of the Servo
    float getAngle();                       // Get angle of the Servo

    /**  Shorthand for the accessor methods */
    MW_Servo& operator= (float angle);
    MW_Servo& operator= (MW_Servo& rhs);
    operator float();

private:
    static const PinName servoPins[];

    PwmOut* Servo;      // PWM object which drives the servo
    float   angle;      // Current angle of the Servo
    float   minAngle;   // Min Angle to which the Servo should turn
    float   maxAngle;   // Max Angle to which the Servo should turn
};

#endif
