#ifndef __MOTOR_H__
#define __MOTOR_H__

class Motor
{
private:
    PwmOut *motorIn1;
    PwmOut *motorIn2;
    static DigitalOut *hBridgeEn;

private:
    void enableHBridge();
    void disableHBridge();
    void initMotorInput(PwmOut *motorIn);
public:
    Motor(int _motorIn1, int _motorIn2);
    ~Motor();

    void setSpeed(float speed);
    float getSpeed();
    Motor& operator= (float speed);
    Motor& operator= (Motor &rhs);
    operator float();
};

#endif // (__MOTOR_H__)
