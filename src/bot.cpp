#include "Motor.h"
#include "pins.h"

class Bot
{
private:
    Motor *LeftMotor;
    Motor *RightMotor;
    PwmOut *SteeringServo;
public:
    Bot();
    ~Bot();
};

Bot::Bot()
{
    this->LeftMotor = new Motor(MOTOR_A_1, MOTOR_A_2);
    this->RightMotor = new Motor(MOTOR_B_1, MOTOR_B_2);
    this->SteeringServo = new PwmOut(SERVO_PIN);
}

Bot::~Bot()
{
    delete this->LeftMotor;
    delete this->RightMotor;
    delete this->SteeringServo;
}

int main()
{
    Bot *bot = new Bot();

    delete bot;
    return 0;
}
