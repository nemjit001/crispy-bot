#include "Motor.h"
#include "Servo.h"

class Bot
{
private:
    Motor *LeftMotor;
    Motor *RightMotor;
    Servo *SteeringServo;
public:
    Bot();
    ~Bot();
};

Bot::Bot()
{
    this->LeftMotor = new Motor();
    this->RightMotor = new Motor();
    this->SteeringServo = new Servo();
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
