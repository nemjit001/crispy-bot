#include "Motor.h"
#include "util.h"

class Bot
{
    private:
    Motor *motor_a;
    Motor *motor_b;

    public:
    Bot();
    ~Bot();
};

Bot::Bot()
{
    this->motor_a = new Motor(PTA01, PTA02);
    this->motor_b = new Motor(PTA03, PTA04);
}

Bot::~Bot()
{
    delete this->motor_a;
    delete this->motor_b;
}

int main()
{
    Bot *bot = new Bot();

    delete bot;
    return 0;
}
