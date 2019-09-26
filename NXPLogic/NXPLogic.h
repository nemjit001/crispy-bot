#pragma once

#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#define EXPOSURE_TIME 2
#define SERVO_PERIOD 0.02
#define SPEED 0.16
#define THRESHOLD 4000
#define DEBUG_FLAG false

typedef struct steeringStruct {
    int neutral = 1210;
    int radius = 200;
} steeringStruct;

class NXPLogic {
private:
    uint16_t camData[128];
    int mid = 64;
    int threshold = THRESHOLD;
    float speed = SPEED;

    steeringStruct steering;

    PwmOut servo = PwmOut(PTA12);
    MW_DC_Motor motorA = MW_DC_Motor('A');
    MW_DC_Motor motorB = MW_DC_Motor('B');
    MW_Camera camera = MW_Camera(0);

    int calcMid();
    int findEdge(int, int);
    int getCamDataDiff(int);
    void setWheels();
public:
    NXPLogic();
    bool step();
};