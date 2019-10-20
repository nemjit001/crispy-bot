#pragma once

#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"
#include "Filter.h"

#define CAMDATA_SIZE 128
#define EXPOSURE_TIME 2
#define SERVO_PERIOD 0.02
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SPEED 0.16
#define THRESHOLD 4000
#define DEBUG_FLAG false

typedef struct steeringStruct {
    int neutral = (SERVO_MIN + SERVO_MAX) / 2;
    int radius = SERVO_MAX - SERVO_MIN;
} steeringStruct;

class NXPLogic {
private:
    int camDataSize = CAMDATA_SIZE;
    uint16_t camData[CAMDATA_SIZE];

    int threshold = THRESHOLD;
    float speed = SPEED;
    int mid = camDataSize / 2;

    steeringStruct steering;
    Filter filter = Filter();

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