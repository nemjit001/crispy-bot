#pragma once

#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"
#include "Filter.h"

#include "math.h"

#define MODE STANDARD

#define CAMDATA_SIZE 128
#define EXPOSURE_TIME 2
#define SERVO_PERIOD 0.12
#define MAX_SPEED 0.20
#define MIN_SPEED 0.14
#define DEBUG_FLAG false

#define CAM_HEIGHT 21.2     // In cm, lens tot grond
#define LINE_DIST 30.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 1  // In cm, lens tot wiel, horizontaal
#define WIDTH_MUL 2
#define STEERING_RANGE 0.525 // In radialen, van midden naar links of rechts, was 0.675

typedef enum Mode {
    STANDARD = 0,
    PRINT,
    TEST
} Mode;

class NXPLogic {
private:
    uint16_t camData[CAMDATA_SIZE];
    int threshold = 0;
    Mode mode = MODE;

    int neutral = 1110;
    int radius = 160;

    float mid = CAMDATA_SIZE / 2;
    int firstEdge = 0;
    int secEdge = CAMDATA_SIZE - 1;
    int pos = neutral;
    
    float lineWidth = WIDTH_MUL * sqrt(CAM_HEIGHT * CAM_HEIGHT + (LINE_DIST - LENS_WHEELS_DIST) * (LINE_DIST - LENS_WHEELS_DIST));   // Totale breedte van de lijn

    PwmOut servo = PwmOut(PTA12);
    MW_DC_Motor motorA = MW_DC_Motor('A');
    MW_DC_Motor motorB = MW_DC_Motor('B');
    MW_Camera camera = MW_Camera(0);

    int stepCounter = 0;
    int testState = 0;

    float calcMid();
    int findEdge(int, int);
    void setWheels();
    void setSpeed();
    float getCamDataAverage();
    void printData();
    void testLoop();
public:
    NXPLogic();
    bool step();
};