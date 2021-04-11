#ifndef NXP_CUP_H_
#define NXP_CUP_H_

extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "Utils/def.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"

#include "Applications/gInput.h"
#include "Applications/gCompute.h"
#include "Applications/gOutput.h"

#include "arm_math.h"
}

/* Pixy 2 */
#include "Pixy/Pixy2SPI_SS.h"
//#include "Pixy/Pixy2line.h"

/* Own modules */
#include "servo_module.h"
#include "engine_module.h"

#define CAM_HEIGHT 39.5     // In cm, lens tot grond
#define LINE_DIST 71.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0  // In cm, lens tot wiel, horizontaal

#define FOV_X (60 * M_PI / 180.0)
#define FOV_Y (40 * M_PI / 180.0)
#define STEERING_RANGE 0.735080959
#define THRESHOLD 10

#define CAM_ANGLE atan2(LINE_DIST + LENS_WHEELS_DIST, CAM_HEIGHT)
// #define CAM_ANGLE (64 * M_PI / 180.0)

#define MAX_SPEED 0.5
#define MIN_SPEED 0.45
#define SPEED_INCREASE_FACTOR 1.05
#define SPEED_DECREASE_FACTOR (1.0f / SPEED_INCREASE_FACTOR)

typedef struct {
    double x;
    double y;
} point;

class rover
{
private:
    Pixy2SPI_SS pixy;
    servoModule *servo;
    engineModule engine;

    bool stopTrackSignal = false;
    int res_x, res_y, line1, line2, depth, frameCounter;
    point midLower, midUpper;
    uint8_t *camData1, *camData2;
    float currentSpeed;

    void engine_kpod();

    int findEdgeHor(int y, int start, int stop);
    int findEdgeVer(int x, int start, int stop);
    int findEdge(uint8_t camData[], int start, int stop);
    point getMid(point prev, int y);
    point getMid(point prev, int y, int &firstEdge, int &secEdge);
    point getDir(point prev);
    int getDepth(int startHeight);
    void getCamData(int y, uint8_t camData[]);

    void setSpeed(bool spee);
    void setWheels(point p);
    
    void checkTrackSignals();

    point convert_point(int x, int y);
    point reverse_point(float x, float y);

public:
    rover() {
        servo = new servoModule(servoPort1);
        pixy.init();
        pixy.setLED(0, 255, 0);

		servo->setRotation(0.0);
		engine.setSpeed(0.0, 0.0);

        res_x = pixy.frameWidth;
        res_y = pixy.frameHeight;

        midLower = midUpper = {0, 100};

        line1 = res_y - 25;

        depth = line1;

        frameCounter = 0;

        camData1 = (uint8_t*)malloc(res_x * sizeof(uint8_t));
        camData2 = (uint8_t*)malloc(res_x * sizeof(uint8_t));
    }

    ~rover()
    {
        delete servo;
        free(camData1);
        free(camData2);
    }

    void test_servo();
    void test_rgb();
    void printLineDist();
    void printLineDist2();
    void printLineDist3();
    void printLineDist4();
    void printCamData();

    void step(bool motors) {
        point mid, p1, p2;
        bool spee = false;

        if (stopTrackSignal)
        {
            stop();
            return;
        }

        mid = midLower = getMid(midLower, line1);

        depth = getDepth(res_y - 1);
        p2 = convert_point(res_x / 2, depth);
        float dist = p2.y;

        if (dist - 50 > mid.y) {
            frameCounter++;
            if (frameCounter >= 3) {
                spee = true;
                pixy.setLamp(1, 1);
                p2 = reverse_point(res_x / 2, dist - 50);
                mid = midUpper = getMid(midUpper, p2.y);
            }
        }
        else {
            frameCounter = 0;
            pixy.setLamp(0, 0);
        }

        // if (dist < 50) {
        //     pixy.setLamp(1, 1);
        // }
        // else {
        //     pixy.setLamp(0, 0);
        // }
        
		setWheels(mid);
        if (motors) setSpeed(spee);
        else engine.setSpeed(0, 0);
        checkTrackSignals();
	};

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
