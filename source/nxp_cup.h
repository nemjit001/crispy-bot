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
#define LINE_DIST 80.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0  // In cm, lens tot wiel, horizontaal

#define FOV_X (68 * M_PI / 180.0)
#define FOV_Y (45 * M_PI / 180.0)
#define STEERING_RANGE 0.735080959
#define THRESHOLD 7

#define CAM_ANGLE atan2(LINE_DIST + LENS_WHEELS_DIST, CAM_HEIGHT)
// #define CAM_ANGLE (64 * M_PI / 180.0)

#define MAX_SPEED 0.55
#define MIN_SPEED 0.43
#define SPEED_INCREASE_FACTOR 1.035
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

    bool stopTrackSignal = false, spee;
    int res_x, res_y, line1, line2, depth_p, frameCounter;
    point midLower, midUpper;
    uint8_t *camData1, *camData2;
    float currentSpeed, dist;

    void engine_kpod();

    int findEdgeHor(int y, int start, int stop);
    int findEdgeVer(int x, int start, int stop);
    int findEdge(uint8_t camData[], int start, int stop);
    point getMid(point prev, int y);
    point getMid(point prev, int y, int &firstEdge, int &secEdge);
    float getAngle(point p);
    int getDepth(int startHeight);
    void getCamData(int y, uint8_t camData[]);
    int getFinish();

    void setSpeed(float dist);
    void setWheels(point p);
    
    void checkTrackSignals();

    point pixel_to_point(int x, int y);
    point point_to_pixel(float x, float y);

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

        line1 = point_to_pixel(0, 50).y;

        depth_p = line1;

        frameCounter = 0;
        currentSpeed = MIN_SPEED;

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
    void printLineDist5();
    void printCamData();

    void step(bool motors) {
        point direction, p1, p2;
        spee = false;
        int firstEdge, secEdge;

        if (stopTrackSignal)
        {
            stop();
            return;
        }

        midLower = getMid(midLower, line1);
        // midLower.y -= 30;

        depth_p = getDepth(res_y - 1);
        p2 = pixel_to_point(res_x / 2, depth_p);
        dist = p2.y;
        // if (dist > 150) dist = 150;
        dist -= 60;
        depth_p = point_to_pixel(res_x / 2, dist).y;

        // float angleDiff = abs(atan2(midLower.x, midLower.y) - atan2(midUpper.x, midUpper.y));
        //  && angleDiff < (20 * (M_PI / 180.0))

        if (dist > midLower.y) {
            spee = true;
            pixy.setLamp(1, 1);
            direction = midUpper = getMid({0, 100}, depth_p);
        }
        else {
            pixy.setLamp(0, 0);
            direction = midLower;
        }
        
		setWheels(direction);
        if (motors) setSpeed(dist);
        else engine.setSpeed(0, 0);
        checkTrackSignals();
	};

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
