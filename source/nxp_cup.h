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
#define SERVO_RANGE 0.7
#define SERVO_CENTER -0.14

#define CAM_ANGLE atan2(LINE_DIST + LENS_WHEELS_DIST, CAM_HEIGHT)
// #define CAM_ANGLE (64 * M_PI / 180.0)

#define MAX_SPEED 0.60
#define MIN_SPEED 0.47
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

    bool stopTrackSignal = false, spee = false, braking = false, canBrake = true;
    int res_x, res_y, line1, line2, depth_p, frameCounter;
    point midLower, midUpper;
    uint8_t *camData1, *camData2;
    float currentSpeed, dist, offset;

    void engine_kpod();

    int findEdgeHor(int y, int start, int stop);
    int findEdgeVer(int x, int start, int stop);
    int findEdge(uint8_t camData[], int start, int stop);
    point getMid(point prev, int y);
    point getMid(point prev, int y, int &firstEdge, int &secEdge);
    float getAngle(point p);
    int getDepth(int x, int startHeight);
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
        point direction, p;

        midLower = getMid(midLower, line1);

        depth_p = getDepth(res_x / 2, res_y - 1);
        p = pixel_to_point(res_x / 2, depth_p);
        dist = p.y;
        dist -= 60;
        depth_p = point_to_pixel(res_x / 2, dist).y;

        if (dist > midLower.y) {
            spee = true;
            pixy.setLamp(1, 1);
            direction = midUpper = getMid({0, 100}, depth_p);
        }
        else {
            pixy.setLamp(0, 0);
            spee = false;
            direction = midLower;
            direction.y -= 10;
        }




        if (dist > 200) canBrake = true;

        if (!braking) {
            if (dist < 100 && canBrake) {
                braking = true;
                canBrake = false;
            }
        }
        else {
            frameCounter++;

            if (frameCounter == 5) {
                braking = false;
                frameCounter = 0;
            }
        }
        
        


		setWheels(direction);
        if (motors) setSpeed(dist);
        else engine.setSpeed(0, 0);
        checkTrackSignals();
	};

    void stepObject() {
        int dist1, dist2;
        point dir, p1, p2;
        int lineOffset = 20, steeringOffset = 20, minDiff = 50;
        int object = false;
        uint8_t c1, c2;

        dist1 = getDepth(res_x / 2 - lineOffset, res_y - 1);
        p1 = pixel_to_point(res_x / 2 - lineOffset, dist1);
        dist2 = getDepth(res_x / 2 + lineOffset, res_y - 1);
        p2 = pixel_to_point(res_x / 2 + lineOffset, dist2);

        pixy.video.getRGB(res_x / 2 - lineOffset, dist1 - 10, &c1, false);
        pixy.video.getRGB(res_x / 2 + lineOffset, dist1 - 10, &c2, false);

        dir = midLower = getMid(midLower, line1);
        float angle = abs(atan2(midLower.x, midLower.y));

        if (abs(p1.y - p2.y) > minDiff) pixy.setLamp(1, 1);
        else pixy.setLamp(0, 0);

        if (object == 0) {
            // pixy.setLamp(0, 0);
            if (p1.y - p2.y > minDiff && c1 > 150)      object = 1;
            else if (p2.y - p1.y > minDiff && c2 > 150) object = 2;
        }

        if (object != 0) {
            // pixy.setLamp(1, 1);
            frameCounter++;

            if (object == 1)        dir = {-100, 100};
            else if (object == 2)   dir = {100, 100};
            
            if (frameCounter == 10) {
                object = 0;
                frameCounter = 0;
            }
        }

        // drawObject(lineOffset);
        setWheels(dir);
        engine.setSpeed(-0.4, -0.4);
        // engine.setSpeed(0, 0);
    };

    void drawObject(int lineOffset) {
    	char buf[32];
        uint8_t c;
        int dist1, dist2;

        for (int y = 0; y < res_y; y++) {
            pixy.video.getRGB(res_x / 2 - lineOffset, y, &c, false);
            sprintf(buf, "%d,", c);
            print_string(buf);
        }

        for (int y = 0; y < res_y; y++) {
            pixy.video.getRGB(res_x / 2 + lineOffset, y, &c, false);
            sprintf(buf, "%d,", c);
            print_string(buf);
        }

        dist1 = getDepth(res_x / 2 - lineOffset, res_y - 1);
        dist2 = getDepth(res_x / 2 + lineOffset, res_y - 1);
        sprintf(buf, "%d,%d,\r\n", dist1, dist2);
        print_string(buf);
    }

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
