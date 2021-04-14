#ifndef ROVER_H_
#define ROVER_H_

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

#include "arm_math.h"
}

#include "camera.h"
#include "engine_module.h"
#include "servo_module.h"

#define MAX_SPEED 0.50
#define MIN_SPEED 0.43

class Rover
{
private:
    Camera camera = Camera();
    servoModule servo = servoModule(servoPort1);
    engineModule engine = engineModule();

    bool finished = false, spee = false, braking = false, canBrake = true;
    int lowerLine, upperLine, frameCounter = 0, lowerLeft, lowerRight, upperLeft, upperRight;
    point midLower = {0, 100}, midUpper = {0, 100};
    float angle, depth;

    void engine_kpod();
    void setBrakes();
    void setSpeed();
    void setWheels();
    void printDepth();
    void printBoth();
    void printMidLine();
    void printFinish();

public:
    Rover() {
		servo.setRotation(0.0);
		engine.setSpeed(0.0, 0.0);

        midLower = midUpper = {0, 100};

        lowerLine = camera.point_to_pixel(0, 50).y;
    }

    void step(bool motors) {
		setWheels();
        if (mSwitch_ReadSwitch(kSw3)) printFinish();
        
        if (mSwitch_ReadSwitch(kSw2)) setBrakes();
        if (mSwitch_ReadSwitch(kSw1)) setSpeed();

        else {
            engine.setSpeed(0, 0);
        }
	};

    // void stepObject() {
    //     int dist1, dist2;
    //     point dir, p1, p2;
    //     int lineOffset = 20, steeringOffset = 20, minDiff = 50;
    //     int object = false;
    //     uint8_t c1, c2;

    //           bottomLine = p.y;  dist1 = getDepth(res_x / 2 - lineOffset, res_y - 1);
    //     p1 = pixel_to_point(res_x / 2 - lineOffset, dist1);
    //     dist2 = getDepth(res_x / 2 + lineOffset, res_y - 1);
    //     p2 = pixel_to_point(res_x / 2 + lineOffset, dist2);

    //     pixy.video.getRGB(res_x / 2 - lineOffset, dist1 - 10, &c1, false);
    //     pixy.video.getRGB(res_x / 2 + lineOffset, dist1 - 10, &c2, false);

    //     dir = midLower = getMid(midLower, line1);
    //     float angle = abs(atan2(midLower.x, midLower.y));

    //     if (abs(p1.y - p2.y) > minDiff) pixy.setLamp(1, 1);
    //     else pixy.setLamp(0, 0);

    //     if (object == 0) {
    //         // pixy.setLamp(0, 0);
    //         if (p1.y - p2.y > minDiff && c1 > 150)      object = 1;
    //         else if (p2.y - p1.y > minDiff && c2 > 150) object = 2;
    //     }

    //     if (object != 0) {
    //         // pixy.setLamp(1, 1);
    //         frameCounter++;

    //         if (object == 1)        dir = {-100, 100};
    //         else if (object == 2)   dir = {100, 100};
            
    //         if (frameCounter == 10) {
    //             object = 0;
    //             frameCounter = 0;
    //         }
    //     }

    //     // drawObject(lineOffset);
    //     setWheels(dir);
    //     engine.setSpeed(-0.4, -0.4);
    //     // engine.setSpeed(0, 0);
    // };

    // void drawObject(int lineOffset) {
    // 	char buf[32];
    //     uint8_t c;
    //     int dist1, dist2;

    //     for (int y = 0; y < res_y; y++) {
    //         pixy.video.getRGB(res_x / 2 - lineOffset, y, &c, false);
    //         sprintf(buf, "%d,", c);
    //         print_string(buf);
    //     }

    //     for (int y = 0; y < res_y; y++) {
    //         pixy.video.getRGB(res_x / 2 + lineOffset, y, &c, false);
    //         sprintf(buf, "%d,", c);
    //         print_string(buf);
    //     }

    //     dist1 = getDepth(res_x / 2 - lineOffset, res_y - 1);
    //     dist2 = getDepth(res_x / 2 + lineOffset, res_y - 1);
    //     sprintf(buf, "%d,%d,\r\n", dist1, dist2);
    //     print_string(buf);
    // }

	// Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { engine.setSpeed(0, 0); };
};

#endif
