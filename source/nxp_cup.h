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
}

/* Pixy 2 */
#include "Pixy/Pixy2SPI_SS.h"
//#include "Pixy/Pixy2line.h"

/* Own modules */
#include "servo_module.h"
#include "engine_module.h"

#define CAM_HEIGHT 39.5     // In cm, lens tot grond
#define LINE_DIST 30.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0  // In cm, lens tot wiel, horizontaal
#define CAM_ANGLE atan2(CAM_HEIGHT, LINE_DIST + LENS_WHEELS_DIST)
#define FOV_X (60 * M_PI / 180.0)
#define FOV_Y (40 * M_PI / 180.0)
#define WIDTH_MUL 1
#define STEERING_RANGE 0.733038

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

	double prev_angle;
	int prev_x0, prev_y0;
	point prev_p0, prev_p1;
    float mid1, mid2;
    float lineWidth = WIDTH_MUL * sqrt(CAM_HEIGHT * CAM_HEIGHT + (LINE_DIST + LENS_WHEELS_DIST) * (LINE_DIST + LENS_WHEELS_DIST));
    float threshold;
    int res_x, res_y, line1, line2;
    int firstEdge, secEdge, thirdEdge, fourEdge;
    uint8_t *camData1, *camData2;
    float speed;
    int finished;

    void set_steering_angle();
    void engine_kpod();
    void set_servo(double angle);
    int feature_left_right(int vector_start);
    bool clear_to_steer(int left_right);

    int findEdge(uint8_t camData[], int start, int stop);
    void setMid();
    void setWheels();
    void setSpeed();
    void setThreshold();
    void setCamData(int y, uint8_t camData[]);
    void printCamData();
    void checkFinish();

public:
    rover() {
        servo = new servoModule(servoPort1);
        pixy.init();
        pixy.setLED(0, 255, 0);

		servo->setRotation(0.0);
		engine.setSpeed(0.0, 0.0);

        res_x = pixy.frameWidth;
        res_y = pixy.frameHeight;
        line1 = res_y * 3/4;
        line2 = res_y * 1/4;
        mid1 = res_x / 2.0;
        mid2 = res_x / 2.0;

        camData1 = (uint8_t*)malloc(res_x * sizeof(uint8_t));
        camData2 = (uint8_t*)malloc(res_x * sizeof(uint8_t));

        threshold = 10;
    }

    ~rover()
    {
        delete servo;
        free(camData1);
        free(camData2);
    }

    void test_servo();
    void test_rgb();

    void step() {
        if(finished == 0){
            setCamData(line1, camData1);
            setCamData(line2, camData2);
            setMid();
            setWheels();
            setSpeed();
            // printCamData();
        }
        else {
            engine.setSpeed(0.0, 0.0);
        }
	};

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
