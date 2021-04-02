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
#define WIDTH_MUL 1
#define STEERING_RANGE 0.733038

class rover
{
private:
    Pixy2SPI_SS pixy;
    servoModule *servo;
    engineModule engine;

	double prev_angle;
	int prev_x0, prev_y0;
	point prev_p0, prev_p1;
    float mid;
    float lineWidth = WIDTH_MUL * sqrt(CAM_HEIGHT * CAM_HEIGHT + (LINE_DIST + LENS_WHEELS_DIST) * (LINE_DIST + LENS_WHEELS_DIST));
    float threshold;
    int res_x, res_y, line1, line2;
    int firstEdge, secEdge;
    uint8_t *camData;
    float speed;

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
    void setCamData(int y);
    void printCamData();

public:
    rover() {
        servo = new servoModule(servoPort1);
        pixy.init();
        pixy.setLED(0, 255, 0);

		servo->setRotation(0.0);
		engine.setSpeed(0.0, 0.0);

        res_x = pixy.frameWidth;
        res_y = pixy.frameHeight;
        line1 = res_y * (3/4);
        line2 = res_y * (1/4);
        mid = x / 2.0;

        camData = (uint8_t*)malloc(x * sizeof(uint8_t));
    }

    ~rover()
    {
        delete servo;
        free(camData);
    }

    void test_servo();
    void test_rgb();

    void step() {
        setCamData();
        setThreshold();
        setMid();
		setWheels();
        setSpeed();
        // printCamData();
	};

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
