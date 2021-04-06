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
#define LINE_DIST 30.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0  // In cm, lens tot wiel, horizontaal

#define FOV_X (60 * M_PI / 180.0)
#define FOV_Y (40 * M_PI / 180.0)
#define WIDTH_MUL 1
#define STEERING_RANGE 0.733038
#define THRESHOLD 10

#define MAX_SPEED 0.5
#define MIN_SPEED 0.45
#define SPEED_INCREASE_FACTOR 1.05
#define SPEED_DECREASE_FACTOR (1.0f / SPEED_INCREASE_FACTOR)

//#define CAM_ANGLE (atan2(CAM_HEIGHT, LINE_DIST + LENS_WHEELS_DIST) + (FOV_Y / 4.0))
#define CAM_ANGLE (70 * M_PI / 180.0)

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
    float lineWidth = WIDTH_MUL * sqrt(CAM_HEIGHT * CAM_HEIGHT + (LINE_DIST + LENS_WHEELS_DIST) * (LINE_DIST + LENS_WHEELS_DIST));
    int res_x, res_y, line1, line2;
    float midLower, midUpper, currentSpeed;
    uint8_t *camData1, *camData2, *frame;

    void engine_kpod();

    int findEdge(uint8_t camData[], int start, int stop);
    void getMid(uint8_t camData[], float &mid);
    int getDepth(int startHeight);
    void getCamData(int y, uint8_t camData[]);

    void setSpeed(int depth);
    void setWheels(float mid);

    void printCamData();
    
    void checkTrackSignals();

    point convert_point(int x, int y);

public:
    rover() {
        servo = new servoModule(servoPort1);
        pixy.init();
        pixy.setLED(0, 255, 0);

		servo->setRotation(0.0);
		engine.setSpeed(0.0, 0.0);

        res_x = pixy.frameWidth;
        res_y = pixy.frameHeight;

        midLower = midUpper = res_x / 2;

        line1 = res_y * 3/4;

        currentSpeed = MIN_SPEED;
    
        frame = (uint8_t*)malloc(res_x * res_y * sizeof(uint8_t));
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

    void step() {
        float usedMid = 0.0f;

        if (stopTrackSignal)
        {
            stop();
            return;
        }

        getCamData(line1, camData1);

        getMid(camData1, midLower);

        printf("mid lower: %d\n", (int)midLower);

        int depth = getDepth(line1);
        point p = convert_point(res_x / 2, depth);
        float dist = p.y;

        usedMid = midLower;

        if (dist > 80) {
            getCamData(depth + 5, camData2);
            getMid(camData2, midUpper);

            if (abs(midUpper - res_x / 2) < abs(midLower - res_x / 2))
                usedMid = midUpper;
        }
        
		setWheels(usedMid);
        setSpeed(depth);
        checkTrackSignals();
        // printCamData();
	};

	Pixy2SPI_SS &getPixy() { return this->pixy; };

    void stop() { this->engine.setSpeed(0.0f, 0.0f); };
};

#endif
