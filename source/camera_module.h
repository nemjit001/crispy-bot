#ifndef __CAMERA_MODULE_H__
#define __CAMERA_MODULE_H__

extern "C"
{
#include <stdlib.h>
#include "Modules/mDelay.h"
}

/* Pixy 2 */
#include "Pixy/Pixy2SPI_SS.h"

class cameraModule
{
private:
    Pixy2SPI_SS pixy;

public:
    cameraModule()
    {
        pixy.init();
        pixy.setLED(0, 255, 0);
    }

    int getVectors(Vector **vectors);
};

#endif // __CAMERA_MODULE_H__
