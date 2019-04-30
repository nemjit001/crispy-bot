#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#include <sstream>
#include <string>

void drawCam(uint16_t* cameraDataVector) {
   std::stringstream ss;
   std::string temp;
   
   for (int i = 0; i < 128; i++) {
       ss << cameraDataVector[i] << ',';
   }
   
   temp = ss.str();
   terminal.printf("%s\r\n", temp.c_str());
}

int main(void) {
    uint_t camVector[128];
    MW_Camera camera = MW_Camera(0);
    while(true) {
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        drawCam(camVector);
    }
}