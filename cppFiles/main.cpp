#include "MW_Camera.h"
#include "MW_Servo.h"
#include "MW_DC_Motor.h"

motorA = MW_DC_Motor(A);
motorB = MW_DC_Motor(B);

servo = MW_Servo(0) //Might need to add min and max angle if error!!!!

camera = MW_Camera(0);

uint16_t* cameraDataVector;

int main(){
    camera.setExposureTime(100);

    while(true){
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);

        for(int i = 0; i < 128; ++i){
            printf("|%d|", cameraDataVector[i]);
        }
    }
}