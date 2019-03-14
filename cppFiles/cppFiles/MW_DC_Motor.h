#include "MW_Camera.h"
#include "MW_Servo.h"
#include "MW_DC_Motor.h"

DigitalOut red(LED_RED);
DigitalOut green(LED_GREEN);
DigitalOut blue(LED_BLUE);

MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

MW_Servo servo = MW_Servo(0); //Might need to add min and max angle if error!!!!

MW_Camera camera = MW_Camera(0);

uint16_t* cameraDataVector = new uint16_t[128];

int main(){
    camera.setExposureTime(100);
    motorA.setSpeed(0.1);
    motorB.setSpeed(-0.1);

    

    while(true){
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);

        for(int i = 0; i < 128; ++i){
            if(cameraDataVector[i] > 40000){
                red = 1;
                green = 0;
                blue = 0;   
            }else if(cameraDataVector[i] > 2000 && cameraDataVector[i] < 40000){
                red = 0;
                green = 0;
                blue = 1; 
            }else if(cameraDataVector[i] < 2000){
                red = 0;
                green = 1;
                blue = 0; 
            }else{
                red = 0;
                green = 0;
                blue = 0;   
            }
        }
    }
}
