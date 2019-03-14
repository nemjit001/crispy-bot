#include "MW_Camera.h"
#include "MW_Servo.h"
#include "MW_DC_Motor.h"

DigitalOut red(LED_RED);
DigitalOut green(LED_GREEN);
DigitalOut blue(LED_BLUE);

MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

MW_Servo servo = MW_Servo(0);

MW_Camera camera = MW_Camera(0);

uint16_t* cameraDataVector;
int threshold;
float prev_mid = 64;

void updateCamData(){
    camera.updateCameraImage();
    camera.getCameraImage(cameraDataVector);
}

float getMid(){
    int firstEdge = 1, secEdge = 128, mean;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean = mean / 40;
    threshold = mean * 0.91;

    //Setting first edge
    for(i = 64; i > 0; --i){
        if(cameraDataVector[i] < threshold){
            firstEdge = cameraDataVector[i];
            break;
        }
    }

    //Setting second edge
    for(i = 64; i < 128; ++i){
        if(cameraDataVector[i] < threshold){
            secEdge = cameraDataVector[i];
            break;
        }
    }

    if((firstEdge > 1) && (secEdge < 128)){
        return ((firstEdge + secEdge) / 2)
    }else if(firstEdge < 3){
        return firstEdge + 50;
    }else if(secondEdge > 124){
        return secondEdge - 50;
    }else{
        return 64;
    }
}

void setWheels(){
    float KP, KDP, mid = getMid(), prev_diff;

    KP = 50;
    KDP = 25;
    
    diff = mid - 64;
    prev_diff = prev_mid - 64;

    int servo_val = (KP * diff) + (KDP (diff - prev_diff));
   
    servo_val = servo_val / 10;

    servo_val /= 100;

    servo.set_angle(servo_val);
}

int main(){
    camera.setExposureTime(100);
    motorA.setSpeed(0.1);
    motorB.setSpeed(0.1);

    

    while(true){
        updateCamData();
        setWheels();
    }
}