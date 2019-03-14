#include "MW_Camera.h"
#include "MW_Servo.h"
#include "MW_DC_Motor.h"

MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');


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
    for(int i = 64; i > 0; --i){
        if(cameraDataVector[i] < threshold){
            firstEdge = cameraDataVector[i];
            break;
        }
    }

    //Setting second edge
    for(int i = 64; i < 128; ++i){
        if(cameraDataVector[i] < threshold){
            secEdge = cameraDataVector[i];
            break;
        }
    }

    if((firstEdge > 1) && (secEdge < 128)){
        return ((firstEdge + secEdge) / 2);
    }else if(firstEdge < 3){
        return firstEdge + 50;
    }else if(secEdge > 124){
        return secEdge - 50;
    }else{
        return 64;
    }
}

void setWheels(){
    float KP, KDP, mid = getMid(), prev_diff, diff;
    MW_Servo servo(0);

    KP = 50;
    KDP = 25;
    
    diff = mid - 64;
    prev_diff = prev_mid - 64;
    
    
    int kp = KP;
    int kdp = KDP;
    int servo_val = (kp * diff) + (kdp * (diff - prev_diff));
   
    servo_val = servo_val / 10;

    servo_val /= 100;

    servo.setAngle(servo_val);
}

int main(){
    camera.setExposureTime(100);
    motorA.setSpeed(0.1);
    motorB.setSpeed(0.1);

    PwmOut servo = PwmOut(PTA12);

    servo.period(0.02f);
    
    while(true){
        servo.pulsewidth_us(1380);
        wait(3);
        servo.pulsewidth_us(1220);
        wait(3);
        servo.pulsewidth_us(1060);
        wait(3);
    }
}