#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

Serial terminal(USBTX, USBRX); // tx, rx
int threshold;
float speed = 0.125;
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

int getMid(uint16_t* cameraDataVector, int prev_mid){
    int firstEdge = 0, secEdge = 127, mean = 0;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean = mean / 40;
    threshold = mean * 0.71;
    //terminal.printf("Threshold: %d\r\n", threshold);

    //Setting first edge
    for(int i = 64; i > 0; --i){
        if(cameraDataVector[i] < threshold){
            firstEdge = i;
            break;
        }
    }

    //Setting second edge
    for(int i = 64; i < 128; ++i){
        if(cameraDataVector[i] < threshold){
            secEdge = i;
            break;
        }
    }
    
    return ((firstEdge + secEdge) / 2);
}

int setWheels(int mid){
    int neutral = 1210, dist = 150, pos;
    float offset, magic = 60, min = 0.02;
    
    offset = -float(mid - 64) / 64.0;
    if (abs(offset) > min) offset = 0;
    
    pos = neutral + offset * float(dist) * magic;
    
    if(pos > neutral + dist) pos = neutral + dist;
    else if(pos < neutral - dist) pos = neutral - dist;
    
    servo.pulsewidth_us(pos);

    return mid;
}


void drawCam(uint16_t* cameraDataVector) {
    for (int i = 0; i < 128; i++) {
        if (cameraDataVector[i] < threshold) printf("#");
        else printf("-");
    }
    printf("\r\n");
}


int main(){
    uint16_t cameraDataVector[128];
    int prev_mid = 64, mid = 64;
    MW_Camera camera = MW_Camera(0);
    servo.period(0.02f);
    
    camera.setExposureTime(10);
    
    motorA.setSpeed(speed);
    motorB.setSpeed(-speed);
    
    while(true){
        if (prev_mid > 62 && prev_mid < 66) {
            motorA.setSpeed(-speed);
            motorB.setSpeed(speed);
        }
        else {
            motorA.setSpeed(speed);
            motorB.setSpeed(-speed);
        }
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        mid = getMid(cameraDataVector, prev_mid);
        prev_mid = setWheels(mid);
        drawCam(cameraDataVector);
    }

    return 0;   
}