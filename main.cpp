#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

Serial terminal(USBTX, USBRX); // tx, rx
int threshold, min = 1210, max = 1210;
float diff = 2000;
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

    //Setting first edge
    for(int i = 64; i > 0; --i){
        if(abs(cameraDataVector[i] - cameraDataVector[i - 1]) > diff){
            firstEdge = i;
            break;
        }
    }

    //Setting second edge
    for(int i = 64; i < 127; ++i){
        if(abs(cameraDataVector[i] - cameraDataVector[i + 1]) > diff){
            secEdge = i;
            break;
        }
    }
    
    return ((firstEdge + secEdge) / 2);
}

int setWheels(int mid){
    int neutral = 1210, dist = 160, pos;
    float offset, magic = 30, speed = 0.125;
    
    offset = -float(mid - 64) / 64.0;
    
    offset = (offset >= 0) ? pow(offset, 2.5f) : -pow(-offset, 2.5f);
    
    pos = neutral + offset * dist * magic;

    if(pos > neutral + dist) pos = neutral + dist;
    else if(pos < neutral - dist) pos = neutral - dist;
    
    servo.pulsewidth_us(pos);
    
    //speed -= abs(offset) * 0.6;
    motorA.setSpeed(speed);
    motorB.setSpeed(-speed);

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
    bool stopped = false;
    
    camera.setExposureTime(2);
    
    while(!stopped){
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        mid = getMid(cameraDataVector, prev_mid);
        prev_mid = setWheels(mid);
        //drawCam(cameraDataVector);
    }

    return 0;   
}