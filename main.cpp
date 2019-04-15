#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

Serial terminal(USBTX, USBRX); // tx, rx
int prev_mid = 64;
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

int getMid(uint16_t* cameraDataVector){
    int firstEdge = -1, secEdge = 130, mean; 
    float threshold;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean = mean / 40;
    threshold = mean * 0.71;
    //terminal.printf("Threshold: %f\n", threshold);

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
    
    //terminal.printf("L: %d R: %d\r\n", firstEdge, secEdge);
    
    if((firstEdge >= 2) && (secEdge <= 127)){
        return ((firstEdge + secEdge) / 2);
    }else if ((firstEdge < 2) && (secEdge > 127)){
        return prev_mid;
    }else if(firstEdge < 2){
        return firstEdge + 50;
    }else if(secEdge > 127){
        return secEdge - 50;
    }
}

int setWheels(int mid){
    int neutral = 1210, dist = 150, pos;
    float offset, magic = 1.5;
    
    //terminal.printf("mid: %d\r\n", mid);
    
    offset = float(mid - 64) / 128.0;
    
    pos = neutral + offset * float(dist) * magic;
    
    terminal.printf("pos: %d", pos);
    
    if(pos > neutral + dist) pos = neutral + dist;
    else if(pos < neutral - dist) pos = neutral - dist;
    
    servo.pulsewidth_us(pos);

    return mid;
}


int main(){
    uint16_t cameraDataVector[128];
    int prev_mid = 64, mid = 64;
    MW_Camera camera = MW_Camera(0);
    servo.period(0.02f);
    
    camera.setExposureTime(10);
    
    motorA.setSpeed(0.1);
    motorB.setSpeed(-0.1);
    
    while(true){
        if (prev_mid > 62 && prev_mid < 66) {
            motorA.setSpeed(0.1);
            motorB.setSpeed(-0.1);
        }
        else {
            motorA.setSpeed(0.1);
            motorB.setSpeed(-0.1);
        }
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        wait_ms(100);
        mid = getMid(cameraDataVector);
        mid = 34;
        prev_mid = setWheels(mid);
    }

    return 0;   
}