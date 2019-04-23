#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#define SPEED 0.125

Serial terminal(USBTX, USBRX); // gebruikt usb connectie voor screen
int threshold;
float diff = 2000;
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

float getMid(uint16_t* cameraDataVector){
    int firstEdge = 0, secEdge = 127;
    float mean = 0;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean /= 40.0;
    threshold = mean * 0.071;
    terminal.printf("%f\r\n", mean);

    //Setting first edge
    for(int i = 64; i > 0; --i){
        if(abs(cameraDataVector[i] - cameraDataVector[i - 1]) > threshold){
            firstEdge = i;
            break;
        }
    }

    //Setting second edge
    for(int i = 64; i < 127; ++i){
        if(abs(cameraDataVector[i] - cameraDataVector[i + 1]) > threshold){
            secEdge = i;
            break;
        }
    }
    
    return (firstEdge == 0 && secEdge == 127) ? 64.0 : ((firstEdge + secEdge) / 2.0);
}

void setWheels(float mid){
    int neutral = 1210, dist = 160, pos;
    float offset, magic = 30;
    
    offset = -float(mid - 64) / 64.0;
    
    offset = (offset >= 0) ? pow(offset, 2.5f) : -pow(-offset, 2.5f);
    
    pos = neutral + offset * dist * magic;

    if (pos > neutral + dist) pos = neutral + dist;
    else if (pos < neutral - dist) pos = neutral - dist;
    
    servo.pulsewidth_us(pos);
    
    //speed -= abs(offset) * 0.6;
    motorA.setSpeed(SPEED);
    motorB.setSpeed(-SPEED);
}


void drawCam(uint16_t* cameraDataVector) {
    for (int i = 0; i < 128; i++) {
        if (cameraDataVector[i] < threshold) terminal.printf("#");
        else terminal.printf("-");
    }
    terminal.printf("\r\n");
}


int main(){
    uint16_t cameraDataVector[128];
    float mid = 64.0;
    MW_Camera camera = MW_Camera(0);
    servo.period(0.02f);
    bool stopped = false;
    
    camera.setExposureTime(2);
    
    while(!stopped){
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        mid = getMid(cameraDataVector);
        setWheels(mid);
        
        //drawCam(cameraDataVector);
    }

    return 0;   
}