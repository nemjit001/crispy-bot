#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#include <string.h>

#define SPEED 0.1

Serial terminal(USBTX, USBRX); // gebruikt usb connectie voor screen
int threshold;
float diff = 2000;
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');

float getMid(uint16_t* cameraDataVector){
    int firstEdge, secEdge, max;
    float mean = 0;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean /= 40.0;
    threshold = diff;

    //Setting first edge
    max = 0;
    for(int i = 64; i > 0; --i){
        if(abs(cameraDataVector[i] - cameraDataVector[i - 1]) > max){
            max = abs(cameraDataVector[i] - cameraDataVector[i - 1]);
            firstEdge = i;
        }
    }
    if (max < threshold) firstEdge = 0;

    max = 0;
    //Setting second edge
    for(int i = 64; i < 127; ++i){
        if(abs(cameraDataVector[i] - cameraDataVector[i + 1]) > max){
            max = abs(cameraDataVector[i] - cameraDataVector[i - 1]);
            secEdge = i;
        }
    }
    if (max < threshold) secEdge = 127;
    
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
    std::string dataString;
    for (int i = 0; i < 128; i++) {
        dataString += cameraDataVector[i] + ",";
    }
    terminal.printf("%s\r\n", dataString);

    /*
    std::stringstream ss;
    std::string temp = "";
    
    for (int i = 0; i < 128; i++) {
        ss << cameraDataVector[i] << ',';
    }
    
    temp += ss.str();
    terminal.printf("%s\r\n", temp.c_str());
    */
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