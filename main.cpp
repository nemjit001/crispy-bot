#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

Serial terminal(USBTX, USBRX); // tx, rx
int prev_mid = 64;
PwmOut servo = PwmOut(PTA12);

int getMid(uint16_t* cameraDataVector){
    int firstEdge = 1, secEdge = 128, mean; 
    float threshold;

    for(int i = 40; i < 80; ++i){
        mean += cameraDataVector[i];
    }

    mean = mean / 40;
    threshold = mean * 0.91;
    terminal.printf("Threshold: %f\n", threshold);

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

    if((firstEdge > 1) && (secEdge < 128)){
        int temp = ((firstEdge + secEdge) / 2);
        return ((firstEdge + secEdge) / 2);
    }else if(firstEdge < 3){
        
        return firstEdge + 50;
    }else if(secEdge > 124){
        return secEdge - 50;
    }else{
        return 64;
    }
}

int setWheels(int mid, int prev_mid){
   int serv_neutral = 1210, diff;
   float position = serv_neutral;
   terminal.printf("getMid: %d", mid);
   
   diff = 64 - mid;
   terminal.printf("diff: %d", diff);
   if(diff != 0){
        position = serv_neutral + (diff * 2.5);
   }else{
        position = serv_neutral;
    }
    
    if(position > 1370){
        position = 1370;
    }else if(position < 1050){
        position = 1050;
    }
        
    terminal.printf("position: %d", position);
    servo.pulsewidth_us(position);

   return mid;
}


int main(){
    uint16_t cameraDataVector[128];
    int prev_mid = 64;
    MW_Camera camera = MW_Camera(0);
    servo.period(0.02f);
    MW_DC_Motor motorA = MW_DC_Motor('A');
    MW_DC_Motor motorB = MW_DC_Motor('B');
    
    motorA.setSpeed(0.0);
    motorB.setSpeed(0.0);
    
    while(true){
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        prev_mid = setWheels(getMid(cameraDataVector), prev_mid);
        //setWheels(servo, );
        //setWheels(servo, );
        /*for(int i = 0; i < 128; ++i){
             terminal.printf("|%d|", cameraDataVector[i]);  
        }*/
        wait(1);  
    }
    
    
    return 0;   
}