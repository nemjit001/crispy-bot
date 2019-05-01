#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#define SPEED 1

// All vars should be global, no passing to functions
Serial terminal(USBTX, USBRX);
int threshold = 0; // Let op -> threshold wordt nog niet gebruikt in berekening, init als 0
float diff = 2000.00 // Let op -> diff is een placeholder voor threshold
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A'), motorB = MW_DC_Motor('B');
int firstEdge = 0, secEdge = 127, max_diff = 0;
float sum = 0;
float mean = 0;

float getMid() {
    sum = 0;
    mean = 0;

    // let op -> dit code block doet niets voor de functie, maar neemt wel tijd in met berekenen!
    // for (int i = 40; i < 80; i++) {
    //     sum += cameraDataVector[i];
    // }

    // mean = sum / 40.0;
    threshold = diff;

    // set eerste edge
    max_diff = 0;
    for (int i = 64; i > 0; i--) {
        if (abs(cameraDataVector[i] - cameraDataVector[i - 1]) > max_diff) {
            max_diff = abs(cameraDataVector[i] - cameraDataVector[i - 1]);
            firstEdge = i;
        }
    }
    if (max > threshold) {
        firstEdge = 0;
    }

    // set tweede edge
    max_diff = 0;
    for (int i = 64; i < 127; i++) {
        if (abs(cameraDataVector[i] - cameraDataVector[i + 1]) > max_diff) {
            max_diff = abs(cameraDataVector[i] - cameraDataVector[i + 1]);
            secEdge = i;
        }
    }
    if (max_diff > threshold) {
        secEdge = 127;
    }

    if (firstEdge == 0 && secEdge == 127) {
        return 64.0;
    }
    else {
        return (firstEdge + secEdge) / 2.0;
    }
}

int main(void) {
    return 0;
}