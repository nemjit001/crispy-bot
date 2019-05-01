#include "mbed.h"
#include "MW_Camera.h"
#include "MW_DC_Motor.h"

#define SPEED 1

// All vars should be global, no passing to functions (is lichter op het ram, ipv de hele tijd redeclaren blijven ze vast staan)
Serial terminal(USBTX, USBRX);
PwmOut servo = PwmOut(PTA12);
MW_DC_Motor motorA = MW_DC_Motor('A');
MW_DC_Motor motorB = MW_DC_Motor('B');
MW_Camera camera = MW_Camera(0);
bool stopped = false; // Geeft aan of de auto moet stoppen
int firstEdge = 0;
int secEdge = 127;
int max_diff = 0;
int threshold = 0; // Let op -> threshold wordt nog niet gebruikt in berekening, init als 0
int mid = 64.0; // midden van de baan
float sum = 0;
float mean = 0;
float diff = 2000.00; // Let op -> diff is een placeholder voor threshold
uint16_t cameraDataVector[128]; // De vector met de gevonden camera waarden.

void getMid() {
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
    if (max_diff > threshold) {
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

    // return een waarde op basis van de (niet) gevonden edges
    if (firstEdge == 0 && secEdge == 127) {
        mid = 64.0;
    }
    else {
        mid = (firstEdge + secEdge) / 2.0;
    }
}

void setWheels() {
    static int neutral = 1210, dist = 160, pos;
    static float offset, magic_num = 30;

    offset = -float(mid - 64) / 64.0;
    offset = (offset > 0) ? pow(offset, 2.5f) : -pow(-offset, 2.5f);

    pos = neutral + (offset * dist * magic_num);

    if (pos > neutral + dist) {
        pos = neutral + dist;
    }
    else if (pos < neutral - dist) {
        pos = neutral - dist;
    }

    servo.pulsewidth_us(pos);
}

int main(void) {
    camera.setExposureTime(2);
    servo.period(0.02f);
    motorA.setSpeed(SPEED);
    motorB.setSpeed(-SPEED);

    while (!stopped) {
        camera.updateCameraImage();
        camera.getCameraImage(cameraDataVector);
        getMid();
        setWheels();
    }

    return 0;
}