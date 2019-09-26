#include "NXPLogic.h"

bool NXPLogic::step() {
    camera.updateCameraImage();
    camera.getCameraImage(camData);
    filter.applyGaussianBlur(camData, camDataSize);
    mid = calcMid();
    setWheels();

    return true;
}

int NXPLogic::calcMid() {
    int firstEdge, secEdge;

    firstEdge = findEdge(mid, 0);
    if (firstEdge == -1) firstEdge = 0;

    secEdge = findEdge(mid, camDataSize);
    if (secEdge == -1) secEdge = camDataSize;

    return ((firstEdge + secEdge) / 2);
}

int NXPLogic::findEdge(int start, int stop) {
    int diff = 0, max_diff = 0, edge = 0;

    for (int i = start; i < stop; start < stop ? i++ : i--) {
        diff = getCamDataDiff(i);
        if (diff > max_diff) {
            max_diff = diff;
            edge = i;
        }
    }

    if (max_diff < threshold) return -1;
    return edge;
}

int NXPLogic::getCamDataDiff(int i) {
    return abs(camData[i] - camData[i + 1]);
}

void NXPLogic::setWheels() {
    int neutral = steering.neutral;
    int radius = steering.radius;
    int pos = neutral;
    float offset, magic = 30;
    
    offset = -float(mid - 64) / 64.0;
    
    offset = (offset >= 0) ? pow(offset, 2.5f) : -pow(-offset, 2.5f);
    
    pos = neutral + offset * radius * magic;

    if (pos > neutral + radius) pos = neutral + radius;
    else if (pos < neutral - radius) pos = neutral - radius;
    
    servo.pulsewidth_us(pos);
}

NXPLogic::NXPLogic() {
    camera.setExposureTime(EXPOSURE_TIME);
    servo.period(SERVO_PERIOD);
    motorA.setSpeed(SPEED);
    motorB.setSpeed(-SPEED);
}