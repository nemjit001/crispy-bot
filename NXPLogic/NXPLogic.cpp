#include "NXPLogic.h"

Serial pc(USBTX, USBRX);

static float quadraticCurve(float, float, float);

bool NXPLogic::step() {
    camera.updateCameraImage();
    camera.getCameraImage(camData);

    switch (mode) {
    case STANDARD:
        threshold = 1000 + (0.075 * getCamDataAverage());
        mid = calcMid();
        setWheels();
        setSpeed();
        break;
    case PRINT:
        mid = calcMid();
        printData();
        break;
    case TEST:
        testLoop();
        break;
    }

    return true;
}

float NXPLogic::calcMid() {
    firstEdge = findEdge(mid - 1, -1);
    if (firstEdge == -1) firstEdge = 0;

    secEdge = findEdge(mid, CAMDATA_SIZE);
    if (secEdge == -1) secEdge = CAMDATA_SIZE - 1;

    return ((firstEdge + secEdge) / 2.0);
}

int NXPLogic::findEdge(int start, int stop) {
    int diff = 0, i = start;
    int sign = (start < stop) ? 1 : -1;

    while (i != stop) {
        diff = sign * (camData[i] - camData[i + 1]);
        if (diff >= threshold) {
            return i;
        }
        i += sign;
    }

    return -1;
}

void NXPLogic::setWheels() {
    float offset, deg;

    offset = -(mid - 63.5) / 63.5;
    offset *= (lineWidth / 2.0);

    deg = atan2(offset, LINE_DIST);

    offset = quadraticCurve(deg / STEERING_RANGE, 1.4, 1.7);
    if (offset > 1) offset = 1;
    else if (offset < -1) offset = -1;

    pos = neutral + offset * radius;
    
    servo.pulsewidth_us(pos);
}

void NXPLogic::setSpeed() {
    float offset = abs(pos - neutral) / float(radius);

    float speed = (1 - abs(offset)) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
    motorA.setSpeed(speed);
    motorB.setSpeed(-speed);
}

NXPLogic::NXPLogic() {
    camera.setExposureTime(EXPOSURE_TIME);
    servo.period(SERVO_PERIOD);
    servo.pulsewidth_us(neutral);
    pc.baud(460800);

    switch (mode) {
    case standard:
        motorA.setSpeed(MIN_SPEED);
        motorB.setSpeed(-MIN_SPEED);
        break;
    case print:
        motorA.setSpeed(0);
        motorB.setSpeed(0);
        break;
    case test:
        motorA.setSpeed(0);
        motorB.setSpeed(0);
        break;
    }
}

static float quadraticCurve(float offset, float a, float b) {
    int sign = (offset < 0 ? -1 : 1);

    return sign * a * pow(abs(offset), b);
}

float NXPLogic::getCamDataAverage() {
    float result = 0;
    for (int i = 0; i < CAMDATA_SIZE; i++) {
        result += camData[i];
    }
    result /= CAMDATA_SIZE;
    return result;
}

void NXPLogic::printData() {
    for (int i = 0; i < CAMDATA_SIZE; i++) {
        pc.printf("%d,", camData[i]);
    }
    pc.printf("%d,%d,%d,", int(mid), firstEdge, secEdge);
    pc.printf("\r\n");
}

void NXPLogic::testLoop() {
    if (stepCounter % 200 == 0) {
        testState = (testState + 1) % 4;
    }

    switch (testState) {
    case 0:
        servo.pulsewidth_us(neutral + radius);
        break;
    case 1:
        servo.pulsewidth_us(neutral - radius);
        break;
    default:
        servo.pulsewidth_us(neutral);
        break;
    }

    stepCounter++;
}