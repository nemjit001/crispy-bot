/*
 * Copyright 2003-20xx Haute �cole ARC Ing�ni�rie, Switzerland. 
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Haute �cole ARC Ing�ni�rie nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS	uint8_t camData[res_x]; FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    nxpcup_ARC.c
 * @brief   Application entry point.
 */

#include "rover.h"

#define _NORMAL_RUN		0b00000000
#define _CHECK_BATTERY 	0b00000001
#define _CHECK_SERVO 	0b00000010
#define _PAUSE_ALL		0b00001000
#define _LINE_DIST		0b00000011
#define _CAM_DATA		0b00001100

void leds_off()
{
	mLeds_Write(kMaskLed1, kLedOff);
	mLeds_Write(kMaskLed2, kLedOff);
	mLeds_Write(kMaskLed3, kLedOff);
	mLeds_Write(kMaskLed4, kLedOff);
}

void Rover::setWheels() {
    point direction;

    midLower = camera.getMid(midLower, bottomLine);

    depth = camera.getDepth(0);

    if (depth - 60 > midLower.y && false) {
        spee = true;
        direction = midUpper = camera.getMid({0, 100}, depth - 60);
        // pixy.setLamp(1, 1);
    }
    else {
        // pixy.setLamp(0, 0);
        spee = false;
        direction = midLower;
        direction.y -= 10;
    }

    angle = atan2(direction.x, direction.y);

    servo.setServo(angle);
}

void Rover::setSpeed() {
	float speed1, speed2;

	if (depth > 100){
		speed1 = MAX_SPEED;
		speed2 = MAX_SPEED;
	} else if (braking) {
		speed1 = speed2 = 0;
	}
	else{
		speed1 = MIN_SPEED;
		speed2 = MIN_SPEED;

		if (angle < TO_RADIANS(-15)) {
            speed2 += 0.10; speed1 -= 0.05;
        }
		else if (angle > TO_RADIANS(15)) {
            speed1 += 0.10; speed2 -= 0.05;
        }
	}

	engine.setSpeed(-speed1, -speed2);
}

void Rover::setBrakes() {
    if (depth > 200) canBrake = true;

    if (!braking) {
        if (depth < 100 && canBrake) {
            braking = true;
            canBrake = false;
        }
    }
    else {
        frameCounter++;

        if (frameCounter == 5) {
            braking = false;
            frameCounter = 0;
        }
    }
}

void Rover::engine_kpod()
{ 
	engine.setSpeed(mAd_Read(kPot1), mAd_Read(kPot1));
}

void Rover::checkFinish(){
	int beamLeft =  camera.res_x - 20, beamRight = camera.res_x + 20;
	int finishLeft = camera.findEdgeVer(beamLeft, camera.res_y - 10, camera.res_y / 2), finishRight = findEdgeVer(beamRight, camera.res_y - 10, camera.res_y / 2);
	int pointBefore, pointAfter; 
	uint8_t c1, c2;

	if(finishLeft != -1){
		pointBefore = camera.getGrayScale(beamLeft, finishLeft - 10);
		pointAfter = camera.getGrayScale(beamLeft, finishLeft + 10);

		if((c1 - c2) < 10){
			camera.setLamp(true);
		}
	}
	else if(finishRight != -1) {
		pointBefore = camera.getGrayScale(beamRight, finishLeft - 10);
		pointAfter = camera.getGrayScale(beamRight, finishLeft + 10);

		if((c1 - c2) < 10){
			camera.setLamp(true);
		}
	}
	else {
		camera.setLamp(false);
	}
	return;
}

void display_battery_level()
{
	float voltage = mAd_Read(kUBatt);
	mLeds_Write(kMaskLed1, kLedOn);

	if (voltage > 6)
	{
		mLeds_Write(kMaskLed2, kLedOn);
	}
	if (voltage > 6.9)
	{
		mLeds_Write(kMaskLed3, kLedOn);
	}
	else if (voltage > 7.32)
	{
		mLeds_Write(kMaskLed4, kLedOn);
	}
}

uint8_t get_switch_state()
{
	return 0x0 |
	(mSwitch_ReadSwitch(kSw1) << 0) |
	(mSwitch_ReadSwitch(kSw2) << 1) |
	(mSwitch_ReadSwitch(kSw3) << 2) |
	(mSwitch_ReadSwitch(kSw4) << 3);
}
