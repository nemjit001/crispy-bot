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

    midLower = camera.getMid(midLower, lowerLine, lowerLeft, lowerRight);

    depth = camera.getDepth(0);
	upperLine = camera.point_to_pixel(0, depth - 60).y;

	midUpper = camera.getMid({0, 100}, upperLine, upperLeft, upperRight);

    if (upperLine < lowerLine) {
        spee = true;
        direction = midUpper;
    }
    else {
        spee = false;
        direction = midLower;
        direction.y -= 15;
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

void Rover::printDepth() {
	uint8_t camData[camera.res_y];
	char str[16];

	camera.getCol(camera.res_x / 2, camData);

	for (int i = 0; i < camera.res_y; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	point p = camera.point_to_pixel(0, depth);

	sprintf(str, "%d,\r\n", (int)p.y);
	print_string(str);
}

void Rover::printMidLine() {
	uint8_t camData[camera.res_x];
	char str[16];

	camera.getRow(camera.res_y / 2, camData);

	for (int i = 0; i < camera.res_x; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	print_string("\r\n");
}

void Rover::printBoth() {
	uint8_t camData[camera.res_x];
	char str[32];
	int mid;

	// Lower
	camera.getRow(lowerLine, camData);

	for (int i = 0; i < camera.res_x; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	mid = (int)camera.point_to_pixel(midLower).x;

	sprintf(str, "%d,%d,%d,", mid, lowerLeft, lowerRight);
	print_string(str);

	// Upper
	camera.getRow(upperLine, camData);

	for (int i = 0; i < camera.res_x; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	mid = (int)camera.point_to_pixel(midUpper).x;

	sprintf(str, "%d,%d,%d,%d,%d,\r\n", mid, upperLeft, upperRight, (int)depth, spee);
	print_string(str);
}

void Rover::printFinish() {
	uint8_t camData[camera.res_y];
	char str[32];

	camera.getCol(camera.res_x / 2 - 10, camData);

	for (int i = 0; i < camera.res_y; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	camera.getCol(camera.res_x / 2 + 10, camData);

	for (int i = 0; i < camera.res_y; i++) {
		sprintf(str, "%d,", camData[i]);
		print_string(str);
	}

	int finishLeft = camera.findEdgeVer(-10, camera.res_y - 10, camera.res_y / 2);
	int finishRight = camera.findEdgeVer(10, camera.res_y - 10, camera.res_y / 2);
	sprintf(str, "%d,%d,\r\n", finishLeft, finishRight);
	print_string(str);
}

uint8_t get_switch_state()
{
	return 0x0 |
	(mSwitch_ReadSwitch(kSw1) << 0) |
	(mSwitch_ReadSwitch(kSw2) << 1) |
	(mSwitch_ReadSwitch(kSw3) << 2) |
	(mSwitch_ReadSwitch(kSw4) << 3);
}
