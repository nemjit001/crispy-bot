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
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include "nxp_cup.h"

#define _NORMAL_RUN		0b00000000
#define _CHECK_BATTERY 	0b00000001
#define _CHECK_SERVO 	0b00000010
#define _PAUSE_ALL		0b00001000
#define _LINE_DIST		0b00000011
#define _CAM_DATA		0b00001100

#define DEBUG_PRINT_ENABLED 1

#define K_MAIN_INTERVAL (100 / kPit1Period)

point rover::convert_point(int x, int y) {
    double angleX = FOV_X * (x / (double)res_x) - (FOV_X / 2.0);
    double angleY = CAM_ANGLE + (FOV_Y * ((res_y - y) / (double)res_y)) - (FOV_Y / 2.0);

    double pointY = CAM_HEIGHT * tan(angleY);
    
    double beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + pointY * pointY);
    
    double pointX = beamLength * tan(angleX);

    point p;
    p.x = pointX;
    p.y = pointY;

    return p;
}

void leds_off()
{
	mLeds_Write(kMaskLed1, kLedOff);
	mLeds_Write(kMaskLed2, kLedOff);
	mLeds_Write(kMaskLed3, kLedOff);
	mLeds_Write(kMaskLed4, kLedOff);
}

static float quadraticCurve(float offset, float a, float b) {
    int sign = (offset < 0 ? -1 : 1);

    return sign * a * pow(abs(offset), b);
}

void rover::getCamData(int y, uint8_t camData[])
{
	uint8_t c;

	for (int i = 0; i < res_x; i++) {
		pixy.video.getRGB(i, y, &c, 0);

		camData[i] = c;
	}
}

point rover::getMid(float &mid, int y) {
	uint8_t camData[res_x];
	point p;

	getCamData(y, camData);

	int firstEdge, secEdge;

	firstEdge = findEdge(camData, mid, 0);
	secEdge = findEdge(camData, mid, res_x - 1);

	if (firstEdge == -1 && secEdge != -1) {
		p = convert_point(secEdge, y);
		p.x -= 35;
	}
	else if (secEdge == -1 && firstEdge != -1) {
		p = convert_point(firstEdge, y);
		p.x += 35;
	}
	else if (firstEdge == -1 && secEdge == -1) {
		p = convert_point(res_x / 2, y);
	}
	else {
		p = convert_point((firstEdge + secEdge) / 2.0, y);
	}

	if (firstEdge == -1) firstEdge = 0;
	if (secEdge == -1) secEdge = res_x - 1;

	mid = (firstEdge + secEdge) / 2.0;

    return p;
}

void rover::setWheels(point p) {
	float deg, offset;

    deg = atan2(p.x, p.y);

    // offset = quadraticCurve(deg / STEERING_RANGE, 4 - ((speed - 0.42) * 30), 2);
	// offset = quadraticCurve(deg / STEERING_RANGE, 3, 2);

	// offset = quadraticCurve(deg / STEERING_RANGE, 3, 2);

	offset = deg / STEERING_RANGE;

    if (offset > 1) offset = 1;
    else if (offset < -1) offset = -1;

	offset = offset * 0.9 + 0.1;

	offset = 0;
    
    servo->setRotation(offset);
}

int rover::findEdge(uint8_t data[], int start, int stop) {
    int diff = 0, i = start;
    int sign = (start < stop) ? 1 : -1;

    while (i != stop) {
        diff = sign * (data[i] - data[i + 1]);
        if (diff >= THRESHOLD) {
            return i;
        }
        i += sign;
    }

    return -1;
}

void rover::setSpeed(int depth) {
	// if (depth == -1)
	// {
	// 	currentSpeed *= SPEED_INCREASE_FACTOR;

	// 	if (currentSpeed > MAX_SPEED)
	// 		currentSpeed = MAX_SPEED;
	// }
	// else
	// {
	// 	// TODO: lineare decrease hier?
	// 	currentSpeed = MIN_SPEED;
	// }

	float currentSpeed;

	if (depth > 100 || depth == -1) {
		currentSpeed = 0.5;
	}
	else currentSpeed = 0.42;

	currentSpeed = 0;

	engine.setSpeed(-currentSpeed, -currentSpeed);
}

int rover::getDepth(int startHeight) {
	uint8_t data[startHeight];
	uint8_t c;

	for (int i = 0; i < startHeight; i++) {
		pixy.video.getRGB(res_x / 2, startHeight - i - 1, &c, 0);
		data[i] = c;
	}

	int dist = findEdge(data, 0, startHeight - 1);

	if (dist == -1) return -1;

	return res_y - dist;

	// if (edge == -1) return -1;

	// point p = convert_point(res_x / 2, edge);
	// return p.y;
}

void rover::checkTrackSignals()
{
	// TODO: check track for codes
	// |  -   -  | == stop
	// |   |||   | == start fast track
	// |  || ||   | == stop fast track

	// float32_t *percent_camdata = (float32_t *)calloc(sizeof(float32_t), res_x);
	// float32_t *out_fft = (float32_t *)calloc(sizeof(float32_t), res_x * 2);
	// arm_rfft_fast_instance_f32 fft_instance;

	// arm_status status = arm_rfft_fast_init_f32(&fft_instance, res_x);

	// if (status == ARM_MATH_ARGUMENT_ERROR)
	// 	return;

	// for (int i = 0; i < res_x; i++)
	// 	percent_camdata[i] = (float32_t)(camData1[i]) / 255.0f;

	// arm_rfft_fast_f32(&fft_instance, percent_camdata, out_fft, 0);

	// for (int i = 0; i < res_x * 2; i++)
	// {
	// 	printf("%d,", (int)out_fft[i]);
	// }
	// printf("\n");

	// free(percent_camdata);
	// free(out_fft);

	return;
}

void rover::engine_kpod()
{ 
	engine.setSpeed(mAd_Read(kPot1), mAd_Read(kPot1));
	printf("kPot1: %d\n", (int)(mAd_Read(kPot1) * 1000));
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

void rover::test_servo()
{
	static float duty = 0.0;
	bool 	btn1 = mSwitch_ReadPushBut(kPushButSW1),
			btn2 = mSwitch_ReadPushBut(kPushButSW2);

	if (btn1 && btn2)
	{
		duty = 0;
		servo->setRotation(duty);
		return;
	}

	if (btn1)
	{
		duty += 0.05;
		if (duty > 1)
			duty = 1;
	}
	else if (btn2)
	{
		duty -= 0.05;
		if (duty < -1)
			duty = -1;
	}

	servo->setRotation(duty);
}

void rover::printLineDist() {
	getCamData(res_y / 2, camData1);

	char buf[32];

	sprintf(buf, "%d,", res_x);
    print_string(buf);

    for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData1[i]);
        print_string(buf);
    }

	print_string("\r\n");


	// char buf[16];
	// int depth = getDepth(line1);

	// sprintf(buf, "%d\r\n", depth);
	// print_string(buf);
}

void rover::printCamData() {
	// Print mid, firstEdge en secEdge van beide lijnen, en het verste punt (dist)
	// mid1, firstEdge1, secEdge1, dist1, mid2, firstEdge2, secEdg2, dist2, maxdistX, maxdistY

	point p, p1, p2;
	int mid1, mid2;
	char buf[128];

	// Lower line
	getCamData(line1, camData1);

	int firstEdge, secEdge;

	firstEdge = findEdge(camData1, midLower, -1);
	secEdge = findEdge(camData1, midLower, res_x);

	if (firstEdge == -1) firstEdge = 0;
	if (secEdge == -1) secEdge = res_x - 1;

	mid1 = (firstEdge + secEdge) / 2;
	
	p = convert_point(mid1, line1);
	p1 = convert_point(firstEdge, line1);
	p2 = convert_point(secEdge, line1);

	sprintf(buf, "%d,%d,%d,%d,", (int)p.x, (int)p1.x, (int)p2.x, (int)p.y);
	print_string(buf);


	// Dist
	int depth = getDepth(line1);

	printf("depth: %d\n", depth);

	if (depth == -1) {
		print_string("-1,-1,-1,-1,-1,-1,\r\n");
		return;
	}

		
	// Upper line
	getCamData(depth + 5, camData2);

	firstEdge = findEdge(camData2, midUpper, -1);
	secEdge = findEdge(camData2, midUpper, res_x);

	if (firstEdge == -1) firstEdge = 0;
	if (secEdge == -1) secEdge = res_x - 1;

	mid2 = (firstEdge + secEdge) / 2;

	p = convert_point(mid2, depth + 5);
	p1 = convert_point(firstEdge, depth + 5);
	p2 = convert_point(secEdge, depth + 5);

	sprintf(buf, "%d,%d,%d,%d,", (int)p.x, (int)p1.x, (int)p2.x, (int)p.y);
	print_string(buf);

	p = convert_point(res_x / 2, depth);

	sprintf(buf, "%d,%d,\r\n", (int)p.x, (int)p.y);
	print_string(buf);
}

uint8_t get_switch_state()
{
	return 0x0 |
	(mSwitch_ReadSwitch(kSw1) << 0) |
	(mSwitch_ReadSwitch(kSw2) << 1) |
	(mSwitch_ReadSwitch(kSw3) << 2) |
	(mSwitch_ReadSwitch(kSw4) << 3);
}

void board_device_setup()
{
	//--------------------------------------------------------------------
	// Device and card setup
	//--------------------------------------------------------------------
	// PLL Config --> CPU 100MHz, bus and periph 50MHz
	mCpu_Setup();

	// Config and start switches and pushers
	mSwitch_Setup();
	mSwitch_Open();

	// Config and start of LEDs
	mLeds_Setup();
	mLeds_Open();

	// Config and start of ADC
	mAd_Setup();
	mAd_Open();

	// Config and start of SPI
	mSpi_Setup();
	mSpi_Open();

	// Config and start non-blocking delay by PIT
	mDelay_Setup();
	mDelay_Open();

	// Timer Config for Speed Measurement and PWM Outputs for Servos
	mTimer_Setup();
	mTimer_Open();

	// Setup FXOS8700CQ
	mAccelMagneto_Setup();
	mAccelMagneto_Open();

	// Setup FXAS21002C
	mGyro_Setup();
	mGyro_Open();

	// Config and start of the DAC0 used to drive the driver LED lighting
	mDac_Setup();
	mDac_Open();

	// Setup and start of motor and servo PWM controls and speed measurement
	mTimer_Setup();
	mTimer_Open();

	// Enable IRQ at the CPU -> Primask
	__enable_irq();

	// UART 4 monitoring image
	mRs232_Setup();
	mRs232_Open();
}

/*
 * @brief   Application entry point.
 */
int main(void)
{

	board_device_setup();
	// print_string("INITIALIZING BOARD\n\r\0");

	// bitmask containing board switch state
	static uint8_t switch_state = 0x0;
	// main loop delay
	static Int16 delay = 0;

	static rover car;

	// line camera
	Pixy2SPI_SS test_pixy = car.getPixy();

	mDelay_ReStart(kPit1, delay, K_MAIN_INTERVAL);

	mLeds_Write(kMaskLed1, kLedOn);

	for (;;)
	{
		if(!mDelay_IsDelayDone(kPit1, delay))
			continue;
		
		mDelay_ReStart(kPit1, delay, K_MAIN_INTERVAL);

		leds_off();

		switch_state = get_switch_state();

		switch (switch_state)
		{
		case _NORMAL_RUN:
			car.step();
			break;
		case _CHECK_BATTERY:
			car.stop();
			display_battery_level();
			break;
		case _CHECK_SERVO:
			mLeds_Write(kMaskLed1, kLedOn);
			car.stop();
			car.test_servo();
			break;
		case _PAUSE_ALL:
			mLeds_Write(kMaskLed2, kLedOn);
			car.stop();
			__asm("nop");
			break;
		case _LINE_DIST:
			car.stop();
			car.printLineDist();
			break;
		case _CAM_DATA:
			car.step();
			car.printCamData();
			break;
		default:
			break;
		}
	}

	return 0;
}
