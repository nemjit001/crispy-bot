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

#define DEBUG_PRINT_ENABLED 1

#define K_MAIN_INTERVAL (100 / kPit1Period)

point rover::convert_point(int x, int y) {
    double angleX = FOV_X * (x / (double)res_x) - (FOV_X / 2.0);
    double angleY = CAM_ANGLE - ((FOV_Y * ((res_y - y) / (double)res_y)) - (FOV_Y / 2.0));

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
	uint8_t r, g, b;

	for (int i = 0; i < res_x; i++) {
		pixy.video.getRGB(i, y, &r, &g, &b, 0);
		camData[i] = (r + g + b) / 3;
	}
}

float rover::getMid(uint8_t camData[], float mid) {
	int firstEdge, secEdge;

	firstEdge = findEdge(camData, mid, -1);
	secEdge = findEdge(camData1, mid, res_x);

	if (firstEdge == -1) firstEdge = 0;
	if (secEdge == -1) secEdge = res_x - 1;

    return (firstEdge + secEdge) / 2.0;
}

void rover::setWheels(float mid) {
	float offset, deg;

    offset = (mid - res_x / 2.0) / (res_x / 2.0);
    offset *= (lineWidth / 2.0);

    deg = atan2(offset, LINE_DIST);

    // offset = quadraticCurve(deg / STEERING_RANGE, 4 - ((speed - 0.42) * 30), 2);
	offset = quadraticCurve(deg / STEERING_RANGE, 3, 2);
    if (offset > 1) offset = 1;
    else if (offset < -1) offset = -1;

	offset = offset * 0.9 + 0.1;
    
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
	float speed;

	if (depth == -1) speed = 0.50;
	else speed = 0.40;

	engine.setSpeed(-speed, -speed);
}

int rover::getDepth(int startHeight) {
	uint8_t data[startHeight];
	uint8_t r, g, b;

	for (int i = 0; i < startHeight; i++) {
		pixy.video.getRGB(res_x / 2, startHeight - i - 1, &r, &g, &b, 0);
		data[i] = (r + g + b) / 3;
	}

	return findEdge(data, 0, startHeight - 1);

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

	float32_t *percent_camdata = (float32_t *)calloc(sizeof(float32_t), res_x);
	float32_t *out_fft = (float32_t *)calloc(sizeof(float32_t), res_x * 2);
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

	free(percent_camdata);
	free(out_fft);


	// if(secEdge - firstEdge < 50) stopTrackSignal = 1;

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

// void rover::printCamData() {
// 	char buf[32];

// 	sprintf(buf, "%d,", res_x);
//     print_string(buf);

//     for (int i = 0; i < res_x; i++) {
// 		sprintf(buf, "%d,", camData1[i]);
//         print_string(buf);
//     }

//     sprintf(buf, "%d,%d,%d,\r\n", int(mid1), firstEdge, secEdge);
// 	print_string(buf);
// }

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
		default:
			break;
		}
	}

	return 0;
}
