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

#define DEBUG_PRINT_ENABLED 1

#define K_MAIN_INTERVAL (100 / kPit1Period)

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

void rover::setCamData(int y, )
{
	uint8_t r, g, b;

	for (int i = 0; i < res_x; i++) {
		pixy.video.getRGB(i, y, &r, &g, &b, 0);
		camData[i] = (r + g + b) / 3;
	}
}

void rover::setMid() {
	firstEdge = findEdge(camData, mid, -1);
	secEdge = findEdge(camData, mid, x);

	if (firstEdge == -1) firstEdge = 0;
	if (secEdge == -1) secEdge = x - 1;

    mid = (firstEdge + secEdge) / 2.0;
}

void rover::setWheels() {
	float offset, deg;

    offset = (mid - x / 2.0) / (x / 2.0);
    offset *= (lineWidth / 2.0);

    deg = atan2(offset, LINE_DIST);

    offset = quadraticCurve(deg / STEERING_RANGE, 4 - ((speed - 0.42) * 30), 2);
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
        if (diff >= threshold) {
            return i;
        }
        i += sign;
    }

    return -1;
}

void rover::setSpeed() {
	uint8_t data[y];
	uint8_t r, g, b;

	for (int i = 0; i < y; i++) {
		pixy.video.getRGB(x / 2, y - i - 1, &r, &g, &b, 0);
		data[i] = (r + g + b) / 3;
	}

	int edge = findEdge(data, 0, y - 1);

	if (edge == -1) speed = 0.50;
	else {
		float temp = edge / float(y);
		speed = 0.42 + 0.05 * temp;
	}

	speed = 0.42;

	engine.setSpeed(-speed, -speed);
}

void rover::setThreshold() {
	int total = 0;

	for (int i = 0; i < x - 1; i++) {
		total += camData[i] - camData[i + 1];
	}

	// threshold = 10 + 0.025 * total / (float)x;
	threshold = 10;
}

// void rover::set_servo(double angle){
// 	servo->setRotation(angle * STEP_PER_DEGREE);
// }

//Returns 1 if starting point of vector is on the right of the centre and 0 if it is on the left.
int rover::feature_left_right(int vector_start){
	int centre = pixy.frameWidth / 2;

	if(vector_start < centre) return 0;
	else if (vector_start > centre) return 1;
}

void rover::engine_kpod()
{ 
	engine.setSpeed(mAd_Read(kPot1), mAd_Read(kPot1));
	printf("kPot1: %d\n", (int)(mAd_Read(kPot1) * 1000));
}

bool rover::clear_to_steer(int angle){
	if (angle < 0){
		for(int i = 1; i < pixy.line.numVectors; i++){
			if(feature_left_right(pixy.line.vectors[i].m_x0) == 0 && pixy.line.vectors[i].m_y1 < (pixy.frameHeight * 0.75)) return false;
			else return true;
		}
	}
	else if (angle > 0) {
		for(int i = 1; i < pixy.line.numVectors; i++){
			if(feature_left_right(pixy.line.vectors[i].m_x0) == 1 && pixy.line.vectors[i].m_y1 < (pixy.frameHeight * 0.75)) return false;
			else return true;
		}
	}
	
	return false;
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

void rover::printCamData() {
	char buf[32];

	sprintf(buf, "%d,", x);
    print_string(buf);

    for (int i = 0; i < x; i++) {
		sprintf(buf, "%d,", camData[i]);
        print_string(buf);
    }

    sprintf(buf, "%d,%d,%d,\r\n", int(mid), firstEdge, secEdge);
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
		default:
			break;
		}

		// car.test_rgb();
	}

	return 0;
}
