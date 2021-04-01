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
#define _CHECK_ENGINE	0b00000011
#define _CHECK_CAM		0b00000100
#define _PAUSE_ALL		0b00001000

#define STEERING_RANGE 84
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


void rover::set_steering_angle()
{
	engine_kpod();
	static servoModule servo(servoPort1);

	pixy.line.getAllFeatures();
	for(int i = 0; i < pixy.line.numVectors; i++){
		pixy.line.vectors[i].print();

		if(pixy.line.vectors[0].m_y1 > pixy.line.vectors[0].m_y0){
			int temp = pixy.line.vectors[0].m_x0;
			int temp2 = pixy.line.vectors[0].m_y0;
			pixy.line.vectors[0].m_x0 = pixy.line.vectors[0].m_x1;
			pixy.line.vectors[0].m_y0 = pixy.line.vectors[0].m_y1;
			pixy.line.vectors[0].m_x1 = temp;
			pixy.line.vectors[0].m_y1 = temp2;
		}
	}

	point p0 = convert_point(pixy.line.vectors[0].m_x0, pixy.line.vectors[0].m_y0);
	point p1 = convert_point(pixy.line.vectors[0].m_x1, pixy.line.vectors[0].m_y1);

	char data1[64];
	double angle1 = vector_to_angle(p0.x, p0.y, p1.x, p1.y);
	sprintf(data1, "vector: (%d %d) (%d %d) angle: %d reso: %d x %d\r\n", (int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, (int)angle1, pixy.frameWidth, pixy.frameHeight);

	print_string(data1);

	char data2[64];
	sprintf(data2, "------------------------------------------------\n\r");

	print_string(data2);

	angle1 = quadraticCurve(angle1 / STEERING_RANGE, 1.4, 1.7);

	set_servo(angle1);
	prev_angle = angle1;
	prev_p0 = p0;
	prev_p1 = p1;
	prev_x0 = pixy.line.vectors[0].m_x0;
	prev_y0 = pixy.line.vectors[0].m_y0;
}

void rover::set_servo(double angle){
	servo->setRotation(angle);
}

//Returns 1 if starting point of vector is on the right of the centre and 0 if it is on the left.
int rover::feature_left_right(int vector_start){
	int centre = pixy.frameWidth / 2;

	if(vector_start < centre) return 0;
	else if (vector_start > centre) return 1;
}

void rover::engine_kpod()
{ 
	engine.setSpeed(mAd_Read(kPot1), mAd_Read(kPot1));
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
	// PLL Config --> CPU 100MHz, bus and periph 50MH z
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
	print_string("INITIALIZING BOARD\n\r\0");

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
			display_battery_level();
			break;
		case _CHECK_SERVO:
			mLeds_Write(kMaskLed1, kLedOn);
			car.test_servo();
			break;
		/*case _CHECK_ENGINE:
			mLeds_Write(kMaskLed4, kLedOn);
			test_engines();
			break;
		case _CHECK_CAM:
			mLeds_Write(kMaskLed3, kLedOn);
			cam_test(test_pixy);*/
			break;
		case _PAUSE_ALL:
			mLeds_Write(kMaskLed2, kLedOn);
			__asm("nop");
			break;
		default:
			break;
		}
	}

	return 0;
}
