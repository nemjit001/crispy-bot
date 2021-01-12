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

extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "Utils/def.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"

#include "Applications/gInput.h"
#include "Applications/gCompute.h"
#include "Applications/gOutput.h"
}

/* Pixy 2 */
#include "Pixy/Pixy2SPI_SS.h"
#include "Pixy/Pixy2Line.h"

/* Own modules */
#include "servo_module.h"
#include "engine_module.h"
#include "Utils/util.h"

#define _NORMAL_RUN		0b00000000
#define _CHECK_BATTERY 	0b00000001
#define _CHECK_SERVO 	0b00000010
#define _CHECK_ENGINE	0b00000011
#define _PAUSE_ALL		0b00001000

#define DEBUG_PRINT_ENABLED 1

#define K_MAIN_INTERVAL (100 / kPit1Period)

// line camera
static Pixy2SPI_SS pixy;

void leds_off()
{
	mLeds_Write(kMaskLed1, kLedOff);
	mLeds_Write(kMaskLed2, kLedOff);
	mLeds_Write(kMaskLed3, kLedOff);
	mLeds_Write(kMaskLed4, kLedOff);
}

void normal_run()
{
	pixy.line.getAllFeatures(LINE_ALL_FEATURES, 1);

  	char test_str2[15];
  	sprintf(test_str2, "numVectors: %d\n\r", pixy.line.numVectors);
  	print_string(test_str2);

	for(int i = 0; i < pixy.line.numVectors; i ++)
	{
		point p0 = convert_point(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0);
		point p1 = convert_point(pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);

		char data[64];
		sprintf(data, "(%d, %d) -> (%d, %d)\r\n", (int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y);

		print_string(data);
	}
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

void test_servo()
{
	static servoModule servo(servoPort1);
	static float duty = 0.0;
	bool 	btn1 = mSwitch_ReadPushBut(kPushButSW1),
			btn2 = mSwitch_ReadPushBut(kPushButSW2);

	if (btn1 && btn2)
	{
		duty = 0;
		servo.setRotation(duty);
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

	servo.setRotation(duty);
}

void test_engines()
{
	static engineModule engine;

	engine.setSpeed(mAd_Read(kPot1), mAd_Read(kPot1));
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

	pixy.init();
	pixy.setLamp(1, 1);

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
			normal_run();
			break;
		case _CHECK_BATTERY:
			display_battery_level();
			break;
		case _CHECK_SERVO:
			mLeds_Write(kMaskLed1, kLedOn);
			test_servo();
			break;
		case _CHECK_ENGINE:
			mLeds_Write(kMaskLed4, kLedOn);
			test_engines();
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
