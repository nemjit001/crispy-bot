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

#define K_MAIN_INTERVAL (100 / kPit1Period)

// Measuring speed and meaning
static float sSpeedMotLeft;
static float sSpeedMotRight;
#define kSpeedTabSize 100
static UInt16 sSpeedIndex;
static float sSpeedTab[kSpeedTabSize][2];
// Table containing the image (and various infos) of the
// digital camera, the test board is used for the Labview app
static UInt16 sImageTabTest[200];
// Measurement of the accelerometer and the magnetometer
static SRAWDATAEnum sAccel;   // in g
static SRAWDATAEnum sMagneto; // in micro teslas
//static UInt8 sAccelMagnetoStatus;
static float sYaw;   // in degree
static float sRoll;  // in degree
static float sPitch; // in degree

// Angular velocity of gyro in degrees
static float sAngVel_X;
static float sAngVel_Y;
static float sAngVel_Z;

// Measurement of motor current and battery voltage
static float sIMotLeft;
static float sIMotRight;
static float sUBatt;
static bool sFaultLeft;
static bool sFaultRight;

// Distance measured by the LIDAR in mm
static UInt8 sDistFront;

/*
 * @brief   Application entry point.
 */
int main(void)
{

	static UInt8 sPixelValMoy;
	static bool sImageOk = false;
	static Int16 sDly;
	static UInt16 sIntTime = 25000;
	Int32 aWakeIntMain;
	Int8 aCharTab[50];
	UInt32 i = 0;
	bool aRet;
	float aDuty = 0;

	// Table containing the image (and various infos) of the digital camera
	UInt8 aImageTab[200];
	// Sensor value
	SRAWDATAEnum aAccel;   // in g
	SRAWDATAEnum aMagneto; // in micro teslas
	float aYaw;			   // in degree
	float aRoll;		   // in degree
	float aPitch;		   // in degree
	// Measuring speed and meaning
	float aSpeedMotLeft;
	float aSpeedMotRight;

#if (kWithLidar)
	UInt8 aDistFront;
#endif

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

	while(1)
	{
		if (mSwitch_ReadSwitch(kSw1) == true)
		{
			mLeds_Write(kMaskLed1, kLedOn);
		}
		else
		{
			mLeds_Write(kMaskLed1, kLedOff);
		}
	}

	return 0;
}
