#include "nxp_cup.h"

int main(void)
{
	board_device_setup();
	print_string("INITIALIZING BOARD\n\r\0");

	// bitmask containing board switch state
	static uint8_t switch_state = 0x0;
	// main loop delay
	static Int16 delay = 0;

	static Rover car;

	mDelay_ReStart(kPit1, delay, K_MAIN_INTERVAL);

	mLeds_Write(kMaskLed1, kLedOn);

	for (;;)
	{
		if(!mDelay_IsDelayDone(kPit1, delay))
			continue;
		
		mDelay_ReStart(kPit1, delay, K_MAIN_INTERVAL);

		// leds_off();

		// switch_state = get_switch_state();

		// switch (switch_state)
		// {
		// case _NORMAL_RUN:
		// 	car.step(true);
		// 	break;
		// case _CHECK_BATTERY:
		// 	car.stop();
		// 	display_battery_level();
		// 	break;
		// case _CHECK_SERVO:
		// 	mLeds_Write(kMaskLed1, kLedOn);
		// 	car.stop();
		// 	car.test_servo();
		// 	break;
		// case _PAUSE_ALL:
		// 	mLeds_Write(kMaskLed2, kLedOn);
		// 	car.stop();
		// 	__asm("nop");
		// 	break;
		// case _LINE_DIST:
		// 	car.step(false);
		// 	car.printLineDist5();
		// 	break;
		// case _CAM_DATA:
		// 	car.step(false);
		// 	car.printCamData();
		// 	break;
		// default:
		// 	break;
		// }

		car.step(true);
	}

	return 0;
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
