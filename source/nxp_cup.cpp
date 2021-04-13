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

		leds_off();

		switch_state = get_switch_state();

		switch (switch_state)
		{
		case _NORMAL_RUN:
			car.step(true);
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
			car.step(false);
			car.printLineDist5();
			break;
		case _CAM_DATA:
			car.step(false);
			car.printCamData();
			break;
		default:
			break;
		}
	}

	return 0;
}
