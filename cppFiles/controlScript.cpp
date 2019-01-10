#include "mbed.h"
#include <cstdlib>

/*
 *
 * TODO:
 * based on velocity calc time needed before turning?
 * write camera function
 * write driving function
 * find out which pins are needed for controlling servo and motors
 *
 */

 // Declaring all functions
void steer(int deg);
int calcDeg();

// Declaring global variables
Serial terminal(USBTX, USBRX);
int carSpeed = 0;

int main() {
	bool drive = true;
	while (drive) {
		/* write drive chain */
		if (/* no turn */) {
			// set motor values to drive with speed carSpeed;
		}
		else if (/* turn */) {
			steer(calcDeg(), carSpeed);
		}
		else {
			drive = false;
			carSpeed = 0;
		}
	}
	// stop all motors and shut down cam
	return 0;
}

void steer(int deg, int speed) {
	if (deg >= -360 && deg <= 360) {
		// Set servo to degree value
	}
	else {
		temrinal.fprintf(stderr, "servo value out of range!\r\nStopping Program...");
	}
}

int calcDeg() {
	int degrees = 0;

	// take in cam data and calculate degrees

	return degrees;
}