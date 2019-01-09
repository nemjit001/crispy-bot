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
serial terminal(USBTX, USBRX);
int carSpeed = 0;

int main() {
	try {
		while (true) {
			/* write drive chain */
		}
	}
	catch (std::runtime_error& excpt) {
		std::cout << "Runtime Error: " << excpt.what() << std::endl;
	}
    return 0;
}

void steer(int deg, int speed){
	if (deg >= -360 && deg <= 360) {
		wait_ms(timeToTurn());
	}
	else {
		throw std::runtime_error("servo value out of range!\r\nStopping Program");
	}
}

int calcDeg () {
	int degrees = 0;

	// take in cam data and calculate degrees

	return degrees;
}
