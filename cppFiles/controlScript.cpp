#include "mbed.h"
#include "derivative.h"
#include <cstdlib>
#include <vector>

/*
 *
 * TODO:
 * based on velocity calc time needed before turning?
 * write camera function
 * write driving function
 * find out which pins are needed for controlling servo and motors
 *
 */

// Defining threshold for black/white recognition
#define THRESHOLD 100;

// Defining vals for linescan cam
#define TAOS_DELAY				asm ("nop")				// minimal delay time
#define	TAOS_SI_HIGH			GPIOB_PDOR |= (1<<8)	// SI on PTB8
#define	TAOS_SI_LOW				GPIOB_PDOR &= ~(1<<8)	// SI on PTB8
#define	TAOS_CLK_HIGH			GPIOB_PDOR |= (1<<9)	// CLK on PTB9
#define	TAOS_CLK_LOW			GPIOB_PDOR &= ~(1<<9)	// CLK on PTB9

// defining coefficients for servo control
#define KP						50			// Proportional coefficient
#define KDP						25			// Differential coefficient

 // Declaring all functions
void ImCap(void);

// Declaring global variables
Serial terminal(USBTX, USBRX);
int temp_reg;								// temporary register
vector<int> ImageData(128, 0);				// array to store the LineScan image
vector<int> ImageDataDifference(128, 0);	// array to store the PineScan pixel difference
int BlackLineRight;							// position of the black line on the right side
int BlackLineLeft;							// position of the black line on the left side
int RoadMiddle = 0;							// calculated middle of the road
int diff = 0;								// actual difference from line middle position
int diff_old = 0;							// previous difference from line middle position
int servo_position = 0;						// actual position of the servo relative to middle
int CompareData;							// set data for comparison to find max

// main function
int main() {
	// Add in port and processor bullshit mah dudes
	while (true) {
		// infinite loop
	}
	return 0;
}

// Capture LineScan Image
void ImCap(void)
{
	ADC0_CFG2 |= 0x10;							// select B side of the MUX // WHY WOULD WE WANT TO DO THAT????
	TAOS_SI_HIGH;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_SI_LOW;
	TAOS_DELAY;
	// inputs data from camera (first pixel)
	ADC0_SC1A = 11;							// set ADC0 channel 11
	while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0) {	// wait until ADC is ready
		ImageData.at(0) = ADC0_RA;							// return value
	}
	TAOS_CLK_LOW;

	for (unsigned int i = 1; i < 128; i++)
	{
		TAOS_DELAY;
		TAOS_DELAY;
		TAOS_CLK_HIGH;
		TAOS_DELAY;
		TAOS_DELAY;
		// inputs data from camera (one pixel each time through loop)
		ADC0_SC1A = 11;							// set ADC0 channel 11
		while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0) {	// wait until ADC is ready
			ImageData.at(i) = ADC0_RA;					// return value
		}
		TAOS_CLK_LOW;
	}

	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_LOW;
}