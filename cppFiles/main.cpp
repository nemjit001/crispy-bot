#include "mbed.h"
#include "MKL25Z4.h"

// Defining macros for important values
#define THRESHOLD 100;      // Threshold for black/white recognition
#define CAR_SPEED 100;      // Global setting for controlling the car's speed

// Defining macro values for linescan camera
#define TAOS_DELAY              asm ("nop")             // minimal delay time
#define TAOS_SI_HIGH            GPIOB_PDOR |= (1<<8)    // SI on PTB8
#define TAOS_SI_LOW             GPIOB_PDOR &= ~(1<<8)   // SI on PTB8
#define TAOS_CLK_HIGH           GPIOB_PDOR |= (1<<9)    // CLK on PTB9
#define TAOS_CLK_LOW            GPIOB_PDOR &= ~(1<<9)   // CLK on PTB9


// Defining servo coefficients
const int KP = 50;             // This is the Proportional Coefficient | Check this value and edit appropriately
const int KDP = 25;             // This is the Differential Coefficient | Check this value and edit appropriately

// Declaring used functions
void ImCap();
void FTM1_IRQHandler();

// Declaring globals needed
int temp_reg;                       // Temporary register for storing data

int lineLeft;                       // The left black line of the track
int lineRight;                      // The right black line of the track
int lineMid = 0;                    // The calculated middle of the track
int diff = 0;                       // The difference from the real middle
int prev_diff = 0;                  // The previous value of diff
int servo_pos = 0;                  // The change in servo position, used for steering
int dataComp = 0;                   // Comparison for data gotten from the cam
int ImData[128];                    // The image data acquired from the linescan camera
int ImDataDiff[128];                // The difference of image data acquired from the linescan camera


// Main function, program entrypoint
int main() {
	// Preparing the MCU and MPU

	// configure clock to 48 MHz from a 8 MHz crystal
	MCG_C2 = (MCG_C2_RANGE0(1) | MCG_C2_EREFS0_MASK);   // configure the oscillator settings
	MCG_C1 = (MCG_C1_CLKS(2) | MCG_C1_FRDIV(3));        // divider for 8 MHz clock

	for (int i = 0; i < 24000; i++) {                       // wait for OSCINIT to set
		// now in FBE mode
		MCG_C6 |= MCG_C6_CME0_MASK;                     // enable the clock monitor
	}

	MCG_C5 |= MCG_C5_PRDIV0(1);                         // set PLL ref divider to divide by 2
	temp_reg = MCG_C6;                                  // store present C6 value (as CME0 bit was previously set)
	temp_reg &= ~MCG_C6_VDIV0_MASK;                     // clear VDIV settings
	temp_reg |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);     // write new VDIV and enable PLL
	MCG_C6 = temp_reg;                                  // update MCG_C6

	for (int i = 0; i < 4000; i++) {                        // wait for PLLST status bit to set
		// now in PBE mode
		SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1));    // core clock, bus clock div by 2
	}

	MCG_C1 &= ~MCG_C1_CLKS_MASK;    // switch CLKS mux to select the PLL as MCGCLKOUT   
	for (int i = 0; i < 2000; i++)  // Wait for clock status bits to update
	// now in PEE mode, core and system clock 48 MHz, bus and flash clock 24 MHz

	// Turning on all port clocks
		SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	// Turning on all TPM clocks
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

	// Turning on ADC0 clock for converting input signals
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// set GPIOs
	PORTC_PCR0 |= PORT_PCR_MUX(1);  // PTC0 Push Button SW2 (Reset button)
	PORTB_PCR8 |= PORT_PCR_MUX(1);  // PTB8 Camera SI
	PORTB_PCR9 |= PORT_PCR_MUX(1);  // PTB9 Camera Clock
	PORTA_PCR4 = 0;                 // Set PTA4 Pin Control Register to Zero
	PORTA_PCR4 |= PORT_PCR_MUX(3);  // PTA4 Motor 1 In 1 (speed) PTA4 TPM0_CH1
	PORTA_PCR5 |= PORT_PCR_MUX(1);  // PTA5 Motor 1 In 2 (direction)
	PORTC_PCR8 |= PORT_PCR_MUX(1);  // PTC8 Motor 2 In 1 (direction)
	PORTC_PCR9 |= PORT_PCR_MUX(3);  // PTC9 Motor 2 In 2 (speed) PTC8 TPM0_CH5

	// set GPIO outputs to output
	GPIOB_PDDR |= (1 << 8);         // PTB8 Camera SI
	GPIOB_PDDR |= (1 << 9);         // PTB9 Camera Clock
	GPIOA_PDDR |= (1 << 4);         // PTA4 Motor A In 1 (speed)
	GPIOA_PDDR |= (1 << 5);         // PTA5 Motor A In 2 (direction)
	GPIOC_PDDR |= (1 << 8);         // PTC9 Motor B In 1 (direction)
	GPIOC_PDDR |= (1 << 9);         // PTC9 Motor B In 2 (speed)

	// set GPIO inputs to input
	GPIOC_PDDR &= ~(1 << 0);        // Setting all 32 bits to 0, except bit 1, then flipping, thus creating 1 input for the SW2

	// set PWM outputs
	PORTA_PCR12 |= PORT_PCR_MUX(3); // Servo Motor Output on PTA12 TPM1_CH0

	// configure TPM clock source to be 48 MHz
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);       // MCGFLLCLK clock or MCGPLLCLK/2
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;  // use MCGPLLCLK/2 clock when running from 8 MHz and PLL

	// set TPM prescaler before enabling the timer
	TPM0_SC |= 3;                   // prescaler for TPM0 (Motors) is 8
	TPM1_SC |= 3;                   // prescaler for TPM1 (Servo) is 8

	// TPM modulo register, set frequency
	TPM0_MOD = 600;                 // modulo TPM0 (Motors), period = 0.10 ms (10000 Hz)
	TPM1_MOD = 60000;               // modulo TPM0 (Servo), period = 10 ms (100 Hz)

	// set TPM clock mode to enable timer
	TPM0_SC |= TPM_SC_CMOD(1);      // enable TPM0 (Motors)
	TPM1_SC |= TPM_SC_CMOD(1);      // enable TPM1 (Servo)

	TPM0_C1SC = 0x28;               // TPM0 channel1 Motor A In 1 speed left
	TPM0_C5SC = 0x28;               // TPM0 channel5 Motor B In 2 speed right
	TPM1_C0SC = 0x28;               // TPM1 channel0 Servo 1

	// TPM channel value registers, sets duty cycle
	TPM1_C0V = 9000;                // TPM1 channel0 Servo 1 ca. 1.5 ms (middle)

	// initial configuration of motors
	TPM0_C1V = CAR_SPEED;                // TPM0 channel1 left Motor A In 1 slow forward
	TPM0_C5V = CAR_SPEED;                // TPM0 channel5 right Motor B In 2 slow forward
	GPIOA_PDOR &= ~(1 << 5);            // Set PTA5 left Motor 1 In 2 forward
	GPIOC_PDOR &= ~(1 << 8);            // Set PTC8 right Motor 2 In 1 forward

	// configure interrupts in TPM0
	TPM1_SC |= TPM_SC_TOIE_MASK;    // enable overflow interrupt in TPM1 (10 ms rate)

	// ADC0 clock configuration
	ADC0_CFG1 |= 0x01;              // clock is bus clock divided by 2 = 12 MHz

	// ADC0 resolution    
	ADC0_CFG1 |= 0x08;              // resolution 10 bit, max. value is 1023

	// ADC0 conversion mode
	ADC0_SC3 = 0x00;                // single conversion mode

	// enable interrupts 18 (TPM = FTM1)  in NVIC, no interrupt levels
	NVIC_ICPR |= (1 << 18);         // clear pending interrupt 18
	NVIC_ISER |= (1 << 18);         // enable interrupt 18

	// enable interrupts globally
	asm(" CPSIE i ");               // enable interrupts on core level

	for (;;) {

	}
}

void FTM1_IRQHandler()                          // TPM1 ISR
{
	TPM1_SC |= 0x80;                            // clear TPM0 ISR flag

	// Capture Line Scan Image
	ImCap();                                    // capture LineScan image from camera

	// Find black line on the right side
	for (int i = 64; i < 127; i++)                  // calculate difference between the pixels
	{
		*(ImDataDiff) = abs(*(ImData + i) - *(ImData + (i + 1)));
	}
	dataComp = THRESHOLD;                       // threshold
	lineRight = 126;
	for (int i = 64; i < 127; i++)
	{
		if (ImDataDiff[i] > dataComp)           // If the threshold is reached, then the lineRight is set to that pixel
		{
			dataComp = ImDataDiff[i];
			lineRight = i;
		}
	}

	// Find black line on the left side
	for (int i = 64; i > 0; i--)                    // calculate difference between the pixels
	{
		*(ImDataDiff) = abs(*(ImData + i) - *(ImData + (i - 1)));
	}
	dataComp = THRESHOLD;                    // threshold
	lineLeft = 1;
	for (int i = 64; i > 1; i--)                    // only down to pixel 2, not 1
	{
		if (ImDataDiff[i] > dataComp)
		{
			dataComp = ImDataDiff[i];
			lineLeft = i;
		}
	}

	// Find middle of the road, 64 for is a straight road
	lineMid = (lineLeft + lineRight) / 2;

	// if a line is only on the the left side
	if (lineRight > 124)
	{
		lineMid = lineLeft + 50;
	}
	// if a line is only on the the right side
	if (lineLeft < 2)
	{
		lineMid = lineRight - 50;
	}
	// if no line on left and right side
	if ((lineRight > 124) && (lineLeft < 3))
	{
		lineMid = 64;
	}

	// Store old value
	prev_diff = diff;                           // store old difference

	// Find difference from real middle
	diff = lineMid - 64;                        // calculate actual difference

	// Checking if the fotten value is in a good range
	if (abs(diff - prev_diff) > 50)
	{
		diff = prev_diff;
	}

	// Direction Control Loop: PD Controller
	servo_pos = (KP * diff) + (KDP * (diff - prev_diff));

	// Set channel 0 PWM_Servo position
	TPM1_C0V = 9000 - servo_pos;            // set channel 0 PWM_Servo

	// Set Motor Speed
	TPM0_C1V = CAR_SPEED;                    // TPM0 channel1 left Motor 1 In 1 slow forward
	TPM0_C5V = CAR_SPEED;                    // TPM0 channel5 right Motor 2 In 2 slow forward
}

// Capture LineScan Image
void ImCap(void)
{
	ADC0_CFG2 |= 0x10;                              // select B side of the MUX
	TAOS_SI_HIGH;
	TAOS_DELAY;
	TAOS_CLK_HIGH;                                  // Turning on the clock
	TAOS_DELAY;
	TAOS_SI_LOW;
	TAOS_DELAY;
	// inputs data from camera (first pixel)
	ADC0_SC1A = 11;                                 // set ADC0 channel 11
	while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0) {  // wait until ADC is ready
		ImData[0] = ADC0_RA;                     // function edits the global array ImData
	}
	TAOS_CLK_LOW;                                   // Turning off the clock

	for (int i = 1; i < 128; i++)                   // Getting the rest of the pixels
	{
		TAOS_DELAY;
		TAOS_DELAY;
		TAOS_CLK_HIGH;
		TAOS_DELAY;
		TAOS_DELAY;
		// inputs data from camera (one pixel each time through loop)
		ADC0_SC1A = 11;                                 // set ADC0 channel 11
		while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0) {  // wait until ADC is ready
			ImData[i] = ADC0_RA;                  // return value
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
