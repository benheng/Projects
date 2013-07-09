#include "msp430g2553.h"
#include <stdio.h>

#define LIGHT_THRESH 799
#define TIMER0_FREQ 999
#define TIMER1_FREQ 999
#define LED0_SPD 200
#define LED1_SPD 250
#define LED2_SPD 300
#define LED0_STP 50
#define LED1_STP 100
#define LED2_STP 150

// GLOBAL VARIABLES
volatile unsigned int latest_adc_result;
volatile unsigned int timer_count = 0;
volatile unsigned int pulsedir0 = 1;
volatile unsigned int pulsedir1 = 1;
volatile unsigned int pulsedir2 = 1;
volatile unsigned int off0 = 0;
volatile unsigned int off1 = 0;
volatile unsigned int off2 = 0;
volatile unsigned int state0 = 0;
volatile unsigned int state1 = 0;
volatile unsigned int T0_pause = 0;
volatile unsigned int T1_pause = 0;

// FUNCTION PROTOTYPES
void init_led();
void init_uart();
void init_timer();
void init_adc();
void start_conversion();
void init_motion();

// MAIN FUNCTION
void main(void) {
  // clock initializers
	BCSCTL1  = CALBC1_1MHZ;	 	// calibration for basic clock system
	DCOCTL 	 = CALDCO_1MHZ;	 	// calibration for digitally controlled oscillator
	WDTCTL 	 = WDTPW + WDTTMSEL + WDTCNTCL;	// WDT 1MHz/32K ~= 0.03s per interrupt
	IE1 |= WDTIE;

	// initiate peripherals
	init_timer();
	init_adc();

	// enable interrupts and put the CPU to sleep
	_bis_SR_register(GIE+LPM0_bits);
}

// HELPER FUNCTIONS
void init_timer() {
	// TimerA_0
	TA0CTL |= TACLR;		// reset timer
	TA0CTL  = TASSEL_2		// SMCLK
			+ ID_0			// input divider = 1
			+ MC_1;			// up mode, interrupt disabled, timer on
	TA0CCTL0 = CCIE;		// nothing in CCR0
	TA0CCTL1 = OUTMOD_7;	// reset/set mode
	TA0CCR0 = TIMER0_FREQ;	// period - 1 in CCR0
	TA0CCR1 = 100;			// duty cycle in CCR1
	P1SEL |= 0x40;			// P1.6
	P1DIR |= 0x40;

	// TimerA_1
	TA1CTL |= TACLR;		// reset timer
	TA1CTL  = TASSEL_2		// SMCLK
			+ ID_0			// input divider = 1
			+ MC_1;			// up mode, interrupt disabled, timer on
	TA1CCTL0 = CCIE;		// nothing in CCR1
	TA1CCTL1 = OUTMOD_7;	// reset/set mode
	TA1CCTL2 = OUTMOD_7;	// reset/set mode
	TA1CCR0  = TIMER1_FREQ;	// period - 1 in CCR0
	TA1CCR1  = 100;			// duty cycle in CCR1
	TA1CCR2  = 100;			// duty cycle in CCR2
	P2SEL |= 0x02 + 0x20;	// P2.1, P2.5
	P2DIR |= 0x02 + 0x20;
}
void init_adc() {
	ADC10AE0 = 0x10;		// P1.4

	ADC10CTL1 = INCH_4		// P1.4, A4 input
			+ SHS_0			// use ADC10SC bit to trigger sampling
			+ ADC10DIV_7	// clock divider = 8
			+ ADC10SSEL_3	// clock source = SMCLK
			+ CONSEQ_0;		// single channel, single conversion
	ADC10DTC1 = 1;			// one block per transfer

	ADC10CTL0 = SREF_0		// reference voltages are Vss and Vcc
			+ ADC10SHT_3	// 64 ADC10 clocks for sample and hold time (slowest)
			+ REF2_5V		// reference generator voltage 2.5V
			+ REFON			// reference generator on
			+ ADC10ON		// turn on ADC10
			+ ENC;			// enable (but not yet start) conversions
}
void start_conversion() {
	if ((ADC10CTL1 & ADC10BUSY) == 0) {	// if not already converting
		ADC10CTL0 |= ADC10SC;
		ADC10SA = (unsigned) &latest_adc_result;
	}
}

// INTERRUPT HANDLERS
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0(void) {
	T0_pause++;
	if (T0_pause > 20) {
		T0_pause = 0;
		switch (state0) {
		case 0:
			if (latest_adc_result <= LIGHT_THRESH) state0 = 10;
			else {
				// P1.6
				if (pulsedir0) {
					TA0CCR1 += latest_adc_result/LED0_SPD;
					if (TA0CCR1 >= latest_adc_result) pulsedir0 = 0;
				}
				else {
					if (TA0CCR1 > latest_adc_result/LED0_SPD) TA0CCR1 -= latest_adc_result/LED0_SPD;
					else TA0CCR1 = 0;
					if (TA0CCR1 <= 1) {
						if (off0 < LED0_STP) off0++;
						else {
							off0 = 0;
							pulsedir0 = 1;
						}
					}
				}
			}
			break;
		case 10:
			if (TA0CCR1 > 0) {
				if (TA0CCR1 > LED0_SPD/100) TA0CCR1 -= LED0_SPD/100;
				else TA0CCR1 = 0;
				if (TA0CCR1 <= 1) pulsedir0 = 1;
			}
			if (latest_adc_result > LIGHT_THRESH) state0 = 0;
			break;
		}
	}
}
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0(void) {
	T1_pause++;
	if (T1_pause > 20) {
		T1_pause = 0;
		switch (state1) {
		case 0:
			if (latest_adc_result <= LIGHT_THRESH) state1 = 10;
			else {
				// P2.1
				if (pulsedir1) {
					TA1CCR1 += latest_adc_result/LED1_SPD;
					if (TA1CCR1 > latest_adc_result) pulsedir1 = 0;
				}
				else {
					if (TA1CCR1 > latest_adc_result/LED1_SPD) TA1CCR1 -= latest_adc_result/LED1_SPD;
					else TA1CCR1 = 0;
					if (TA1CCR1 <= 1) {
						if (off1 < LED1_STP) off1++;
						else {
							off1 = 0;
							pulsedir1 = 1;
						}
					}
				}
				// P2.5
				if (pulsedir2) {
					TA1CCR2 += latest_adc_result/LED2_SPD;
					if (TA1CCR2 > latest_adc_result) pulsedir2 = 0;
				}
				else {
					if (TA1CCR2 > latest_adc_result/LED2_SPD) TA1CCR2 -= latest_adc_result/LED2_SPD;
					else TA1CCR2 = 0;
					if (TA1CCR2 <= 1) {
						if (off2 < LED2_STP) off2++;
						else {
							off2 = 0;
							pulsedir2 = 1;
						}
					}
				}
			}
			break;
		case 10:
			if (TA1CCR1 > 0) {
				if (TA1CCR1 > LED1_SPD/100) TA1CCR1 -= LED1_SPD/100;
				else TA1CCR1 = 0;
				if (TA1CCR1 <= 1) pulsedir1 = 1;
			}
			if (TA1CCR2 > 0) {
				if (TA1CCR2 > LED2_SPD/100) TA1CCR2 -= LED2_SPD/100;
				else TA1CCR2 = 0;
				if (TA1CCR2 <= 1) pulsedir2 = 1;
			}
			if (latest_adc_result > LIGHT_THRESH) state1 = 0;
			break;
		}
	}
}

interrupt void WDT_checkState() {
	timer_count++;
	// Send ADC value over UART every ~1.91s
	if (timer_count > 64) {
		timer_count = 0;
		start_conversion();
	}
}
ISR_VECTOR(WDT_checkState, ".int10")
