//1pps_calibrator.ino
//An Arduino Leonardo sketch to measure and calibrate its own oscillator frequency
//Hardware requirement:
//1. Arduino Pro Mini, Uno or compatable
//2. a 1PPS generator, like a GPS
//
//v0.1: 4/24/2018 - initial release
//v0.2: 4/25/2018 - fixed a minor bug in initialization
//v0.3: 4/26/2018 - ported to Leonardo/ATmega32U4
//v0.4: 4/27/2018 - ported to PIC32MX
//v0.5: 4/28/2018 - numerous minor improvements
//
//Connections:
//
//                              |---------------------|
//      1PPS generator -------->|ICP1/RA4             |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |  PIC32MX250F120B    |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                  TX |-------------> PC via USB/TTL converter
//                              |                     |
//                              |---------------------|
//
//
//

#include <string.h>						//we use strcpy
#include "config.h"						//fuse settings: PRI/FRC, PRIPLL/FRCPLL, PBDIV=1
#include "gpio.h"						//we use gpio
#include "delay.h"						//we use software delays
#include "uart1.h"						//we use uart
#include "pwm4.h"						//we use pwm

//hardware configuration
#define F_CLK       F_PHB				//clock of oscillator to be calibrated
#define PPS_CNT		1					//number of 1pps pulses to count
#define PPS_PIN()	PPS_IC1_TO_RPA4()	//1pps input pin assignment: A2/B6/A4/B13/B2/C6/C1/A3
#define FREQ_CNT	10					//weight used in smoothing algorithm
#define SET_PBDIV	2					//current setting of PBDIV, 1/2/4/8 (default)

#define LED_PORT	LATB
#define LED_DDR		TRISB
#define LED			(1<<7)				//led on pb7
//end hardware configuration

//global defines
#define TxMD		PMD4bits.T2MD
#define TxCON		T2CON
#define TMRx		TMR2
#define PRx			PR2
#define ICxMD		PMD3bits.IC1MD
#define ICxCON		IC1CON
#define ICxIF		IFS0bits.IC1IF
#define ICxIE		IEC0bits.IC1IE
#define ICxIP		IPC1bits.IC1IP
#define ICxBUF		IC1BUF

//global variables
volatile uint16_t tick0, tick1;			//16-bit captures
volatile  int16_t freq_error;			//frequency error
volatile  int32_t freq;					//frequency measurement
volatile  uint8_t freq_available=0;		//1->new data available
volatile  uint8_t pps_cnt = PPS_CNT;	//current 1pps pulse count, downcounter
		  int32_t freq_sum, freq_avg, freq_i, freq_f;
char uRAM[80];							//transmitt buffer for uart
const char str0[]="freq =         Hz.\n\r";

//input capture ISR
void __ISR(_INPUT_CAPTURE_1_VECTOR/*, ipl7*/) _IC1Interrupt(void) {
	//clear the flag
	tick1 = ICxBUF;						//read the capture buffer first
	ICxIF = 0;							//clear the flag after the buffer has been read (the interrupt flag is persistent)
	pps_cnt -= 1;						//decrement pps_cnt
	if (pps_cnt == 0) {
		pps_cnt = PPS_CNT;				//reset pps_cnt
		freq_error = tick1 - (tick0 + F_CLK * PPS_CNT); freq = F_CLK * PPS_CNT + freq_error; freq = freq << OSCCONbits.PBDIV;
		tick0 = tick1;					//update tick0
		freq_available = 1;				//new data available
		IO_FLP(LED_PORT, LED);			//flip led
	}
}
	
//reset timer2 as timebase for input capture
//free running, 16-bit
void tmr2_init(void) {
	TxMD = 0;							//0->enable power to timer, 
	//stop the timer
	//TxCON &=~(1<<15);					//1->start the timer, 0->stop the timer
	//TxCON &=~(1<<7);					//1->gating enabled, 0->gating disabled
	//TxCON  = (TxCON &~(7<<4)) | (0<<4);	//0->1:1 prescaler, 1->2x prescaler, ...
	//TxCON &=~(1<<3);					//0->16 bit mode, 1->32-bit mode
	//TxCON &=~(1<<1);					//0->count on internal clock, 1->count on external clock
	TxCON  = 	(0<<15) |				//1->start the timer, 0->stop the timer
				(0<<13) |				//0->operate in idle, 1->don't operate in idle
				(0<< 7) |				//1->gating enabled, 0->gating disabled
				(0<< 4) |				//0->1:1 prescaler, 1->2x prescaler, 2->4x prescaler, 3->8x prescaler, ..., 7->256x prescaler
				(0<< 3) |				//0->16 bit mode, 1->32-bit mode
				(1<< 0) |				//0->count on internal clock, 1->count on external clock
				0x00;
	//TMRx = 0;							//reset the counter - optional
	PRx  =0xffff;						//period = 0xffff
	//now start the timer
	TxCON |= (1<<15);					//1->start the timer, 0->stop the timer
	//timer now running
}


//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1_init(void) {
	ICxMD = 0;							//0->enable power to input capture
	//disable the input captur emodule
	//ICxCON &=~(1<<15);					//1->enable the module, 0->disable the module
	//ICxCON |= (1<<9);					//1-.capture rising edge first (only used for ICM110)
	//ICxCON &=~(1<<8);					//1->32-bit mode, 0->16-bit mode
	//ICxCON |= (1<<7);					//1->timer2 as timebase, 0->timer3 as timebase
	//ICxCON  = (ICxCON &~(3<<5)) | (0<<5);	//0->interrupt on every capture event, 1->on every second capture event, ...
	//ICxCON  = (ICxCON &~(7<<0)) | (3<<0);	//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
	ICxCON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, ...
				(0<<4) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	ICxIF   = 0;						//0->clear the flag
	ICxIE   = 0;						//1->enable the interrupt, 0->disable the interrupt
	ICxIP   = 1;
	//enable the input capture
	ICxCON |= (1<<15);					//1->enable the module, 0->disable the module
	//input capture running now
}
	
//reset frequency calibrator
void freqc_init(void) {
	//tick0=tick1=0;
	freq_available = 0;					//0->no new data
	freq = F_CLK;						//initial value of freq
	pps_cnt = PPS_CNT;					//reset 1pps pulse counter, downcounter
	freq_sum = (F_CLK << OSCCONbits.PBDIV) * FREQ_CNT;	//initialize freq_sum: PBDIV=8
	freq_avg = freq_sum / FREQ_CNT;		//optional
	
	//optional - calibrate FRC
	//DMA / interupts assumed disabled here
	SYSKEY = 0xaa996655ul; SYSKEY = 0x556699aaul;	//unlock sequence
	OSCTUN = -5;						//change osctun: 12.5% / 32
	OSCCON = (OSCCON &~(3<<19)) | 		//trim FRC
	//set PBDIV
#if   SET_PBDIV==1
				(0<<19);	//PBDIV: 0->1x
#elif SET_PBDIV==2
				(1<<19);	//PBDIV: 1->2x
#elif SET_PBDIV==4
				(2<<19);	//PBDIV: 2->4x
#else
				(3<<19);	//PBDIV: 3->8x (default)
#endif
	SYSKEY = 0x33333333ul;				//lock by writing any non critical value
	
	tmr2_init();						//reset tmr2
	//configure the input capture pin ICP1
	PPS_PIN();
	ic1_init();							//reset ic1
	//wait for the first capture event
	while (ICxIF == 0) continue;
	tick0 = ICxBUF;						//read the first capture
	ICxIF = 0;							//clear the flag after having read the buffer - flag is persistent so the read order has to be maintained
	ICxIE = 1;							//enable the interrupt
}
	
int main(void) {
	uint32_t tmp;
	
	mcu_init();							//reset the mcu
	IO_SET(LED_PORT, LED); IO_OUT(LED_DDR, LED);				//led as output
	freqc_init();						//reset the frequency calibrator
	uart1_init(9600);					//reset uart
	ei();								//enable global interrupts
	while (1) {
		if (freq_available) {			//new data has arrived
			freq_available = 0;			//data has been read, no new data now
			
			//smoothing the reading
			freq_sum += freq - freq_avg;
			freq_avg = freq_sum / FREQ_CNT;
			//freq_i   = freq_sum / FREQ_CNT;
			//calculate the fractional frequency
			freq_f   = freq_sum - freq_avg * FREQ_CNT;
			
			//convert freq for transmission
			tmp = freq;					//display freq
			//forming the display string
#if 0
			strcpy(uRAM, str0);			//initialize uart buffer
			uRAM[14]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[13]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[12]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[11]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[10]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[ 9]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[ 8]=(tmp % 10) + '0'; tmp /= 10;
			uRAM[ 7]=(tmp % 10) + '0'; tmp /= 10;
#else
			//sprintf(uRAM, "freq = %8dHz.\n\r");
			//sprintf(uRAM, "freq_sum=%12ld, freq_i=%12ld, freq_f=%12ld\n\r", freq_sum, freq_i, freq_f);
			sprintf(uRAM, "freq = %10ldHz, freq = %10ld.%03dHz.\n\r", freq, freq_avg, freq_f * 1000 / FREQ_CNT);
#endif
			uart1_puts(uRAM);			//start transmission

			//IO_FLP(LED_PORT, LED);		//flip the led
		}	
		//delay_ms(100);
		//uart1_puts("testing...\n\r");
    }
    
    return 0;
}