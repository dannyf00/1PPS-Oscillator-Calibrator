//gpio for PIC24
//primarily to read DEVID and DEVREV

#include "gpio.h"						//not really used

uint32_t SystemCoreClock=8000000ul;		//systemcoreclock = Peripheral clock
//uint32_t F_CPU=8000000ul;			//CPU clock
uint32_t F_REFO=8000000ul;		//refclock output

//read devid @ 0xff0000
unsigned int DEVID(void) {
	TBLPAG = 0x00ff;
	return __builtin_tblrdl(0x0000);	//for DEVID @ 0xff0000
}

//read devrev @ 0xff0002
unsigned int DEVREV(void) {
	TBLPAG = 0x00ff;
	return __builtin_tblrdl(0x0002);	//for DEVID @ 0xff0000
}

//read device id / rev
//devid at low 16 bits and rev high 16-bits, to be compatible with pic32
unsigned long DEVIDREV(void) {
	unsigned long tmp;
	tmp=DEVREV();						//read the rev
	tmp=(tmp << 16) | DEVID();			//read id
	return tmp;
}

//reset the mcu
void mcu_init(void) {
	//turn off all peripherals
	PMD1=0xffff;
	PMD2=0xffff;
	PMD3=0xffff;
#if defined(PMD4)
	PMD4=0xffff;
#endif

	//all pins digital
#if defined(AD1PCFG)
	AD1PCFG = 0xffff;					//all pins digital
#endif
	//or AD1PCFGH
#if defined(AD1PCFGH)
	AD1PCFGH = 0xffff;					//all pins digital
#endif
	//or AD1PCFGL
#if defined(AD1PCFGL)
	AD1PCFGL = 0xffff;					//all pins digital
#endif
}

