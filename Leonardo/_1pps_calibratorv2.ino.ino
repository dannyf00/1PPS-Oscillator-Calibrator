//1pps_calibrator.ino
//An Arduino Leonardo sketch to measure and calibrate its own oscillator frequency
//Hardware requirement:
//1. Arduino Pro Mini, Uno or compatable
//2. a 1PPS generator, like a GPS
//
//v0.1: 4/24/2018 - initial release
//v0.2: 4/25/2018 - fixed a minor bug in initialization
//v0.3: 4/26/2018 - ported to Leonardo/ATmega32U4
//
//Connections:
//
//                              |---------------------|
//      1PPS generator -------->|ICP1/PD4/D4          |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |                     |
//                              |  Arduino Leonardo   |
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

//global defines
#define F_CLK      		(F_CPU)   	//estimated clock speed
#define PPS_CNT    		10       	//Number of 1PPS pulses to count
#define F_OVERSAMPLE  	4     		//number of oversamples

//global variable
volatile uint16_t tick0, tick1;   //tick0=previous tick / capture, tick1 = current tick / capture
volatile uint32_t freq=F_CLK;   //current capture
volatile  int32_t freq_error;   //frequency error 
volatile uint8_t  freq_available=0; //frequency available indicator: 1->new data available, 0->no new data available
          int32_t freq_sum;     //moving sumes
          int32_t freq_i, freq_f; //interger + fractional parts of freq
		  uint8_t pps_cnt=PPS_CNT;	//current count of 1PPS pulses, downcounter
char uRAM[80];            //uart buffer

//timer1 icp ISR
ISR(TIMER1_CAPT_vect) {
  //clear the flag - done automatically
  //read capture values first to avoid overrun
  //tick1 = ICR1L;        //read ICPL first
  //tick1|= ICR1H << 8;     //read ICPH second
  tick1 = ICR1;
  pps_cnt -= 1;				//decrement pps_cnt, downcounter
  if (pps_cnt == 0) {
	pps_cnt = PPS_CNT;		//reset current count
	//freq = ((F_CLK * PPS_CNT) & 0xffff0000ul) + (int16_t) (tick1 - tick0);  //calculate the frequency
	freq_error = (int16_t) (tick1 - (tick0 + F_CLK * PPS_CNT)); freq = F_CLK * PPS_CNT + freq_error;  //calculate the frequency / frequency error
	tick0 = tick1;          //update captured tick
	freq_available = 1;       //1->new data available
	}
}

//initialize tmr1
//ICP1/PB0/D8 enabled, 
//ICP interrupt not yet enabled
void tmr1_init(void) {
  //stop timer1
  //TCCR1B = (TCCR1B &~0x07) | (0x00 & 0x07); //0->stop the timer1
  TCCR1A = TCCR1B = TCCR1C = 0; //reset to default values, and stops the timer1
  TCCR1B&=~(1<<ICNC1);      //0->disable noise cancelling, 1->enable noise cancelling
  TCCR1B|= (1<<ICES1);      //0->trigger on falling edge, 1->trigger on rising edge 
  TCNT1 = 0;            	//reset timer1: optional

  //clear the flag 
  TIFR1 |= (1<<ICF1);       //1->clear the flag, 0->no effect
  TIMSK1&=~(1<<ICIE1);      //0->disable the interrupt, 1->enable the interrupt

  //enable the clock 
  TCCR1B = (TCCR1B &~0x07) | (0x01 & 0x07); //0->stop the timer1, 0x01->1:1 prescaler 
  //timer1 is now running

}

//initialize frequency calibrator 
void freqc_init(void) {
  //initialize the variables
  freq_available = 0;       //0->no new data available
  freq = F_CLK;         //freq initial value estimate
  freq_sum = freq * F_OVERSAMPLE; //over sample
  freq_i = freq_sum / F_OVERSAMPLE;
  pps_cnt = PPS_CNT;		//reset current count, downcounter

  //initialize the pin - optional
  //pinMode(4, INPUT_PULLUP);   //ICP1/D4 as input
  //digitalWrite(4, HIGH);    //activate pull-up

  tmr1_init();          //initialize tmr1 icp, interrupt disabled

  //wait for the first capture event
  while ((TIFR1 & (1<<ICF1)) == 0) continue;
  TIFR1 |= (1<<ICF1);       //1->clear the flag
  //read capture values first to avoid overrun
  //tick0 = ICR1L;        //read ICPL first
  //tick0|= (ICR1H << 8);     //read ICPH second
  tick0 = ICR1;
  TIMSK1|= (1<<ICIE1);      //enable capture interrupt for future events
}

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);       //initialize serial transmitter
  
  pinMode(13, OUTPUT);

  freqc_init();         //initialize the frequency calibrator

  //enable global interrupts
  interrupts();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (freq_available) {     //new data is available
    freq_available = 0;     //no new data is available
    
    //update moving average
    freq_sum += (freq - freq_i);  //update the sum
    freq_i = freq_sum / F_OVERSAMPLE;
    freq_f = freq_sum - freq_i * F_OVERSAMPLE;

    //send the string
    //Serial.print("freq_error = "); Serial.print(freq_error); Serial.print("Hz.\n\r");
    Serial1.print("freq = "); Serial1.print(freq); Serial1.print("Hz.\n\r");
    //Serial.print("freq = "); Serial.print(freq_i); Serial.print("."); Serial.print((freq_f * 1000 + F_OVERSAMPLE / 2) / F_OVERSAMPLE); Serial.print("Hz, error = "); Serial.print(freq_i - F_CLK); Serial.print("Hz.\n\r");
    //sprintf(uRAM, "freq = %8ld.%03dHz", freq_i, (freq_f * 1000 + F_OVERSAMPLE / 2) / F_OVERSAMPLE); Serial.print(uRAM); Serial.print(", error = "); Serial.print(freq_i - F_CLK); Serial.print("Hz.\n\r");
    //blink the led
    digitalWrite(13, !digitalRead(13)); //flip pin 13
  }
    
}

