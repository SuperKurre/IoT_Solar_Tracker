/*
 * DCmotor.cpp
 *
 *  Created on: 24 Dec 2018
 *      Author: Lauri
 */

#include "DCmotor.h"

DCmotor::DCmotor(PavelIoPin* h1pin, PavelIoPin* h2pin, int enaPort, int enaPin, uint16_t pwm_hz)   {
	h1 = h1pin;
	h2 = h2pin;
	enable_port = enaPort;
	enable_pin = enaPin;
	match0value = convertToMatch0(pwm_hz);
	setupEnablePinPWM(match0value);
}

DCmotor::~DCmotor() {
	/*note!:: program will continue with the DCmotor object
	 * being alive for the duration of the program, so that
	 * proper destructor isn't that much necessary
	 *
	 * same is true for IoPin objects*/
}

uint16_t DCmotor::convertToMatch0(uint16_t pwmfreqhz){
	/*converts the pwmfreq_hz into match0 delayvalue into sctimer*/
	double pwmperiod = 1.0 / (static_cast<double>(pwmfreqhz));
	double res =  std::round(pwmperiod * 1000000);
	uint16_t res1 = static_cast<uint16_t> (res);
	return res1;
}

void DCmotor::setupEnablePinPWM(uint16_t match0val ){
	Chip_SCT_Init(LPC_SCT1);
	LPC_SCT1->CONFIG |= (1 << 17);	 // two 16-bit timers, auto limit, use  the lowcounter enable pwm
	LPC_SCT1->CTRL_L |= (72 - 1) << 5; 	// set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz
	LPC_SCT1->MATCHREL[0].L = match0val - 1; 	//pwm freq
	LPC_SCT1->MATCHREL[1].L = 0; 	// sct1  pulsewidth, initially off
	Chip_SWM_MovablePortPinAssign(SWM_SCT1_OUT1_O, enable_port, enable_pin);  //set enablepin pwm
	/*configure events, freq event , and pulsewidthmatch event */
	LPC_SCT1->EVENT[2].STATE = 0xFFFFFFFF; //all states allowed event2, use for freqmatch
	LPC_SCT1->EVENT[3].STATE = 0xFFFFFFFF; //all states allowed event3, use for pulsewidthmatch
	LPC_SCT1->EVENT[2].CTRL = (1<<12)  ; 	//event2 sct1 frequency match, select reg0 (matchrel[0].l)
	LPC_SCT1->EVENT[3].CTRL = (1<<12) | (1<<0); 	//event3 sct1 pulsewidthmatch, HEVENTbitTrue,  select reg1 (matchrel[1].l)
	/*set outputs with freq events*/
	LPC_SCT1->OUT[1].SET = (1<<2); //event2 sets sct1output1 (on)
	/*clears outputs with pulsewidthmatch events*/
	LPC_SCT1->OUT[1].CLR = 	1<<3;	//event3 clears sct1 output1 (off)
	/*unhalt timers*/
	LPC_SCT1->CTRL_L &= ~(1<<2);  //unhalt sct1 counter
}



void DCmotor::runMotor(uint16_t dutycycle_percent){
	if(dutycycle_percent == 0 ){
		LPC_SCT1->MATCHREL[1].L = 0;
	}else{
		double newmatch1 = dutycycle_percent/100.0;
		newmatch1 = std::round(match0value * newmatch1);
		LPC_SCT1->MATCHREL[1].L = static_cast<uint16_t>(newmatch1);
	}

}
