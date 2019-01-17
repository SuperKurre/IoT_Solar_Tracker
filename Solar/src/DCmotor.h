/*
 * DCmotor.h
 *
 *  Created on: 24 Dec 2018
 *      Author: Lauri
 */
#include "PavelIoPin.h"
#ifndef DCMOTOR_H_
#define DCMOTOR_H_
#include <cstdint>
#include <cmath>
#include "board.h"




class DCmotor {
public:
	DCmotor(PavelIoPin*h1, PavelIoPin*h2, int ena_port, int ena_pin, uint16_t pwmfreq);
	virtual ~DCmotor();

	uint16_t convertToMatch0(uint16_t pwmfreqhz);
	void setupEnablePinPWM(uint16_t pwmfreq_hz );
	void stopMotor();
	void runMotor(uint16_t dutypercent);

private: //datamembers

	PavelIoPin*  h1; //outputpin control motordir
	PavelIoPin*  h2; //outputpin control motordir
	//enable pin should be P0_14, but sctimer pulses it with pwm, so that pin object not needed
	int enable_port; //enable is pulsed with pwm sctimer
	int enable_pin; //enable is pulsed with pwm sctimer
	uint16_t match0value;
};

#endif /* DCMOTOR_H_ */
