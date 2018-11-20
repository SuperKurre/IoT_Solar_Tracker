/*
 * MAX1249.h
 *
 *  Created on: 12.11.2018
 *      Author: pavel
 */

#ifndef MAXIM1249_H_
#define MAXIM1249_H_

#include "SPI.h"

class MAXIM1249 {

private:
	SPI* spi;
	const uint16_t READ_CH0 = 0x9F;		/*command for reading channel 0*/
	const uint16_t READ_CH1 = 0xDF;		/*command for reading channel 1*/
	const uint16_t READ_CH2 = 0xAF;		/*command for reading channel 2*/
	const uint16_t READ_CH3 = 0xEF;		/*command for reading channel 3*/

public:
	MAXIM1249(SPI* spi);
	uint16_t readChannel(int channel);
	uint16_t getChannelAvrg(int samples, int channel);
	virtual ~MAXIM1249();
};

#endif /* MAXIM1249_H_ */
