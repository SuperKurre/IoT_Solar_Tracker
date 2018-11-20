/*
 * SPI.h
 *
 *	SPI class for SPI communication.
 *
 *  Created on: 6.11.2018
 *      Author: pavel
 */

#ifndef SPI_H_
#define SPI_H_

#include "board.h"

class SPI {

private:
	const uint8_t POST_DELAY 	   = 2;
	const uint8_t PRE_DELAY        = 2;
	const uint8_t TRANSFER_DELAY   = 2;
	const uint8_t FRAME_DELAY 	   = 2;

public:
	SPI();
	void Init_PinMux();
	void Setup_Master();
	bool Write_SPI_Data(uint16_t* tx_data_buff);
	bool Read_SPI_Data(uint16_t* rx_data_buff);
	bool RW_SPI_Data(uint16_t* tx_data_buff, uint16_t* rx_data_buff);
	virtual ~SPI();
};

#endif /* SPI_H_ */
