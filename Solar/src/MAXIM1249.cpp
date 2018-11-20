/*
 * MAX1249.cpp
 *
 *  Created on: 12.11.2018
 *      Author: pavel
 */

#include "MAXIM1249.h"

/*constructor*/
MAXIM1249::MAXIM1249(SPI* spi) {
	this->spi = spi;

}

/*Read from specified channel*/
uint16_t MAXIM1249::readChannel(int channel){

	uint16_t tx_buffer[3] = {0x00, 0x00, 0x00};		/*data to be sent*/
	uint16_t rx_buffer[3] = {0};					/*data received*/
	uint16_t rx_data = 0;							/*parsed received (actual)data*/

	if(channel == 0){
		tx_buffer[0] = READ_CH0;
	}
	if(channel == 1){
		tx_buffer[0] = READ_CH1;
	}
	if(channel == 2){
		tx_buffer[0] = READ_CH2;
	}
	if(channel == 3){
		tx_buffer[0] = READ_CH3;
	}
	spi->RW_SPI_Data(tx_buffer, rx_buffer);

	rx_buffer[1] = rx_buffer[1] << 3;
	rx_buffer[2] = rx_buffer[2] >> 5;
	rx_data = rx_buffer[1] | rx_buffer[2];

	return rx_data;
}

/* Calculates average value of the values received from specified channel*/
uint16_t MAXIM1249::getChannelAvrg(int samples, int channel){

	int sum = 0;

	for(int i = 0; i < samples;i++){
		sum += readChannel(channel);
	}
	return sum / samples;
}

/*destructor*/
MAXIM1249::~MAXIM1249() {
	// TODO Auto-generated destructor stub
}

