/*
 * RapiData.cpp
 *
 *  Created on: 27 Nov 2018
 *      Author: Lauri
 */

#include "RapiData.h"




RapiData::~RapiData() {
	// TODO Auto-generated destructor stub
}

std::string RapiData::getStringRapiData(){
	std::string str(std::to_string(tiltDir)); //initialize string
	str += ' '; //append whitespace to delimit the categories of rapidata values

	str += std::to_string(rotateDir);
	str += ' ';
	str += std::to_string(northLDR);
	str += ' ';
	str += std::to_string(southLDR);
	str += ' ';
	str += std::to_string(westLDR);
	str += ' ';
	str += std::to_string(eastLDR);
	str += '\n'; //always end rapidata strings with newline
	//for uart_transmission purposes (and uart_receiving on raspberry)

	return str;
}

