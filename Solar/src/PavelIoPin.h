/*
 * PavelIoPin.h
 *
 *  Created on: 18 Dec 2018
 *      Author: Lauri
 */

#ifndef PAVELIOPIN_H_
#define PAVELIOPIN_H_

class PavelIoPin {
public:
	PavelIoPin(int port, int pin, bool input = true, bool pullup = true, bool invert = false);
	virtual ~PavelIoPin();
	bool read();
void write(bool value);
private:
int pin_b,port_b;
bool inv,in;
};

#endif /* PAVELIOPIN_H_ */
