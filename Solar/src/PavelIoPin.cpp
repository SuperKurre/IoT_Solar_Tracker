/*
 * PavelIoPin.cpp
 *
 *  Created on: 18 Dec 2018
 *      Author: Lauri
 */

#include "PavelIoPin.h"
#include "chip.h"
PavelIoPin::PavelIoPin(int port, int pin, bool input, bool pullup,bool invert) {
	// TODO Auto-generated constructor stub
	port_b = port;
	pin_b = pin;
	inv = invert;
	in=input;
	if (input) {
		if (pullup) {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_MODE_PULLUP | IOCON_DIGMODE_EN | IOCON_INV_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);

			}
		}

		else {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_DIGMODE_EN | IOCON_INV_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_DIGMODE_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			}
		}
	} else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,IOCON_MODE_INACT | IOCON_DIGMODE_EN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
		Chip_GPIO_SetPinState(LPC_GPIO, port, pin, true);

}
}
bool PavelIoPin::read(){
	bool state = Chip_GPIO_GetPinState(LPC_GPIO, port_b, pin_b);
	if (!in) {
		if (inv) {
			state = !state;
		}

	}
return state;
}

void PavelIoPin::write(bool value){
	if (!in) {
				if (inv) {
					value = !value;
				}

			}
	Chip_GPIO_SetPinState(LPC_GPIO, port_b, pin_b, value);
}

PavelIoPin::~PavelIoPin() {
	// TODO Auto-generated destructor stub
}

