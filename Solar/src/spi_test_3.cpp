/*
 * spi_test_3.cpp
 *
 *  Created on: 13.11.2018
 *      Author: pavel
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "stdlib.h"
#include <string>
#include "MAXIM1249.h"
#include "FreeRTOS.h"
#include "task.h"

#define DIFF_LIM 40

/* Sets up system hardware */
static void prvSetupHardware(void){

	SystemCoreClockUpdate();
	Board_Init();
	/* Initial LED0 state is on */
	Board_LED_Set(0, true);
}


/*Calculate steps*/
uint16_t calcSteps(uint16_t ldr1, uint16_t ldr2){

	if(ldr1 > ldr2){
		return ((float)ldr1/(float)ldr2)*(float)100;
	}
	else{
		return ((float)ldr2/(float)ldr1)*(float)100;
	}
}


/*Task that reads ADC values*/
static void vTask_ADC(void* pvPrameters){

	SPI spi;							/*SPI communication class*/
	MAXIM1249 maxim(&spi);				/*ADC(MAXIM 1249) class*/

	uint16_t north_ldr = 0;
	uint16_t south_ldr = 0;
	uint16_t west_ldr = 0;
	uint16_t east_ldr = 0;

	std::string tilt_dir = "";
	std::string rot_dir = "";


	int tilt_steps = 0;					/*amount of steps for tilting*/
	int rot_steps = 0;					/*amount of steps for rotation*/

	char uart_buff[30] = {0};
	char steps_buff[30] = {0};
	char dir_buff[30]={0};

	while(1){

		north_ldr = maxim.getChannelAvrg(20, 0);
		south_ldr = maxim.getChannelAvrg(20, 3);
		west_ldr = maxim.getChannelAvrg(20, 1);
		east_ldr = maxim.getChannelAvrg(20, 2);

		/*calculate steps if difference between ldrs exceeds DIFF_LIM*/
		if(abs(north_ldr - south_ldr) > DIFF_LIM){

			tilt_steps = calcSteps(north_ldr, south_ldr);

			/*if north_ldr gets more light than south_ldr move down*/
			if(north_ldr > south_ldr){

				/*move down*/
				tilt_dir = "down";
			}
			else{
				/*move up*/
				tilt_dir = "up";
			}
		}
		else{
			tilt_dir = "stop";
		}
		if(abs(east_ldr - west_ldr) > DIFF_LIM){

			rot_steps = calcSteps(east_ldr, west_ldr);

			if(east_ldr > west_ldr){
				rot_dir = "right";
			}
			else{
				rot_dir = "left";
			}
		}
		else{
			rot_dir = "stop";
		}

		sprintf(uart_buff,"N: %d	S: %d	W: %d	E:%d\n\r", north_ldr, south_ldr, west_ldr, east_ldr);
		sprintf(dir_buff, "tilt_dir: %s		rot_dir: %s\n\r", tilt_dir.c_str(),rot_dir.c_str());

		Board_UARTPutSTR(uart_buff);
		Board_UARTPutSTR(dir_buff);

		vTaskDelay(500);
	}
}

int main(){

	prvSetupHardware();

	/*ADC Task*/
	xTaskCreate(vTask_ADC, "vTask_ADC",
			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	return 0;
}

