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
#include "queue.h"
#include "RapiData.h"

#define DIFF_LIM 40

QueueHandle_t rapiQueue; //send rapi_data into the queue, send the enqueued datapackets into the rapi device



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


static void vTask_send_data_to_rapi(void* pvPrameters){

	RapiData rapidata;
	char starting[2]{2, 0}; //start-of-text ascii
	char ending[2]{4, 0}; //end-of-transmission ascii eot
	std::string message;

	while(1){
		xQueueReceive(rapiQueue, &rapidata, portMAX_DELAY); //block until  get rapidata

		Board_UARTPutSTR(starting); //initiate transimission round

		if(rapidata.tiltDir==1)
			message += "tiltdir=up,";
		else
			message += "tiltdir=down,";

		if(rapidata.rotateDir==1)
			message += "rotatedir=clockwise,";
		else
			message += "rotatedir=counterclockwise,";

		message += "north_ldr=";
		message += std::to_string(rapidata.northLDR);
		message += ',';

		message += "south_ldr=";
		message += std::to_string(rapidata.southLDR);
		message += ',';

		message += "west_ldr=";
		message += std::to_string(rapidata.westLDR);
		message += ',';

		message += "east_ldr=";
		message += std::to_string(rapidata.eastLDR);
		//last part into message was formatted above


		Board_UARTPutSTR( message.c_str() ); // then send message to uart

		/*send the ending part of uart message transmission*/
		Board_UARTPutSTR(ending);

		message=""; //reset the message in preparation of next round


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

		int tdir, rdir;	//temp variables for initalize rapidata
		if (tilt_dir=="down")
			tdir = 0;
		else
			tdir = 1;

		if(rot_dir == "right")
			rdir = 1;
		else
			rdir = 0;

		/*send rapidata to the queue, so the sendingtask can send the data*/
		RapiData rd(tdir, rdir, (int)north_ldr, (int)south_ldr, (int)west_ldr, (int)east_ldr );
		xQueueSendToBack(rapiQueue, &rd, 0 ); //send to queue, but dont block, and dont wait
//		sprintf(uart_buff,"N: %d	S: %d	W: %d	E:%d\n\r", north_ldr, south_ldr, west_ldr, east_ldr);
//		sprintf(dir_buff, "tilt_dir: %s		rot_dir: %s\n\r", tilt_dir.c_str(),rot_dir.c_str());
//
//		Board_UARTPutSTR(uart_buff);
//		Board_UARTPutSTR(dir_buff);

		vTaskDelay(500);
	}
}

int main(){

	prvSetupHardware();
	rapiQueue = xQueueCreate(20, sizeof(RapiData));
	/*ADC Task*/
	xTaskCreate(vTask_ADC, "vTask_ADC",
			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(vTask_send_data_to_rapi, "sendRapiData",
			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	return 0;
}

