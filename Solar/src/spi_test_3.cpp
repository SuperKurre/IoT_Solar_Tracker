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
#include <cstring>
#include "SerialPort.h"
#include "DigitalIoPin.h"


#include <mutex>
#include "semphr.h"
#include "queue.h"

#define DIFF_LIM 40

QueueHandle_t rapiQueue; //send rapi_data into the queue, send the enqueued datapackets into the rapi device


#define LED_ON 				1
#define LED_OFF 			0

#define LEFT				1
#define RIGHT				0
#define UP					1
#define DOWN				0
#define MOTOR_ON			1
#define MOTOR_OFF			0
#define REVERSE				20


struct motorCmd {
	uint32_t cmd;
	uint32_t pulses;
	uint32_t pps;
};

DigitalIoPin *Motor1Direction;
DigitalIoPin *Motor1;
//DigitalIoPin *Motor2Direction;
//DigitalIoPin *Motor2;


SemaphoreHandle_t xUARTSemaphore;
SemaphoreHandle_t sbRIT;
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xQueueSemaphore;
QueueHandle_t xQueue;

int direction = 1;
volatile int motorState = 0;
volatile bool m1state=false;
int ppsValue = 1500;
volatile uint32_t motor1steps = 0;
volatile uint32_t RIT_count;


extern "C" {

void RIT_IRQHandler(void) {
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
	if ( motor1steps > 0 ) {
		Motor1->write(m1state);
		motor1steps--;
//		if (motor2steps != 0) {
//			Motor2->write(motorState);
//			motor2steps--;
//		}
//		motorState ^= 1;
		m1state = !m1state;
		// do something useful here...
	} else {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		Motor1->write(false);
		m1state=false;
//		Motor2->write(MOTOR_OFF);
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

}




/* Sets up system hardware */
static void prvSetupHardware(void){

	SystemCoreClockUpdate();
	Board_Init();
	/* Initial LED0 state is on */
	Board_LED_Set(0, true);
}


void RIT_start(int steps) {
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * 275 / 1000000;
	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	motor1steps = steps;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	} else {
		// unexpected error
	}
}


//void RIT_start(int steps) {
//	uint64_t cmp_value;
//	// Determine approximate compare value based on clock rate and passed interval
//	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * 275 / 1000000;
//	// disable timer during configuration
//	Chip_RIT_Disable(LPC_RITIMER);
//	// enable automatic clear on when compare value==timer value
//	// this makes interrupts trigger periodically
//	Chip_RIT_EnableCompClear(LPC_RITIMER);
//	// reset the counter
//	Chip_RIT_SetCounter(LPC_RITIMER, 0);
//	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
//	// start counting
//	Chip_RIT_Enable(LPC_RITIMER);
//	// Enable the interrupt signal in NVIC (the interrupt controller)
//	NVIC_EnableIRQ(RITIMER_IRQn);
//	// wait for ISR to tell that we're done
//	if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
//		// Disable the interrupt signal in NVIC (the interrupt controller)
//		NVIC_DisableIRQ(RITIMER_IRQn);
//	} else {
//		// unexpected error
//	}
//}


/*Calculate steps*/
uint16_t calcSteps(uint16_t ldr1, uint16_t ldr2){

	if(ldr1 > ldr2){
		return ((float)ldr1/(float)ldr2)*(float)100;
	}
	else{
		return ((float)ldr2/(float)ldr1)*(float)100;
	}
}

static void vTask_real_rapi_uart_task(void* pvParameters){
	RapiData rp;
	std::string tx_message;
	SerialPort serial; //initialize serial usart_1 for uart.
	serial.begin(9600); //baud rate


	for(;;){
		xQueueReceive(rapiQueue, &rp, portMAX_DELAY); //blocking queue-read
		tx_message = rp.getStringRapiData();
		serial.write(tx_message.c_str(), strlen(tx_message.c_str()) ); //send char array and length
		vTaskDelay(1000); //necessary delay (of some size) after transmitting (delay > 10)
		//if there is no delay at all after transmitting, then you can overfill the uart tx_ring buffers
		//which will fuck it up.
		//typically if the buffer overfill happens, then only the first char is ever sent always...?! (debugged with logicanalyzer)
	}
}

static void vTask_rapi_communication(void* pvPrameters){

	RapiData rapidata;
	const char startingchar = 2; //start-of-text ascii
	const char endingchar = 4; //end-of-transmission ascii eot
	std::string tx_message;
	int rx_letter;
	int rx_count = 0;
	int counter=0;
	//int sentCounter=0; for debugging uart only!
	SerialPort serial; //create serialport object to allow uart transmissions (using UART_1)
	serial.begin(9600); //start at 9600 speed baud
	char inst1[]="FIRSTMESSAGE...";


	while(true){
			if(counter == 0){
				tx_message="hello12345\n"; //send newline terminated strings !!! must send newline_terminated
				serial.write(tx_message.c_str(), strlen(tx_message.c_str()) ); //send char array and length
				counter++;
				//sentCounter++;  for debugging uart only!
				tx_message=""; //reset the message in preparation of next round
			} else if(counter == 1){
				tx_message="shit123\n";
				serial.write(tx_message.c_str(), strlen(tx_message.c_str()) ); //send char array and length
				counter++;
				//sentCounter++;  for debugging uart only!
				tx_message=""; //reset the message in preparation of next round
			} else if(counter == 2){
				tx_message="piss456\n";
				serial.write(tx_message.c_str(), strlen(tx_message.c_str()) ); //send char array and length
				counter++;
				//sentCounter++;  for debugging uart only!
				tx_message=""; //reset the message in preparation of next round
			}else{
				counter=0;
				tx_message=""; //reset the message in preparation of next round
			}
			vTaskDelay(1000); //necessary delay (of some size) after transmitting (delay > 10) //NOTE::! you must increase delay proportionally compared to the tx_data_length


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
	Board_UARTPutSTR("HellowWorld!\n\r");

	while(1){

		north_ldr = maxim.getChannelAvrg(20, 0);
		south_ldr = maxim.getChannelAvrg(20, 3);
		west_ldr = maxim.getChannelAvrg(20, 1);
		east_ldr = maxim.getChannelAvrg(20, 2);

		/*calculate steps if difference between ldrs exceeds DIFF_LIM*/
		if(abs(north_ldr - south_ldr) > DIFF_LIM){
//			tilt_steps = calcSteps(north_ldr, south_ldr);
//			motor1steps=tilt_steps;
			/*if north_ldr gets more light than south_ldr move down*/
			if(north_ldr > south_ldr){
				Motor1Direction->write(false);
				/*move down*/
				tilt_dir = "down";
			}
			else{
				/*move up*/
				Motor1Direction->write(true);
				tilt_dir = "up";
			}
			RIT_start(1000);
		}
		else{
			tilt_dir = "stop";
//			motor1steps=0;
		}
//		if(abs(east_ldr - west_ldr) > DIFF_LIM){
//
//			rot_steps = calcSteps(east_ldr, west_ldr);
////			motor2steps=rot_steps;
//			if(east_ldr > west_ldr){
//				rot_dir = "right";
////				Motor2Direction->write(false);
//			}
//			else{
//				rot_dir = "left";
////				Motor2Direction->write(true);
//			}
//		}
//		else{
//			rot_dir = "stop";
////			motor2steps=0;
//		}

	//	motor1steps = tilt_steps;
		//motor2steps= rot_steps;
//		RIT_start(tilt_steps);

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
//
//		sprintf(uart_buff,"N: %d	S: %d	W: %d	E:%d\n\r", north_ldr, south_ldr, west_ldr, east_ldr);
//		sprintf(dir_buff, "tilt_dir: %s		rot_dir: %s\n\r", tilt_dir.c_str(),rot_dir.c_str());
//		Board_UARTPutSTR(uart_buff);
//		Board_UARTPutSTR(dir_buff);

		vTaskDelay(10);
	}
}

int main(){

	prvSetupHardware();
	rapiQueue = xQueueCreate(20, sizeof(RapiData));

	sbRIT = xSemaphoreCreateBinary();
	vQueueAddToRegistry(sbRIT, "RITMutex");

	// initialize RIT (= enable clocking etc.)
		Chip_RIT_Init(LPC_RITIMER);
		// set the priority level of the interrupt
		// The level must be equal or lower than the maximum priority specified in FreeRTOS config
		// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
		NVIC_SetPriority(RITIMER_IRQn,
				configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);

		Motor1Direction = new DigitalIoPin(1, 0, DigitalIoPin::output);
		Motor1 = new DigitalIoPin(0, 24, DigitalIoPin::output);



	/*ADC Task*/
	xTaskCreate(vTask_ADC, "vTask_ADC",
			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(vTask_real_rapi_uart_task, "rapi_comm",
			configMINIMAL_STACK_SIZE*8 , NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	return 0;
}

