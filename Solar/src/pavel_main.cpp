/*
 *	IoT Solar Tracker
 *
 *	by Pavel Dounaev, Miikka Käkelä & Lauri Solin
 *
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
#include "semphr.h"
#include "PavelIoPin.h"
#include <deque>
#include "LimitedCounter.h"

#define DEAD_ZONE_V 200
#define DEAD_ZONE_H 200


volatile uint32_t RIT_count;
SemaphoreHandle_t sbRIT;
volatile bool move = false;

/*Creating DigitalIoPins*/
PavelIoPin* STEP;
PavelIoPin* DIR;
PavelIoPin* h_pin1;
PavelIoPin* h_pin2;

extern "C" { void RIT_IRQHandler(void)
{
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER);
	// clear IRQ flag

	if(RIT_count > 0) {
		RIT_count--;
		move = !(bool)move;
		STEP->write(move);
	}
	else {
		Chip_RIT_Disable(LPC_RITIMER);
		// disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}



void setupH1PWM(){
	Chip_SCT_Init(LPC_SCT1);

	LPC_SCT1->CONFIG |= (1 << 17);	 // two 16-bit timers, auto limit, use  the lowcounter h1pin
	LPC_SCT1->CTRL_L |= (72 - 1) << 5; 	// set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz
	LPC_SCT1->MATCHREL[0].L = 1000 - 1; 	//sct1   freq 1khz
	LPC_SCT1->MATCHREL[1].L = 0; 	// sct1  pulsewidth, initially off
	Chip_SWM_MovablePortPinAssign(SWM_SCT1_OUT1_O, 0, 22);  //set h1pin to sct1 output1

	/*configure events, freq event , and pulsewidthmatch event */
	LPC_SCT1->EVENT[2].STATE = 0xFFFFFFFF; //all states allowed event2, use for freqmatch
	LPC_SCT1->EVENT[3].STATE = 0xFFFFFFFF; //all states allowed event3, use for pulsewidthmatch
	LPC_SCT1->EVENT[2].CTRL = (1<<12)  ; 	//event2 sct1 frequency match, select reg0 (matchrel[0].l)
	LPC_SCT1->EVENT[3].CTRL = (1<<12) | (1<<0); 	//event3 sct1 pulsewidthmatch, HEVENTbitTrue,  select reg1 (matchrel[1].l)

	/*set outputs with freq events*/
	LPC_SCT1->OUT[1].SET = (1<<2); //event2 sets sct1output1 (on)

	/*clears outputs with pulsewidthmatch events*/
	LPC_SCT1->OUT[1].CLR = 	1<<3;	//event3 clears sct1 output1 (off)

	/*unhalt timers*/
	LPC_SCT1->CTRL_L &= ~(1<<2);  //unhalt sct1 counter

}

void setupH2PWM(){
	Chip_SCT_Init(LPC_SCT0);

	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit, use the lowcounter for h2pin
	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz SCtimer clockfreq

	//matchrel_1 controls duty cycle , min=1000, max=2000, center=1500
	//matchrel_0 controls the main frequency
	LPC_SCT0->MATCHREL[0].L = 1000-1; // match 0 @ 1000 / 1000000Hz =  0,001s period = 1kHz drivingfrequency, as required
	LPC_SCT0->MATCHREL[1].L = 0; // match 1 used for duty cycle, initially off

	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,23);	//set h2pin to sct0 output0

	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 register, condition only
	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 register, condition only
	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0 output, (on)
	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0 (off)
	LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg
}


void setH1Value(int newval){
	if(newval >= 0 && newval <= 1000)
		LPC_SCT1->MATCHREL[1].L = newval;
}

void setH2Value(int newval){
	if(newval >= 0 && newval <= 1000)
		LPC_SCT0->MATCHREL[1].L = newval;
}


void RIT_start(int count, int us) {

	uint64_t cmp_value;

	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us / 1000000;

	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	RIT_count = count;
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
	if(xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	}
	else {
		// unexpected error
	}
}
/* Sets up system hardware */
static void prvSetupHardware(void){

	SystemCoreClockUpdate();
	Board_Init();

	// initialize RIT (= enable clocking etc.)
	Chip_RIT_Init(LPC_RITIMER);

	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority( RITIMER_IRQn, 8 + 1 );

	/* Initial LED0 state is on */
	Board_LED_Set(0, true);
}

// calculates delay based on pps
int calc_delay(int pps){

	// 1s = 1000000us
	const int mil_us = 1000000;

	return (mil_us / 2) / pps;
}


/*Calculates the running average*/
uint16_t getRunAverage(std::deque<uint16_t>ldr_dq, uint16_t ldr_val){

	std::deque<uint16_t>::iterator it;
	const int BUFFER_SIZE = 50;
	int sum = 0;

	if(ldr_dq.size() < BUFFER_SIZE){

		ldr_dq.push_back(ldr_val);
	}
	else{
		ldr_dq.pop_front();
		ldr_dq.push_back(ldr_val);
	}

	for(auto it = ldr_dq.begin(); it != ldr_dq.end(); ++it){
		sum += *it;
	}

	return sum / ldr_dq.size();
}


static void vPWM_test_task(void* taskParameters){
	PavelIoPin sw1(0,17,true, true, true);
	PavelIoPin sw2(1,11,true, true, true);
	PavelIoPin sw3(1,9,true, true, true);

/* test_task for oscilloscope PWM */
	for(;;){
		if(sw1.read()){ //set h1pin true, and h2pin false
			setH1Value(900);
			setH2Value(100);
			vTaskDelay(500);
		}else if(sw2.read()){ //set h1pin false and h2pin true
			setH1Value(100);
			setH2Value(900);
			vTaskDelay(500);
		}else if(sw3.read()){ //set both pins false
			setH1Value(0);
			setH2Value(0);
			vTaskDelay(500);
		} else{
			vTaskDelay(20);
		}
	}
}


/*Task that reads ADC values*/
static void vTask_ADC(void* pvPrameters){

	SPI spi;							/*SPI communication class*/
	MAXIM1249 maxim(&spi);				/*ADC(MAXIM 1249) class*/

	std::deque<uint16_t>north_ldr_dq;
	std::deque<uint16_t>south_ldr_dq;
	std::deque<uint16_t>west_ldr_dq;
	std::deque<uint16_t>east_ldr_dq;

	uint16_t north_ldr = 0;
	uint16_t south_ldr = 0;
	uint16_t west_ldr = 0;
	uint16_t east_ldr = 0;

	char uart_buff[30] = {0};			/*contains ldr values(used for debugging)*/
	const int PPS = 400;				/*pulses per second - "speed of the motor"*/
	const int steps = 200;

	while(1){

		north_ldr = getRunAverage(north_ldr_dq, maxim.readChannel(0));
		east_ldr = getRunAverage(east_ldr_dq, maxim.readChannel(1));
		west_ldr = getRunAverage(south_ldr_dq, maxim.readChannel(2));
		south_ldr = getRunAverage(west_ldr_dq,maxim.readChannel(3));


		/*calculate steps if difference between ldrs exceeds DIFF_LIM*/
		if(abs(north_ldr - south_ldr) > DEAD_ZONE_V){

			/*if north_ldr gets more light than south_ldr move down*/
			if(north_ldr > south_ldr){
				DIR->write(false);

			}
			else{
				DIR->write(true);
			}
			RIT_start(steps, calc_delay(PPS));
		}
		if(abs(east_ldr - west_ldr) > DEAD_ZONE_H){

			if(east_ldr > west_ldr){

//				h_pin1->write(true);
//				h_pin2->write(false);
				setH1Value(500);
				setH2Value(0);
			}
			else{

//				h_pin1->write(false);
//				h_pin2->write(true);
				setH1Value(0);
				setH2Value(500);
			}
		}
		else{
//			h_pin1->write(false);
//			h_pin2->write(false);
			setH1Value(0);
			setH2Value(0);
		}

		 sprintf(uart_buff,"N: %d	S: %d	W: %d	E:%d\n\r", north_ldr, south_ldr, west_ldr, east_ldr);
		 Board_UARTPutSTR(uart_buff);
		 Board_UARTPutSTR("\n\r");

		vTaskDelay(10);
	}
}

int main(){

	prvSetupHardware();

	sbRIT = xSemaphoreCreateBinary();
	setupH2PWM();
	setupH1PWM();
	STEP = new PavelIoPin(0, 24, false, true, false);
	DIR = new PavelIoPin(1, 0, false, true, false);

	h_pin1 = new PavelIoPin(0, 22, false, true, false);
	h_pin2 = new PavelIoPin(0, 23, false, true, false);

//	h_pin1->write(false);
//	h_pin2->write(false);
	/*ADC Task*/
//	xTaskCreate(vTask_ADC, "vTask_ADC",
//			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
//			(TaskHandle_t *) NULL);

	xTaskCreate(vPWM_test_task, "pwm__testtask",
			configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);


	vTaskStartScheduler();

	return 0;
}



