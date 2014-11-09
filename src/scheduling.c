#include "scheduling.h"
#include "sys_init.h"
#include "semphr.h"


MotorSpeeds MS = {.m1 = 0, .m2 = 0, .m3 = 0, .m4 = 0};
SemaphoreHandle_t semaphore;
uint16_t 

void ChangeMotorSpeed(MotorSpeeds* p_motorSpeedsPtr)
{
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m1 * SPEED;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m2 * SPEED;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m3 * SPEED;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m4 * SPEED;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure); 

}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) {
	while (1) {

	}
}

void task1(void *pvParameters){
	TickType_t xTimer;
	const TickType_t xFeq = 10;
	xTimer = xTaskGetTickCount();
	while (1) {
		vTaskDelayUntil(&xTimer, xFeq);
		detectEmergency();
		//LED_green_toggle();
	}
 }

void task2(void *pvParameters){
	TickType_t xTimer;
	const TickType_t xFeq = 100;
	xTimer = xTaskGetTickCount();
	while (1) {
		vTaskDelayUntil(&xTimer, xFeq);
		refreshSensorData();
		xSemaphoreGive( semaphore );
		//LED_red_toggle();
	}
 }

void task3(void *pvParameters){
	while (1) {
		if( xSemaphoreTake( semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			calculateOrientation();			
		}
	}
 }

void task4(void *pvParameters){
	TickType_t xTimer;
	const TickType_t xFeq = 1000;
	xTimer = xTaskGetTickCount();
	while (1) {
		vTaskDelayUntil(&xTimer, xFeq);
		updatePid(&MS);
		ChangeMotorSpeed(&MS);
	}
}

void vApplicationIdleHook( void ){
	while (1) {
		logDebugInfo();
	}
}


void main (void){

	init_system_clk();
	init_blink();
	init_motors();
 	init_TIM2();
 	

	semaphore = xSemaphoreCreateBinary();
	//xSemaphoreTake( semaphore ,( TickType_t ) 10  );


 	xTaskCreate(	task1,
 					"task1",
 					200,
 					NULL,
 					1,
 					NULL
 					);

  	xTaskCreate(	task2,
 					"task2",
 					200,
 					NULL,
 					2,
 					NULL
 					);

  	xTaskCreate(	task3,
 					"task3",
 					200,
 					NULL,
 					3,
 					NULL
 					);

  	xTaskCreate(	task4,
 					"task4",
 					200,
 					NULL,
 					4,
 					NULL
 					);

 	vTaskStartScheduler();

	while (1) {
  }

}