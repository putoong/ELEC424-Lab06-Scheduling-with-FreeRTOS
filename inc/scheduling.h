#ifndef __SCHEDULING_H
#define __SCHEDULING_H

#include "FreeRTOS.h"
#include "task.h"
#include "lab06_task.h"

#define SPEED 							100
static uint32_t counter;

void ChangeMotorSpeed(MotorSpeeds* p_motorSpeedsPtr);
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
void main(void);



#endif