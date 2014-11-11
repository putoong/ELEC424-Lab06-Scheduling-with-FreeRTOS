We modified GCC/ARM_CM3/port.c in the FreeRTOS library. Please also add the following lines in that file.

void SVC_Handler(void) __attribute__ ((alias("vPortSVCHandler")));
void PendSV_Handler(void) __attribute__ ((alias("xPortPendSVHandler")));
void SysTick_Handler(void) __attribute__ ((alias("xPortSysTickHandler")));