#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "SEGGER_RTT.h" //for jlink log

#include "demo.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	
	//task init
  	demo_task_init();

	vTaskStartScheduler();
  	for(;;);
}

#if configCHECK_FOR_STACK_OVERFLOW
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	SEGGER_RTT_printf(0, "------------------------------------------------\n");
	SEGGER_RTT_printf(0, "xxxxxx--stack over flow: taskName = %s\n", pcTaskName);
	vTaskDelay(1000/portTICK_PERIOD_MS);
}
#endif

