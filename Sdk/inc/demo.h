#ifndef __DEMO__H
#define __DEMO__H

#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_rcc.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"


#include "SEGGER_RTT.h" //for jlink log

#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


extern void demo_task_init(void);





#endif
