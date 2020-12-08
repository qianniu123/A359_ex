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



typedef enum 
{
    M_STOP,
    M_FORWARD,
    M_BACKWARD,
}m_state_e;

typedef enum
{
    M_ab,
    M_c,
    M_d,
}m_index_e;

typedef enum
{
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_PRESS,
    
}button_state_t;

typedef enum 
{
    WINDOW_OPEN,
    WINDOW_CLOSE,
    //-------------
    WINDOW_FORWARD,
    WINDOW_BACKWARD,
    WINDOW_UP,
    WINDOW_DOWN,
}window_state_t;

typedef enum 
{
    SW_NULL,
    SW1_DOWN,
    SW2_DOWN,
    SW3_DOWN,
    SW4_DOWN,
    SW5_DOWN,
    SW6_DOWN,
}sw_state_t;


typedef struct 
{
    uint8_t msg_id;
    uint8_t value;
}msg_t;



extern void demo_task_init(void);

#endif
