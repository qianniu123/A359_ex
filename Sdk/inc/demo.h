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
    M_STOP  = 0,
    M_FORWARD,
    M_BACKWARD,
    M_BRAKE,
}m_state_e;

typedef enum
{
    M_ab,
    M_c,
    M_d,
}m_index_e;

typedef enum
{
    BUTTON_DOWN = 0,
    BUTTON_UP   = 1,
    BUTTON_PRESS,
    
}button_state_t;

typedef enum 
{
    SW_DOWN = 0, 
    SW_UP   = 1,

}switch_state_t;

typedef enum 
{
    WINDOW_STOP = 0,  
     
    WINDOW_OPEN,
    WINDOW_FORWARD,
    WINDOW_UP,
    WINDOW_OPENED,
    
    WINDOW_CLOSE,
    WINDOW_DOWN,
    WINDOW_BACKWARD,
    WINDOW_CLOSED,
}window_state_t;

typedef enum 
{
    SW_STATE_NULL = 0,
    SW1_STATE_CHANGE,
    SW2_STATE_CHANGE,
    SW3_STATE_CHANGE,
    SW4_STATE_CHANGE,
    SW5_STATE_CHANGE,
    SW6_STATE_CHANGE,
}sw_state_t;


typedef struct 
{
    uint8_t msg_id;
    uint8_t value;
}msg_t;


typedef struct 
{
    uint8_t state;//open, close

}window_t;

extern void demo_task_init(void);

#endif
