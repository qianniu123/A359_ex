#include "demo.h"

#define demo_printf(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

#define DEMO_TASK_NAME          "DEMO_TASK"
#define DEMO_TASK_PRI           3 
#define DEMO_TASK_STK_SIZE      128

#define BUTTON_TASK_NAME        "BUTTON_TASK"
#define BUTTON_TASK_PRI         2          //low pri 
#define BUTTON_TASK_STK_SIZE    128

#define WINDOW_TASK_NAME         "MOTOR_TASK"
#define WINDOW_TASK_PRI          3 
#define WINDOW_TASK_STK_SIZE     128

#define SW_TASK_NAME            "SW_TASK"
#define SW_TASK_PRI             5           //high pri 
#define SW_TASK_STK_SIZE        128

#define SW_QUEUE_SIZE       5
QueueHandle_t queue_sw = NULL;
#define MOTOR_QUEUE_SIZE    5
QueueHandle_t queue_motor = NULL;

window_t window = {0};
//-------------------------------------------------------------------------------
//PWM -> PA8 PA9 PA10 PA11   PC8 PC9
#define M_ab(state, pwm)  do{}while(0); //state: 0->stop;1->forward;2->backward


//EXTI-> PC0 PC1 PC2 PC3 PC4 PC5
#define SWITCH_PORT         GPIOC
#define SW2_PIN             GPIO_Pin_0
#define SW1_PIN             GPIO_Pin_1
#define SW4_PIN             GPIO_Pin_2
#define SW3_PIN             GPIO_Pin_3
#define SW6_PIN             GPIO_Pin_4
#define SW5_PIN             GPIO_Pin_5

#define SW1_IRQn            EXTI1_IRQn
#define SW2_IRQn            EXTI0_IRQn
#define SW3_IRQn            EXTI3_IRQn
#define SW4_IRQn            EXTI2_IRQn
#define SW5_IRQn            EXTI9_5_IRQn
#define SW6_IRQn            EXTI4_IRQn
//---
#define SW_BAC_DATA         GPIO_ReadInputDataBit(SWITCH_PORT, SW1_PIN)
#define SW_FOR_DATA         GPIO_ReadInputDataBit(SWITCH_PORT, SW2_PIN)
#define SW_DOW1_DATA        GPIO_ReadInputDataBit(SWITCH_PORT, SW3_PIN)
#define SW_UP1_DATA         GPIO_ReadInputDataBit(SWITCH_PORT, SW4_PIN)
#define SW_DOW2_DATA        GPIO_ReadInputDataBit(SWITCH_PORT, SW5_PIN)
#define SW_UP2_DATA         GPIO_ReadInputDataBit(SWITCH_PORT, SW6_PIN)

//BUTTON -> PB0 PB1
#define BUTTON_PORT         GPIOB
#define BUTTON_FORWARD_PIN  GPIO_Pin_9
#define BUTTON_REVERSE_PIN  GPIO_Pin_8
//---
#define BUTTON_FORWARD_DATA GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_FORWARD_PIN)
#define BUTTON_REVERSE_DATA GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_REVERSE_PIN)

//led -> PB3
#define LED_PORT            GPIOB
#define LED_PIN             GPIO_Pin_3
#define LED_RD_DATA         GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) //GPIO_ReadInputDataBit(LED_PORT, LED_PIN) //
#define LED_WR_DATA(val)    GPIO_WriteBit(LED_PORT, LED_PIN, val)
//----------------------------------------------------------------------
static void demo_task(void *param);
static void button_task(void *param);
static void switch_task(void *param);
static void window_task(void *param);
//----------------------------------------------------------------------
static void motor_ctrl(uint8_t motor_index, uint8_t state, uint8_t speed)
{
    switch(state)
    {
        case M_STOP:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare4(TIM1, 0); //FOR_F
                TIM_SetCompare3(TIM1, 0);   //BAC_BS
            }
            if(motor_index == M_c)
            {
                TIM_SetCompare2(TIM1, 0); //UP1_F
                TIM_SetCompare1(TIM1, 0);   //DOW1_B 
            }
            if(motor_index == M_d)
            {
                TIM_SetCompare4(TIM3, 0); //UP2_F
                TIM_SetCompare3(TIM3, 0);   //DOW2_B 
            } 
        }
        break;
        case M_FORWARD:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare4(TIM1, 255); //FOR_F
                TIM_SetCompare3(TIM1, 0);   //BAC_BS
            }
            if(motor_index == M_c)
            {
                TIM_SetCompare2(TIM1, 255); //UP1_F
                TIM_SetCompare1(TIM1, 0);   //DOW1_B 
            }
            if(motor_index == M_d)
            {
                TIM_SetCompare4(TIM3, 255); //UP2_F
                TIM_SetCompare3(TIM3, 0);   //DOW2_B
            } 
        }
        break;
        case M_BACKWARD:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare4(TIM1, 0);   //FOR_F
                TIM_SetCompare3(TIM1, 255); //BAC_BS
            }
            if(motor_index == M_c)
            {
                TIM_SetCompare2(TIM1, 0);   //UP1_F
                TIM_SetCompare1(TIM1, 255); //DOW1_B //255
            }
            if(motor_index == M_d)
            {
                TIM_SetCompare4(TIM3, 0);   //UP2_F
                TIM_SetCompare3(TIM3, 255); //DOW2_B //255
            } 
        }
        break;
        case M_BRAKE:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare4(TIM1, 128); //FOR_F
                TIM_SetCompare3(TIM1, 128);   //BAC_BS
            }
            if(motor_index == M_c)
            {
                TIM_SetCompare2(TIM1, 128); //UP1_F
                TIM_SetCompare1(TIM1, 128);   //DOW1_B 
            }
            if(motor_index == M_d)
            {
                TIM_SetCompare4(TIM3, 128); //UP2_F
                TIM_SetCompare3(TIM3, 128);   //DOW2_B
            } 
        }
        break;
        default:
        break;
    }
}

static void led_toggle(void)
{
    if(LED_RD_DATA == 0)
    {
        LED_WR_DATA(Bit_SET);
        //demo_printf("led on\n");
    }
    else 
    {
        LED_WR_DATA(Bit_RESET);
        //demo_printf("led off\n");
    }
}

static void exti_irq_set(IRQn_Type type, FunctionalState state)
{
    uint32_t exti_linex = 0;
    switch(type)
    {
        case EXTI0_IRQn:
        {
            exti_linex = EXTI_Line0;
        }
        break;
        case EXTI1_IRQn:
        {
            exti_linex = EXTI_Line1;
        }
        break;
        case EXTI2_IRQn:
        {
            exti_linex = EXTI_Line2;
        }
        break;
        case EXTI3_IRQn:
        {
            exti_linex = EXTI_Line3;
        }
        break;
        case EXTI4_IRQn:
        {
            exti_linex = EXTI_Line4;
        }
        break;
        case EXTI9_5_IRQn:
        {
            exti_linex = EXTI_Line5;
        }
        break;
        default:
        break;
    }
    EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line    = exti_linex;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = state;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = type;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = state;
	NVIC_Init(&NVIC_InitStructure);
}

static void change_window_state(uint8_t new_state)
{
    msg_t msg = {new_state, 0};
    xQueueSend(queue_motor, &msg, 0);
}
static uint8_t get_window_state(void)
{
    return window.state;
}
//----------------------------------------------------------------------

static void bsp_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
//--------------------------------------------------------------
//pwm
#if 0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //TIM_CH1-4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
#elif 1
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //TIM_CH1-4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    //GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);

	TIM_TimeBaseStructure.TIM_Period = 255;//SystemCoreClock/1000;// ???
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE); 
    
    #if 0
    TIM_SetCompare1(TIM1, 50);//SystemCoreClock/(2*1000)); // ???
    TIM_SetCompare2(TIM1, 50);//SystemCoreClock/(2*1000)); //
    TIM_SetCompare3(TIM1, 100);//SystemCoreClock/(2*1000)); //
    TIM_SetCompare4(TIM1, 100);//SystemCoreClock/(2*1000)); //
    #endif
#endif
//--------------------------------------------------------------
//pwm
#if 1
    //PC8,PC9 -> TIM3 -> TIM_CH3-4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    //TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_TimeBaseStructure.TIM_Period = 255;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    
    TIM_OC3Init(TIM3, &TIM_OCInitStructure); 
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); 
	
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
    
    #if 0
    TIM_SetCompare3(TIM3, 100);//SystemCoreClock/(2*1000)
    TIM_SetCompare4(TIM3, 100);//SystemCoreClock/(2*1000)
    #endif
#endif
//--------------------------------------------------------------
//switch
#if 1
    //GPIO_InitTypeDef        GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //exti, need enable AFIO clk

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);//SW2
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);//SW1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);//SW4
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);//SW3
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);//SW6
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);//SW5

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
//----------------------------------------------------------------------
//button
#if 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BUTTON_FORWARD_PIN|BUTTON_REVERSE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
    GPIO_SetBits(BUTTON_PORT, BUTTON_FORWARD_PIN|BUTTON_REVERSE_PIN);
#endif
//----------------------------------------------------------------------
//LED
#if 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    #if 1 //jtag or io (disable JTAG-DP, enable SW-DP, enable PB3 IO)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    #endif
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
    //GPIO_SetBits(LED_PORT, LED_PIN);
    GPIO_ResetBits(LED_PORT, LED_PIN);
#endif
    demo_printf("%s: bsp init\n", __func__);
}

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0))
    {
        demo_printf("%s: 22222\n", __func__);
        exti_irq_set(SW2_IRQn, DISABLE);
        msg_t msg = {SW2_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1))
    {
        demo_printf("%s: 11111\n", __func__);
        exti_irq_set(SW1_IRQn, DISABLE);
        msg_t msg = {SW1_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2))
    {
        demo_printf("%s: 44444\n", __func__);
        exti_irq_set(SW4_IRQn, DISABLE);
        msg_t msg = {SW4_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3))
    {
        demo_printf("%s: 33333\n", __func__);
        exti_irq_set(SW3_IRQn, DISABLE);
        msg_t msg = {SW3_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4))
    {
        demo_printf("%s: 666\n", __func__);
        exti_irq_set(SW6_IRQn, DISABLE);
        msg_t msg = {SW6_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5))
    {
        demo_printf("%s: 555\n", __func__);
        exti_irq_set(SW5_IRQn, DISABLE);
        msg_t msg = {SW5_STATE_CHANGE, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

void demo_task_init(void)
{
    bsp_init();
    if(pdFALSE == xTaskCreate(demo_task, 
                                DEMO_TASK_NAME,
                                DEMO_TASK_STK_SIZE,
                                NULL,
                                DEMO_TASK_PRI,
                                NULL))
    {
        demo_printf("create demo_task error\n");
    }
    
    if(pdFALSE == xTaskCreate(button_task, 
                                BUTTON_TASK_NAME,
                                BUTTON_TASK_STK_SIZE,
                                NULL,
                                BUTTON_TASK_PRI,
                                NULL))
    {
        demo_printf("create button_task error\n");
    }

    queue_motor = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(msg_t));
    if(queue_motor == NULL)
    {
       demo_printf("create queue_motor error\n"); 
    }
    if(pdFALSE == xTaskCreate(window_task, 
                                WINDOW_TASK_NAME,
                                WINDOW_TASK_STK_SIZE,
                                NULL,
                                WINDOW_TASK_PRI,
                                NULL))
    {
        demo_printf("create window_task error\n");
    }

    queue_sw = xQueueCreate(SW_QUEUE_SIZE, sizeof(msg_t));
    if(queue_sw == NULL)
    {
       demo_printf("create queue_sw error\n"); 
    }
    if(pdFALSE == xTaskCreate(switch_task, 
                                SW_TASK_NAME,
                                SW_TASK_STK_SIZE,
                                NULL,
                                SW_TASK_PRI,
                                NULL))
    {
        demo_printf("create switch_task error");
    }

    change_window_state(WINDOW_STOP);
    demo_printf("demo task init \n");
}

//===============================================
#if 1  //demo
void demo_task(void *param)
{
    uint32_t demo_cnt = 0;
    while(1)
    {
        //demo_printf("%s: .... %d\n", __func__, demo_cnt);
        demo_cnt ++;
        led_toggle();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

static void button_task(void *param)
{
    while(1)
    {
        if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
        {
            vTaskDelay(20/portTICK_PERIOD_MS);
            if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
            {
                demo_printf("%s: button forward down\n", __func__);
                //start forward -> send msg
                if(window.state < WINDOW_OPEN || window.state > WINDOW_OPENED)
                {
                    change_window_state(WINDOW_OPEN);
                }
            }
        }
        else if(BUTTON_REVERSE_DATA == BUTTON_DOWN)
        {
            vTaskDelay(20/portTICK_PERIOD_MS);
            if(BUTTON_REVERSE_DATA == BUTTON_DOWN)
            {
                demo_printf("%s: button reverse down\n", __func__);
                //start backward -> send msg
                if(window.state < WINDOW_CLOSE || window.state > WINDOW_CLOSED)
                {
                   change_window_state(WINDOW_CLOSE);
                }
            }
        }
        else 
        {
            //demo_printf("%s: button_forward_data = %d, button_backward_data = %d\n", __func__, BUTTON_FORWARD_DATA, BUTTON_REVERSE_DATA);
            if(BUTTON_FORWARD_DATA == BUTTON_UP && BUTTON_REVERSE_DATA == BUTTON_UP )
            {
                vTaskDelay(20/portTICK_PERIOD_MS);
                if(BUTTON_FORWARD_DATA == BUTTON_UP && BUTTON_REVERSE_DATA == BUTTON_UP)
                {
                    //demo_printf("%s: button forward&backward up\n", __func__);
                    //stop close -> send msg
                    if(window.state != WINDOW_STOP)
                    {
                        change_window_state(WINDOW_STOP);
                    }
                }
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);//100
    }

}

static void window_task(void *param)
{
    msg_t msg = {0};
    while(1)
    {
        if(pdTRUE == xQueueReceive(queue_motor, &msg, portMAX_DELAY))
        {
            window.state = msg.msg_id;
            //demo_printf("%s: window.state = %d\n", __func__, window.state);
            switch(window.state)
            {
                case WINDOW_STOP:
                {
                    demo_printf("%s: WINDOW_STOP\n", __func__);
                    #if 1
                    motor_ctrl(M_ab, M_STOP, 0);
                    motor_ctrl(M_c, M_STOP, 0);
                    motor_ctrl(M_d, M_STOP, 0);
                    #endif
                }
                break;
                case WINDOW_OPEN:
                {
                    demo_printf("%s: WINDOW_OPEN\n", __func__);
                    if(SW_FOR_DATA != SW_DOWN)
                    {
                        change_window_state(WINDOW_FORWARD);
                    }
                    else 
                    {
                        if(SW_UP1_DATA != SW_DOWN || SW_UP2_DATA != SW_DOWN)
                        {
                            change_window_state(WINDOW_UP);
                        }
                        else 
                        {
                            change_window_state(WINDOW_OPENED);
                        }
                    }
                }
                break;
                case WINDOW_FORWARD:
                {
                    demo_printf("%s: WINDOW_FORWARD\n", __func__);
                    if(SW_FOR_DATA != SW_DOWN)
                    {
                        motor_ctrl(M_ab, M_FORWARD, 255);
                        motor_ctrl(M_c, M_STOP, 0);
                        motor_ctrl(M_d, M_STOP, 0);
                    }
                }
                break;
                case WINDOW_UP:
                {
                    demo_printf("%s: WINDOW_UP\n", __func__);
                    if(SW_FOR_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_ab, M_STOP, 0);
                        if(SW_UP1_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_c, M_FORWARD, 255);
                        }
                        if(SW_UP2_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_d, M_FORWARD, 255);
                        }
                    }
                }
                break;
                case WINDOW_OPENED:
                {
                    demo_printf("%s: WINDOW_OPENED\n", __func__);
                    motor_ctrl(M_ab, M_STOP, 0);
                    if(SW_UP1_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_c, M_STOP, 0); 
                    }
                    if(SW_UP2_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_d, M_STOP, 0);
                    }
                }
                break;
                //--------------------------------------
                case WINDOW_CLOSE:
                {
                    demo_printf("%s: WINDOW_CLOSE\n", __func__);
                    if(SW_DOW1_DATA != SW_DOWN || SW_DOW2_DATA != SW_DOWN)
                    {
                        change_window_state(WINDOW_DOWN);
                    }
                    else 
                    {
                        if(SW_BAC_DATA != SW_DOWN) 
                        {
                            change_window_state(WINDOW_BACKWARD);
                        }
                        else 
                        {
                            change_window_state(WINDOW_CLOSED);
                        }
                    }
                }
                break;
                case WINDOW_DOWN:
                {
                    demo_printf("%s: WINDOW_DOWN\n", __func__);
                    motor_ctrl(M_ab, M_STOP, 0);
                    if(SW_DOW1_DATA != SW_DOWN)
                    {
                        motor_ctrl(M_c, M_BACKWARD, 255);
                    }
                    if(SW_DOW2_DATA != SW_DOWN)
                    {
                        motor_ctrl(M_d, M_BACKWARD, 255);
                    }
                }
                break;
                case WINDOW_BACKWARD:
                {
                    demo_printf("%s: WINDOW_BACKWARD\n", __func__);
                    if(SW_DOW1_DATA == SW_DOWN && SW_DOW2_DATA == SW_DOWN)
                    {
                        if(SW_BAC_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_ab, M_BACKWARD, 255);
                        }
                    } 
                    if(SW_DOW1_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_c, M_STOP, 0);
                    }
                    if(SW_DOW2_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_d, M_STOP, 0);
                    }
                }
                break;
                case WINDOW_CLOSED:
                {
                    demo_printf("%s: WINDOW_CLOSED\n", __func__);
                    if(SW_BAC_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_ab, M_STOP, 0);
                        motor_ctrl(M_c, M_STOP, 0);
                        motor_ctrl(M_d, M_STOP, 0); 
                    }
                }
                break;
                
                default:
                break;
            }
        }
    }
}

static void switch_task(void *param)
{
    msg_t msg = {0};
    while(1) //window state change
    {
        if(pdTRUE == xQueueReceive(queue_sw, &msg, portMAX_DELAY)) //recv msg from exti irq
        {
            uint8_t sw_state = msg.msg_id;
            uint8_t change_flag  = (get_window_state() == WINDOW_STOP?0:1);
           
            vTaskDelay(20/portTICK_PERIOD_MS);//debounce
            
            switch(sw_state)
            {
                case SW_STATE_NULL:
                {
                    demo_printf("%s: SW_STATE_NULL\n", __func__);
                }
                break;
                case SW1_STATE_CHANGE:
                {
                    demo_printf("%s: sw1 %s\n", __func__, (SW_BAC_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW1_IRQn, ENABLE);

                    if(change_flag && SW_BAC_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_CLOSED);
                    }
                    else 
                    {
                        //window opening
                    }
                }
                break;
                case SW2_STATE_CHANGE:
                {
                    demo_printf("%s: sw2 %s\n", __func__, (SW_FOR_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW2_IRQn, ENABLE);

                    if(change_flag && SW_FOR_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_UP);
                    }
                }
                break;

                case SW3_STATE_CHANGE:
                {
                    demo_printf("%s: sw3 %s\n", __func__, (SW_DOW1_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW3_IRQn, ENABLE);

                    if(change_flag && SW_DOW1_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_BACKWARD);
                    }
                }
                break;
                case SW5_STATE_CHANGE:
                {
                    demo_printf("%s: sw5 %s\n", __func__, (SW_DOW2_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW5_IRQn, ENABLE);

                    if(change_flag && SW_DOW2_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_BACKWARD);
                    }
                }
                break;

                case SW4_STATE_CHANGE:
                {
                    demo_printf("%s: sw4 %s\n", __func__, (SW_UP1_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW4_IRQn, ENABLE);

                    if(change_flag && SW_UP1_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_OPENED);
                    }
                }
                break;
                case SW6_STATE_CHANGE:
                {
                    demo_printf("%s: sw6 %s\n", __func__, (SW_UP2_DATA==SW_DOWN?"down":"up"));
                    exti_irq_set(SW6_IRQn, ENABLE);

                    if(change_flag && SW_UP2_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_OPENED);
                    }
                }
                break;
                default:
                break;
            }
        }
    }
}

#endif


