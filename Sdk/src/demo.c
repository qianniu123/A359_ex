#include "demo.h"

#define demo_printf(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

#define LED_TASK_NAME          "DEMO_TASK"
#define LED_TASK_PRI           2 
#define LED_TASK_STK_SIZE      128

#define WINDOW_TASK_NAME        "MOTOR_TASK"
#define WINDOW_TASK_PRI         3 
#define WINDOW_TASK_STK_SIZE    128

#define SW_TASK_NAME            "SW_TASK"
#define SW_TASK_PRI             4           //high pri 
#define SW_TASK_STK_SIZE        256

#define SW_QUEUE_SIZE       10
QueueHandle_t queue_sw = NULL;
#define MOTOR_QUEUE_SIZE    5
QueueHandle_t queue_motor = NULL;

window_t window = {0};
char *software_version = "V0.3";
//-------------------------------------------------------------------------------
//PWM -> (Mc)PA8 PA9, (Mab)PA10 PA11, (Md)PC8 PC9
#define MOTOR_CTR_PWM_ENABLE       0

#if !MOTOR_CTR_PWM_ENABLE
#define M_AB_PORT           GPIOA
#define M_AB_F_PIN          GPIO_Pin_11
#define M_AB_B_PIN          GPIO_Pin_10

#define M_C_PORT            GPIOA
#define M_C_F_PIN           GPIO_Pin_9
#define M_C_B_PIN           GPIO_Pin_8

#define M_D_PORT            GPIOC
#define M_D_F_PIN           GPIO_Pin_9
#define M_D_B_PIN           GPIO_Pin_8
#endif

//EXTI-> PC0 PC1 PC2 PC3 PC4 PC5; PB8 PB9
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

#define BUTTON_FORWARD_IRQn EXTI9_5_IRQn
#define BUTTON_REVERSE_IRQn EXTI9_5_IRQn
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
#define BUTTON_DATA_SETBIT  GPIO_SetBits(BUTTON_PORT, BUTTON_FORWARD_PIN|BUTTON_REVERSE_PIN);
#define BUTTON_FORWARD_DATA GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_FORWARD_PIN)
#define BUTTON_REVERSE_DATA GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_REVERSE_PIN)

//led -> PB3
#define LED_PORT            GPIOB
#define LED_PIN             GPIO_Pin_3
#define LED_RD_DATA         GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) //GPIO_ReadInputDataBit(LED_PORT, LED_PIN)
#define LED_WR_DATA(val)    GPIO_WriteBit(LED_PORT, LED_PIN, val)
//----------------------------------------------------------------------
static void led_task(void *param);
static void switch_task(void *param);
static void window_task(void *param);
//----------------------------------------------------------------------
static void motor_ctrl(uint8_t motor_index, uint8_t state, uint8_t speed)
{
    #if MOTOR_CTR_PWM_ENABLE
    switch(motor_index)
    {
        case M_ab:
        {
            if(state == M_STOP)
            {
                TIM_SetCompare4(TIM1, 0); //FOR_F
                TIM_SetCompare3(TIM1, 0); //BAC_B
            }
            else if(state == M_FORWARD)
            {
                TIM_SetCompare4(TIM1, 255); //FOR_F
                TIM_SetCompare3(TIM1, 0);   //BAC_B
            }
            else if(state == M_BACKWARD) 
            {
                TIM_SetCompare4(TIM1, 0); //FOR_F
                TIM_SetCompare3(TIM1, 255); //BAC_B
            }
            else if(state == M_BRAKE)
            {
                TIM_SetCompare4(TIM1, 255); //FOR_F
                TIM_SetCompare3(TIM1, 255); //BAC_B
            }
        }
        break;
        case M_c:
        {
            if(state == M_STOP)
            {
                TIM_SetCompare2(TIM1, 0); //UP1_F
                TIM_SetCompare1(TIM1, 0); //DOW1_B
            }
            else if(state == M_FORWARD)
            {
                TIM_SetCompare2(TIM1, 255); //UP1_F
                TIM_SetCompare1(TIM1, 0); //DOW1_B
            }
            else if(state == M_BACKWARD)
            {
                TIM_SetCompare2(TIM1, 0); //UP1_F
                TIM_SetCompare1(TIM1, 255); //DOW1_B
            }
            else if(state == M_BRAKE)
            {
                TIM_SetCompare2(TIM1, 255); //UP1_F
                TIM_SetCompare1(TIM1, 255); //DOW1_B
            }
        }
        break;
        case M_d:
        {
            if(state == M_STOP)
            {
                TIM_SetCompare4(TIM3, 0); //UP2_F
                TIM_SetCompare3(TIM3, 0); //DOW2_B 
            }
            else if(state == M_FORWARD)
            {
                TIM_SetCompare4(TIM3, 255); //UP2_F
                TIM_SetCompare3(TIM3, 0); //DOW2_B 
            }
            else if(state == M_BACKWARD)
            {
                TIM_SetCompare4(TIM3, 0); //UP2_F
                TIM_SetCompare3(TIM3, 255); //DOW2_B 
            }
            else if(state == M_BRAKE)
            {
                TIM_SetCompare4(TIM3, 255); //UP2_F
                TIM_SetCompare3(TIM3, 255); //DOW2_B 
            }
        }
        break;
        default:
        break;
    }
    #else 
    switch(motor_index)
    {
        case M_ab:
        {
            if(state == M_STOP)
            {
                GPIO_WriteBit(M_AB_PORT, M_AB_F_PIN, Bit_RESET); //FOR_F
                GPIO_WriteBit(M_AB_PORT, M_AB_B_PIN, Bit_RESET); //BAC_B
            }
            else if(state == M_FORWARD)
            {
                GPIO_WriteBit(M_AB_PORT, M_AB_F_PIN, Bit_SET); //FOR_F
                GPIO_WriteBit(M_AB_PORT, M_AB_B_PIN, Bit_RESET); //BAC_B
            }
            else if(state == M_BACKWARD) 
            {
                GPIO_WriteBit(M_AB_PORT, M_AB_F_PIN, Bit_RESET); //FOR_F
                GPIO_WriteBit(M_AB_PORT, M_AB_B_PIN, Bit_SET); //BAC_B
            }
            else if(state == M_BRAKE)
            {
                GPIO_WriteBit(M_AB_PORT, M_AB_F_PIN, Bit_SET); //FOR_F
                GPIO_WriteBit(M_AB_PORT, M_AB_B_PIN, Bit_SET); //BAC_B
            }
        }
        break;
        case M_c:
        {
            if(state == M_STOP)
            {
                GPIO_WriteBit(M_C_PORT, M_C_F_PIN, Bit_RESET); //UP1_F
                GPIO_WriteBit(M_C_PORT, M_C_B_PIN, Bit_RESET); //DOW1_B
            }
            else if(state == M_FORWARD)
            {
                GPIO_WriteBit(M_C_PORT, M_C_F_PIN, Bit_SET); //UP1_F
                GPIO_WriteBit(M_C_PORT, M_C_B_PIN, Bit_RESET); //DOW1_B
            }
            else if(state == M_BACKWARD)
            {
                GPIO_WriteBit(M_C_PORT, M_C_F_PIN, Bit_RESET); //UP1_F
                GPIO_WriteBit(M_C_PORT, M_C_B_PIN, Bit_SET); //DOW1_B
            }
            else if(state == M_BRAKE)
            {
                GPIO_WriteBit(M_C_PORT, M_C_F_PIN, Bit_SET); //UP1_F
                GPIO_WriteBit(M_C_PORT, M_C_B_PIN, Bit_SET); //DOW1_B
            }
        }
        break;
        case M_d:
        {
            if(state == M_STOP)
            {
                GPIO_WriteBit(M_D_PORT, M_D_F_PIN, Bit_RESET); //UP2_F
                GPIO_WriteBit(M_D_PORT, M_D_B_PIN, Bit_RESET); //DOW2_B 
            }
            else if(state == M_FORWARD)
            {
                GPIO_WriteBit(M_D_PORT, M_D_F_PIN, Bit_SET); //UP2_F
                GPIO_WriteBit(M_D_PORT, M_D_B_PIN, Bit_RESET); //DOW2_B 
            }
            else if(state == M_BACKWARD)
            {
                GPIO_WriteBit(M_D_PORT, M_D_F_PIN, Bit_RESET); //UP2_F
                GPIO_WriteBit(M_D_PORT, M_D_B_PIN, Bit_SET); //DOW2_B 
            }
            else if(state == M_BRAKE)
            {
                GPIO_WriteBit(M_D_PORT, M_D_F_PIN, Bit_SET); //UP2_F
                GPIO_WriteBit(M_D_PORT, M_D_B_PIN, Bit_SET); //DOW2_B 
            }
        }
        break;
        default:
        break;
    }
    #endif
}

#if 0
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
            exti_linex = EXTI_Line5|EXTI_Line8|EXTI_Line9;
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
#endif

static void change_window_state(uint8_t new_state)
{
    msg_t msg = {new_state, 0};
    xQueueSend(queue_motor, &msg, 0);
}
static void update_window_state(void)
{
    msg_t msg = {window.state, 0};
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
//pwm or io
#if !MOTOR_CTR_PWM_ENABLE
    //PA8,PA9,PA10,PA11, //PC8,PC9 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //TIM_CH1-4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_8|GPIO_Pin_9);
#else
    //PA8,PA9,PA10,PA11 -> TIM1 -> TIM_CH1-4
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
#endif
//--------------------------------------------------------------
//EXTI
#if 1
//switchs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //exti, need enable AFIO clk

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//50MHZ		 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);//SW2
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);//SW1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);//SW4
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);//SW3
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);//SW6
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);//SW5
//buttons
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //exti, need enable AFIO clk
    GPIO_InitStructure.GPIO_Pin = BUTTON_FORWARD_PIN|BUTTON_REVERSE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
    GPIO_SetBits(BUTTON_PORT, BUTTON_FORWARD_PIN|BUTTON_REVERSE_PIN);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);//button forward
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);//button reverse

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5|EXTI_Line8|EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling;//!!!???
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

#define DEBOUNCE_TICK   (20*portTICK_PERIOD_MS) //ms
void EXTI0_IRQHandler(void)
{
    static TickType_t tick_cnt = 0;
    if(EXTI_GetITStatus(EXTI_Line0))
    {
        //exti_irq_set(SW2_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt + DEBOUNCE_TICK)
        {
            demo_printf("%s: 22222\n", __func__);
            tick_cnt = tick_cnt_cur;
            msg_t msg = {SW2_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
    }
}

void EXTI1_IRQHandler(void)
{
    static TickType_t tick_cnt = 0;
    if(EXTI_GetITStatus(EXTI_Line1))
    {
        //exti_irq_set(SW1_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt + DEBOUNCE_TICK)
        {
            demo_printf("%s: 11111\n", __func__);
            tick_cnt = tick_cnt_cur;
            msg_t msg = {SW1_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line1);
        }
    }
}
void EXTI2_IRQHandler(void)
{
    static TickType_t tick_cnt = 0;
    if(EXTI_GetITStatus(EXTI_Line2))
    {
        //exti_irq_set(SW4_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt + DEBOUNCE_TICK)
        {
            demo_printf("%s: 44444\n", __func__);
            tick_cnt = tick_cnt_cur;
            msg_t msg = {SW4_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line2);
        }
    }
}
void EXTI3_IRQHandler(void)
{
    static TickType_t tick_cnt = 0;
    if(EXTI_GetITStatus(EXTI_Line3))
    {
        //exti_irq_set(SW3_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt + DEBOUNCE_TICK)
        {
            demo_printf("%s: 33333\n", __func__);
            tick_cnt = tick_cnt_cur;
            msg_t msg = {SW3_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line3);
        }
    }
}
void EXTI4_IRQHandler(void)
{
    static TickType_t tick_cnt = 0;
    if(EXTI_GetITStatus(EXTI_Line4))
    {
        //exti_irq_set(SW6_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt + DEBOUNCE_TICK)
        {
            demo_printf("%s: 66666\n", __func__);
            tick_cnt = tick_cnt_cur;
            msg_t msg = {SW6_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line4);
        }
    }
}
void EXTI9_5_IRQHandler(void)
{
    static TickType_t tick_cnt5 = 0;
    static TickType_t tick_cnt8 = 0;
    static TickType_t tick_cnt9 = 0;

    if(EXTI_GetITStatus(EXTI_Line5))
    {
        //exti_irq_set(SW5_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt5 + DEBOUNCE_TICK)
        {
            demo_printf("%s: 55555\n", __func__);
            tick_cnt5 = tick_cnt_cur;
            msg_t msg = {SW5_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line5);
        }
    }
    else if(EXTI_GetITStatus(EXTI_Line8))
    {
        //exti_irq_set(BUTTON_REVERSE_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt8 + DEBOUNCE_TICK)
        {
            demo_printf("%s: button reverse\n", __func__);
            tick_cnt8 = tick_cnt_cur;
            msg_t msg = {BUTTON_R_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line8);
        }
    }
    else if(EXTI_GetITStatus(EXTI_Line9))
    {
        //exti_irq_set(BUTTON_FORWARD_IRQn, DISABLE);
        TickType_t tick_cnt_cur = xTaskGetTickCountFromISR();
        if(tick_cnt_cur > tick_cnt9 + DEBOUNCE_TICK)
        {   
            demo_printf("%s: button forward\n", __func__);
            tick_cnt9 = tick_cnt_cur;
            msg_t msg = {BUTTON_F_STATE_CHANGE, 0};
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line9);
        }
    }
}

void demo_task_init(void)
{
    bsp_init();
    if(pdFALSE == xTaskCreate(led_task, 
                                LED_TASK_NAME,
                                LED_TASK_STK_SIZE,
                                NULL,
                                LED_TASK_PRI,
                                NULL))
    {
        demo_printf("create led_task error\n");
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
    demo_printf("demo task init ; sw_version = %s\n", software_version);
}

//===============================================
#if 1  //demo
void led_task(void *param)
{
    uint32_t demo_cnt = 0;
    while(1)
    {
        demo_printf("%s: .... %d\n", __func__, demo_cnt++);
        LED_WR_DATA(Bit_SET);
        vTaskDelay(100/portTICK_PERIOD_MS);
        LED_WR_DATA(Bit_RESET);
        vTaskDelay(900/portTICK_PERIOD_MS);
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
                    #if 0 //for motor test
                        motor_ctrl(M_ab, M_FORWARD, 255);
                        motor_ctrl(M_c, M_FORWARD, 255);
                        motor_ctrl(M_d, M_FORWARD, 255);
                    #else
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
                    #endif
                }
                break;
                case WINDOW_FORWARD:
                {
                    demo_printf("%s: WINDOW_FORWARD\n", __func__);
                    motor_ctrl(M_c, M_STOP, 0);
                    motor_ctrl(M_d, M_STOP, 0);
                    if(SW_FOR_DATA != SW_DOWN)
                    {
                        motor_ctrl(M_ab, M_FORWARD, 255);
                    }
                    else 
                    {
                        motor_ctrl(M_ab, M_STOP, 0);
                        change_window_state(WINDOW_UP);
                    }
                }
                break;
                case WINDOW_UP:
                {
                    demo_printf("%s: WINDOW_UP\n", __func__);
                    //if(SW_FOR_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_ab, M_STOP, 0);
                        if(SW_UP1_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_c, M_FORWARD, 255);
                        }
                        else 
                        {
                            motor_ctrl(M_c, M_STOP, 0);
                        }
                        if(SW_UP2_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_d, M_FORWARD, 255);
                        }
                        else 
                        {
                            motor_ctrl(M_d, M_STOP, 0);
                        }

                        if(SW_UP1_DATA == SW_DOWN && SW_UP2_DATA == SW_DOWN)
                        {
                            change_window_state(WINDOW_OPENED);
                        }
                    }
                    // else 
                    // {
                    //     change_window_state(WINDOW_FORWARD);
                    // }
                }
                break;
                case WINDOW_OPENED:
                {
                    demo_printf("%s: WINDOW_OPENED\n", __func__);
                    motor_ctrl(M_ab, M_STOP, 0);
                    motor_ctrl(M_c, M_STOP, 0); 
                    motor_ctrl(M_d, M_STOP, 0);
                    change_window_state(WINDOW_STOP);
                }
                break;
                //--------------------------------------
                case WINDOW_CLOSE:
                {
                    demo_printf("%s: WINDOW_CLOSE\n", __func__);
                    #if 0 // for motor test
                        motor_ctrl(M_ab, M_BACKWARD, 255);
                        motor_ctrl(M_c, M_BACKWARD, 255);
                        motor_ctrl(M_d, M_BACKWARD, 255);
                    #else
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
                    #endif
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
                    else 
                    {
                        motor_ctrl(M_c, M_STOP, 0);
                    }
                    if(SW_DOW2_DATA != SW_DOWN)
                    {
                        motor_ctrl(M_d, M_BACKWARD, 255);
                    }
                    else 
                    {
                        motor_ctrl(M_d, M_STOP, 0);
                    }

                    if(SW_DOW1_DATA == SW_DOWN && SW_DOW2_DATA == SW_DOWN)
                    {
                        change_window_state(WINDOW_BACKWARD);
                    }
                }
                break;
                case WINDOW_BACKWARD:
                {
                    demo_printf("%s: WINDOW_BACKWARD\n", __func__);
                    //if(SW_DOW1_DATA == SW_DOWN && SW_DOW2_DATA == SW_DOWN)
                    {
                        motor_ctrl(M_c, M_STOP, 0);
                        motor_ctrl(M_d, M_STOP, 0);
                        if(SW_BAC_DATA != SW_DOWN)
                        {
                            motor_ctrl(M_ab, M_BACKWARD, 255);
                        }
                        else 
                        {
                            motor_ctrl(M_ab, M_STOP, 0);
                            change_window_state(WINDOW_CLOSED);
                        } 
                    }
                    // else 
                    // {
                    //     change_window_state(WINDOW_DOWN);
                    // }
                }
                break;
                case WINDOW_CLOSED:
                {
                    demo_printf("%s: WINDOW_CLOSED\n", __func__);
                    motor_ctrl(M_ab, M_STOP, 0);
                    motor_ctrl(M_c, M_STOP, 0);
                    motor_ctrl(M_d, M_STOP, 0); 
                    change_window_state(WINDOW_STOP);
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
                    //exti_irq_set(SW1_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_BACKWARD && SW_BAC_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;
                case SW2_STATE_CHANGE:
                {
                    demo_printf("%s: sw2 %s\n", __func__, (SW_FOR_DATA==SW_DOWN?"down":"up"));
                    //exti_irq_set(SW2_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_FORWARD && SW_FOR_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;

                case SW3_STATE_CHANGE:
                {
                    demo_printf("%s: sw3 %s\n", __func__, (SW_DOW1_DATA==SW_DOWN?"down":"up"));
                    //exti_irq_set(SW3_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_DOWN && SW_DOW1_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;
                case SW5_STATE_CHANGE:
                {
                    demo_printf("%s: sw5 %s\n", __func__, (SW_DOW2_DATA==SW_DOWN?"down":"up"));
                    //exti_irq_set(SW5_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_DOWN && SW_DOW2_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;

                case SW4_STATE_CHANGE:
                {
                    demo_printf("%s: sw4 %s\n", __func__, (SW_UP1_DATA==SW_DOWN?"down":"up"));
                    //exti_irq_set(SW4_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_UP && SW_UP1_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;
                case SW6_STATE_CHANGE:
                {
                    demo_printf("%s: sw6 %s\n", __func__, (SW_UP2_DATA==SW_DOWN?"down":"up"));
                    //exti_irq_set(SW6_IRQn, ENABLE);

                    if(get_window_state() == WINDOW_UP && SW_UP2_DATA == SW_DOWN)
                    {
                        update_window_state();
                    }
                }
                break;

                case BUTTON_F_STATE_CHANGE:
                {
                    demo_printf("%s: button forward %s\n", __func__, (BUTTON_FORWARD_DATA == BUTTON_DOWN?"down":"up"));
                    //exti_irq_set(BUTTON_FORWARD_IRQn, ENABLE);
                    
                    if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
                    {
                        //open window
                        if(get_window_state() == WINDOW_STOP)
                        {
                            change_window_state(WINDOW_OPEN);
                        }
                    }
                    else if(BUTTON_REVERSE_DATA == BUTTON_UP)//all button up
                    { 
                        if(get_window_state() != WINDOW_STOP)
                        {
                            change_window_state(WINDOW_STOP);
                        }
                    }
                }
                break;
                case BUTTON_R_STATE_CHANGE:
                {
                    demo_printf("%s: button reverse %s\n", __func__, (BUTTON_REVERSE_DATA == BUTTON_DOWN?"down":"up"));
                    //exti_irq_set(BUTTON_REVERSE_IRQn, ENABLE);
                    
                    if(BUTTON_REVERSE_DATA == BUTTON_DOWN)
                    {
                        //close window
                        if(get_window_state() == WINDOW_STOP)
                        {
                           change_window_state(WINDOW_CLOSE);
                        }
                    }
                    else if(BUTTON_FORWARD_DATA == BUTTON_UP)//all button up
                    {
                        if(get_window_state() != WINDOW_STOP)
                        {
                            change_window_state(WINDOW_STOP);
                        }
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
