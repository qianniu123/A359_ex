#include "demo.h"

#define demo_printf(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

#define DEMO_TASK_NAME       "DEMO_TASK"
#define DEMO_TASK_PRI        3 
#define DEMO_TASK_STK_SIZE   256

//PWM -> PA8 PA9 PA10 PA11   PC8 PC9
#define M_ab(state, pwm)  do{}while(0); //state: 0->stop;1->forward;2->backward


//EXTI-> PC0 PC1 PC2 PC3 PC4 PC5
#define SWITCH_PORT         GPIOC
#define SW1_PIN             GPIO_Pin_0
#define SW2_PIN             GPIO_Pin_1
#define SW3_PIN             GPIO_Pin_2
#define SW4_PIN             GPIO_Pin_3
#define SW5_PIN             GPIO_Pin_4
#define SW6_PIN             GPIO_Pin_5
//---
#define SW1_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW1_PIN)
#define SW2_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW2_PIN)
#define SW3_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW3_PIN)
#define SW4_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW4_PIN)
#define SW5_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW5_PIN)
#define SW6_DATA            GPIO_ReadInputDataBit(SWITCH_PORT, SW6_PIN)

//BUTTON -> PB0 PB1
#define BUTTON_PORT         GPIOB
#define BUTTON_FORWARD_PIN  GPIO_Pin_0
#define BUTTON_BACKWARD_PIN GPIO_Pin_1
//---
#define BUTTON_FORWARD_DATA     GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_FORWARD_PIN)
#define BUTTON_BACKWARD_DATA    GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_BACKWARD_PIN)


//----------------------------------------------------------
void motor_ctrl(uint8_t motor_index, uint8_t state, uint8_t speed)
{
    switch(state)
    {
        case M_STOP:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare1(TIM1, 0);
                TIM_SetCompare2(TIM1, 0);
            }
            else if(motor_index == M_c)
            {
                TIM_SetCompare3(TIM1, 0);
                TIM_SetCompare4(TIM1, 0);
            }
            else if(motor_index == M_d)
            {
                TIM_SetCompare3(TIM3, 0);
                TIM_SetCompare4(TIM3, 0);
            }
        }
        break;
        case M_FORWARD:
        {
            if(motor_index == M_ab)
            {

            }
            else if(motor_index == M_c)
            {

            }
            else if(motor_index == M_d)
            {
                
            }
        }
        break;
        case M_BACKWARD:
        {
            if(motor_index == M_ab)
            {

            }
            else if(motor_index == M_c)
            {

            }
            else if(motor_index == M_d)
            {
                
            }
        }
        break;
        default:
        break;
    }
}

//-----------------------------------------------

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
    
    TIM_SetCompare1(TIM1, 50);//SystemCoreClock/(2*1000)); // ???
    TIM_SetCompare2(TIM1, 50);//SystemCoreClock/(2*1000)); //
    TIM_SetCompare3(TIM1, 100);//SystemCoreClock/(2*1000)); //
    TIM_SetCompare4(TIM1, 100);//SystemCoreClock/(2*1000)); //
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
    TIM_TimeBaseStructure.TIM_Period = 255;//SystemCoreClock/1000;//64000000/1000
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
    
    TIM_SetCompare3(TIM3, 100);//SystemCoreClock/(2*1000)
    TIM_SetCompare4(TIM3, 100);//SystemCoreClock/(2*1000)
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

    EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
    
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);
	EXTI_InitStructure.EXTI_Line=EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

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
//--------------------------------------------------------------
//button
#if 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    //GPIO_ResetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1);
#endif
    demo_printf("%s: bsp init\n", __func__);
}

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0))
    {

        demo_printf("%s: 000\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1))
    {

        demo_printf("%s: 111\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2))
    {

        demo_printf("%s: 222\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3))
    {

        demo_printf("%s: 333\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4))
    {

        demo_printf("%s: 444\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5))
    {

        demo_printf("%s: 555\n", __func__);
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

void demo_task(void *param)
{
    while(1)
    {
        demo_printf("%s: ....\n", __func__);
        
        vTaskDelay(3000/portTICK_PERIOD_MS);
        //vTaskDelay(1000/portTICK_PERIOD_MS);
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
        demo_printf("create demo_task error");
    }
      
    demo_printf("demo task init \n");
}

//===============================================
#if 1  //demo
void button_task(void *param)
{
    while(1)
    {
        if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
        {
            //start forward
        }
        else if(BUTTON_BACKWARD_DATA == BUTTON_DOWN)
        {
            //start backward
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }

}

void motor_task(void *param)
{
    while(1)
    {


    }
}

#endif


