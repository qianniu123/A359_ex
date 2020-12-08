#include "demo.h"

#define demo_printf(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

#define DEMO_TASK_NAME          "DEMO_TASK"
#define DEMO_TASK_PRI           3 
#define DEMO_TASK_STK_SIZE      128

#define BUTTON_TASK_NAME        "BUTTON_TASK"
#define BUTTON_TASK_PRI         3 
#define BUTTON_TASK_STK_SIZE    128

#define MOTOR_TASK_NAME         "MOTOR_TASK"
#define MOTOR_TASK_PRI          3 
#define MOTOR_TASK_STK_SIZE     128

#define SW_TASK_NAME            "SW_TASK"
#define SW_TASK_PRI             3 
#define SW_TASK_STK_SIZE        128

#define SW_QUEUE_SIZE       5
QueueHandle_t queue_sw = NULL;
#define MOTOR_QUEUE_SIZE    5
QueueHandle_t queue_motor = NULL;

//-------------------------------------------------------------------------------
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
static void button_task(void *param);
static void switch_task(void *param);
static void motor_task(void *param);
//----------------------------------------------------------------------
static void motor_ctrl(uint8_t motor_index, uint8_t state, uint8_t speed)
{
    switch(state)
    {
        case M_STOP:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare3(TIM3, 0);//FOR_FSS
                TIM_SetCompare4(TIM3, 0);//BAC_B
            }
            else if(motor_index == M_c)
            {
                TIM_SetCompare1(TIM1, 0);//UP1_F
                TIM_SetCompare2(TIM1, 0);//DOW1_B
            }
            else if(motor_index == M_d)
            {
                TIM_SetCompare3(TIM1, 0);//UP2_F
                TIM_SetCompare4(TIM1, 0);//DOW2_B
            }
        }
        break;
        case M_FORWARD:
        {
           if(motor_index == M_ab)
            {
                TIM_SetCompare3(TIM3, 255);//FOR_FSS
                TIM_SetCompare4(TIM3, 0);//BAC_B
            }
            else if(motor_index == M_c)
            {
                TIM_SetCompare1(TIM1, 255);//UP1_F
                TIM_SetCompare2(TIM1, 0);//DOW1_B
            }
            else if(motor_index == M_d)
            {
                TIM_SetCompare3(TIM1, 255);//UP2_F
                TIM_SetCompare4(TIM1, 0);//DOW2_B
            } 
        }
        break;
        case M_BACKWARD:
        {
            if(motor_index == M_ab)
            {
                TIM_SetCompare3(TIM3, 0);//FOR_FSS
                TIM_SetCompare4(TIM3, 255);//BAC_B
            }
            else if(motor_index == M_c)
            {
                TIM_SetCompare1(TIM1, 0);//UP1_F
                TIM_SetCompare2(TIM1, 255);//DOW1_B
            }
            else if(motor_index == M_d)
            {
                TIM_SetCompare3(TIM1, 0);//UP2_F
                TIM_SetCompare4(TIM1, 255);//DOW2_B
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
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = type;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = state;
	NVIC_Init(&NVIC_InitStructure);
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
    
    #if 1
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
    
    #if 1
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
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_PORT, LED_PIN);
#endif
    demo_printf("%s: bsp init\n", __func__);
}

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0))
    {
        demo_printf("%s: 000\n", __func__);
        exti_irq_set(EXTI0_IRQn, DISABLE);
        msg_t msg = {SW1_DOWN, 0};
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
        demo_printf("%s: 111\n", __func__);
        exti_irq_set(EXTI1_IRQn, DISABLE);
        msg_t msg = {SW2_DOWN, 0};
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
        demo_printf("%s: 222\n", __func__);
        exti_irq_set(EXTI2_IRQn, DISABLE);
        msg_t msg = {SW3_DOWN, 0};
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
        demo_printf("%s: 333\n", __func__);
        exti_irq_set(EXTI3_IRQn, DISABLE);
        msg_t msg = {SW4_DOWN, 0};
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
        demo_printf("%s: 444\n", __func__);
        exti_irq_set(EXTI4_IRQn, DISABLE);
        msg_t msg = {SW5_DOWN, 0};
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
        exti_irq_set(EXTI9_5_IRQn, DISABLE);
        msg_t msg = {SW6_DOWN, 0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sw, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

void demo_task(void *param)
{
    uint32_t demo_cnt = 0;
    while(1)
    {
        demo_printf("%s: .... %d\n", __func__, demo_cnt);
        demo_cnt ++;
        led_toggle();
        vTaskDelay(3000/portTICK_PERIOD_MS);
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
    if(pdFALSE == xTaskCreate(motor_task, 
                                MOTOR_TASK_NAME,
                                MOTOR_TASK_STK_SIZE,
                                NULL,
                                MOTOR_TASK_PRI,
                                NULL))
    {
        demo_printf("create motor_task error\n");
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

    demo_printf("demo task init \n");
}

//===============================================
#if 1  //demo
static void button_task(void *param)
{
    msg_t msg = {0};
    while(1)
    {
        if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
        {
            vTaskDelay(20/portTICK_PERIOD_MS);
            if(BUTTON_FORWARD_DATA == BUTTON_DOWN)
            {
                demo_printf("%s: button forward down\n", __func__);
                //start forward -> send msg
                msg.msg_id = WINDOW_FORWARD;
                //xQueueSend(queue_motor, &msg, 0);
            }
        }
        else if(BUTTON_REVERSE_DATA == BUTTON_DOWN)
        {
            vTaskDelay(20/portTICK_PERIOD_MS);
            if(BUTTON_REVERSE_DATA == BUTTON_DOWN)
            {
                demo_printf("%s: button backward down\n", __func__);
                //start backward -> send msg
                msg.msg_id = WINDOW_DOWN;
                //xQueueSend(queue_motor, &msg, 0);
            }
        }
        else 
        {
            demo_printf("%s: button_forward_data = %d, button_backward_data = %d\n", __func__, BUTTON_FORWARD_DATA, BUTTON_REVERSE_DATA);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);//100
    }

}

static void motor_task(void *param)
{
    msg_t msg = {0};
    while(1)
    {
        if(pdTRUE == xQueueReceive(queue_motor, &msg, portMAX_DELAY))
        {
            uint8_t window_state = msg.msg_id;//back->forward->up  or up->down->back
            demo_printf("%s: window_state = %d\n", __func__, window_state);
            switch(window_state)
            {
                case WINDOW_FORWARD:
                {
                    if(SW_BAC_DATA != 0)
                    {
                        motor_ctrl(M_ab, M_FORWARD, 255);
                    }
                }
                break;
                case WINDOW_BACKWARD:
                {
                    if(SW_FOR_DATA != 0)
                    {
                        motor_ctrl(M_ab, M_BACKWARD, 255);
                    }
                }
                break;
                case WINDOW_UP:
                {
                    if(SW_UP1_DATA != 0)
                    {
                        motor_ctrl(M_c, M_FORWARD, 255);
                    }
                    if(SW_UP2_DATA != 0)
                    {
                        motor_ctrl(M_d, M_FORWARD, 255);
                    }
                }
                break;
                case WINDOW_DOWN:
                {
                    if(SW_DOW1_DATA != 0)
                    {
                        motor_ctrl(M_c, M_BACKWARD, 255);
                    }
                    if(SW_DOW2_DATA != 0)
                    {
                        motor_ctrl(M_d, M_BACKWARD, 255);
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
            vTaskDelay(20/portTICK_PERIOD_MS);//debounce
            demo_printf("%s: sw_state = %d\n", __func__, sw_state);
            switch(sw_state)
            {
                case SW1_DOWN:
                {
                    demo_printf("%s: sw1_data = %d\n", __func__, SW_BAC_DATA);
                    exti_irq_set(EXTI0_IRQn, ENABLE);
                }
                break;

                case SW2_DOWN:
                {
                    demo_printf("%s: sw2_data = %d\n", __func__, SW_FOR_DATA);
                    exti_irq_set(EXTI1_IRQn, ENABLE);
                }
                break;

                case SW3_DOWN:
                {
                    demo_printf("%s: sw3_data = %d\n", __func__, SW_DOW1_DATA);
                    exti_irq_set(EXTI2_IRQn, ENABLE);
                }
                break;
                case SW5_DOWN:
                {
                    demo_printf("%s: sw5_data = %d\n", __func__, SW_DOW2_DATA);
                    exti_irq_set(EXTI4_IRQn, ENABLE);
                }
                break;

                case SW4_DOWN:
                {
                    demo_printf("%s: sw4_data = %d\n", __func__, SW_UP1_DATA);
                    exti_irq_set(EXTI3_IRQn, ENABLE);
                }
                break;
                case SW6_DOWN:
                {
                    demo_printf("%s: sw6_data = %d\n", __func__, SW_UP2_DATA);
                    exti_irq_set(EXTI9_5_IRQn, ENABLE);
                }
                break;
                default:
                break;
            }
        }
    }
}

#endif


