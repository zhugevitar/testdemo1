#include "drv_timer.h"
#include "drv_led.h"

// 系统定时器计数器
INT32U  g_SysTimerCounter = 0;

__IO uint32_t ms_tick = 0;


//============================================================================
// 名称：drv_timer_init
// 功能：初始化定时器
// 参数：无
// 返回：无
// 说明：系统初始化后调用，调用该函数后即开始计数，初始化为1ms中断
//============================================================================
void drv_timer_init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef  TIM_OCInitStructure = {0};
    uint16_t CCR1_Val = 0;
    uint16_t PrescalerValue = 0;

    /* TIM10 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    /* GPIOF clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOF Configuration: TIM10 CH1 (PF6) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    /* Connect TIM pins to AF3 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
      /* ---------------------------------------------------------------------------
    TIM10 Configuration: generate 1 PWM signal:

    In this example TIM10 input clock (TIM10CLK) is set to 2 * APB2 clock (PCLK2), 
    since APB2 prescaler is different from 1.   
      TIM10CLK = 2 * PCLK2  
      PCLK2 = HCLK / 2 
      => TIM10CLK = HCLK = SystemCoreClock
          
    To get TIM10 counter clock at 21 MHz, the prescaler is computed as follows:
       Prescaler = (TIM10CLK / TIM10 counter clock) - 1
       Prescaler = (SystemCoreClock /21 MHz) - 1
                                              
    To get TIM10 output clock at 31.530 KHz, the period (TIM10_ARR) is computed as follows:
       ARR = (TIM10 counter clock / TIM10 output clock) - 1
           = 665
                  
    TIM10 Channel1 duty cycle = (TIM10_CCR1/ TIM10_ARR)* 100 = 50%

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
    --------------------------------------------------------------------------- */   


    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / 200000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1999;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM12, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM12, ENABLE);

    /* TIM10 enable counter */
    TIM_Cmd(TIM12, ENABLE);
}

void set_pwm_config(uint32_t compare)
{
    TIM_SetCompare1(TIM12, compare);
}

//============================================================================
// 名称：sys_timer_irq
// 功能：定时器中断服务函数
// 参数：无
// 返回：无
// 说明：无
//============================================================================
void sys_timer_irq(void)
{
    if( TIM_GetITStatus(SysTimer_Base, TIM_IT_Update) != RESET )
    {
        // 清中断标志
        TIM_ClearITPendingBit(SysTimer_Base, TIM_IT_Update);

        // 处理的事情
        g_SysTimerCounter++;
        if(g_SysTimerCounter%500 == 0)
        {
            drv_led_toggle(LED2);
        }
    }

}



/*
*********************************************************************************************************
*                                         systick_init
*
* Description: 对系统的systick进行初始化
*
* Arguments  : void
*
* Returns    : void

*********************************************************************************************************
*/
void systick_init(void)
{
    /* 将systick中断时间设置成1ms */
    if (SysTick_Config(SystemCoreClock / TICK_PER_SEC))
      { 
        /* Capture error */ 
        while (1);
      }
}

void systick_increase(void)
{
    ms_tick += 1;
}
/**
  * @fun_name   sys_now
  * @brief      This function get systick.
  * @param      None
  * @retval     uint32_t systick
  */
uint32_t sys_now(void)
{
    return ms_tick;
}


