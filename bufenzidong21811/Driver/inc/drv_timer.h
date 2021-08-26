#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include "stm32f4xx.h"
#include "typedefine.h"


/****************宏定义区-Start*********************/
#define TICK_PER_SEC 1000  /* 每秒钟几个tick，由于systick是1ms一个中断，所以这个值是1000 */
/****************宏定义区-End××********************/

/****************全局变量定义区-Start*********************/
/* 该结构体用来表示机器时间 */
/****************全局变量定义区-End***********************/

/****************函数声明区-Start********************/
void systick_init(void);
uint32_t sys_now(void);
void systick_increase(void);
void set_pwm_config(uint32_t compare);
/****************函数声明区-End**********************/



#define SysTimer_Base               TIM7
#define SysTimer_CLK                RCC_APB1Periph_TIM7
#define SysTimer_IRQn               TIM7_IRQn
#define SysTimer_IrqHandler         TIM7_IRQHandler
#define DBGMCU_TIM_STOP             DBGMCU_TIM7_STOP

#define sys_timer_irq               SysTimer_IrqHandler

extern INT32U  g_SysTimerCounter;

#define drv_timer_int_enable()      SysTimer_Base->DIER |= TIM_IT_Update;
#define drv_timer_int_disable()     SysTimer_Base->DIER &= ~TIM_IT_Update;

extern void drv_timer_init( void );


#endif


