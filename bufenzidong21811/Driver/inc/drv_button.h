#ifndef __DRV_BUTTON_H__
#define __DRV_BUTTON_H__

#include "stm32f4xx.h"
#include "typedefine.h"

#define BUTTONn                         2

#define Button2_PIN                     GPIO_Pin_5
#define Button2_GPIO_PORT               GPIOE
#define Button2_GPIO_CLK                RCC_AHB1Periph_GPIOE
#define Button2_EXTI_PortSource         EXTI_PortSourceGPIOE
#define Button2_EXTI_PinSource          EXTI_PinSource5
#define Button2_EXTI_Line               EXTI_Line5
#define Button2_EXTI_IRQn               EXTI9_5_IRQn
#define button2_irq                     EXTI9_5_IRQHandler

#define Button3_PIN                     GPIO_Pin_6
#define Button3_GPIO_PORT               GPIOE
#define Button3_GPIO_CLK                RCC_AHB1Periph_GPIOE
#define Button3_EXTI_PortSource         EXTI_PortSourceGPIOE
#define Button3_EXTI_PinSource          EXTI_PinSource6
#define Button3_EXTI_Line               EXTI_Line6
#define Button3_EXTI_IRQn               EXTI9_5_IRQn
#define button3_irq                     EXTI9_5_IRQHandler
#define KEY0 					GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7) //PE07
typedef enum
{
    Button2 = 0,
    Button3 = 1,
} ButtonType;

#define button2_3_irq                   EXTI9_5_IRQHandler      // 两个按键位于同一个中断，所以重新定义统一的中断函数


extern void     drv_button_init( void );
extern INT8U    drv_button_get_status( ButtonType Button );
extern void     drv_button_init( void );
void     KEY_Init(void);
#endif


