#ifndef __DRV_LED_H__
#define __DRV_LED_H__

#include "stm32f4xx.h"
#include "typedefine.h"


//-------------------------------------------------------------------
//                          指示灯相关定义
//-------------------------------------------------------------------
#define LEDn                             9

#define LED1_PIN                         GPIO_Pin_3
#define LED1_GPIO_PORT                   GPIOE
#define LED1_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED2_PIN                         GPIO_Pin_4
#define LED2_GPIO_PORT                   GPIOE
#define LED2_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED3_PIN                         GPIO_Pin_0
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK                    RCC_AHB1Periph_GPIOA

#define LED4_PIN                         GPIO_Pin_9  //上升
#define LED4_GPIO_PORT                   GPIOE
#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED5_PIN                         GPIO_Pin_10  //上升
#define LED5_GPIO_PORT                   GPIOE
#define LED5_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED6_PIN                         GPIO_Pin_11    //下降
#define LED6_GPIO_PORT                   GPIOE
#define LED6_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED7_PIN                         GPIO_Pin_12    //下降
#define LED7_GPIO_PORT                   GPIOE
#define LED7_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED9_PIN                         GPIO_Pin_13   //泵
#define LED9_GPIO_PORT                   GPIOE
#define LED9_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED10_PIN                        GPIO_Pin_14    //泵
#define LED10_GPIO_PORT                  GPIOE
#define LED10_GPIO_CLK                   RCC_AHB1Periph_GPIOE

//#define LED11_PIN                        GPIO_Pin_15        //急停按钮
//#define LED11_GPIO_PORT                  GPIOE
//#define LED11_GPIO_CLK                   RCC_AHB1Periph_GPIOE

/*
* LED1:板载绿灯
* LED2:板载红灯

*/

typedef enum
 {
		   LED1,//板载绿灯
		   LED2,//板载红灯
		   LED3,//喷洒头
		   LED4,//上升PE9
		   LED5,//上升PE10
		   LED6,//下降PE11
		   LED7,//下降PE12
		   LED9,//泵PE13
		   LED10//泵PE14
	 
	 
 } LedType;

//-------------------------------------------------------------------
//                              函数声明
//-------------------------------------------------------------------
extern void drv_led_init( void );
extern void drv_led_on( LedType Led );
extern void drv_led_off( LedType Led );
extern void drv_led_toggle( LedType Led );


/**
  * @fun_name   drv_get_lc_state
  * @brief      获取当前液位状态.
  * @param      None
  * @retval     uint8_t uint8_t:0,缺药；1,有药
  */
uint8_t drv_get_lc_state(void);
//uint8_t JiTing(void);  //急停按钮函数


#endif

