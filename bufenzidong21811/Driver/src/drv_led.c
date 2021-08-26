#include "drv_led.h"



//LED端口资源数组定义
GPIO_TypeDef *LED_GPIO_PORT[LEDn] = { LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT, LED4_GPIO_PORT, LED5_GPIO_PORT, LED6_GPIO_PORT, LED7_GPIO_PORT,LED9_GPIO_PORT,LED10_GPIO_PORT};
const INT16U    LED_GPIO_PIN[LEDn] = { LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN, LED6_PIN, LED7_PIN,LED9_PIN,LED10_PIN };
const INT32U    LED_GPIO_CLK[LEDn] = { LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK, LED4_GPIO_CLK, LED5_GPIO_CLK, LED6_GPIO_CLK, LED7_GPIO_CLK,LED9_GPIO_CLK,LED10_GPIO_CLK };

//============================================================================
// 名称：drv_led_init
// 功能：所有LED口线初始化
// 参数：无
// 返回：无
// 说明：系统初始化时调用
//============================================================================
void drv_led_init( void )
{
    INT8U i;
    GPIO_InitTypeDef  GPIO_InitStructure;

    for( i = 0; i < LEDn; i++ )
    {
        RCC_AHB1PeriphClockCmd(LED_GPIO_CLK[i], ENABLE);

        GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    //GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

        GPIO_Init(LED_GPIO_PORT[i], &GPIO_InitStructure);

        drv_led_off((LedType )i);
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* 液位检测输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//============================================================================
// 名称：drv_led_on
// 功能：LED亮
// 参数：Led：LED编号，LED1~LEDn
// 返回：无
// 说明：无
//============================================================================
void drv_led_on( LedType Led )
{
    GPIO_SetBits(LED_GPIO_PORT[Led], LED_GPIO_PIN[Led]);
}

//============================================================================
// 名称：drv_led_off
// 功能：LED灭
// 参数：Led：LED编号，LED1~LEDn
// 返回：无
// 说明：无
//============================================================================
void drv_led_off( LedType Led )
{
    GPIO_ResetBits(LED_GPIO_PORT[Led], LED_GPIO_PIN[Led]);
}

//============================================================================
// 名称：drv_led_toggle
// 功能：LED状态翻转
// 参数：Led：LED编号，LED1~LEDn
// 返回：无
// 说明：无
//============================================================================
void drv_led_toggle( LedType Led )
{
    LED_GPIO_PORT[Led]->ODR ^= LED_GPIO_PIN[Led];
}


/**
  * @fun_name   drv_get_lc_state
  * @brief      获取当前液位状态.
  * @param      None
  * @retval     uint8_t uint8_t:1,缺药；0,有药
  */

uint8_t drv_get_lc_state(void)
{
    uint8_t ret = 0;

    ret = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0);

    return ret;
}

    /** 获取急停按钮状态  **/
//uint8_t JiTing(void)
//{
//    uint8_t ret = 0;

//    ret = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);

//    return ret;
//}

