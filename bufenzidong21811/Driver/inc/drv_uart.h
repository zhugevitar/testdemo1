#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "stm32f4xx.h"
#include "typedefine.h"


//串口相关定义
#define SCOM2_BASE                                       USART2
#define SCOM2_CLK                                        RCC_APB1Periph_USART2
#define SCOM2_GPIO_PORT                                  GPIOA


#define SCOM2_TX_PIN                                     GPIO_Pin_2
#define SCOM2_TX_SOURCE                                  GPIO_PinSource2
#define SCOM2_TX_AF                                      GPIO_AF_USART2

#define SCOM2_RX_PIN                                     GPIO_Pin_3
#define SCOM2_RX_SOURCE                                  GPIO_PinSource3
#define SCOM2_RX_AF                                      GPIO_AF_USART2


#define RS485_DIR_PIN_2                                     GPIO_Pin_2
#define RS485_DIR_PORT_2                                    GPIOB

/*  */

#define SCOM3_BASE                                       USART3
#define SCOM3_CLK                                        RCC_APB1Periph_USART3
#define SCOM3_GPIO_PORT                                  GPIOB


#define SCOM3_TX_PIN                                     GPIO_Pin_10
#define SCOM3_TX_SOURCE                                  GPIO_PinSource10
#define SCOM3_TX_AF                                      GPIO_AF_USART3

#define SCOM3_RX_PIN                                     GPIO_Pin_11
#define SCOM3_RX_SOURCE                                  GPIO_PinSource11
#define SCOM3_RX_AF                                      GPIO_AF_USART3

#define SCOM4_BASE                                       UART4
#define SCOM4_CLK                                        RCC_APB1Periph_UART4
#define SCOM4_GPIO_PORT                                  GPIOC


#define SCOM4_TX_PIN                                     GPIO_Pin_10
#define SCOM4_TX_SOURCE                                  GPIO_PinSource10
#define SCOM4_TX_AF                                      GPIO_AF_UART4

#define SCOM4_RX_PIN                                     GPIO_Pin_11
#define SCOM4_RX_SOURCE                                  GPIO_PinSource11
#define SCOM4_RX_AF                                      GPIO_AF_UART4

#define SCOM6_BASE                                       USART6
#define SCOM6_CLK                                        RCC_APB2Periph_USART6
#define SCOM6_GPIO_PORT                                  GPIOC


#define SCOM6_TX_PIN                                     GPIO_Pin_6
#define SCOM6_TX_SOURCE                                  GPIO_PinSource6
#define SCOM6_TX_AF                                      GPIO_AF_USART6

#define SCOM6_RX_PIN                                     GPIO_Pin_7
#define SCOM6_RX_SOURCE                                  GPIO_PinSource7
#define SCOM6_RX_AF                                      GPIO_AF_USART6



extern uint8_t usart2_data[255];
extern uint8_t usart2_idx;
extern uint8_t usart2_flag;

extern uint8_t usart3_data[255];
extern uint8_t usart3_idx;
extern uint8_t usart3_flag;


extern uint8_t usart6_data[255];
extern uint8_t usart6_idx;
extern uint8_t usart6_flag;

extern uint8_t usart4_data[255];
extern uint8_t usart4_idx;
extern uint8_t usart4_flag;


enum GPIO_STATUS_EM
{
    GPIO_STATUS_OFF  = 0,
    GPIO_STATUS_ON   = 1,
};



typedef enum
{
    UART_BAUDRATE_300 = 0 ,
    UART_BAUDRATE_1200,
    UART_BAUDRATE_2400,
    UART_BAUDRATE_4800,
    UART_BAUDRATE_9600,
    UART_BAUDRATE_19200,
    UART_BAUDRATE_38400,
    UART_BAUDRATE_57600,
    UART_BAUDRATE_115200,
}UartBaudType, *PUartBaudType;

void drv_uart_2_init(void);
void drv_uart_3_init(void);
void drv_uart_6_init(void);

void drv_uart_4_init(void);

/*
*********************************************************************************************************
*                                         usart2_send_buff
*
* Description:  使用usart2进行数据发送
*
* Arguments  :  uint8_t *buff               指向需要发送的数据的首地址
*               uint8_t buff_len            需要发送数据的长度
*
* Returns    :  uint8_t               成功发送数据的长度

*********************************************************************************************************
*/
uint8_t usart2_send_buff(uint8_t *buff, uint8_t buff_len);


uint8_t usart3_send_buff(uint8_t *buff, uint8_t buff_len);


uint8_t usart6_send_buff(uint8_t *buff, uint8_t buff_len);

uint8_t usart4_send_buff(uint8_t *buff, uint8_t buff_len);


#endif


