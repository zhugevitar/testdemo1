#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "stm32f4xx.h"
#include "typedefine.h"


// SCAN1
#define SCAN_BASE                       CAN1
#define SCAN_CLK                        RCC_APB1Periph_CAN1
#define SCAN_RX_PIN                     GPIO_Pin_8
#define SCAN_TX_PIN                     GPIO_Pin_9
#define SCAN_GPIO_PORT                  GPIOB
#define SCAN_GPIO_CLK                   RCC_AHB1Periph_GPIOB
#define SCAN_AF_PORT                    GPIO_AF_CAN1
#define SCAN_RX_SOURCE                  GPIO_PinSource8
#define SCAN_TX_SOURCE                  GPIO_PinSource9
#define SCAN1_RX0_IRQn                  CAN1_RX0_IRQn

#define drv_scan_irq                    CAN1_RX0_IRQHandler


extern void     drv_can_init( void );
extern INT8U    drv_can_send_msg( CanTxMsg TxMessage );

#endif


