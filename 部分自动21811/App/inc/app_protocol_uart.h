#ifndef __APP_PROTOCOL_UART_H__
#define __APP_PROTOCOL_UART_H__

#include "stm32f4xx.h"
#include "typedefine.h"


typedef enum {
    // 特殊命令
    UARTCMD_SetMotorSpeed = 41,             // 自定义电机速度
    UARTCMD_SetPlatformSpeed,               // 自定义平台速度
}UARTCMD_Type;

// 系统串口消息数据包
#define UART_MSG_PACKAGE_STARTCHAR       0xAA        // 数据包开始字符
#define UART_MSG_PACKAGE_ENDCHAR         0x0D        // 数据包结束字符

extern void app_protocol_uart_send_msg( INT8U DeviceType, INT8U Address, INT8U Cmd, INT8U *Dat, INT8U Len );

#endif


