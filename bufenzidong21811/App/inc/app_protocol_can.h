#ifndef __APP_PROTOCOL_CAN_H__
#define __APP_PROTOCOL_CAN_H__

#include "stm32f4xx.h"
#include "typedefine.h"


// 根据地址、功能码和索引计算扩展ID
#define app_protocol_can_extid_set(index, cmd, address)  ((INT32U)((index<<16) | (cmd<<8) | address))
#define app_protocol_can_extid_get_index(ID)             ((INT8U)((ID & 0X00FF0000) >> 16))  // 从扩展ID解析出功能码索引
#define app_protocol_can_extid_get_funcode(ID)           ((INT8U)((ID & 0X0000FF00) >> 8))   // 从扩展ID解析出功能码
#define app_protocol_can_extid_get_address(ID)           ((INT8U)(ID & 0X000000FF))          // 从扩展ID解析出CAN地址


typedef enum
{
    // 特殊命令
    CANCMD_SetMotorSpeed = 41,          // 自定义电机速度
    CANCMD_SetPlatformSpeed,            // 自定义平台速度
}CANCMD_Type;

typedef struct structCanCommandType
{
    INT8U   CmdAddress;
    INT8U   CmdFunc;
    INT8U   CmdIndex;
    INT8U   MsgLen;
    INT8U   *pData;
}CanCommandType, *PCanCommandType;


enum MOTOR_CMD_STATE_EN
{
    MOTOR_CMD_IDLE = 0,
    MOTOR_CMD_WAIT_RES,
    MOTOR_CMD_WAIT_STOP,
    MOTOR_CMD_ERROR
};

extern enum MOTOR_CMD_STATE_EN motor_cmd_status;


extern INT8U app_protocol_can_send_package( INT8U Address, INT8U Cmd, INT8U Index, INT8U *Data, INT8U Len );

#endif


