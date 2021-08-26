#include "app_protocol_can.h"
#include "drv_can.h"

enum MOTOR_CMD_STATE_EN motor_cmd_status = MOTOR_CMD_IDLE;


//============================================================================
// 名称：app_protocol_can_send_package
// 功能：通过CAN发送数据包
// 参数：Address：地址
//      Cmd：功能码
//      Index：功能码分包传输索引
//      Data：数据
//      Len：数据字节数
// 返回：无
// 说明：无
//============================================================================
INT8U app_protocol_can_send_package( INT8U Address, INT8U Cmd, INT8U Index, INT8U *Data, INT8U Len )
{
    CanTxMsg  l_CanTxMsg;

    l_CanTxMsg.ExtId = app_protocol_can_extid_set(Index, Cmd, Address);
    l_CanTxMsg.StdId = 0;
    l_CanTxMsg.DLC = Len;
    l_CanTxMsg.RTR = CAN_RTR_DATA;
    l_CanTxMsg.IDE = CAN_ID_EXT;
    if( Len > 0 )
    {
        INT8U i;

        for( i = 0; i < Len; i++ )
        {
            l_CanTxMsg.Data[i] = Data[i];
        }
    }

    drv_can_send_msg(l_CanTxMsg);


    return ( 0 );
}

//============================================================================
// 名称：app_protocol_can_protocol_parse
// 功能：CAN协议解析
// 参数：Cmd：功能码
//      Index：索引，子命令
//      Msg：数据
// 返回：0为成功，其他为失败
// 说明：无
//============================================================================
INT8U app_protocol_can_protocol_parse( INT8U Cmd, INT8U Index, INT8U *Data )
{
    switch ( Cmd )
    {
    case CANCMD_SetMotorSpeed:          // 自定义电机速度
        {
        }
        break;
    case CANCMD_SetPlatformSpeed:       // 自定义平台速度
        {
            motor_cmd_status = MOTOR_CMD_IDLE;
        }
        break;
    default:
        {
            return ( 1 );
        }
    }
    return ( 0 );
}

//============================================================================
// 名称：app_protocol_can_msg_process
// 功能：CAN消息处理
// 参数：无
// 返回：无
// 说明：无
//============================================================================
void app_protocol_can_msg_process( CanRxMsg RxMsg )
{
    CanCommandType l_CanCmd;

    l_CanCmd.CmdAddress = app_protocol_can_extid_get_address(RxMsg.ExtId);
    l_CanCmd.CmdFunc = app_protocol_can_extid_get_funcode(RxMsg.ExtId);
    l_CanCmd.CmdIndex = app_protocol_can_extid_get_index(RxMsg.ExtId);
    l_CanCmd.pData = RxMsg.Data;
    l_CanCmd.MsgLen = RxMsg.DLC;

    app_protocol_can_protocol_parse(l_CanCmd.CmdFunc, l_CanCmd.CmdIndex, l_CanCmd.pData);
}



