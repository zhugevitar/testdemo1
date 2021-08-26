#include "arm_math.h"
#include "app_robot_control.h"
#include "app_protocol_can.h"
#include "app_protocol_uart.h"
#include "drv_delay.h"

// 从UART发送命令，注释后为从CAN发送命令
//#define SEND_MSG_FROM_UART                            1

#ifndef SEND_MSG_FROM_UART
#include "stdio.h"
#endif



// 定义平台电机驱动器地址
#define MOTOR_DIRVER_ADDRESS                            0x1    // 需要根据具体的底盘型号说明书设置

// 定义平台电机驱动器设备类型
#define MOTOR_DRIVER_DEVICE_TYPE                        0x40

float speed_x_reg = 0.0f;   /* 记录上次发送的x轴速度参数 */
float speed_y_reg = 0.0f;   /* 记录上次发送的y轴速度参数 */
float rotate_reg = 0.0f;    /* 记录上次发送的旋转速度参数 */


//============================================================================
// 名称：app_robot_control_move_abort
// 功能：控制机器人平台停止动作
// 参数：无
// 返回：无
// 说明：将所有电机速度设置为0
//============================================================================
void app_robot_control_move_abort( void )
{
    INT8U ZeroSpeed[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetMotorSpeed, 0, ZeroSpeed, 8);

    drv_delay_ms(10);
}

//============================================================================
// 名称：app_robot_control_move_abort_jiting
// 功能：控制机器人平台停止动作
// 参数：无
// 返回：无
// 说明：将所有电机速度设置为0
//============================================================================
void app_robot_control_move_abort_jiting( void )
{
    INT8U ZeroSpeed[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetMotorSpeed, 0, ZeroSpeed, 8);

   // drv_delay_ms(10);
}


/**
  * @fun_name   app_robot_control_go
  * @brief      This function set car's move speed and rotate speed.
  * @param      float speed_x：x轴方向速度(mm/s)
                float speed_y：Y轴方向速度(mm/s)
                float rotate：自转弧度速度(rad/s)
  * @retval     None
  */
void app_robot_control_go(float speed_x, float speed_y, float rotate)
{
    LineMoveDataType LineCmdBuffer = {0};

    /* 只有处于空闲状态下才能继续发送行走命令 */
    if (MOTOR_CMD_IDLE == motor_cmd_status)
    {
        motor_cmd_status = MOTOR_CMD_WAIT_RES;
    }
    else
    {
        return;
    }

    speed_x_reg = speed_x;
    speed_y_reg = speed_y;
    rotate_reg = rotate;

    LineCmdBuffer.SpeedX = (INT16S)(speed_x * 10.0f);
    LineCmdBuffer.SpeedY = (INT16S)(speed_y * 10.0f);
    LineCmdBuffer.SpeedRotate = (INT16S)(rotate * 10000.0f);
    LineCmdBuffer.Acceleration = 0;

    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetPlatformSpeed, 0, (INT8U *)(&LineCmdBuffer), 8);
}

/**
  * @fun_name   app_robot_control_go_again
  * @brief      This function set car's move speed and rotate speed.
  * @param      None
  * @retval     None
  */
void app_robot_control_go_again(void)
{
    LineMoveDataType LineCmdBuffer = {0};

    LineCmdBuffer.SpeedX = (INT16S)(speed_x_reg * 10.0f);
    LineCmdBuffer.SpeedY = (INT16S)(speed_y_reg * 10.0f);
    LineCmdBuffer.SpeedRotate = (INT16S)(rotate_reg * 10000.0f);
    LineCmdBuffer.Acceleration = 0;

    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetPlatformSpeed, 0, (INT8U *)(&LineCmdBuffer), 8);
}


void app_robot_control_rotate(float speed)
{
    LineMoveDataType LineCmdBuffer = {0};

    /* 只有处于空闲状态下才能继续发送旋转命令 */
    if (MOTOR_CMD_IDLE == motor_cmd_status)
    {
        motor_cmd_status = MOTOR_CMD_WAIT_RES;
    }
    else
    {
        return;
    }

    LineCmdBuffer.SpeedX = 0;
    LineCmdBuffer.SpeedY = 0;
    LineCmdBuffer.SpeedRotate = (INT16S)(speed * 10000);
    LineCmdBuffer.Acceleration = 0;

    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetPlatformSpeed, 0, (INT8U *)(&LineCmdBuffer), 8);
}

//============================================================================
// 名称：app_robot_control_move
// 功能：控制机器人轮子
// 参数：UsrSpeed_1：用户自定义轮子速度1，单位m/s，后续以此类推
//      MoveTimeMs：动作ms时间数
// 返回：无
// 说明：使用用户自定义速度
//轮子安装为C形，右前为1号，左前为2号
//向前行走时1号数据为正,2号为负，3号为负，4号为正
//车辆向右偏移说明1号和4号轮子转动较慢
//============================================================================
//INT8U app_robot_control_move( float UsrSpeed_1,float UsrSpeed_2,float UsrSpeed_3,float UsrSpeed_4)//, INT32U MoveTimeMs )
//{
//    INT8U SpeedDataBuffer[8];
//    INT16S tmpSpeed_1,tmpSpeed_2,tmpSpeed_3,tmpSpeed_4;

//    tmpSpeed_1 = (INT16S)(UsrSpeed_1 * 10000);    // 轮子线速度需要乘以10000倍发送
//    SpeedDataBuffer[0] = tmpSpeed_1 & 0xFF;
//    SpeedDataBuffer[1] = tmpSpeed_1 >> 8;
 
//     tmpSpeed_2 = (INT16S)(UsrSpeed_2 * 10000);    // 轮子线速度需要乘以10000倍发送
//    SpeedDataBuffer[2] = tmpSpeed_2 & 0xFF;
//    SpeedDataBuffer[3] = tmpSpeed_2 >> 8;

//     tmpSpeed_3 = (INT16S)(UsrSpeed_3 * 10000);    // 轮子线速度需要乘以10000倍发送
//    SpeedDataBuffer[4] = tmpSpeed_3 & 0xFF;
//    SpeedDataBuffer[5] = tmpSpeed_3 >> 8;

//     tmpSpeed_4 = (INT16S)(UsrSpeed_4 * 10000);    // 轮子线速度需要乘以10000倍发送
//     SpeedDataBuffer[6] = tmpSpeed_4 & 0xFF;
//    SpeedDataBuffer[7] = tmpSpeed_4 >> 8;
 


//    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetMotorSpeed, 0, SpeedDataBuffer, 8);
		//drv_delay_ms(MoveTimeMs);
    //app_robot_control_move_abort();

//    return ( 0 );
//}

//============================================================================
// 名称：app_robot_control_move
// 功能：控制机器人轮子
// 参数：UsrSpeed_1：用户自定义轮子速度1，单位m/s，后续以此类推
// 返回：无
// 说明：使用用户自定义速度
//轮子安装为C形，右前为1号，左前为2号
//向前行走时1号数据为正,2号为负，3号为负，4号为正
//车辆向右偏移说明1号和4号轮子转动较慢
//============================================================================
INT8U app_robot_control_move( float UsrSpeed_1,float UsrSpeed_2,float UsrSpeed_3,float UsrSpeed_4)
{
    INT8U SpeedDataBuffer[8];
    INT16S tmpSpeed;

    tmpSpeed= (INT16S)(UsrSpeed_1 * 10000);    // 轮子线速度需要乘以1000倍发送
    SpeedDataBuffer[0] = tmpSpeed& 0xFF;
    SpeedDataBuffer[1] = tmpSpeed>> 8;
 
     tmpSpeed= (INT16S)(UsrSpeed_2 * 10000);    // 轮子线速度需要乘以1000倍发送
    SpeedDataBuffer[2] = tmpSpeed& 0xFF;
    SpeedDataBuffer[3] = tmpSpeed>> 8;

     tmpSpeed = (INT16S)(UsrSpeed_3 * 10000);    // 轮子线速度需要乘以1000倍发送
    SpeedDataBuffer[4] = tmpSpeed& 0xFF;
    SpeedDataBuffer[5] = tmpSpeed>> 8;

     tmpSpeed = (INT16S)(UsrSpeed_4 * 10000);    // 轮子线速度需要乘以10000倍发送
    SpeedDataBuffer[6] = tmpSpeed & 0xFF;
    SpeedDataBuffer[7] = tmpSpeed>> 8;
 

#ifdef SEND_MSG_FROM_UART
    app_protocol_uart_send_msg( MOTOR_DRIVER_DEVICE_TYPE, MOTOR_DIRVER_ADDRESS, UARTCMD_SetMotorSpeed, SpeedDataBuffer, 8 );
#else
    app_protocol_can_send_package(MOTOR_DIRVER_ADDRESS, CANCMD_SetMotorSpeed, 0, SpeedDataBuffer, 8);
#endif
    return ( 0 );
}
