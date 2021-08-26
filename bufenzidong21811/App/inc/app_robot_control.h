#ifndef __APP_ROBOT_CONTROL_H__
#define __APP_ROBOT_CONTROL_H__


#include "typedefine.h"


typedef struct _LineMoveDataType
{
    INT16S SpeedX;          // X方向速度，m/s，浮点速度*10000
    INT16S SpeedY;          // Y方向速度，m/s，浮点速度*10000
    INT16S SpeedRotate;     // 自转角速度，rad/s，浮点速度*10000
    INT16S Acceleration;    // 未用，为0
}LineMoveDataType;


extern void  app_robot_control_move_abort( void );
 void app_robot_control_move_abort_jiting( void );

void app_robot_control_rotate(float speed);
/**
  * @fun_name   app_robot_control_go
  * @brief      This function set car's move speed and rotate speed.
  * @param      float speed_x：x轴方向速度(mm/s)
                float speed_y：Y轴方向速度(mm/s)
                float rotate：自转弧度速度(rad/s)
  * @retval     None
  */
void app_robot_control_go(float speed_x, float speed_y, float rotate);

/**
  * @fun_name   app_robot_control_go_again
  * @brief      This function set car's move speed and rotate speed.
  * @param      None
  * @retval     None
  */
void app_robot_control_go_again(void);


INT8U app_robot_control_move( float UsrSpeed_1,float UsrSpeed_2,float UsrSpeed_3,float UsrSpeed_4);
#endif

