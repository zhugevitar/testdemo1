/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : main.c
* Author             : 成都航发
* Version            : V1.0.1
* Date               : 2013-12-14
* Description        : 控制 Discovery Q2 机器人平台底盘例程
*    2   3
*1           0
*******************************************************************************/
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_it.h"
#include "app_protocol_can.h"
#include "app_protocol_uart.h"
#include "drv_button.h"
#include "drv_can.h"
#include "drv_delay.h"
#include "drv_led.h"
#include "drv_uart.h"
#include "drv_timer.h"
#include "app_robot_control.h"
#include "arm_math.h"

#define ABS_DIFF(x, y) (x > y) ? (x - y) : (y - x);
#define ABS(x) (x > 0) (x ? (-x));


#define ULM_NUM                         (0x04)
#define CAR_SPEED_X                     (100.0f)
#define CAR_SPEED_Y                     (150.0f)
#define ROTATE_RAD                      (0.1f)
#define ULM_DATA_NUM                    (6)

#define AVOID_DELAY_TICK                (8000)

#define MOVE_DESLAY_TIME                (100)

#define CAR_DIS_DIFF_MAX                (50.0f)
#define CAR_WARNING_DIS                 (100.0f)

#define CAR_WIDTH_MM                    (420.0f)
#define DISTANCE_RATIO                  (0.6f)


#define DELTA_VAR_MAX                   (100.0f)
#define DELTA_EXP_MAX                   (1.0f)
#define DELTA_EXP_MIN                   (-1.0f)

#define WAIT_LEAVE_TIME                 (3000)     /* 等待操作员离开延时 */

#define ENTER_LOW_POWER                 (1861)  /* 1.5 / 3.3 * 4096 */
#define OUT_LOW_POWER                   (2606)  /*  */

#define DIFF_LENGTH                     (500)

uint8_t ulm_addr_arr[ULM_NUM]           = {0xA0, 0xA2, 0xA4, 0xA6};
uint16_t ulm_distance_arr[ULM_NUM]      = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t ulm_old_distance_arr[ULM_NUM]  = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint8_t get_distance_cmd[3]             = {0xA0, 0x02, 0xBC};

float ulm0_old_distance_data            = 0;
float ulm1_old_distance_data            = 0;

float ulm_distance_sum[4]               = {0.0f};  /* 一个周期内距离之和 */
uint8_t ulm_distance_num[4]             = {0};   /* 一个周期内测得距离的次数 */

uint8_t sm120bl_state = 0;
uint16_t sm120bl_angle = 0;

/* 记录两侧传感器每个时间片内采集的数据 */
struct ulm_distance_st
{
    float ulm0_delta_data[ULM_DATA_NUM];
    float ulm1_delta_data[ULM_DATA_NUM];
    float ulm0_distance_data[ULM_DATA_NUM];
    float ulm1_distance_data[ULM_DATA_NUM];
    float ulm0_delta_exp;
    float ulm0_delta_var;
    float ulm1_delta_exp;
    float ulm1_delta_var;

    float ulm0_distance_exp;
    float ulm1_distance_exp;
    uint8_t start_idx;
    uint8_t end_idx;
};

struct ulm_distance_st ulm_distance_data = {0};


enum CAR_STATE_EN
{
    CAR_WAIT_SENSOR_OK = 0,     /* 初始化阶段等待传感器正常工作 */
    CAR_WAIT_START,             /* 等待操作员按下开始按钮状态 */
    CAR_WAIT_OPERATOR_LEAVE,    /* 通过延时等待操作员离开状态 */
    CAR_INIT_SM120BL_START,
    CAR_SM120BL_ROTATE_START,
    CAR_START_ROTATE,           /* 机器人开始运动前旋转360度 */
    CAR_SENSOR_INIT,            /* 等待传感器都正常状态 */
    CAR_GO,                     /* 机器人开始行走状态 */
    CAR_STATE_INIT,             /* 机器人状态初始化 */
    CAR_MOVE_INIT,              /* 机器人开始行走前的初始化 */
    CAR_MOVING,                 /* 机器人行走过程中 */
    CAR_ADJUSTING,              /* 通过自转调整机器人方向 */
    CAR_MOVE_STOP,              /* 机器人运动停止状态 */
    CAR_INIT_SM120BL_END,
    CAR_SM120BL_ROTATE_END,
    CAR_LACK_WATER,             /* 水箱缺药 */
    CAR_LOW_POWER,              /* 电压过低 */
    CAR_ADJUST_SENSOR,          /* 长时间有传感器处于异常状态时进入该状态 */
    CAR_OBSTACLE_AVOID,         /* 机器人通过左右运动来躲避障碍物 */
    CAR_OBSTACLE_CHECK,         /* 躲避障碍物之后，再次检查当前是否还有障碍物 */
    CAR_GO_BACK,                /* 到达终点之后需要先回退然后自转 */
    CAR_END_ROTATE,             /* 到达终点之后的自转运动 */
    CAR_GET_POSITION,
    CAR_CHANGE_POSITION         /* 到达终点自转之后调整位置 */
};

enum ULM_STATE_EN
{
    ULM_SEND_CMD = 0,
    ULM_WAIT_RES,
    ULM_DELAY
};

enum BUTTON_STATE_EN
{
    BUTTON_IDLE = 0,    /* 按键没有被按下，处于空闲状态 */
    BUTTON_DELAY,       /* 检测到按键按下，处于防抖状态 */
    BUTTON_ACTIVE       /* 已经经过防抖处理，按键确实已经按下 */
};


//============================================================================
// 名称： app_main_system_init
// 功能： 系统初始化
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================
void app_main_system_init( void )
{
    drv_led_init();
    drv_button_init();  // 注意按键初始化为查询模式
    drv_delay_init();
    systick_init();
    drv_can_init();
    drv_uart_2_init();
    drv_uart_3_init();
    drv_uart_6_init();

    drv_timer_init();
}




/**
  * @fun_name   
  * @brief      This function .
  * @param      None
  * @retval     
  */
float calculate_angle_two(float v1, float v2)   //没有使用
{
    
    float angle1_f = (v1 > 0) ? v1 : (0 - v1);
    float angle2_f = (v2 > 0) ? v2 : (0 - v2);

    // angle1_f = (1.414 * CAR_SPEED) + angle1_f;
    // angle2_f = (1.414 * CAR_SPEED) + angle2_f;

    if ((0.0f == angle1_f) || (0.0f == angle2_f))
    {
        return 0.0f;
    }
    else
    {
        angle1_f = v1 / angle1_f;
        angle1_f = atan(angle1_f);

        angle2_f = v2 / angle2_f;
        angle2_f = atan(angle2_f);

        if (angle1_f < angle2_f)
        {
            return angle2_f;
        }
        else
        {
            return angle1_f;
        }
    }
}

/**
  * @fun_name   celculate_distance_exp_fun
  * @brief      计算两侧超声模块平均距离.
  * @param      None
  * @retval     None
  */
void celculate_distance_exp_fun(void) //
{
    uint8_t idx = ulm_distance_data.start_idx;
    float ulm0_exp_f = 0.0f;
    float ulm1_exp_f = 0.0f;

    while (idx != ulm_distance_data.end_idx)
    {
        ulm0_exp_f += ulm_distance_data.ulm0_distance_data[idx];
        ulm1_exp_f += ulm_distance_data.ulm1_distance_data[idx];

        ++idx;
        if (ULM_DATA_NUM <= idx)
        {
            idx = 0;
        }
    }

    ulm_distance_data.ulm0_distance_exp = ulm0_exp_f / (float)(ULM_DATA_NUM - 1);
    ulm_distance_data.ulm1_distance_exp = ulm1_exp_f / (float)(ULM_DATA_NUM - 1);
}


void calculate_exp_var_fun()  //求两侧墙壁斜率，左右两边超声距离差的平均值和方差
{
    uint8_t idx = ulm_distance_data.start_idx;
    float ulm0_exp_f = 0.0f;
    float ulm0_var_f = 0.0f;
    float ulm1_exp_f = 0.0f;
    float ulm1_var_f = 0.0f;

    while (idx != ulm_distance_data.end_idx)
    {
        ulm0_exp_f += ulm_distance_data.ulm0_delta_data[idx];
        ulm0_var_f += ulm_distance_data.ulm0_delta_data[idx] * ulm_distance_data.ulm0_delta_data[idx];

        ulm1_exp_f += ulm_distance_data.ulm1_delta_data[idx];
        ulm1_var_f += ulm_distance_data.ulm1_delta_data[idx] * ulm_distance_data.ulm1_delta_data[idx];

        ++idx;
        if (ULM_DATA_NUM <= idx)
        {
            idx = 0;
        }
    }

    ulm_distance_data.ulm0_delta_exp = ulm0_exp_f / (float)(ULM_DATA_NUM - 1);
    ulm_distance_data.ulm0_delta_var = ((ulm0_var_f - (ulm_distance_data.ulm0_delta_exp * ulm_distance_data.ulm0_delta_exp)) / (float)(ULM_DATA_NUM - 2));


    ulm_distance_data.ulm1_delta_exp = ulm1_exp_f / (float)(ULM_DATA_NUM - 1);
    ulm_distance_data.ulm1_delta_var = ((ulm1_var_f - (ulm_distance_data.ulm1_delta_exp * ulm_distance_data.ulm1_delta_exp)) / (float)(ULM_DATA_NUM - 2));
}
void ulm_refresh_fun_1(void)    //获取两侧两个超声距离
{
    static uint8_t ulm_idx                  = 0;
    static uint8_t get_dis_idx             = 0;
    uint16_t temp_distance                  = 0;
    uint32_t now_tick                       = 0;
    static uint32_t ulm_tick                = 0;
    static enum ULM_STATE_EN ulm_status     = ULM_SEND_CMD;

    now_tick = sys_now();

    /* 得到传感器数据 */
    switch (ulm_status)
    {
        case ULM_SEND_CMD:
            if (0x00 == get_dis_idx)
            {
                get_dis_idx = 0x01;
                ulm_idx = 0x01;
            }
            else
            {
                get_dis_idx = 0x00;
                ulm_idx = 0x00;
            }
            get_distance_cmd[0] = ulm_addr_arr[ulm_idx];
            usart2_send_buff(get_distance_cmd, 3);
            ulm_tick = now_tick + 40;
            ulm_status = ULM_WAIT_RES;
            break;
        case ULM_WAIT_RES:
            if ((1 == usart2_flag) || (ulm_tick <= now_tick))
            {
                if (1 == usart2_flag)
                {
                    temp_distance = (usart2_data[3] << 8) + usart2_data[2];
                    if ((0x02 == usart2_data[1]) && (0x04 <= usart2_idx) && (0xFFFF != temp_distance))
                    {
                        
                        switch (usart2_data[0])
                        {
                            case 0xA0:       //0号
                                ulm_distance_sum[0] += temp_distance;
                                ulm_distance_num[0] += 1;
                                break;
                            case 0xA2:       //1号
                                ulm_distance_sum[1] += temp_distance;
                                ulm_distance_num[1] += 1;
                            break;
                            case 0xA4:      //2号
                                ulm_distance_sum[2] += temp_distance;
                                ulm_distance_num[2] += 1;
                            break;
                            case 0xA6:     //3号
                                ulm_distance_sum[3] += temp_distance;
                                ulm_distance_num[3] += 1;
                            break;
                            default:
                                break;
                        }
                    }

                    usart2_flag = 0;
                    usart2_idx = 0;
                }
                else
                {
                    ulm_distance_arr[ulm_idx] = 0xFFFF;
                }

                ulm_status = ULM_DELAY;
                ulm_tick = now_tick + 10;
            }
            break;
        case ULM_DELAY:
            if (ulm_tick <= now_tick)
            {
                ulm_status = ULM_SEND_CMD;
            }
            break;
        default:
            break;
    }
}


void ulm_refresh_fun_2(void)    //获取前面超声距离
{
    static uint8_t ulm_idx                  = 0;
    static uint8_t get_dis_idx             = 0;
    uint16_t temp_distance                  = 0;
    uint32_t now_tick                       = 0;
    static uint32_t ulm_tick                = 0;
    static enum ULM_STATE_EN ulm_status     = ULM_SEND_CMD;

    now_tick = sys_now();

    /* 得到传感器数据 */
    switch (ulm_status)
    {
        case ULM_SEND_CMD:
            if (0x00 == get_dis_idx)
            {
                get_dis_idx = 0x01;
                ulm_idx = 0x02;
            }
            else
            {
                get_dis_idx = 0x00;
                ulm_idx = 0x03;
            }
            get_distance_cmd[0] = ulm_addr_arr[ulm_idx];
            usart3_send_buff(get_distance_cmd, 3);
            ulm_tick = now_tick + 40;
            ulm_status = ULM_WAIT_RES;
            break;
        case ULM_WAIT_RES:
            if ((1 == usart3_flag) || (ulm_tick <= now_tick))
            {
                if (1 == usart3_flag)
                {
                    temp_distance = (usart3_data[3] << 8) + usart2_data[2];
                    if ((0x02 == usart3_data[1]) && (0x04 <= usart3_idx) && (0xFFFF != temp_distance))
                    {
                        
                        switch (usart3_data[0])
                        {
                            case 0xA0:
                                ulm_distance_sum[0] += temp_distance;
                                ulm_distance_num[0] += 1;
                                break;
                            case 0xA2:
                                ulm_distance_sum[1] += temp_distance;
                                ulm_distance_num[1] += 1;
                            break;
                            case 0xA4:
                                ulm_distance_sum[2] += temp_distance;
                                ulm_distance_num[2] += 1;
                            break;
                            case 0xA6:
                                ulm_distance_sum[3] += temp_distance;
                                ulm_distance_num[3] += 1;
                            break;
                            default:
                                break;
                        }
                    }

                    usart3_flag = 0;
                    usart3_idx = 0;
                }
                else
                {
                    ulm_distance_arr[ulm_idx] = 0xFFFF;
                }

                ulm_status = ULM_DELAY;
                ulm_tick = now_tick + 10;
            }
            break;
        case ULM_DELAY:
            if (ulm_tick <= now_tick)
            {
                ulm_status = ULM_SEND_CMD;
            }
            break;
        default:
            break;
    }
}


void ulm_distance_para_init(void)   //在每次计算后需要清零
{
    ulm_distance_sum[0] = 0.0f;
    ulm_distance_sum[1] = 0.0f;
    ulm_distance_num[0] = 0x00;
    ulm_distance_num[1] = 0x00;
}

void sm120bl_init(uint16_t angle)  //舵机，不使用
{
    sm120bl_angle = angle;
    sm120bl_state = 0;
}

uint8_t sm120bl_check_anwser_packet(uint8_t *data, uint8_t len)
{
    uint8_t check_sum = 0;
    uint8_t i = 0;

    if ((0xFF != data[0]) || (0xFF != data[1]) || (6 > len))
    {
        return 0;
    }

    for (i = 2; i < len - 1; ++i)
    {
        check_sum += data[i];
    }

    check_sum = ~check_sum;

    if (data[len - 1] == check_sum)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t sm120bl_snd_data[20] = {0};
uint8_t sm120bl_rotate()
{
    uint8_t i = 0;
    uint8_t ret = 0;
    uint8_t check_sum = 0;
    uint16_t angle = 0;
    uint16_t diff_angle = 0;
    uint32_t now_tick = 0;
    static uint32_t delay_tick = 0;

    now_tick = sys_now();


    switch (sm120bl_state)
    {
        case 0: /* 设置加速度，目标位置，时间参数，速度 */
            sm120bl_snd_data[0] = 0xFF;
            sm120bl_snd_data[1] = 0xFF;
            sm120bl_snd_data[2] = 0x01;
            sm120bl_snd_data[3] = 0x0A;
            sm120bl_snd_data[4] = 0x03;
            sm120bl_snd_data[5] = 0x29;
            sm120bl_snd_data[6] = 0x05;
            sm120bl_snd_data[7] = (uint8_t)(sm120bl_angle);
            sm120bl_snd_data[8] = (uint8_t)(sm120bl_angle >> 8);
            sm120bl_snd_data[9] = 0x00;
            sm120bl_snd_data[10] = 0x00;
            sm120bl_snd_data[11] = 0xF4;
            sm120bl_snd_data[12] = 0x01;
            check_sum = 0;
            for (i = 2; i < 13; ++i)
            {
                check_sum += sm120bl_snd_data[i];
            }
            sm120bl_snd_data[13] = ~check_sum;

            usart6_send_buff(sm120bl_snd_data, 14);

            delay_tick = now_tick + 50; /* 50ms没有反应就从新发送写寄存器命令 */
            sm120bl_state = 1;
            break;
        case 1: /* 等待返回OK */
            if (1 == usart6_flag)
            {
                ret = sm120bl_check_anwser_packet(usart6_data, usart6_idx);
                if (1 == ret) 
                {
                    if (0x00 == usart6_data[4])
                    {
                        sm120bl_state = 2;
                    }
                    else
                    {
                        sm120bl_state = 4;
                    }
                }

                usart6_idx = 0;
                usart6_flag = 0x00;
            }
            else if (now_tick > delay_tick)
            {
                sm120bl_state = 0;
            }
            break;
        case 2: /* 查询当前位置 */
            sm120bl_snd_data[0] = 0xFF;
            sm120bl_snd_data[1] = 0xFF;
            sm120bl_snd_data[2] = 0x01;
            sm120bl_snd_data[3] = 0x04;
            sm120bl_snd_data[4] = 0x02;
            sm120bl_snd_data[5] = 0x38;
            sm120bl_snd_data[6] = 0x002;
            check_sum = 0;
            for (i = 2; i < 7; ++i)
            {
                check_sum += sm120bl_snd_data[i];
            }
            sm120bl_snd_data[7] = ~check_sum;

            usart6_send_buff(sm120bl_snd_data, 8);

            now_tick = sys_now();
            delay_tick = now_tick + 50; /* 50ms没有反应就从新发送写寄存器命令 */
            sm120bl_state = 3;
            break;
        case 3:   /* 获得当前位置 */
            if (1 == usart6_flag)
            {
                ret = sm120bl_check_anwser_packet(usart6_data, usart6_idx);
                if (1 == ret) 
                {
                    if (0x00 == usart6_data[4])
                    {
                        angle = (usart6_data[6] << 8) + usart6_data[5];
                        diff_angle = ABS_DIFF(angle, sm120bl_angle);
                        if (5 >= diff_angle)
                        {
                            sm120bl_state = 5;
                        }
                        else
                        {
                            sm120bl_state = 2;
                        }
                    }
                    else
                    {
                        sm120bl_state = 4;
                    }
                }
                else
                {
                    sm120bl_state = 2;
                }

                usart6_idx = 0;
                usart6_flag = 0x00;
            }
            else if (now_tick > delay_tick)
            {
                /* 超时没有旋转到指定位置就在此发送执行命令 */
                sm120bl_state = 0;
            }
            break;
        case 4: /* 出现错误 */
            break;
        case 5: /* 运动到指定位置完成 */
            break;
        default:
            break;
    }
    return sm120bl_state;
}


// 配置ADC
void adc_config(void)     //采集电池电压
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能端口外设时钟和DMA2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    // 使能ADC1模块时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // ------------------ADC相关配置--------------------
    // GPIO配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC通用配置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC通道10配置
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);


    // DMA2 流0 通道0配置
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcvalue;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream0_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;                     //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;                            //响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
    {
        ;
    }

    // 使能DMA2流0
    DMA_Cmd(DMA2_Stream0, ENABLE);

    // 使能DMA请求
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);


    // 使能ADC1的DMA
    ADC_DMACmd(ADC1, ENABLE);

    // 使能ADC1
    ADC_Cmd(ADC1, ENABLE);
}

#define DISTANCE_ARR_NUM    (400)
float left_distance_arr[DISTANCE_ARR_NUM] = {0};
uint8_t left_distance_idx_start = 0;
uint8_t left_distance_idx_end = 0;


float right_distance_arr[DISTANCE_ARR_NUM] = {0};
uint8_t right_distance_idx_start = 0;
uint8_t right_distance_idx_end = 0;

void init_distance(float left, float right)     //没有使用
{
    left_distance_arr[0] = left;
    right_distance_arr[0] = right;

    left_distance_idx_start = 0;
    left_distance_idx_end = 1;

    right_distance_idx_start = 0;
    right_distance_idx_end = 1;
}

void add_items_to_distance(float left, float right)  //没有使用
{
    left_distance_arr[0] = left;
    right_distance_arr[0] = right;
}

uint8_t back_flag = 0;

//============================================================================
// 名称： main
// 功能： C主函数
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================
int main( void )
{
    uint32_t led1_tim = 500;
    uint32_t led2_tim = 0;
    uint32_t led5_tim = 0;
    uint32_t lack_delay_tim = 0;
    uint32_t low_tim = 0;          /* 当机器人处于缺药状态红色指示灯闪烁定时器 */
    uint32_t now_tick       = 0;    /* 当前systick值 */
    uint32_t after_avoid_tick = 0;        /* 避障后停止需找中线延时 */
    uint32_t bt1_tick = 0;                  /* 按键1延时tick */
    uint32_t bt2_tick = 0;                  /* 按键2延时tick */
    uint32_t adc_tick = 0;
    uint8_t led1_flag = 0;
    uint8_t led2_flag = 0;
    uint8_t led5_flag = 0;
    uint8_t red_led_flag = 0;
    uint8_t u8_ret = 0;
    uint8_t stop_flag = 0;              /* 是否为按键导致的停止 */
    uint8_t cmd_err_cnt = 0;
    uint8_t error_sides = 0;            /* 两侧传感器错误次数计数 */
    uint8_t error_front = 0;            /* 前方传感器错误次数计数 */
    enum CAR_STATE_EN main_state = CAR_WAIT_START;
    enum BUTTON_STATE_EN bt1_status = BUTTON_IDLE;  /* 按键1状态机 */
    enum BUTTON_STATE_EN bt2_status = BUTTON_IDLE;  /* 按键2状态机 */
    uint32_t move_tick = MOVE_DESLAY_TIME;
    uint32_t cmd_tick = 0;
    float car_rotate = 0.0f;
    float temp_distance = 0.0f;
    float temp_distance1 = 0.0f;
    float temp_distance2 = 0.0f;
    float temp_distance3 = 0.0f;
    float avoid_time = 0.0f;



    // 初始化
    app_main_system_init();

    adc_config();
    // 强制使底盘停止动作
    app_robot_control_move_abort();

    set_pwm_config(111);  //设置水泵PWM

    // 启动ADC转换
    ADC_SoftwareStartConv(ADC1);


    while ( 1 )
    {
        now_tick = sys_now(); //1ms


        /* pcb绿色led闪烁，表示系统运行正常 */
        if (led1_tim <= now_tick)
        {
            led1_tim = now_tick + 2000;
            if (0 == led1_flag)
            {
                led1_flag = 1;
                drv_led_on(LED1);
            }
            else
            {
                led1_flag = 0;
                drv_led_off(LED1);
            }
        }

        /* 计算ADC值 ,输入电池电压*/
        if (0x01 == voltage_flag)
        {
            if (ENTER_LOW_POWER >= voltage_value)
            {
                if ((now_tick >= adc_tick) && (CAR_LOW_POWER != main_state))
                {
                    /* 当前处于缺药状态，直接进入停止状态 */
                    drv_led_on(LED2);           /* 打开板载红灯 */
                    drv_led_off(LED3);          /* 关闭雾化器 */
                    drv_led_off(LED5);          /* 关闭绿色指示灯 */
                    drv_led_off(LED6);          /* 关闭黄色指示灯 */
                    drv_led_on(LED7);           /* 打开红色指示灯 */

                    /* 关闭抽水泵 */
                    set_pwm_config(110);

                    /* 机器人停止运动 */
                    app_robot_control_move_abort();

                    red_led_flag = 0;  /* 缺药状态红色指示灯闪烁状态 */

                    main_state = CAR_LOW_POWER; /* 直接进入停止状态 */
                }
            }
            else
            {
                adc_tick = move_tick + 500;
            }
        }
        /* 开始按钮的按键检测 ，按下20ms以上且松开*/
        switch (bt1_status)
        {
            case BUTTON_IDLE:   /* 按键没有被按下，处于空闲状态 */
                u8_ret = drv_button_get_status(Button2);
                if (0x00 == u8_ret)
                {
                    bt1_tick = now_tick + 20;   /* 20ms防抖 */
                    bt1_status = BUTTON_DELAY;
                }
                break;
            case BUTTON_DELAY:  /* 检测到按键按下，处于防抖状态 */
                if (bt1_tick <= now_tick)
                {
                    bt1_status = BUTTON_ACTIVE;
                }
                break;
            case BUTTON_ACTIVE: /* 已经经过防抖处理，按键确实已经按下 */
                u8_ret = drv_button_get_status(Button2);
                if (0x01 == u8_ret)
                {
                    /* 需要等到按键抬起才算按键结束 */
                    bt1_status = BUTTON_IDLE;
                }
                break;
            default:
                bt1_status = BUTTON_IDLE;
                break;
        }

        /* 结束按钮的按键检测 ，按下20ms以上且松开*/
        switch (bt2_status)
        {
            case BUTTON_IDLE:   /* 按键没有被按下，处于空闲状态 */
                u8_ret = drv_button_get_status(Button3);
                if (0x00 == u8_ret)
                {
                    bt2_tick = now_tick + 20;   /* 20ms防抖 */
                    bt2_status = BUTTON_DELAY;
                }
                break;
            case BUTTON_DELAY:  /* 检测到按键按下，处于防抖状态 */
                if (bt2_tick <= now_tick)
                {
                    bt2_status = BUTTON_ACTIVE;
                    stop_flag = 1;
                }
                break;
            case BUTTON_ACTIVE: /* 已经经过防抖处理，按键确实已经按下 */
                u8_ret = drv_button_get_status(Button3);
                if (0x01 == u8_ret)
                {
                    /* 需要等到按键抬起才算按键结束 */
                    bt2_status = BUTTON_IDLE;
                }
                break;
            default:
                bt2_status = BUTTON_IDLE;
                break;
        }

        u8_ret = drv_get_lc_state();   //获取当前液位状态.0,缺药；1,有药

        if (0x00 == u8_ret)  //缺药2s以上认为缺药
        {
            if ((now_tick >= lack_delay_tim) && (CAR_LACK_WATER != main_state) && (CAR_LOW_POWER != main_state))
            {
                /* 当前处于缺药状态，直接进入停止状态 */
                drv_led_on(LED2);           /* 打开板载红灯 */
                drv_led_off(LED3);          /* 关闭雾化器 */
                drv_led_off(LED5);          /* 关闭绿色指示灯 */
                drv_led_off(LED6);          /* 关闭黄色指示灯 */
                drv_led_on(LED7);           /* 打开红色指示灯 */

                /* 关闭抽水泵 */
                set_pwm_config(110);

                /* 机器人停止运动 */
                app_robot_control_move_abort();

                red_led_flag = 0;  /* 缺药状态红色指示灯闪烁状态 */

                main_state = CAR_LACK_WATER; /* 直接进入停止状态 */
            }
        }
        else
        {
            lack_delay_tim = now_tick + 2000;
        }

        if (BUTTON_ACTIVE == bt2_status)
        {
            /* 用户按下停止按钮 */
            drv_led_on(LED2);           /* 打开板载红灯 */
            drv_led_off(LED3);          /* 关闭雾化器 */
            drv_led_off(LED5);          /* 关闭绿色指示灯 */
            drv_led_off(LED6);          /* 关闭黄色指示灯 */
            drv_led_on(LED7);           /* 打开红色指示灯 */

            set_pwm_config(110);        /* 关闭抽水泵 */

            main_state = CAR_MOVE_STOP; /* 直接进入停止状态 */

            /* 机器人停止运动 */
            app_robot_control_move_abort();
        }

        ulm_refresh_fun_1();  //刷新两侧两个测距
        ulm_refresh_fun_2();  //测距

        switch (main_state)
        {
            case CAR_WAIT_SENSOR_OK:
                /* 四个传感器都正常之后才能继续下一步操作 */
                if ((0x00 != ulm_distance_num[0]) && (0x00 != ulm_distance_num[1])
                    && (0x00 != ulm_distance_num[2]) && (0x00 != ulm_distance_num[3]))
                {
                    drv_led_off(LED6);         /* 关闭黄色指示灯 */
                    main_state = CAR_WAIT_START;
                }
                else if (now_tick >= move_tick)
                {
                    /* 一段时间内传感器还没有收到数据，就认为传感器异常，亮起黄灯 */
                    drv_led_on(LED6);         /* 打开黄色指示灯 */
                }
                break;
            case CAR_WAIT_START:    /* 等待操作员按下启动按钮 */
                drv_led_off(LED6);         /* 关闭黄色指示灯 */
                drv_led_off(LED7);         /* 关闭红色指示灯 */

                /* 等待用户按下开始按键时绿灯闪烁 */
                if (now_tick >= move_tick)
                {
                    move_tick = now_tick + 1000;
                    if (0x00 == led5_flag)
                    {
                        led5_flag = 0x01;
                        drv_led_on(LED5);
                    }
                    else
                    {
                        led5_flag = 0x00;
                        drv_led_off(LED5);
                    }
                }
                if (BUTTON_ACTIVE == bt1_status)
                {
                    main_state = CAR_WAIT_OPERATOR_LEAVE;  /* 开始阶段先不执行自转 */
                    move_tick = now_tick + WAIT_LEAVE_TIME;
                    led5_tim = now_tick + 200;
                }
                break;
/*            case CAR_INIT_SM120BL_START:
                sm120bl_init(2048);
                main_state = CAR_SM120BL_ROTATE_START;
                break;
            case CAR_SM120BL_ROTATE_START:
                u8_ret = sm120bl_rotate();

                if (5 == u8_ret)
                {
                    main_state = CAR_START_ROTATE;
                    move_tick = now_tick + 200;   // 等待操作员离开 
                }
                break;*/
            case CAR_WAIT_OPERATOR_LEAVE:   /* 等待操作员离开 */
                if (move_tick <= now_tick)
                {
                    // app_robot_control_go(0.0f, 0.0f, 0.1f);  /* 发送自转命令 */
                    main_state = CAR_START_ROTATE;
                    move_tick = now_tick + MOVE_DESLAY_TIME;
                }
                else
                {
                    /* 快闪绿灯提醒操作员赶紧离开 */
                    if (led5_tim <= now_tick)
                    {
                        led5_tim = now_tick + 200;
                        if (0x00 == led5_flag)
                        {
                            led5_flag = 0x01;
                            drv_led_on(LED5);
                        }
                        else
                        {
                            led5_flag = 0x00;
                            drv_led_off(LED5);
                        }
                    }
                }
                break;
            case CAR_START_ROTATE:  /* 空操作 */
                if (move_tick <= now_tick)
                {
                    /* 自转结束，机器人停止运动，打开黄色指示灯直到传感器正常 */
                    app_robot_control_move_abort();
                    move_tick = now_tick + MOVE_DESLAY_TIME;
                    main_state = CAR_SENSOR_INIT;
                }
                break;
            case CAR_SENSOR_INIT:   /* 等待所有传感器都正常工作 */
                if (move_tick <= now_tick)
                {
                    /* 四个传感器都正常之后才能继续下一步操作 */
                    if ((0x00 != ulm_distance_num[0]) && (0x00 != ulm_distance_num[1])
                        && (0x00 != ulm_distance_num[2]) && (0x00 != ulm_distance_num[3]))   
                    {
                        move_tick = now_tick + MOVE_DESLAY_TIME;
                        main_state = CAR_GO;
                        drv_led_off(LED6);   /* 关闭黄灯 */
                    }
                    else                       //一次数据都没有收到
                    {
                        drv_led_on(LED6);  /* 打开黄色指示灯直到所有传感器都已经能够正常工作 */
                        move_tick = now_tick + MOVE_DESLAY_TIME;
                    }

                    drv_led_off(LED7);   /* 关闭红灯 */
                    set_pwm_config(110);    /* 关闭喷洒头 */
                }
                break;
            case CAR_GO:    /* 机器人开始行走，并开始喷雾作业 */
                app_robot_control_go(0.0f, CAR_SPEED_Y, 0.0f);  /* 发送向前行走命令 */
                drv_led_off(LED2);                              /* 关闭板载红色指示灯 */
                /* 返回的时候不喷药 */
                if (0 == back_flag)
                {
                    drv_led_on(LED3);                               /* 打开喷洒头 */
                    set_pwm_config(117);                            /* 设置水泵速度 */
                }
                drv_led_on(LED5);                               /* 打开绿色指示灯 */
                drv_led_off(LED6);                              /* 关闭黄色指示灯，表示传感器已经正常工作 */
                ulm_distance_para_init();                       /* 将左右两侧超声超时传感器数据清零 */
                main_state = CAR_STATE_INIT;                    /* 跳转到下一状态 */
                move_tick = now_tick + MOVE_DESLAY_TIME;        /* 定义延时时间 */
                break;
            case CAR_STATE_INIT:    /* 状态初始化，通过延时时间段内传感器采集的数据，获得机器人状态 */
                if (now_tick >= move_tick)
                {
                    /* 通过求取平均值的方法来获得当前机器人的状态 */
                    ulm0_old_distance_data = (float)(ulm_distance_sum[0] / ulm_distance_num[0]);
                    ulm1_old_distance_data = (float)(ulm_distance_sum[1] / ulm_distance_num[1]);
                    ulm_distance_para_init();   /* 计算完两侧的距离之后就清除累加数据 */

                    /* 记录数据清零 */
                    ulm_distance_data.start_idx = 0;
                    ulm_distance_data.end_idx   = 0;
                    move_tick = now_tick + MOVE_DESLAY_TIME;
                    main_state = CAR_MOVE_INIT;
                }
                break;
            case CAR_MOVE_INIT:   //计算数据组初始化
                if (now_tick >= move_tick)
                {
                    /* 两侧传感器200ms收不到数据就需要重新确认传感器 */
                    if ((0x00 == ulm_distance_num[0]) || (0x00 == ulm_distance_num[1]))
                    {
                        ++error_sides;
                        if (2 <= error_sides)
                        {
                            error_sides = 0;
                            move_tick = now_tick + MOVE_DESLAY_TIME;
                            main_state = CAR_ADJUST_SENSOR;
                            continue;
                        }
                    }
                    else
                    {
                        error_sides = 0;
                    }

                    /* 前面传感器400ms收不到数据就需要重新确认传感器 */
                    if ((0x00 == ulm_distance_num[0]) || (0x00 == ulm_distance_num[1]))
                    {
                        ++error_front;
                        if (4 <= error_front)
                        {
                            error_front = 0;
                            move_tick = now_tick + MOVE_DESLAY_TIME;
                            main_state = CAR_ADJUST_SENSOR;
                            continue;
                        }
                    }
                    else
                    {
                        error_front = 0;
                    }

                    temp_distance = (float)(ulm_distance_sum[0] / ulm_distance_num[0]);
                    ulm_distance_data.ulm0_distance_data[ulm_distance_data.end_idx] = temp_distance;
                    ulm_distance_data.ulm0_delta_data[ulm_distance_data.end_idx] = temp_distance - ulm0_old_distance_data;
                    ulm0_old_distance_data = temp_distance;
                    temp_distance = (float)(ulm_distance_sum[1] / ulm_distance_num[1]);
                    ulm_distance_data.ulm1_distance_data[ulm_distance_data.end_idx] = temp_distance;
                    ulm_distance_data.ulm1_delta_data[ulm_distance_data.end_idx] = temp_distance - ulm1_old_distance_data;
                    ulm1_old_distance_data = temp_distance;

                    ulm_distance_para_init();
                    ++ulm_distance_data.end_idx;
                    if ((ULM_DATA_NUM - 1) <= ulm_distance_data.end_idx)
                    {
                        main_state = CAR_MOVING;
                        move_tick = now_tick + MOVE_DESLAY_TIME;
                    }
                    else
                    {
                        main_state = CAR_MOVE_INIT;
                        move_tick = now_tick + MOVE_DESLAY_TIME;
                    }
                }

                break;
            case CAR_MOVING:    //行走过程
                if (now_tick >= move_tick)
                {
                    /* 两侧传感器200ms收不到数据就需要重新确认传感器 */
                    if ((0x00 == ulm_distance_num[0]) || (0x00 == ulm_distance_num[1]))
                    {
                        ++error_sides;
                        if (2 <= error_sides)
                        {
                            error_sides = 0;
                            move_tick = now_tick + MOVE_DESLAY_TIME;
                            main_state = CAR_ADJUST_SENSOR;
                            continue;
                        }
                    }
                    else
                    {
                        error_sides = 0;
                    }

                    /* 前面传感器400ms收不到数据就需要重新确认传感器 */
                    if ((0x00 == ulm_distance_num[0]) || (0x00 == ulm_distance_num[1]))
                    {
                        ++error_front;
                        if (4 <= error_front)
                        {
                            error_front = 0;
                            move_tick = now_tick + MOVE_DESLAY_TIME;
                            main_state = CAR_ADJUST_SENSOR;
                            continue;
                        }
                    }
                    else
                    {
                        error_front = 0;
                    }

                    celculate_distance_exp_fun();
                    /* 先判断前面是否有障碍物 */
                    if ((0x00 != ulm_distance_num[2]) && (0x00 != ulm_distance_num[3]))
                    {
                        temp_distance = (float)(ulm_distance_sum[2]/ ulm_distance_num[2]);
                        temp_distance1 = (float)(ulm_distance_sum[3]/ ulm_distance_num[3]);

                        ulm_distance_sum[2] = 0.0f;
                        ulm_distance_sum[3] = 0.0f;
                        ulm_distance_num[2] = 0x00;
                        ulm_distance_num[3] = 0x00;

                        temp_distance *= DISTANCE_RATIO;
                        temp_distance1 *= DISTANCE_RATIO;
                        if (CAR_WIDTH_MM > temp_distance)
                        {
                            if (CAR_WIDTH_MM > temp_distance1)
                            {
                                /* 两侧传感器都检测到障碍物 */
                                if (((CAR_WIDTH_MM - temp_distance) < ulm_distance_data.ulm0_distance_exp)
                                    && ((CAR_WIDTH_MM - temp_distance1) < ulm_distance_data.ulm1_distance_exp))
                                {
                                    temp_distance2 = ulm_distance_data.ulm0_distance_exp + temp_distance - CAR_WIDTH_MM;
                                    temp_distance3 = ulm_distance_data.ulm1_distance_exp + temp_distance1 - CAR_WIDTH_MM;
                                    if (temp_distance2 > temp_distance3)
                                    {
                                        main_state = CAR_OBSTACLE_AVOID;
                                        after_avoid_tick = now_tick + AVOID_DELAY_TICK;
                                        avoid_time = ((CAR_WIDTH_MM - temp_distance + 200.0f) * 900.0f) / CAR_SPEED_X;
                                        move_tick = now_tick + (uint32_t)(avoid_time);
                                        app_robot_control_go(CAR_SPEED_X, 0.0f, 0.0f);
                                        continue;
                                    }
                                    else
                                    {
                                        main_state = CAR_OBSTACLE_AVOID;
                                        after_avoid_tick = now_tick + AVOID_DELAY_TICK;
                                        avoid_time = ((CAR_WIDTH_MM - temp_distance1 + 200.0f) * 900.0f) / CAR_SPEED_X;
                                        move_tick = now_tick + (uint32_t)(avoid_time);
                                        app_robot_control_go((0 - CAR_SPEED_X), 0.0f, 0.0f);
                                        continue;
                                    }
                                }
                                else
                                {
                                    /* 两侧数据异常 */
                                    app_robot_control_move_abort();
                                    main_state = CAR_MOVE_STOP;
                                    stop_flag = 1;
                                    continue;
                                }
                            }
                            else
                            {
                                /* 左边检测到障碍物，右边没有检测到障碍物 */
                                temp_distance2 = ulm_distance_data.ulm0_distance_exp - temp_distance + CAR_WIDTH_MM;
                                temp_distance3 = ulm_distance_data.ulm1_distance_exp + temp_distance;
                                if (temp_distance2 > temp_distance3)
                                {
                                    avoid_time = ((temp_distance + 200.0f) * 900.0f) / CAR_SPEED_X;
                                    app_robot_control_go(CAR_SPEED_X, 0.0f, 0.0f);
                                }
                                else
                                {
                                    avoid_time = ((CAR_WIDTH_MM - temp_distance + 200.0f) * 900.0f) / CAR_SPEED_X;
                                    app_robot_control_go((0 - CAR_SPEED_X), 0.0f, 0.0f);
                                }
                                move_tick = now_tick + (uint32_t)(avoid_time);
                                after_avoid_tick = now_tick + AVOID_DELAY_TICK;
                                main_state = CAR_OBSTACLE_AVOID;
                                continue;
                            }
                        }
                        else if (CAR_WIDTH_MM > temp_distance1)
                        {
                            /* 右边检测到障碍物，左边边没有检测到障碍物 */
                            temp_distance2 = ulm_distance_data.ulm0_distance_exp + temp_distance1;
                            temp_distance3 = ulm_distance_data.ulm1_distance_exp - temp_distance1 + CAR_WIDTH_MM;
                            if (temp_distance2 > temp_distance3)
                            {
                                avoid_time = ((CAR_WIDTH_MM - temp_distance1 + 200.0f) * 900.0f) / CAR_SPEED_X;
                                app_robot_control_go(CAR_SPEED_X, 0.0f, 0.0f);
                            }
                            else
                            {
                                avoid_time = ((temp_distance1 + 200.0f) * 800.0f) / CAR_SPEED_X;
                                app_robot_control_go((0 - CAR_SPEED_X), 0.0f, 0.0f);
                            }
                            move_tick = now_tick + (uint32_t)(avoid_time);
                            after_avoid_tick = now_tick + AVOID_DELAY_TICK;
                            main_state = CAR_OBSTACLE_AVOID;
                            continue;
                        }
                    }
                    /* 在前面没有障碍物的情况下，判断两侧距离是否过小 */
                    if (CAR_WARNING_DIS >= ulm_distance_data.ulm0_distance_exp)
                    {
                        if (CAR_WARNING_DIS >= ulm_distance_data.ulm1_distance_exp)
                        {
                            /* 两侧都小于警告值，就停止运动 */
                            app_robot_control_move_abort();
                            main_state = CAR_MOVE_STOP;
                            stop_flag = 1;
                            continue;
                        }
                        else
                        {
                            /* 右侧距离过小 */
                            main_state = CAR_ADJUSTING;
                            move_tick = now_tick + 500;
                            app_robot_control_go((0 - CAR_SPEED_X), 0.0f, 0.0f);
                            continue;
                        }
                    }
                    else if (CAR_WARNING_DIS >= ulm_distance_data.ulm1_distance_exp)
                    {
                            /* 左侧距离过小 */
                            main_state = CAR_ADJUSTING;
                            move_tick = now_tick + 500;
                            app_robot_control_go(CAR_SPEED_X, 0.0f, 0.0f);
                            continue;
                    }

                    temp_distance = (float)(ulm_distance_sum[0] / ulm_distance_num[0]);
                    ulm_distance_data.ulm0_distance_data[ulm_distance_data.end_idx] = temp_distance;
                    ulm_distance_data.ulm0_delta_data[ulm_distance_data.end_idx] = temp_distance - ulm0_old_distance_data;
                    ulm0_old_distance_data = temp_distance;
                    temp_distance = (float)(ulm_distance_sum[1] / ulm_distance_num[1]);
                    ulm_distance_data.ulm1_distance_data[ulm_distance_data.end_idx] = temp_distance;
                    ulm_distance_data.ulm1_delta_data[ulm_distance_data.end_idx] = temp_distance - ulm1_old_distance_data;
                    ulm1_old_distance_data = temp_distance;

                    ulm_distance_para_init();
                    ++ulm_distance_data.end_idx;
                    ++ulm_distance_data.start_idx;
                    if (ULM_DATA_NUM == ulm_distance_data.start_idx)
                    {
                        ulm_distance_data.start_idx = 0;
                    }
                    if (ULM_DATA_NUM == ulm_distance_data.end_idx)
                    {
                        ulm_distance_data.end_idx = 0;
                    }
                    calculate_exp_var_fun();

                    if (DELTA_VAR_MAX >= ulm_distance_data.ulm0_delta_var)
                    {
                        if (DELTA_VAR_MAX >= ulm_distance_data.ulm1_delta_var)
                        {
                            if (((DELTA_EXP_MAX >= ulm_distance_data.ulm0_delta_exp) && (DELTA_EXP_MIN <= ulm_distance_data.ulm0_delta_exp))
                                || ((DELTA_EXP_MAX >= ulm_distance_data.ulm1_delta_exp) && (DELTA_EXP_MIN <= ulm_distance_data.ulm1_delta_exp))
                                || ((DELTA_EXP_MAX <= ulm_distance_data.ulm0_delta_exp) && (DELTA_EXP_MAX <= ulm_distance_data.ulm1_delta_exp))
                                || ((DELTA_EXP_MIN >= ulm_distance_data.ulm0_delta_exp) && (DELTA_EXP_MIN >= ulm_distance_data.ulm1_delta_exp)))
                            {
                                /* 如果处于调整后禁止平移阶段就不能进行平移操作 */
                                /* 两边都没用明显的偏向,就计算两侧距离是否相差过大 */
                                temp_distance = ABS_DIFF(ulm_distance_data.ulm0_distance_exp, ulm_distance_data.ulm1_distance_exp);
                                if ((CAR_DIS_DIFF_MAX <= temp_distance) && (now_tick >= after_avoid_tick))
                                {
                                    avoid_time = (ulm_distance_data.ulm0_distance_exp - ulm_distance_data.ulm1_distance_exp);
                                    if (ulm_distance_data.ulm0_distance_exp > ulm_distance_data.ulm1_distance_exp)
                                    {
                                        app_robot_control_go(CAR_SPEED_X, (CAR_SPEED_Y / 3.0f), 0.0f);
                                    }
                                    else
                                    {
                                        app_robot_control_go((0 - CAR_SPEED_X), (CAR_SPEED_Y / 3.0f), 0.0f);
                                        avoid_time = 0.0f - avoid_time;
                                    }
                                    avoid_time = avoid_time * 600.0f;
                                    main_state = CAR_ADJUSTING;
                                    move_tick = now_tick + (uint32_t)(avoid_time / CAR_SPEED_Y);
                                }
                                else
                                {
                                    main_state = CAR_MOVING;
                                    move_tick = now_tick + MOVE_DESLAY_TIME;
                                }
                            }
                            else
                            {
                                if (0 < ulm_distance_data.ulm0_delta_exp)
                                {
                                    car_rotate = (0.0f - ROTATE_RAD);
                                }
                                else
                                {
                                    car_rotate = ROTATE_RAD;
                                }
                                app_robot_control_go(0.0f, CAR_SPEED_Y, car_rotate);

                                main_state = CAR_ADJUSTING;
                                move_tick = now_tick + 500;
                            }
                        }
                        else
                        {
                            if (0 < ulm_distance_data.ulm0_delta_exp)
                            {
                                car_rotate = (0.0f - ROTATE_RAD);
                            }
                            else
                            {
                                car_rotate = ROTATE_RAD;
                            }
                            app_robot_control_go(0.0f, CAR_SPEED_Y, car_rotate);

                            main_state = CAR_ADJUSTING;
                            move_tick = now_tick + 500;
                        }
                    }
                    else if (DELTA_VAR_MAX >= ulm_distance_data.ulm1_delta_var)
                    {
                        if (0 < ulm_distance_data.ulm1_delta_exp)
                        {
                            car_rotate = ROTATE_RAD;
                        }
                        else
                        {
                            car_rotate = (0.0f - ROTATE_RAD);
                        }
                        app_robot_control_go(0.0f, CAR_SPEED_Y, car_rotate);

                        main_state = CAR_ADJUSTING;
                        move_tick = now_tick + 500;
                    }
                    else
                    {
                        main_state = CAR_MOVING;
                        move_tick = now_tick + MOVE_DESLAY_TIME;
                    }
                }
                break;
            case CAR_ADJUSTING:
                /* 在调整的过程中也需要时刻关注两侧距离和前方时候有障碍物 */
                /* 如果有就需要立刻停止调整 */
                /* 先判断前面是否有障碍物 */
                if ((0x00 != ulm_distance_num[2]) && (0x00 != ulm_distance_num[3]))
                {
                    temp_distance = (float)(ulm_distance_sum[2]/ ulm_distance_num[2]);
                    temp_distance1 = (float)(ulm_distance_sum[3]/ ulm_distance_num[3]);

                    /* 这里不能清空两侧传感器数据 */

                    temp_distance *= DISTANCE_RATIO;
                    temp_distance1 *= DISTANCE_RATIO;

                    if ((CAR_WIDTH_MM >= temp_distance) || (CAR_WIDTH_MM >= temp_distance1))
                    {
                        main_state = CAR_MOVING;
                    }
                }
                if (now_tick >= move_tick)
                {
                    move_tick = now_tick + MOVE_DESLAY_TIME;
                    ulm_distance_para_init();
                    main_state = CAR_GO;
                }
                break;
            case CAR_ADJUST_SENSOR:     /* 传感器异常处理 */
                app_robot_control_move_abort();     /* 机器人停止运动 */
                drv_led_on(LED2);                   /* 打开板子红灯 */
                drv_led_off(LED3);                  /* 关闭喷洒头 */
                drv_led_off(LED5);                  /* 关闭绿色指示灯 */
                drv_led_on(LED6);                   /* 打开黄色指示灯 */
                set_pwm_config(110);                /* 关闭水泵 */

                ulm_distance_para_init();           /* 清空传感器数据，从而得到全新的数据 */

                move_tick = now_tick + MOVE_DESLAY_TIME;
                main_state = CAR_SENSOR_INIT;
                break;
            case CAR_OBSTACLE_AVOID:
                /* 横向移动时需要时刻关注两侧距离 */
                if ((0x00 < ulm_distance_num[0])
                    && (0x00 < ulm_distance_num[1]))
                {
                    temp_distance = (float)(ulm_distance_sum[0] / (float)ulm_distance_num[0]);
                    temp_distance1 = (float)(ulm_distance_sum[1] / (float)ulm_distance_num[1]);

                    if ((CAR_WARNING_DIS >= temp_distance) || (CAR_WARNING_DIS >= temp_distance1))
                    {
                        ulm_distance_para_init();   /* 清空传感器数据，从而得到全新的数据 */
                        ulm_distance_sum[2] = 0.0f;
                        ulm_distance_sum[3] = 0.0f;
                        ulm_distance_num[2] = 0x00;
                        ulm_distance_num[3] = 0x00;
                        main_state = CAR_OBSTACLE_CHECK;
                    }
                }
                if (move_tick <= now_tick)
                {
                    ulm_distance_para_init();   /* 清空传感器数据，从而得到全新的数据 */
                    ulm_distance_sum[2] = 0.0f;
                    ulm_distance_sum[3] = 0.0f;
                    ulm_distance_num[2] = 0x00;
                    ulm_distance_num[3] = 0x00;
                    main_state = CAR_OBSTACLE_CHECK;
                }
                break;
            case CAR_OBSTACLE_CHECK:
                if ((0x00 < ulm_distance_num[0]) && (0x00 < ulm_distance_num[1])
                    && (0x00 < ulm_distance_num[2]) && (0x00 < ulm_distance_num[3]))
                {
                    temp_distance = (float)(ulm_distance_sum[2] / ulm_distance_num[2]);
                    temp_distance1 = (float)(ulm_distance_sum[3] / ulm_distance_num[3]);

                    ulm_distance_sum[2] = 0.0f;
                    ulm_distance_sum[3] = 0.0f;
                    ulm_distance_num[2] = 0x00;
                    ulm_distance_num[3] = 0x00;

                    temp_distance *= DISTANCE_RATIO;   //计算前面障碍物距当前传感器
                    temp_distance1 *= DISTANCE_RATIO;           

                    if ((CAR_WIDTH_MM < temp_distance) && (CAR_WIDTH_MM < temp_distance1))
                    {
                        /* 两侧都能通过 */
                        main_state = CAR_GO;
                    }
                    else
                    {
                        /* 已经经过了避障阶段之后，前方还有障碍物 */
                        /* 那么就向后运动一定距离，然后自转一周 */
                        app_robot_control_move_abort();
                        move_tick = now_tick + 2000;
                        main_state = CAR_MOVE_STOP;
                        stop_flag = 0;
                    }
                }
                break;
            case CAR_MOVE_STOP:
                drv_led_off(LED3);      /* 关闭雾化器 */
                drv_led_on(LED2);       /* 打开板载红灯 */
                drv_led_off(LED5);      /* 关闭绿色指示灯灯 */
                drv_led_on(LED7);       /* 打开红色指示灯 */
                set_pwm_config(110);    /* 关闭水泵 */
                /* 停止运动，LED2闪烁 */
                if (led2_tim <= now_tick)
                {
                    led2_tim = now_tick + 300;

                    if (0 == led2_flag)
                    {
                        led2_flag = 1;
                        drv_led_on(LED2);
                    }
                    else
                    {
                        led2_flag = 0;
                        drv_led_off(LED2);
                    }
                }
                if ((0x00 == stop_flag) && (0x00 == back_flag))
                {
                    back_flag = 0x01;
                    ulm_distance_para_init();
                    main_state = CAR_GET_POSITION;
                }
                
                /*
                if (BUTTON_ACTIVE == bt1_status)
                {
                    main_state = CAR_SENSOR_INIT;
                    move_tick = now_tick + MOVE_DESLAY_TIME;
                    drv_led_off(LED7);
                }
                */
                break;
            case CAR_INIT_SM120BL_END:
                sm120bl_init(0);
                main_state = CAR_SM120BL_ROTATE_END;
                break;
            case CAR_SM120BL_ROTATE_END:
                u8_ret = sm120bl_rotate();

                if (5 == u8_ret)
                {
                    main_state = CAR_GO_BACK;
                    move_tick = now_tick + 200;   /* 等待操作员离开 */
                }
                break;
            case CAR_LACK_WATER:    /* 药箱处于缺药状态 */
                /* 缺药，红色指示灯闪烁 */
                if (low_tim <= now_tick)
                {
                    low_tim = now_tick + 1000;

                    if (0 == red_led_flag)
                    {
                        red_led_flag = 1;
                        drv_led_on(LED6);
                    }
                    else
                    {
                        red_led_flag = 0;
                        drv_led_off(LED6);
                    }
                }

                /* 时刻判断当前液位计状态，如果已经添加液体，就进入停止状态 */
                /* 等待操作员重新按下开始按钮就可以从新进行消毒作业 */
                u8_ret = drv_get_lc_state();

                /* 需要等到连续500ms有水才退出 */
                if (0x01 == u8_ret)
                {
                    if (now_tick >= move_tick)
                    {
                        main_state = CAR_WAIT_START;
                        drv_led_off(LED6);
                    }
                }
                else
                {
                    move_tick = now_tick + 500;
                }
                break;
            case CAR_LOW_POWER:
                /* 缺药，红色指示灯闪烁 */
                if (low_tim <= now_tick)
                {
                    low_tim = now_tick + 1000;

                    if (0 == red_led_flag)
                    {
                        red_led_flag = 1;
                        drv_led_on(LED7);
                    }
                    else
                    {
                        red_led_flag = 0;
                        drv_led_off(LED7);
                    }
                }

                /* 充电过程中超过21v就退出低电压模式 */
                if ((0x01 == voltage_flag) && (OUT_LOW_POWER <= voltage_value))
                {
                    main_state = CAR_WAIT_START;
                    drv_led_off(LED7);
                }
                break;
            case CAR_GO_BACK:       /* 自转运动 */
                if (now_tick >= move_tick)
                {
                    app_robot_control_go(0.0f, 0.0f, 0.2f);
                    move_tick = now_tick + 15707;
                    main_state = CAR_END_ROTATE;
                }
                break;
            case CAR_END_ROTATE:    /* 自转运动 */
                if (now_tick >= move_tick)
                {
                    app_robot_control_move_abort();
                    move_tick = now_tick + 100;
                    main_state = CAR_SENSOR_INIT;
                }
                break;
            case CAR_GET_POSITION:    /* 自转运动 */
                if ((0x04 <= ulm_distance_num[0]) && (0x04 <= ulm_distance_num[1]))
                {
                    temp_distance = (float)(ulm_distance_sum[0] / (float)ulm_distance_num[0]);
                    temp_distance1 = (float)(ulm_distance_sum[1] / (float)ulm_distance_num[1]);

                    temp_distance3 = ABS_DIFF(temp_distance, temp_distance1);
                    if (CAR_DIS_DIFF_MAX <= temp_distance3)
                    {
                        if (temp_distance > (temp_distance1))
                        {
                            app_robot_control_go(CAR_SPEED_X, 0.0f, 0.0f);
                        }
                        else
                        {
                            app_robot_control_go((0 - CAR_SPEED_X), 0.0f, 0.0f);
                        }
                        move_tick = now_tick + (temp_distance3 / CAR_SPEED_X) * 600.0f;
                        main_state = CAR_CHANGE_POSITION;
                    }
                }
                break;
            case CAR_CHANGE_POSITION:
                if (now_tick >= move_tick)
                {
                    ulm_distance_para_init();
                    app_robot_control_move_abort();
                    main_state = CAR_GO_BACK;
                }
                break;
            default:
                main_state = CAR_WAIT_START;
                break;
        }


        switch (motor_cmd_status)
        {
            case MOTOR_CMD_IDLE:
                cmd_err_cnt = 0;
                cmd_tick = now_tick + 200;
                break;
            case MOTOR_CMD_WAIT_RES:
                if (now_tick >= cmd_tick)
                {
                    ++cmd_err_cnt;
                    if (3 == cmd_err_cnt)
                    {
                        cmd_err_cnt = 0;
                        cmd_tick = now_tick + 200;
                        app_robot_control_move_abort();
                        motor_cmd_status = MOTOR_CMD_WAIT_STOP;
                    }
                    else
                    {
                        cmd_tick = now_tick + 200;
                        app_robot_control_go_again();
                    }
                }
                break;
            case MOTOR_CMD_WAIT_STOP:
                if (now_tick >= cmd_tick)
                {
                    ++cmd_err_cnt;
                    if (3 == cmd_err_cnt)
                    {
                        cmd_err_cnt = 0;
                        main_state = CAR_MOVE_STOP;
                        stop_flag = 1;
                        motor_cmd_status = MOTOR_CMD_ERROR;
                    }
                    else
                    {
                        cmd_tick = now_tick + 200;
                        app_robot_control_go_again();
                    }
                }
                break;
            case MOTOR_CMD_ERROR:
                break;
            default:
                cmd_err_cnt = 0;
                motor_cmd_status = MOTOR_CMD_IDLE;
                break;
        }
    }
}


