#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_it.h"

#include "iwdg.h"

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

#define MOVE_DESLAY_TIME                (100)

#define CAR_WIDTH_MM                    (420.0f)
#define DISTANCE_RATIO                  (0.6f)


#define WAIT_LEAVE_TIME                 (3000)     /* 等待操作员离开延时 */

#define ENTER_LOW_POWER                 (1861)  /* 1.5 / 3.3 * 4096 */
#define OUT_LOW_POWER                   (2606)  /*  */


uint8_t ulm_addr_arr[ULM_NUM]           = {0xA0, 0xA2, 0xA4, 0xA6};  /*各模块地址*/
uint16_t ulm_distance_arr[ULM_NUM]      = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t ulm_old_distance_arr[ULM_NUM]  = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint8_t get_distance_cmd[3]             = {0xA2, 0x02, 0xBC};   /*正前方超声波模块感应距离命令*/
uint8_t upload_data[20]                 = {0};  //{0x7e,0x0a,0x01,0x00,0x00,0x01,0x02,0x00,0x08,0x7e,0x00};
uint8_t upload_num                      = 0;

float ulm0_old_distance_data            = 0;
float ulm1_old_distance_data            = 0;
float ulm2_distance_data            = 0;
float ulm3_distance_data            = 0;

float    ulm_distance_sum[4]               = {0.0f};  /* 一个周期内距离之和 */
uint8_t ulm_distance_num[4]             = {0};   /* 一个周期内测得距离的次数 */

uint8_t usart2_overtime                 = 0;
uint8_t usart3_overtime                 = 0;
uint8_t error_front = 0;        //传感器错误次数
uint8_t back_flag = 0;
uint32_t now_tick        =  0;    /* 当前systick值 */
uint32_t old_tick        =  0;    /* 上一个systick值 */	
uint8_t  tick_1ms        =  0;	
uint32_t	tick_comm      =  0;	  //上传函数的计时器
 uint32_t	tick_request     =  0;	  //数据请求计时器

	uint32_t T=0;           //计时30秒
	uint8_t i=0;
   BOOLEAN state_zidong;

   uint16_t temp_distance              = 0; //感应距离

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
		CAR_WAIT_START=0,             /* 等待操作员按下开始按钮状态 */
		CAR_WAIT_OPERATOR_LEAVE,    /* 通过延时等待操作员离开状态 */
		CAR_START_ROTATE,           /* 机器人开始运动前旋转360度 */
		CAR_GO,                     /* 机器人开始行走状态 */
			CAR_TURN_R,                 /* 机器人向右旋转 */
			CAR_TURN_L,                 /* 机器人向左旋转 */    	
		CAR_MOVING,                 /* 机器人行走过程中 */
		CAR_ADJUSTING,              /* 通过自转调整机器人方向 */
		  CAR_MOVE_BACK,              /* 机器人运动停止状态 */
		CAR_MOVE_STOP,              /* 机器人运动停止状态 */
		CAR_LACK_WATER,             /* 水箱缺药 */
		CAR_LOW_POWER,              /* 电压过低 */
		CAR_ADJUST_SENSOR,          /* 长时间有传感器处于异常状态时进入该状态 */
		CAR_VOID,                   /* 蔽障 */
		CAR_VOID2,                   /* 蔽障 */	
		  CAR_VOID3,                   /* 蔽障 */
		  CAR_END_ROTATE,             /* 到达终点之后的自转运动 */
			CAR_GO_FRONT,
			CAR_GO_BACK,
			CAR_GO_LEFT,
			CAR_GO_RIGHT,
			CAR_GO_TURNLEFT,
			CAR_GO_TURNRIGHT,
			CAR_GO_STOP,
			CAR_SPRAY_ON,
			CAR_SPRAY_OFF,
			CAR_GO_AUTO,
			CAR_GO_MAN,
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
    drv_uart_2_init();//超声波传感器A2
    drv_uart_3_init();
    drv_uart_4_init();	
    drv_uart_6_init();	
    KEY_Init();          //急停按钮	
    drv_timer_init();
	 IWDG_Init(6,625);//看门狗初始化,4秒无连接自动复位
}

//============================================================================
// 名称： void ulm_refresh_fun_2(void)
// 功能： 获取超声波感应距离
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================
void ulm_refresh_fun_2(void)    //获取前面超声距离
{
    static uint8_t ulm_idx                        = 0;
    static uint8_t get_dis_idx                 = 0;
    uint16_t temp_distance                    = 0;
    uint32_t now_tick                              = 0;
    static uint32_t ulm_tick                    = 0;
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
            usart3_send_buff(get_distance_cmd, 3); //串口3请求传感器返回数值
            ulm_tick = now_tick + 40;
            ulm_status = ULM_WAIT_RES;
            break;
        case ULM_WAIT_RES:
					
			 if(ulm_tick > now_tick)	
				{              
                if (1 == usart3_flag)
                {
                    temp_distance = (usart3_data[3] << 8) + usart2_data[2]; //获取返回数据值
                    if ( (0x02 == usart3_data[1]) && (0x04 <= usart3_idx) && (0xFFFF != temp_distance) )//,,数据不为空
                    {   
						error_front = 0;  //错误次数0                    
                        switch (usart3_data[0])
                        {
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
										ulm_status = ULM_DELAY;
                    ulm_tick = now_tick + 10;
                }
							}
						else
							{     usart3_overtime = 1;
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


 /*//获取两侧两个超声距离*/
void ulm_refresh_fun_1(void)    
{
    static uint8_t ulm_idx                  = 0;//
    static uint8_t get_dis_idx           = 0;//
   // uint16_t temp_distance              = 0;//
    uint32_t now_tick                        = 0;//当前时间
    static uint32_t ulm_tick               = 0;//  
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
            usart2_send_buff(get_distance_cmd, 3);//串口2
            ulm_tick = now_tick + 40;
            ulm_status = ULM_WAIT_RES;
            break;
        case ULM_WAIT_RES:
				    if(ulm_tick > now_tick)	
							{              
								if (1 == usart2_flag)
                {
                    temp_distance = (usart2_data[3] << 8) + usart2_data[2];  //得到探测距离
                    if ((0x02 == usart2_data[1]) && (0x04 <= usart2_idx) )       //&& (0xFFFF != temp_distance)
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
                                default:
                                break;
                        }
                    }
                    usart2_flag = 0;
                    usart2_idx = 0;
					ulm_status = ULM_DELAY;
                    ulm_tick = now_tick + 10;
                }
							}
						else
							{     usart2_overtime = 1;
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




//============================================================================
// 名称：// 配置ADC
// 功能： 
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================

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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC通用配置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//8 改为4
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC通道10配置
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//
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


//============================================================================
// 名称： avoiding(void)
// 功能： 距离解析函数
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================

	  float  ulm2_distance_sumdata                   = 0;  //全局变量,存储三次的距离之和
	  float  ulm2_distance_average                    = 0;  //三次平均值
				uint8_t  distance_sumdata_num      = 0;  //获得距离的次数

  void  avoiding (void )   //
    {			 
	      temp_distance = (usart2_data[3] << 8) + usart2_data[2];  //得到单次探测距离			 
		  ulm2_distance_sumdata   +=  temp_distance;//距离之和
		  distance_sumdata_num ++;//获取距离的次数
		 
		if(  distance_sumdata_num  == 3 ) //三次后求平均值
			{
			   ulm2_distance_average  	=  ulm2_distance_sumdata   / 3 ;//平均值
             if(ulm2_distance_average   <  250 )   drv_led_off(LED4) ; //平均值小于25CM关闭led4
				
				distance_sumdata_num      = 0; 
				ulm2_distance_average      = 0;
				ulm2_distance_sumdata     = 0;
				
			}        		
	      usart2_flag = 0;
          usart2_idx = 0;
     }

//============================================================================
// 名称： data_request
// 功能： 距离数据请求函数
// 参数： 无
// 返回： 无
// 说明： 无
//============================================================================
 void Data_request (  void )  
 {       
              now_tick = sys_now(); //1ms
			  if(old_tick != now_tick)  tick_1ms = 1; 
				  else     tick_1ms = 0; 
			  old_tick = now_tick;
	          if(tick_1ms > 0) tick_request ++;
			  if(tick_request > 100)		//100ms请求一次数据	 
			  	{ 
					
				 tick_request = 0;
				 usart2_send_buff (get_distance_cmd, 3);		//正前方超声波传感器数据请求指令	
			     avoiding( );    
					
				}
 }
	
  /***  数据上传函数  ***/

void Upload_Data(void)
{
              now_tick = sys_now(); //1ms
			   if(old_tick != now_tick)  tick_1ms = 1; 
				 else     tick_1ms = 0; 
				 old_tick = now_tick;
	           if(tick_1ms > 0)tick_comm ++;
				 if(tick_comm > 1000)			//时间1000改为500 
			  	{ 
	                   T++;  //计时，用于推杆通电时间
					    tick_comm = 0;
						upload_data[2]                 = 0x01;	//机器人类别
						upload_data[3]                 = 0x08;	//机器人编号
 
		          upload_data[6]                 = (uint8_t)(voltage_value >> 8);	
				  	if(upload_data[6] == 0x7e)   upload_data[6] = 0x7d;
		          upload_data[7]                 = (uint8_t)voltage_value;		
				  	if(upload_data[7] == 0x7e)   upload_data[7] = 0x7d;		
					 upload_data[8]  = 0;
					 for(i=1; i<8; i++)
				  	{
					  	upload_data[8] = upload_data[8] ^ upload_data[i];
					  }

					  usart4_send_buff(upload_data,upload_num);//串口4上传当前机器状态

				  }

}
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
	    uint32_t lack_delay_tim  = 0;      //缺药持续时间 
        uint8_t led1_flag = 0;  
        uint8_t u8_ret = 0;   
		uint8_t UP=0;        //推杆上升标志位
		uint8_t DOWN=0;
		uint8_t BENG=0;      //泵开启标志位
		uint8_t TUIDAODI=0;  //推杆到底部标志位
  	    //uint8_t  JITING=0;   //急停标志位
		BOOLEAN state_man; 
		BOOLEAN state_move;
        //BOOLEAN state_zidong;
       enum CAR_STATE_EN main_state = CAR_WAIT_START;
 


    // 初始化
    app_main_system_init();
	
	
    //采集电压
    adc_config();
    // 强制使底盘停止动作
    app_robot_control_move_abort();

    // 启动ADC转换
    ADC_SoftwareStartConv(ADC1);
    main_state = CAR_WAIT_START;
		upload_data[0]                 = 0x7e;
		upload_data[1]                 = 0x0a;	
		upload_data[5]                 = 0x00; //故障		
		upload_data[9]                 = 0x7e;		
		upload_num                      = 10;
 		usart4_send_buff(upload_data,upload_num);
    while ( 1 )   
    {       
		
		     
				 
			/*** 紧急停止按钮 ****/
			   if( KEY0 == 1 ) 
				{
					if (1 == usart4_flag)
				{ 
			     usart4_flag=0;
					 usart4_idx=0;
					if(usart4_data[0] == 0x00 && usart4_data[1] == 0xFF && usart4_data[2] == 0x00) //收到心跳包
		       {
			        IWDG_Feed( );   //喂狗
						  
		        }
					}
				  drv_led_off(LED9);        //关泵
				  drv_led_off(LED10);     //关泵
					drv_led_off(LED4);     //推杆上升电源关闭
					drv_led_off(LED5);     //推杆上升电源关闭
					drv_led_off(LED6);     //关闭推杆下降电源
				  drv_led_off(LED7);       //关闭推杆下降电源
					main_state = CAR_SPRAY_OFF; 
					upload_data[4]=upload_data[4]&0x0B;//上位机无作业显示	
					
	
					/**小车停止**/
					main_state = CAR_GO_STOP;
					state_move=0;
					app_robot_control_move_abort(); 
					upload_data[4]= 0x00 ;
						
						upload_data[2]                 = 0x01;	//机器人类别
						upload_data[3]                 = 0x08;	//机器人编号
 
		       upload_data[6]                 = (uint8_t)(voltage_value >> 8);	
				  	if(upload_data[6] == 0x7e)   upload_data[6] = 0x7d;
		       upload_data[7]                 = (uint8_t)voltage_value;		
				  	if(upload_data[7] == 0x7e)   upload_data[7] = 0x7d;		
					 upload_data[8]  = 0;
					 for(i=1; i<8; i++)
				  	{
					  	upload_data[8] = upload_data[8] ^ upload_data[i];
					  }

		
					  usart4_send_buff(upload_data,upload_num);					
					 drv_delay_ms(500);
					
				} 
       //**  正常工作		**//		
				else{
					
			    Upload_Data( );	//上传信息函数
                Data_request ( ) ;//A2模块距离请求函数
			 					
			 
			   /* 开始消毒作业 */
				if(T==16 && UP==1 && DOWN==0)//判断推杆升起时间，16秒
				{
								UP=0;
						drv_led_off(LED4);//推杆上升电源关闭
			      		drv_led_off(LED5);//推杆上升电源关闭    
					                                                             drv_led_on(LED9);//开喷泵
							  drv_led_on(LED10);//开喷泵
							  BENG=1;           //泵启动标志位
							  main_state = CAR_SPRAY_ON;
					      upload_data[4]=upload_data[4]|0x04; //上位机消毒作业显示
					
				 }
				
				if(T==16 && DOWN==1)  //下降16秒
				{
				  drv_led_off(LED6); //关闭推杆下降电源
				  drv_led_off(LED7); 
				  DOWN=0;	
				  BENG=0;                //泵开启标志位
				  TUIDAODI=1;        //推杆到底标志位1
					
				}
				
				
				if (1 == usart4_flag)
				{ 
			          usart4_flag=0;
					 usart4_idx=0;
					
					/***看门狗检测***/
					 if(usart4_data[0] == 0x00 && usart4_data[1] == 0xFF && usart4_data[2] == 0x00)//收到心跳包
		       {
			        IWDG_Feed( );   //喂狗						  
		        }
				
						/***串口命令检测***/
						if ( usart4_data[0]==0x6e && usart4_data[4]==0x6e && usart4_data[1]==0x05 && KEY0 == 0  )//急停按钮松开才能工作
						{  
							 //JITING=0;//复位急停标志位
							
							 /*按键1，遥控*/
							if (usart4_data[2]==0x01 && usart4_data[3]==0x06)
							{
								main_state = CAR_GO_MAN;								
								state_man=1;
								state_move=0;
                       
								usart4_flag=0;
								usart4_idx=0;
							}
							/***自动前进部分***/
							if (usart4_data[2]==0x02 && usart4_data[3]==0x07)
							{
								 /*按键2，自动*/
								main_state = CAR_GO_AUTO;							
								state_man=0;
								drv_led_on(LED4);
								
									if (state_move==0)
									{
										state_zidong = 1;
										state_move=1;
										app_robot_control_go(0.0f,200.0f, 0.0f); 
										upload_data[4]=upload_data[4]|0x01;	
										//avoiding( );
									}
								
                               
								
								usart4_flag=0;
								usart4_idx=0;
							}
							if (usart4_data[2]==0x03 && usart4_data[3]==0x08 )
							{
								/*按键W，前进*/
								main_state = CAR_GO_FRONT;								
								if (state_man==1)
								{
									if (state_move==0)
									{
										state_move=1;
										app_robot_control_go(0.0f,200.0f, 0.0f); 
										upload_data[4]=upload_data[4]|0x01;	
									}
								}

								usart4_flag=0;
								usart4_idx=0;
							}
							/*按键S，后退*/
							if (usart4_data[2]==0x04 && usart4_data[3]==0x09 )
							{
								main_state = CAR_GO_BACK;								
								if (state_man==1)
								{if (state_move==0)
									{state_move=1;
										app_robot_control_go(0.0f, -200.0f, 0.0f); 
										upload_data[4]=upload_data[4]|0x01;	
									}
								}
							
								usart4_flag=0;
								usart4_idx=0;
							}
								/*按键Q，左移*/
							if (usart4_data[2]==0x05 && usart4_data[3]==0x0A )
							{
								main_state = CAR_GO_LEFT;								
								if (state_man==1)
								{if (state_move==0)
									{state_move=1;
										app_robot_control_go(-150.0f, 0.0f, 0.0f); 
										upload_data[4]=upload_data[4]|0x01;
									}
								}
							
								usart4_flag=0;
								usart4_idx=0;
							}
							/*按键E，右移*/
							if (usart4_data[2]==0x06 && usart4_data[3]==0x0B)
							{
								main_state = CAR_GO_RIGHT;
								if (state_man==1)
								{if (state_move==0)
									{state_move=1;
										app_robot_control_go(150.0f, 0.0f, 0.0f); 
										upload_data[4]=upload_data[4]|0x01;
									}
								}

								usart4_flag=0;
								usart4_idx=0;
							}
							/*按键A，左转*/
							if (usart4_data[2]==0x07 && usart4_data[3]==0x0C )
							{
								main_state = CAR_GO_TURNLEFT;								
								if (state_man==1)
								{if (state_move==0)
									{state_move=1;
										app_robot_control_go(0.0f, 0.0f, 0.4f);
										upload_data[4]=upload_data[4]|0x01;
									}
								}

								usart4_flag=0;
								usart4_idx=0;
							}
							/*按键D，右转*/
							if (usart4_data[2]==0x08 && usart4_data[3]==0x0D )
							{
								main_state = CAR_GO_TURNRIGHT;								
								if (state_man==1)
								{if (state_move==0)
									{state_move=1;
										app_robot_control_go(0.0f, 0.0f, -0.4f);
										upload_data[4]=upload_data[4]|0x01;
									}
								}

								usart4_flag=0;
								usart4_idx=0;
							}
								/*停止*/
							if (usart4_data[2]==0x09 && usart4_data[3]==0x0E)
							{
								main_state = CAR_GO_STOP;								
								state_move=0;
								app_robot_control_move_abort(); //停止
								upload_data[4]=upload_data[4]^0x01;
								usart4_flag=0;
								usart4_idx=0;
								drv_led_off(LED4);
								drv_led_off(LED5);
								drv_led_off(LED6);
								drv_led_off(LED7);
								drv_led_off(LED9);
								drv_led_off(LED10);
								
							}
								/*  按键3，推杆升起*/
							if (usart4_data[2]==0x0A && usart4_data[3]==0x0F && DOWN==0 && BENG==0 )
							{	
								if (state_man==1)
								{T=0;
								 UP=1;
								 TUIDAODI=0;             //推杆正在升起标志位值0
								 drv_led_on(LED4);   //推杆升起，高电平	
								 drv_led_on(LED5);   //推杆升起，高电平	
									
								 drv_led_off(LED6);  //推杆下降电源关闭	
								 drv_led_off(LED7);  //推杆下降电源关闭	
									
								 main_state = CAR_SPRAY_ON;
								 usart4_flag=0;
								 usart4_idx=0;
								}
							 }
								
								/* 按键4，关闭喷泵 */
							if (usart4_data[2]==0x0B && usart4_data[3]==0x10 && TUIDAODI==0  )//喷雾器关
							{ 
               if (state_man==1)								
							 {T=0;
								DOWN=1;
								UP=0;								
								drv_led_off(LED9);//关泵
								drv_led_off(LED10);//关泵
								 
								drv_led_on(LED6); //推杆下降电源开启
								drv_led_on(LED7); //推杆下降电源开启
								 
								drv_led_off(LED4);//推杆上升电源关闭
								drv_led_off(LED5);//推杆上升电源关闭
								 
								main_state = CAR_SPRAY_OFF; 
								upload_data[4]=upload_data[4]&0x0B;//上位机无作业显示								 																	
								usart4_flag=0;
								 
								 
								usart4_idx=0;
							 }
						 }
						} 
				}
				
	   /*****液位计*****/
				
	   u8_ret = drv_get_lc_state ();   //获取当前液位状态.1:缺药；0:有药 //GPIOD0
        
        if (0x01 == u8_ret)            //缺药2s以上认为缺药
        {
            if ((now_tick >= lack_delay_tim) && (CAR_LACK_WATER != main_state) && (CAR_LOW_POWER != main_state))
            {               
                  /** 停止喷药 **/
							
 				  drv_led_off(LED9);  //关泵	
                  drv_led_off(LED10);  //关泵		
				  upload_data[5]   |= 0x01;//故障显示缺液	
                  upload_data[4]=upload_data[4]&0x0B;//上位机无作业显示
            }
         }
        else
        {
                  lack_delay_tim = now_tick + 2000;
			      upload_data[5]    = 0x00;	  //清除故障
        }
        
					
					
      /*** pcb绿色led闪烁，表示系统运行正常 ***/ 
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
	
			}				      				
									       
     }


	}

	

