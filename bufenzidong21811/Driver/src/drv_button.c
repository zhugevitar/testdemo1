#include "drv_button.h"
#include "drv_delay.h"

GPIO_TypeDef *Button_GPIO_PORT[BUTTONn] = { Button2_GPIO_PORT, Button3_GPIO_PORT };
const INT16U    Button_GPIO_PIN[BUTTONn] = { Button2_PIN, Button3_PIN };
const INT32U    Button_GPIO_CLK[BUTTONn] = { Button2_GPIO_CLK, Button3_GPIO_CLK };
const INT8U     Button_EXTI_PortSource[BUTTONn] = { Button2_EXTI_PortSource, Button3_EXTI_PortSource };
const INT8U     Button_EXTI_PinSource[BUTTONn] = { Button2_EXTI_PinSource, Button3_EXTI_PinSource };
const INT32U    Button_EXTI_Line[BUTTONn] = { Button2_EXTI_Line, Button3_EXTI_Line };
const INT8S     Button_EXTI_IRQn[BUTTONn] = { Button2_EXTI_IRQn, Button3_EXTI_IRQn };

//============================================================================
// 名称：drv_button_line_init
// 功能：一位按键口线初始化
// 参数：Button：要初始化的按键别名
//      EnableInt：是否使用中断，1为中断，0为查询
// 返回：无
// 说明：button_init调用
//============================================================================
void drv_button_line_init( ButtonType Button, INT8U EnableInt )
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    // 使能口线端口时钟
    RCC_AHB1PeriphClockCmd(Button_GPIO_CLK[Button], ENABLE);

    // 配置口线为浮空输入模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = Button_GPIO_PIN[Button];
    GPIO_Init(Button_GPIO_PORT[Button], &GPIO_InitStructure);

    if( EnableInt > 0 )
    {
        // 连接中断信号到引脚
        SYSCFG_EXTILineConfig(Button_EXTI_PortSource[Button], Button_EXTI_PinSource[Button]);

        // 配置中断线，中断或事件模式
        EXTI_InitStructure.EXTI_Line = Button_EXTI_Line[Button];
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        // 配置中断优先级
        NVIC_InitStructure.NVIC_IRQChannel = Button_EXTI_IRQn[Button];
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

//============================================================================
// 名称：drv_button_init
// 功能：按键口线初始化
// 参数：无
// 返回：无
// 说明：系统初始化时调用
//============================================================================

void drv_button_init( void )
{
    INT8U i;

    // 使能 SYSCFG 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    for( i = 0; i < BUTTONn; i++ )
    {
        drv_button_line_init((ButtonType )i, 0);
    }
}

//============================================================================
// 名称：drv_button_get_status
// 功能：读取按键状态
// 参数：Button：要读取状态的按键
// 返回：按键按下状态，1为按下，0为释放
// 说明：按键按下时为低电平
//============================================================================
INT8U drv_button_get_status( ButtonType Button )
{
    uint8_t ret = 0;
    ret = GPIO_ReadInputDataBit(Button_GPIO_PORT[Button], Button_GPIO_PIN[Button]);
    return ret;
}

////============================================================================
//// 名称：button2_3_irq
//// 功能：按键2和3中断服务程序
//// 参数：无
//// 返回：无
//// 说明：无
////============================================================================
//void button2_3_irq(void)
//{
//    if( EXTI_GetITStatus(Button_EXTI_Line[Button1]) == SET )
//    {
//        debug_printf("button1 interrupt.\r\n");
//        // 清中断线挂起位
//        EXTI_ClearITPendingBit(Button_EXTI_Line[Button1]);
//    }
//    if( EXTI_GetITStatus(Button_EXTI_Line[Button2]) == SET )
//    {
//        debug_printf("button2 interrupt.\r\n");
//        // 清中断线挂起位
//        EXTI_ClearITPendingBit(Button_EXTI_Line[Button2]);
//    }
//}

////============================================================================
//// 名称：PE15急停按钮
//// 功能：急停按钮按下收到停止信号
//// 参数：无
//// 返回：无
//// 说明：无
////============================================================================
//void KEY_Init(void) //PE15急停按钮
//{ 
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 
// 	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//开启时钟

//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;    //急停按钮输入信号引脚
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   //输入模式
// 	GPIO_Init(GPIOE, &GPIO_InitStructure); 
//	
//}

  //**急停按钮初始化**///
void KEY_Init(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//时钟初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;   //GPIOE 07¦Òý½Å
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//模式下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	  
} 
//u8 KEY_Scan(u8 mode)
//{	 
//	static u8 key_up=1;//
//	//if(mode)key_up=1;  ////Ö§³ÖÁ¬°´		  
//	if(key_up && KEY0==1)
//	{
//		drv_delay_ms(10);//È¥¶¶¶¯ 
//		key_up=0;
//		if(KEY0==1)return KEY0_PRES;
//		else if(WK_UP==1)return WKUP_PRES;
//	}else if(KEY0==0&&WK_UP==0)key_up=1; 	    
// 	return 0;// ÎÞ°´¼ü°´ÏÂ
//}




