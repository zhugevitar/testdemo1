#include "drv_can.h"

// 波特率参数表，所有参数需要加1
// 第1项：SJW，同步跳转宽度，取值范围0～3
// 第2项：BS1，传输段及相位缓冲段1的和，取值范围：0～15
// 第3项：BS2，相位缓冲段2，取值范围：0～7
// 第4项：Prescaler，分频值，取值范围0～1023
//                          Fpclk
// BaudRate = -----------------------------------
//              (3 + BS1 + BS2) * (1 + Prescaler)
//
// 尽量满足推荐条件：
//                      2 + BS1
//  Baudrate        ---------------
//                  (3 + BS1 + BS2)
//
//  >800K,              75%
//  >500K,              80%
//  <=500K,             87.5%
//

//============================================================================
// 名称：drv_can_init
// 功能：CAN口初始化
// 参数：无
// 返回：无
// 说明：配置为机器人平台内部CAN总线默认波特率500Kbps
//============================================================================
void drv_can_init( void )
{
    NVIC_InitTypeDef        NVIC_InitStructure;
    GPIO_InitTypeDef        GPIO_InitStructure;
    CAN_InitTypeDef         CAN_InitStructure;
    CAN_FilterInitTypeDef   CAN_FilterInitStructure;

    // 配置CAN中断
    NVIC_InitStructure.NVIC_IRQChannel = SCAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能端口时钟
    RCC_AHB1PeriphClockCmd(SCAN_GPIO_CLK, ENABLE);

    // 连接引脚到复用功能
    GPIO_PinAFConfig(SCAN_GPIO_PORT, SCAN_RX_SOURCE, SCAN_AF_PORT);
    GPIO_PinAFConfig(SCAN_GPIO_PORT, SCAN_TX_SOURCE, SCAN_AF_PORT);

    // 配置收发引脚
    GPIO_InitStructure.GPIO_Pin = SCAN_RX_PIN | SCAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(SCAN_GPIO_PORT, &GPIO_InitStructure);

    // 使能CAN模块时钟
    RCC_APB1PeriphClockCmd(SCAN_CLK, ENABLE);

    // CAN模块寄存器恢复默认值
    CAN_DeInit(SCAN_BASE);

    // 初始化CAN单元
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 5;
    CAN_Init(SCAN_BASE, &CAN_InitStructure);

    // 验收滤波设置，主机接收所有ID的报文
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // 使能FIFO0消息中断，接收本机地址和广播地址
    CAN_ITConfig(SCAN_BASE, CAN_IT_FMP0, ENABLE);
}

//============================================================================
// 名称：drv_scan_irq
// 功能：CAN中断服务程序.
// 参数：无
// 返回：无
// 说明：无
//============================================================================
extern void app_protocol_can_msg_process( CanRxMsg RxMsg );
void drv_scan_irq( void )
{
    CanRxMsg l_CanRxMsg;

    // 读出FIFO中所有的数据
    if ( CAN_MessagePending(SCAN_BASE, CAN_FIFO0) )
    {
        CAN_Receive(SCAN_BASE, CAN_FIFO0, &l_CanRxMsg);
        app_protocol_can_msg_process(l_CanRxMsg);
    }
}

//============================================================================
// 名称：drv_can_send_msg
// 功能：通过CAN口发送报文
// 参数：TxMessage：要发送的报文
// 返回：无
// 说明：发送邮箱
//============================================================================
INT8U drv_can_send_msg( CanTxMsg TxMessage )
{
    return ( CAN_Transmit(SCAN_BASE, &TxMessage) );
}



