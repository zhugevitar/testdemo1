#include "drv_uart.h"
#include "string.h"

void rs485_rd_status(enum GPIO_STATUS_EM status);

uint8_t usart2_data[255] = {0}; /* 存储串口接收的数据 */
uint8_t usart2_idx = 0;         /* 当前写入的下标 */
uint8_t usart2_flag = 0;        /* 是否接收到一帧数据标志位 */


uint8_t usart3_data[255] = {0}; /* 存储串口接收的数据 */
uint8_t usart3_idx = 0;         /* 当前写入的下标 */
uint8_t usart3_flag = 0;        /* 是否接收到一帧数据标志位 */


uint8_t usart6_data[255] = {0}; /* 存储串口接收的数据 */
uint8_t usart6_idx = 0;         /* 当前写入的下标 */
uint8_t usart6_flag = 0;        /* 是否接收到一帧数据标志位 */

uint8_t usart4_data[255] = {0}; /* 存储串口接收的数据 */
uint8_t usart4_idx = 0;         /* 当前写入的下标 */
uint8_t usart4_flag = 0;        /* 是否接收到一帧数据标志位 */
//============================================================================
// 名称： drv_uart_init
// 功能： 初始化串口
// 参数： Baudrate：波特率
// 返回： 无
// 说明： 无
//============================================================================
void drv_uart_2_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能RXD和TXD端口外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    // 使能串口外设时钟
    RCC_APB1PeriphClockCmd(SCOM2_CLK, ENABLE);

    // 连接口线到串口发送脚
    GPIO_PinAFConfig(SCOM2_GPIO_PORT, SCOM2_TX_SOURCE, SCOM2_TX_AF);
    // 连接口线到串口接收脚
    GPIO_PinAFConfig(SCOM2_GPIO_PORT, SCOM2_RX_SOURCE, SCOM2_RX_AF);

    // 配置串口发送脚为复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM2_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCOM2_GPIO_PORT, &GPIO_InitStructure);
    // 配置串口接收脚为复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM2_RX_PIN;
    GPIO_Init(SCOM2_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SCOM2_BASE, &USART_InitStructure);

    // 配置RD管脚
    GPIO_InitStructure.GPIO_Pin = RS485_DIR_PIN_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    //GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(RS485_DIR_PORT_2, &GPIO_InitStructure);

    // 配置串口中断
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 使能串口接收中断
    USART_ITConfig(SCOM2_BASE, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SCOM2_BASE, USART_IT_IDLE, ENABLE);

    // 使能串口
    USART_Cmd(SCOM2_BASE, ENABLE);

    /* 拉低rd管脚使485处于接受状态 */
    rs485_rd_status(GPIO_STATUS_OFF);
}


void rs485_rd_status(enum GPIO_STATUS_EM status)
{
    if (GPIO_STATUS_OFF == status)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_2);
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
    }
}


/*
*********************************************************************************************************
*                                         usart2_send_buff
*
* Description:  使用usart2进行数据发送
*
* Arguments  :  uint8_t *buff               指向需要发送的数据的首地址
*               uint8_t buff_len            需要发送数据的长度
*
* Returns    :  uint8_t               成功发送数据的长度

*********************************************************************************************************
*/
uint8_t usart2_send_buff(uint8_t *buff, uint8_t buff_len)
{
    uint8_t i = 0;
    // uint16_t count = 0;
    uint8_t *temp_buff = buff;

    /* 参数合法性判断 */
    if ((buff == NULL) || (0 == buff_len))
    {
        return ERROR;
    }

    /* 拉高RD脚，使得芯片处理发送状态 */
    rs485_rd_status(GPIO_STATUS_ON);

    buff_len += 1;


    // taskENTER_CRITICAL();
    for(i = 0; i < buff_len; ++i)
    {
        USART_SendData(USART2, *(temp_buff + i)); /* 将数据写入到串口的寄存器中 */
        while (RESET == USART_GetFlagStatus(USART2, USART_FLAG_TXE)) /* 等待数据发送完毕 */
        {
        }
    }

    /* 拉低RD脚，使得芯片处理接受状态 */
    rs485_rd_status(GPIO_STATUS_OFF);

    return i;
}



void drv_uart_3_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能串口外设时钟
    // 使能串口外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(SCOM3_CLK, ENABLE);

    // 连接口线到串口发送脚
    GPIO_PinAFConfig(SCOM3_GPIO_PORT, SCOM3_TX_SOURCE, SCOM3_TX_AF);
    // 连接口线到串口接收脚
    GPIO_PinAFConfig(SCOM3_GPIO_PORT, SCOM3_RX_SOURCE, SCOM3_RX_AF);

    // 配置串口发送脚为复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM3_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCOM3_GPIO_PORT, &GPIO_InitStructure);
    // 配置串口接收脚为复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM3_RX_PIN;
    GPIO_Init(SCOM3_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SCOM3_BASE, &USART_InitStructure);

    // 配置串口中断
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 使能串口接收中断
    USART_ITConfig(SCOM3_BASE, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SCOM3_BASE, USART_IT_IDLE, ENABLE);

    // 使能串口
    USART_Cmd(SCOM3_BASE, ENABLE);
}

void drv_uart_4_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能串口外设时钟
    // 使能串口外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(SCOM4_CLK, ENABLE);

    // 连接口线到串口发送脚
    GPIO_PinAFConfig(SCOM4_GPIO_PORT, SCOM4_TX_SOURCE, SCOM4_TX_AF);
    // 连接口线到串口接收脚
    GPIO_PinAFConfig(SCOM4_GPIO_PORT, SCOM4_RX_SOURCE, SCOM4_RX_AF);

    // 配置串口发送脚为复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM4_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCOM4_GPIO_PORT, &GPIO_InitStructure);
    // 配置串口接收脚为复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM4_RX_PIN;
    GPIO_Init(SCOM4_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SCOM4_BASE, &USART_InitStructure);

    // 配置串口中断
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 使能串口接收中断
    USART_ITConfig(SCOM4_BASE, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SCOM4_BASE, USART_IT_IDLE, ENABLE);

    // 使能串口
    USART_Cmd(SCOM4_BASE, ENABLE);
}


void drv_uart_6_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能串口外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(SCOM6_CLK, ENABLE);

    // 连接口线到串口发送脚
    GPIO_PinAFConfig(SCOM6_GPIO_PORT, SCOM6_TX_SOURCE, SCOM6_TX_AF);
    // 连接口线到串口接收脚
    GPIO_PinAFConfig(SCOM6_GPIO_PORT, SCOM6_RX_SOURCE, SCOM6_RX_AF);

    // 配置串口发送脚为复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM6_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCOM6_GPIO_PORT, &GPIO_InitStructure);
    // 配置串口接收脚为复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SCOM6_RX_PIN;
    GPIO_Init(SCOM6_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SCOM6_BASE, &USART_InitStructure);

    // 配置串口中断
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 使能串口接收中断
    USART_ITConfig(SCOM6_BASE, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SCOM6_BASE, USART_IT_IDLE, ENABLE);

    // 使能串口
    USART_Cmd(SCOM6_BASE, ENABLE);
}



uint8_t usart3_send_buff(uint8_t *buff, uint8_t buff_len)
{
    uint8_t i = 0;
    // uint16_t count = 0;
    uint8_t *temp_buff = buff;

    /* 参数合法性判断 */
    if ((buff == NULL) || (0 == buff_len))
    {
        return ERROR;
    }


    // taskENTER_CRITICAL();
    for(i = 0; i < buff_len; ++i)
    {
        USART_SendData(USART3, *(temp_buff + i)); /* 将数据写入到串口的寄存器中 */
        while (RESET == USART_GetFlagStatus(USART3, USART_FLAG_TXE)) /* 等待数据发送完毕 */
        {
        }
    }

    return i;
}


uint8_t usart4_send_buff(uint8_t *buff, uint8_t buff_len)
{
    uint8_t i = 0;
    uint8_t *temp_buff = buff;

    /* 参数合法性判断 */
    if ((buff == NULL) || (0 == buff_len))
    {
        return ERROR;
    }


    // taskENTER_CRITICAL();
    for(i = 0; i < buff_len; ++i)
    {
        USART_SendData(UART4, *(temp_buff + i)); /* 将数据写入到串口的寄存器中 */
        while (RESET == USART_GetFlagStatus(UART4, USART_FLAG_TXE)) /* 等待数据发送完毕 */
        {
        }
    }

    return i;
}

uint8_t usart6_send_buff(uint8_t *buff, uint8_t buff_len)
{
    uint8_t i = 0;
    // uint16_t count = 0;
    uint8_t *temp_buff = buff;

    /* 参数合法性判断 */
    if ((buff == NULL) || (0 == buff_len))
    {
        return ERROR;
    }


    // taskENTER_CRITICAL();
    for(i = 0; i < buff_len; ++i)
    {
        USART_SendData(USART6, *(temp_buff + i)); /* 将数据写入到串口的寄存器中 */
        while (RESET == USART_GetFlagStatus(USART6, USART_FLAG_TXE)) /* 等待数据发送完毕 */
        {
        }
    }

    return i;
}




