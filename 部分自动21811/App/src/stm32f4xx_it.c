/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_dma.h"

#include "drv_timer.h"
#include "drv_uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t adcvalue[4] = {0};
uint16_t voltage_value = 0x00;
uint8_t voltage_flag = 0x00;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   
}

/**
  * @brief  This function handles Hard Fault exception.

  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{

  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    systick_increase();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/**************************************
*函数名称:USART2_IRQHandler
*函数功能:USART2中断的调用函数
*函数输入:无
*函数输出:无
**************************************/
void USART2_IRQHandler(void)
{
    uint16_t k              = 0;

    // USART_ClearFlag(USART2,USART_FLAG_TC);
    if ((RESET != USART_GetITStatus(USART2, USART_IT_RXNE)) || (RESET != USART_GetITStatus(USART2, USART_IT_ORE)))
    {
        k = USART_ReceiveData(USART2);

        /* 只有当前帧数据处理完了之后才能够接收下一帧数据 */
        if (0 == usart2_flag)
        {
            usart2_data[usart2_idx] = k;
            ++usart2_idx;
            usart2_flag = (255 <= usart2_idx)?1:0;;
        }


        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
    }

    if (RESET != USART_GetITStatus(USART2, USART_IT_IDLE))
    {

        k = USART_ReceiveData(USART2);
        usart2_flag = 1;

        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
    }
}


/**************************************
*函数名称:USART3_IRQHandler
*函数功能:USART3中断的调用函数
*函数输入:无
*函数输出:无
**************************************/
void USART3_IRQHandler(void)
{
    uint16_t k              = 0;

    // USART_ClearFlag(USART2,USART_FLAG_TC);
    if ((RESET != USART_GetITStatus(USART3, USART_IT_RXNE)) || (RESET != USART_GetITStatus(USART3, USART_IT_ORE)))
    {
        k = USART_ReceiveData(USART3);

        /* 只有当前帧数据处理完了之后才能够接收下一帧数据 */
        if (0 == usart3_flag)
        {
            usart3_data[usart3_idx] = k;
            ++usart3_idx;
            usart3_flag = (255 <= usart3_idx)?1:0;;
        }


        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
    }

    if (RESET != USART_GetITStatus(USART3, USART_IT_IDLE))
    {

        k = USART_ReceiveData(USART3);
        usart3_flag = 1;

        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
    }
}

/**************************************
*函数名称:UART4_IRQHandler
*函数功能:UART4中断的调用函数
*函数输入:无
*函数输出:无
**************************************/
void UART4_IRQHandler(void)
{
    uint16_t k              = 0;
	 

    // USART_ClearFlag(USART2,USART_FLAG_TC);
    if ((RESET != USART_GetITStatus(UART4, USART_IT_RXNE)) || (RESET != USART_GetITStatus(UART4, USART_IT_ORE)))
    {
        k = USART_ReceiveData(UART4);

        /* 只有当前帧数据处理完了之后才能够接收下一帧数据 */
        if (0 == usart4_flag)
        {
            usart4_data[usart4_idx] = k;
            ++usart4_idx;
            usart4_flag = (255 <= usart4_idx)?1:0;;
        }


        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);
    }

    if (RESET != USART_GetITStatus(UART4, USART_IT_IDLE))
    {

        k = USART_ReceiveData(UART4);
        usart4_flag = 1;

        USART_ClearITPendingBit(UART4, USART_IT_IDLE);
    }
}

/**************************************
*函数名称:USART2_IRQHandler
*函数功能:USART2中断的调用函数
*函数输入:无
*函数输出:无
**************************************/
void USART6_IRQHandler(void)
{
    uint16_t k              = 0;

    // USART_ClearFlag(USART2,USART_FLAG_TC);
    if ((RESET != USART_GetITStatus(USART6, USART_IT_RXNE)) || (RESET != USART_GetITStatus(USART6, USART_IT_ORE)))
    {
        k = USART_ReceiveData(USART6);

        /* 只有当前帧数据处理完了之后才能够接收下一帧数据 */
        if (0 == usart6_flag)
        {
            usart6_data[usart6_idx] = k;
            ++usart6_idx;
            usart6_flag = (255 <= usart6_idx)?1:0;;
        }


        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
    }

    if (RESET != USART_GetITStatus(USART6, USART_IT_IDLE))
    {

        k = USART_ReceiveData(USART6);
        usart6_flag = 1;

        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
    }
}




void DMA2_Stream0_IRQHandler(void) 
{
    if (DMA_GetFlagStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET)  
    {
        voltage_value = adcvalue[0] + adcvalue[1] + adcvalue[2] + adcvalue[3];
        voltage_value = voltage_value >> 2;
        voltage_flag = 0x01;

        DMA_ClearFlag(DMA2_Stream0, DMA_IT_TCIF0); 
    }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
