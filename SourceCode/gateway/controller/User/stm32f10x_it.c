/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "hard_init.h"
#include <stdio.h>

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern FlagStatus RX_status;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
      printf("\r\nERROR_NMI\r\n");
	  __disable_irq();//关总中断
	  __disable_fault_irq();
	  delay_ms(2000);
	  NVIC_SystemReset();
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		printf("\r\nERROR_HardFault\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
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
		printf("\r\nERROR_MEM\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
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
		printf("\r\nERROR_BusFault\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
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
		printf("\r\nERROR_UsageFault\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
		printf("\r\nERROR_SVC\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
		printf("\r\nERROR_DebugMonitor\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
		printf("\r\nERROR_DebugMonitor\r\n");
	    __disable_irq();//关总中断
	    __disable_fault_irq();
	    delay_ms(2000);
	    NVIC_SystemReset();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
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
void USART1_IRQHandler(void)
{
    GPIO_SetBits(GPIOB,GPIO_Pin_6);
	  //确认是否接收到数据
	  RX_status=USART_GetFlagStatus(USART1,USART_FLAG_RXNE);
	  //接收到数据
	  if(RX_status==SET)
		{
		    USART_SendData(USART1,USART_ReceiveData(USART1));	
		//等待数据发送完毕
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
			GPIO_ResetBits(GPIOB,GPIO_Pin_6);	
    }
}


void USART2_IRQHandler(void)   //WIFI
{
	char temp;
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断
    {	
		USART_ClearFlag(USART2,USART_FLAG_ORE);				
        USART_ReceiveData(USART2);		
    }
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 
		temp=USART_ReceiveData(USART2);
		WIFI_BUF[WIFI_COUNT++]=temp;
		WIFI_BUF[WIFI_COUNT]='\0';

		if(WIFI_COUNT > DEFAULT_WIFI_BUF_NUM){
			WIFI_COUNT = 0;
		}


		#ifdef PRINTF_DEBUG
		if(LOG_DIRECTION==USART_WIFI)
		{
			USART_SendData(USART1,temp);	
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 			
		}
		#endif		 
	
		
		USART_ClearFlag(USART2,USART_FLAG_RXNE);

	} 

}


void USART3_IRQHandler(void)   //BLE
{
	char temp;
												
	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断
    {
		USART_ClearFlag(USART3,USART_FLAG_ORE);	
        USART_ReceiveData(USART3);	 		
    }

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 
			
		temp=USART_ReceiveData(USART3);
        
        #ifdef PRINTF_DEBUG
		if(LOG_DIRECTION==USART_BLE)
		{
			USART_SendData(USART1,temp);	
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);			
		}
        #endif
        
		BLE_BUF[BLE_COUNT++]=temp;
		BLE_BUF[BLE_COUNT]='\0';

		if(BLE_COUNT>DEFAULT_BLE_BUF_NUM){

			BLE_COUNT=0;
		} 

//		if(BLUR_OBD){//蓝牙和OBD直通
//
//			USART_SendData(OBD,temp);	
//			while(USART_GetFlagStatus(OBD,USART_FLAG_TC)==RESET);  
//			USART_ClearFlag(BLUE,USART_FLAG_RXNE);
//			return;		
//		}
//		
		
		USART_ClearFlag(USART3,USART_FLAG_RXNE);
     } 
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
