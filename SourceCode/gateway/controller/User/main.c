/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
#include "stm32f10x.h"
#include "hard_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "blueTooth.h"
#include "wifi.h"

// /** @addtogroup STM32F10x_StdPeriph_Template
//   * @{
//   */

// /* Private typedef
// -----------------------------------------------------------*/
// /* Private define
// ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
FlagStatus RX_status;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small dbg_printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

#define MAX_IP 45

#define AP_SSID_MAX_LEN        21
#define AP_PASS_MAX_LEN        21
#define SERVER_HOST_MAX_LEN    21

/* Private functions ---------------------------------------------------------*/
void LED_Blink(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
	delay_ms(200);
	GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void) {
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */

  char* cards = NULL;
  size_t scards = 0;
//   char ap_ssid[AP_SSID_MAX_LEN] = "ASUS";
//   char ap_pass[AP_PASS_MAX_LEN] = "zhenzhen123";
  char ap_ssid[AP_SSID_MAX_LEN] = "";
  char ap_pass[AP_PASS_MAX_LEN] = "";
  char server_host[SERVER_HOST_MAX_LEN] = "";
  unsigned short server_port = 0;
	uint8_t count = 0;

  hardware_init();

  /* Add your application code here
     */

  dbg_printf("Hello\r\n");
	
	  /* initialize BLE */
  while (BLE_Init()) {
    dbg_printf("ERROR: failed to init BLE\r\n");
  }
	
	while (BLE_GetCFG(ap_ssid,ap_pass,server_host,&server_port)) {
		dbg_printf("ERROR: failed to config wifi\r\n");
	}
	
  /* initialize WIFI */
  while (WIFI_Init(ap_ssid, ap_pass)) {
    dbg_printf("ERROR: failed to init WIFI\r\n");
  }
	
// 	while (WIFI_ConnectAP()){
// 		while (BLE_GetCFG(ap_ssid,ap_pass,server_host,&server_port)) {
// 		  dbg_printf("ERROR: failed to config wifi\r\n");
// 	  }
// 		dbg_printf("ERROR: failed to connect to AP\r\n");
// 	}

  /* Infinite loop */
  while (1) {
    count++;
		if(count==20){
			if(BLE_GetCFG(ap_ssid,ap_pass,server_host,&server_port)){
				dbg_printf("ERROR: failed to config the wifi\r\n");
            }
			dbg_printf("INFO: config wifi successfully\r\n");
			count=0;
		}
		LED_Blink();

    if (BLE_ScanCard(&cards, &scards)) {
      dbg_printf("WARN: no cards found.\r\n");
      continue;
    }

    if (WIFI_Send(cards, scards, server_host, server_port)) {
      dbg_printf("ERROR: failed to report cards information\r\n");
			WIFI_Init(ap_ssid, ap_pass);
			WIFI_Send(cards, scards, server_host, server_port);
      continue;
    }

    dbg_printf("INFO: cards reported\r\n");
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t)ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
  }

  return ch;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number,
     ex: dbg_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1) {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
