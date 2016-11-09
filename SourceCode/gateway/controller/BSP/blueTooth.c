#include "blueTooth.h"
#include "stm32f10x.h"
#include "hard_init.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/*******************************************************************************
 * 函数名称:BLE_Call
 * 描    述:蓝牙AT命令执行
 *
 * 输    入:at命令串，返回值查询关键字，命令执行等待时间
 * 输    出:无
 * 返    回:-1：命令未出现期望结果，0：命令出现期望结果
 *******************************************************************************/
int BLE_Call(const char *at, const char *expect, int ms)
{
  int x = 0;
  memset(BLE_BUF, '\0', DEFAULT_BLE_BUF_NUM);
  BLE_COUNT = 0;
  for (x = 0; *(at + x) != '\0'; x++) {
    USART_SendData(USART3, *(at + x));
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      ;
  }
  delay_ms(ms);

  if (expect && NULL == strstr(BLE_BUF, expect)) {
    return -1;
  }

  return 0;
}

int BLE_Init(void)
{
  dbg_printf("\r\nInit BLE...\r\n");
  if (BLE_Call("AT\r\n", "OK", 200)) {
    dbg_printf("\r\nBLE initiation failed!!!\r\n");
    return -1;
  }
  dbg_printf("\r\nBLE is ready!!!\r\n");

  return 0;
}

int BLE_ScanCard(char **pdata, size_t *len)
{
  // number of cards scanned
  int count = 0;
  dbg_printf("BLE scan..\r\n");
  if (BLE_Call("AT+SCAN\r\n", "Discovering", 3000)) {
    dbg_printf("BLE scan..failed\r\n");
    return -1;
  }
  dbg_printf("BLE scan..OK\r\n");

  dbg_printf("BLE read number of cards..\r\n");
  if (BLE_Call("AT+CARDNUM\r", NULL, 200)) {
    dbg_printf("BLE read number of cards..failed\r\n");
    return -2;
  }
  dbg_printf("BLE read number of cards..OK\r\n");
  dbg_printf("INFO: number of cards is: %s\r\n", BLE_BUF);

  count = atoi(BLE_BUF);
  if (0 == count) {
    *pdata = NULL;
    *len = 0;
    dbg_printf("WARN: no cards found\r\n");
    return -3;
  }

  BLE_Call("AT+GETDATA\r", NULL, 500);
  dbg_printf("INFO: cards data:\r\n%s\r\n", BLE_BUF);
  *pdata = BLE_BUF;
  *len = strlen(BLE_BUF);
  
//   BLE_Call("AT+SLAVE\r\n", "slave",1000);
  return 0;
}

int BLE_GetCFG(char *ssid,char *pass,char *host,unsigned short *port)
{
	uint32_t tmp;
	
	if(BLE_Call("AT+SSID\r",NULL,200)){
		dbg_printf("BLE read ssid..failed\r\n");
		return 1;
	}
	dbg_printf("BLE read SSID successfully!!!\r\n");
	dbg_printf("INFO: SSID is: %s\r\n", BLE_BUF);
	strncpy(ssid,BLE_BUF,20);
	
	if(BLE_Call("AT+PASSWORD\r",NULL,200)){
		dbg_printf("BLE read password..failed\r\n");
		return 1;
	}
	dbg_printf("BLE read password successfully!!!\r\n");
	dbg_printf("INFO: PASSWORD is: %s\r\n", BLE_BUF);
	strncpy(pass,BLE_BUF,20);
	
	if(BLE_Call("AT+HOST\r",NULL,200)){
		dbg_printf("BLE read host..failed\r\n");
		return 1;
	}
	dbg_printf("BLE read host successfully!!!\r\n");
	dbg_printf("INFO: HOST is: %s\r\n", BLE_BUF);

	sscanf(BLE_BUF, "%x", &tmp);
	sprintf(host, "%d.%d.%d.%d", (tmp>>24), ((tmp>>16)&0xff), ((tmp>>8)&0xff), (tmp&0xff));
	
	if(BLE_Call("AT+PORT\r",NULL,200)){
		dbg_printf("BLE read PORT..failed\r\n");
		return 1;
	}
	dbg_printf("BLE read port successfully!!!\r\n");
	dbg_printf("INFO: PORT is: %s\r\n", BLE_BUF);
	
  sscanf(BLE_BUF, "%x", port);
	
	
	return 0;
}
