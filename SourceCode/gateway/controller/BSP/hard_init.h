#ifndef	HARD_INIT_H
#define	HARD_INIT_H

#include "stm32f10x.h"

#include <stdio.h>

#define DEBUG
#ifdef  DEBUG
#define dbg_printf(FORMAT, ...) \
    printf("%s() in %s, line %i: " FORMAT, \
      __func__, __FILE__, __LINE__, ##__VA_ARGS__)
#define dbg_puts(MSG) dbg_printf("%s", MSG)
#else
#define dbg_printf(FORMAT, ...) ((void)0)
#define dbg_puts(MSG) ((void)0)
#endif

//全局变量声明区
extern char WIFI_BUF[];
extern char BLE_BUF[];
extern unsigned int WIFI_COUNT;
extern unsigned int BLE_COUNT;

#define DEFAULT_MAX_CARD_DATA_NUM     15
#define DEFAULT_WIFI_BUF_NUM          200
#define DEFAULT_BLE_BUF_NUM           200

#define DEFAULT_SSID                  "ASUS"
#define DEFAULT_PASSWORD              "zhenzhen123"
#define DEFAULT_SERVER_IP             ""
#define DEFAULT_PORT                  ""

enum LOG_TYPE {UASRT_MAIN,USART_WIFI,USART_BLE};
extern enum LOG_TYPE LOG_DIRECTION;


void delay_ms(unsigned int time);

void hardware_init(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void RTC_Configuration(void);
void TIM_Configuration(void);

#endif
