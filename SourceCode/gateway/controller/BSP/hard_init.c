#include <stdio.h>
#include "hard_init.h"

// #define DEFAULT_MAX_CARD_DATA_NUM     15
// #define DEFAULT_WIFI_BUF_NUM          200
// #define DEFAULT_BLE_BUF_NUM           200

// global variables def
char WIFI_BUF[DEFAULT_WIFI_BUF_NUM];
char BLE_BUF[DEFAULT_BLE_BUF_NUM];
unsigned int WIFI_COUNT = DEFAULT_WIFI_BUF_NUM;
unsigned int BLE_COUNT = DEFAULT_BLE_BUF_NUM;
enum LOG_TYPE LOG_DIRECTION = UASRT_MAIN;
// char CardData[DEFAULT_MAX_CARD_DATA_NUM][17];
// AdvDataUp_t CardData[DEFAULT_MAX_CARD_DATA_NUM];

void delay_ms(unsigned int time)
{
  unsigned int x, y;
  for (x = 0; x < time; x++)
    for (y = 0; y < 13000; y++)
      ;
}

void RCC_Configuration(void) //内部
{
  //将外设 RCC寄存器重设为缺省值
  RCC_DeInit();
  RCC_HSICmd(ENABLE);

  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) {
  }
  if (1) {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //使能预取指缓存
    FLASH_SetLatency(FLASH_Latency_2); //设置flash代码延时
    RCC_HCLKConfig(RCC_SYSCLK_Div1); //设置AHB时钟（HCLK）为系统时钟
    RCC_PCLK2Config(RCC_HCLK_Div1); //设置高速AHB时钟（APB2）为HCLK时钟
    RCC_PCLK1Config(RCC_HCLK_Div1); //设置低速AHB时钟（APB1）为HCLK时钟

    //设置 PLL 时钟源及倍频系数
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
    //使能或者失能 PLL,这个参数可以取：ENABLE或者DISABLE
    RCC_PLLCmd(ENABLE); //如果PLL被用于系统时钟,那么它不能被失能
    //等待指定的 RCC 标志位设置成功 等待PLL初始化成功
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
    }
    //设置系统时钟（SYSCLK） 设置PLL为系统时钟源

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //等待PLL成功用作于系统时钟的时钟源
    // 0x00：HSI 作为系统时钟
    // 0x04：HSE作为系统时钟
    // 0x08：PLL作为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08) {
    }
    /*
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    while(RCC_GetSYSCLKSource() != 0x00)
    {
    }
    */
  }

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_USART1 |
                             RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                             RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                             RCC_APB2Periph_AFIO,
                         ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 |
                             RCC_APB1Periph_PWR | RCC_APB1Periph_BKP |
                             RCC_APB1Periph_TIM3,
                         ENABLE);
}

/*********************************GPIO_MAP**************************************

LED
                LED_RED			PB5
                LED_BLUE		PB6

UART1

          TX_UART1      PA9
          RX_UART1      PA10

BLE
                TX_UART3			PB10
                RX_UART3			PB11

          TODO:RST

WIFI
                TX_UART2			PA2
                RX_UART2			PA3

    TODO:RST

******************************************************************************/

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
  // 							   RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,
  // ENABLE );
  //
  // 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_PWR
  // | RCC_APB1Periph_BKP|
  // 							   RCC_APB1Periph_TIM3,
  // ENABLE
  // );

  //-----------------------------设置所有引脚为模拟输入———省电---------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  //-----------------------串口引脚初始化---------------------------------

  //初始化串口1引脚
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //初始化串口2引脚
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //初始化串口3引脚
  // 	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
  // 	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  // 	GPIO_Init(GPIOB,&GPIO_InitStructure);

  // 		/*
  // 	*  USART1_TX -> PC10 , USART1_RX ->	PC11
  // 	*/
  // 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  // 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  // 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // 	GPIO_Init(GPIOC, &GPIO_InitStructure);
  //
  // 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  // 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  // 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // 	GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 		/*
  // 	*  USART1_TX -> PB10 , USART1_RX ->	PB11
  // 	*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-----------------------------------LED
  //输出设置---------------------------------
  /*

  LED
                  LED_RED			PB5
                  LED_BLUE		PB6
  */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //	GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_RESET);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //	GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
}

/*********************************UART_MAP**************************************

UART1

            TX_UART1            PA9
            RX_UART1            PA10

BLE
                TX_UART3			PB10
                RX_UART3			PB11

WIFI
                TX_UART2			PA2
                RX_UART2			PA3

******************************************************************************/
void USART_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /***************************USART1*******************************************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
  /*
  *  USART1_TX -> PA9 , USART1_RX ->	PA10
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  USART_Cmd(USART1, ENABLE);

  /***************************USART2******************************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /*
  *  USART1_TX -> PA2 , USART1_RX ->	PA3
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  //  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  USART_Cmd(USART2, ENABLE);
  //  return ;

  /***************************USART3*********BLE**********************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /*
  *  USART3_TX -> PB10 , USART3_RX ->	PB11
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  //	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  //  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
  USART_Cmd(USART3, ENABLE);
}

void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  //--------------------------------RTC闹钟中断设置---------------------------------
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
#ifdef IAP_FUNCTION
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_VectTab_FLASH_ADDR);
#endif
  //--------------------------------打开串口中断 ---------------------
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //---------------------------------RTC闹钟--------------------------------
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Timer3中断*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void RTC_Configuration(void)
{
  /* RTC clock source configuration ------------------------------------------*/
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  /* Reset Backup Domain */
  BKP_DeInit();
  /* Enable the LSE OSC */
  // RCC_LSEConfig(RCC_LSE_ON);
  RCC_LSICmd(ENABLE);
  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  /* RTC configuration -------------------------------------------------------*/
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();
  /* Set the RTC time base to 1s */
  RTC_SetPrescaler(40000);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  /* Enable the RTC Alarm interrupt */
  RTC_ITConfig(RTC_IT_SEC, ENABLE);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}

void TIM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  /************************************************定时器2设置********************************************************/
  //	/* 基础设置*/
  //	TIM_TimeBaseStructure.TIM_Period = 20000;		//计数值
  //	TIM_TimeBaseStructure.TIM_Prescaler = 800;    	//预分频
  //	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  //	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  ////向上计数
  //	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  //
  //	/*使能预装载*/
  //	TIM_ARRPreloadConfig(TIM2, ENABLE);
  //	/*预先清除所有中断位*/
  //	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
  //	/* 4个通道和溢出都配置中断*/
  //	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  //	/* 允许TIM2开始计数 */
  //	TIM_Cmd(TIM2, DISABLE);

  /************************************************定时器3设置********************************************************/
  /* 基础设置*/
  TIM_TimeBaseStructure.TIM_Period = 10000;   //计数值 1s
  TIM_TimeBaseStructure.TIM_Prescaler = 7200; //预分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /*使能预装载*/
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /*预先清除所有中断位*/
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  /*溢出都配置中断*/
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  /* 允许TIM2开始计数 */
  TIM_Cmd(TIM3, ENABLE);
}

void hardware_init(void)
{
  RCC_Configuration();
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //禁止JTAG
  GPIO_Configuration();

  USART_Configuration();
  //RTC_Configuration();
  //EXTI_Configuration();
  NVIC_Configuration();
  //TIM_Configuration();

  LOG_DIRECTION = USART_BLE;
}
