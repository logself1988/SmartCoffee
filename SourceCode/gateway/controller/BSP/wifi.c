#include "wifi.h"
#include "stm32f10x.h"
#include "hard_init.h"
#include <string.h>
#include <stdio.h>
#include "blueTooth.h"

#define MAX_CMD 128
static char ap_ssid[WIFI_LIMIT_SSID + 1];
static char ap_pass[WIFI_LIMIT_SSID + 1];

// return new length of the string
static int escape_strcat(char *a, const char *b)
{
  char *p = NULL;
  a += strlen(a);
  p = a;
  while (*b) {
    if (*b == ',' || *b == '\\' || *b == '"') {
      *p = '\\';
      p++;
    }
    *p = *b;
    p++;
    b++;
  }
  *p = '\0';
  return p - a;
}

// Send command to WIFI module and wait for response
static int WIFI_Call(const char *at, const char *expect, unsigned int delay_time)
{
  unsigned int x;

  dbg_printf("WIFI<<%s\r\n", at);
  memset(WIFI_BUF, '\0', DEFAULT_WIFI_BUF_NUM);
  WIFI_COUNT = 0;

  for (x = 0; *(at + x) != '\0'; x++) {
    USART_SendData(USART2, *(at + x));
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
      ;
  }
  delay_ms(delay_time);

  if (expect && NULL == strstr(WIFI_BUF, expect)) {
    return -1;
  }

  return 0;
}

int WIFI_Init(const char* ssid, const char* pass)
{
  strncpy(ap_ssid, ssid, WIFI_LIMIT_SSID);
  strncpy(ap_pass, pass, WIFI_LIMIT_PASS);
  dbg_printf("ssid=%s,pass=%s\n", ap_ssid, ap_pass);

  dbg_printf("WIFI module init..\r\n");
  if (WIFI_Call("AT\r\n", "OK", 1000)) {
    dbg_printf("WIFI module init..failed\r\n");
    return -1;
  }
  dbg_printf("WIFI module init..OK\r\n");
  return 0;
}

int WIFI_Restart(void)
{
  dbg_printf("WIFI module reset..\r\n");
  if (WIFI_Call("AT+RST\r\n", "OK", 200)) {
    dbg_printf("WIFI module reset..failed\r\n");
    return -1;
  }
  return 0;
}

int WIFI_Config(void)
{
  /* set wifi mode to station */
  dbg_printf("WIFI switch to station mode..\r\n");
  if (WIFI_Call("AT+CWMODE_CUR=1\r\n", "OK", 200)) {
    dbg_printf("WIFI switch to station mode..failed\r\n");
    return -2;
  }
  dbg_printf("WIFI switch to station mode..OK\r\n");

  /* enable dhcp */
  dbg_printf("WIFI enable DHCP..\r\n");
  if (WIFI_Call("AT+CWDHCP_CUR=1,1\r\n", "OK", 200)) {
    dbg_printf("WIFI enable DHCP..failed\r\n");
    return -3;
  }
  dbg_printf("WIFI enable DHCP..OK\r\n");

  /* enable multiple connection mode */
  dbg_printf("WIFI enable multiple connections mode..\r\n");
  if (WIFI_Call("AT+CIPMUX=1\r\n", "OK", 200)) {
    dbg_printf("WIFI enable multiple connections mode..failed\r\n");
    return -4;
  }
  dbg_printf("WIFI enable multiple connections mode..OK\r\n");

  return 0;
}

int WIFI_ConnectAP()
{
  char cmd[MAX_CMD] = {0};
  char *p = cmd;
  /* connect to AP */
  p += sprintf(p, "AT+CWJAP_CUR=\"");
  p += escape_strcat(p, ap_ssid);
  p += sprintf(p, "\",\"");
  p += escape_strcat(p, ap_pass);
  p += sprintf(p, "\"\r\n");
  if (WIFI_Call(cmd, "OK", 10000)) {
    dbg_printf("Connect failure!!!\r\n");
    return 1;
  }
  dbg_printf("Connected to %s!!!\r\n", ap_ssid);

  if (WIFI_Call("AT+CIFSR\r\n", "OK", 200)) {
    dbg_printf("Get IP failure!!!\r\n");
    return 1;
  }
  dbg_printf("Current IP:%s\r\n", WIFI_BUF);

  return 0;
}

/* Get wifi status
   2. ip ready
   3. connected
   4. disconnected
   */
int WIFI_Status(void)
{
  const char* status = NULL;
  if (WIFI_Call("AT+CIPSTATUS\r\n", "STATUS:", 200)) {
    dbg_printf("failed to get wifi status\r\n");
    return -1;
  }

  dbg_printf("%s\r\n", WIFI_BUF);
  status = strstr(WIFI_BUF, "STATUS:");
  if (NULL == status)
    return -1;

  return status[7] - '0';
}
int WIFI_Send(const char *data, size_t len, const char *ip, unsigned short port)
{
  char cmd[MAX_CMD] = {0};
  int status = 0;

  dbg_printf("WIFI status..\r\n");
  status = WIFI_Status();
  switch (status) {
    case WIFI_STATUS_DISCONNECTED:
      dbg_printf("INFO: WIFI status disconnected\r\n");
      break;
    case WIFI_STATUS_CONNECTED:
      goto final_exit;
    case WIFI_STATUS_GOT_IP:
      dbg_printf("WIFI status..OK\r\n");
      break;
    default:
      dbg_printf("ERROR: invalid WIFI status:%d\r\n", status);
      dbg_printf("WIFI config..\r\n");
      if (WIFI_Config()) {
        dbg_printf("WIFI config..failed\r\n");
        WIFI_Restart();
        return -1;
      }
      dbg_printf("WIFI config..OK\r\n");

      dbg_printf("WIFI connect..\r\n");
      if (WIFI_ConnectAP()) {
        dbg_printf("WIFI connect..failed\r\n");
        goto final_exit;
      }
      dbg_printf("WIFI connect..OK\r\n");
      goto final_exit;
  }

  sprintf(cmd, "AT+CIPSTART=0,\"TCP\",\"%s\",%u\r\n", ip, port);
  dbg_printf("WIFI tcp link..\r\n");
  if (WIFI_Call(cmd, "OK", 500)) {
    dbg_printf("WIFI tcp link..failed\r\n");
    goto final_exit;
  }
  dbg_printf("WIFI tcp link..OK\r\n");

  sprintf(cmd, "AT+CIPSENDEX=0,%lu\r\n", len);
  dbg_printf("WIFI send data..\r\n");
  if (WIFI_Call(cmd, "AT+", 500)) {
    dbg_printf("WIFI send data..failed\r\n");
    goto final_exit;
  }

  if (WIFI_Call(data, "SEND OK", 200)) {
    dbg_printf("WIFI send data..failed\r\n");
    goto final_exit;
  }
  dbg_printf("WIFI send data..OK\r\n");

final_exit:
  if (WIFI_Call("AT+CIPCLOSE=5\r\n", "OK", 500)) {
    dbg_printf("\r\nFailed to close connection!!!\r\n");
    return -1;
  }
  dbg_printf("\r\nconnections closed!!!\r\n");

  return 0;
}
