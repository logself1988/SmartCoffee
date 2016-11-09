#ifndef BLUE_TOOTH_H
#define BLUE_TOOTH_H

#include <stdio.h>
/***********************************蓝牙命令***************************************************
AT
AT+ROLE?
AT+SCAN
AT+CON[x]
AT+RSSI
AT+DISCON
AT+WRITE[0xXX]
AT+CARDNUM
AT+GETDATA
AT+RESET
**********************************************************************************************/

extern int BLE_Init(void);
extern int BLE_ScanCard(char** pdata, size_t* len);
extern int BLE_GetCFG(char *ssid,char *pass,char *host,unsigned short *port);

#endif
