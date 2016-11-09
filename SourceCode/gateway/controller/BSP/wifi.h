#ifndef WIFI_H
#define WIFI_H

#include <stdio.h>

#define WIFI_STATUS_GOT_IP 2
#define WIFI_STATUS_CONNECTED 3
#define WIFI_STATUS_DISCONNECTED 4

#define WIFI_LIMIT_SSID 32
#define WIFI_LIMIT_PASS 64

extern int WIFI_Init(const char* ssid, const char* pass);
extern int WIFI_Send(const char* data, size_t len, const char* ip, unsigned short port);
extern int WIFI_ConnectAP(void);

#endif
