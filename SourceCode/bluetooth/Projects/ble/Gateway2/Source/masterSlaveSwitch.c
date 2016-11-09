/**************************************************************************************************
  Filename:       masterSlaveSwitch.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Master Slave Switch sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
//#include "hal_led.h"
//#include "hal_key.h"
//#include "hal_lcd.h"
#include "hal_flash.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "central.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "masterSlaveSwitch.h"

// Proprietary services
// Profile�ļ�������ʹ�õĺ�SimpleProfile��ͬ���������޸��˲�������
#include "exampleService.h"

#include "SerialApp.h"
#include <string.h>
/*********************************************************************
 * MACROS

// Length of bd addr as a string

#define B_ADDR_STR_LEN                        20

*/

/*********************************************************************
 * CONSTANTS
 */
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/***************************** CENTRAL defines ***********************/

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

#define WANTED_SERVICE_UUID                   0xFFF0

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  15

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY


/*********************************************************************
 * TYPEDEFS
 */
typedef enum {
  ROLE_PERIPHERAL = 1,
  ROLE_CENTRAL    = 2
} deviceRole_t;


typedef enum {
  GAPROLE_CENTRAL_UNITIALIZED,
  GAPROLE_CENTRAL_INIT_DONE,
  GAPROLE_CENTRAL_SCANNING,
  GAPROLE_CENTRAL_CONNECTING,
  GAPROLE_CENTRAL_CONNECTED,
  GAPROLE_CENTRAL_DISCONNECTING,
  GAPROLE_CENTRAL_DISCONNECTED
} gapCentralRole_States_t;

// Discovery states
typedef enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} gapCentralDiscStates_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

//// Discovery states
//enum
//{
//  BLE_DISC_STATE_IDLE,                // Idle
//  BLE_DISC_STATE_SVC,                 // Service discovery
//  BLE_DISC_STATE_CHAR                 // Characteristic discovery
//};


#define WIFI_LIMIT_HOST 4
#define WIFI_LIMIT_PORT 2
#define WIFI_LIMIT_SSID 20
#define WIFI_LIMIT_PASS 20
#define LIMIT_CHAR_LENGTH 20

//static uint32 server_host;
//static uint16 server_port;

static unsigned char server_host[WIFI_LIMIT_HOST];
static unsigned char server_port[WIFI_LIMIT_PORT];
static unsigned char ap_ssid[WIFI_LIMIT_SSID];
static unsigned char ap_pass[WIFI_LIMIT_PASS];

static uint8 ROLE_Flags=1;

static uint8 gPairStatus=0;/*��������ǰ��״̬��������벻��ȷ������ȡ�����ӣ�0��ʾδ��ԣ�1��ʾ�����*/

static uint8 masterSlaveSwitch_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t             gapPeripheralState = GAPROLE_INIT;
//static gapCentralRole_States_t      gapCentralState = GAPROLE_CENTRAL_UNITIALIZED;

static uint8 gLinkStatus=0;
static uint8 gStatus;

//jackpe, devices' rssi
//static uint8 deviceRSSI[DEFAULT_MAX_SCAN_RES] = {100,100,100,100,100,100,100,100};
//static uint8 smallestRSSIIndex;
//static uint16 tableMinor[DEFAULT_MAX_SCAN_RES] = {0};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x08,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'G','A','T','E','W','A','Y',
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "GATEWAY";

// Initial State
//static deviceRole_t deviceRole = ROLE_PERIPHERAL;

// Connection handle of active connection
//static uint16 connHandle = INVALID_CONNHANDLE;


/**************** Variables concerning the Central mode ***************/
/**********************************************************************/
// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
//static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;

// Value to write
static uint8 simpleBLECharVal= 0;

// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

//static char ScanRspName[10]={'0','0','0','0','0','0','0','0','0','0'};

typedef struct
{
    uint8 CardAddr[6];
    uint8 Card_Major_H;
    uint8 Card_Major_L;
    uint8 Table_Minor_H;
    uint8 Table_Minor_L;
} AdvDataUp_t;

static AdvDataUp_t DataUp[DEFAULT_MAX_SCAN_RES]={0};

// RSSI polling state
static uint8 simpleBLERssi = FALSE;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void masterSlaveSwitch_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static void peripheralStateNotificationCB( gaprole_States_t newState );

static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
//static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );

static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
//static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );

static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
//static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );

//static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );

static void simpleBLECentralStartDiscovery( void );
//static void simpleBLECentralStartDiscovery( void );


static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
//static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );

static void exampleServiceChangeCB( uint8 paramID );
char *bdAddr2Str( uint8 *pAddr );
//char *bdAddr2Str ( uint8 *pAddr );



/////////////////////////////////////////////////////////////////////////////



static void ProcessPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );

static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );

unsigned char* itoa(uint8 unum,unsigned char* str,uint8 radix);

void IO_Init(void);


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks PERIPHERAL
static gapRolesCBs_t masterSlaveSwitch_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
//static gapBondCBs_t masterSlaveSwitch_BondMgrCBs =
//{
//  NULL,                     // Passcode callback (not used by application)
//  NULL                      // Pairing / Bonding state Callback (not used by application)
//};

// GAP Role Callbacks CENTRAL
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};


// Example Service Callbacks
static simpleProfileCBs_t masterSlaveSwitch_exampleServiceCBs =
{
  exampleServiceChangeCB    // Charactersitic value change callback
};

//Ҫʵ������󶨣�������Ҫʵ��bongmgr�Ļص�������
// GAP Bond Manager Callbacks
static gapBondCBs_t masterSlaveSwitch_BondMgrCBs =
{
  ProcessPasscodeCB,                     // ����ص�
  ProcessPairStateCB                     // ��״̬�ص�
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
	delay function, seems no impact on the osal stack
  */

void DelayMS(uint16 msec)
{
	uint16 i,j;
	for (i=0; i<msec; i++)
	for (j=0; j<536; j++);
}


/*********************************************************************
 * @fn      MasterSlaveSwitch_Init
 *
 * @brief   Initialization function for the Master Slave Switch App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void MasterSlaveSwitch_Init( uint8 task_id )
{
   
    IO_Init();

  P1_0 = 1;
  DelayMS(500);
  P1_0 = 0;
  DelayMS(500);
  P1_0 = 1;
  DelayMS(500);
  P1_0 = 0;
  
    HCI_EXT_ExtendRfRangeCmd();
    TXPOWER=0XF1;
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  
  osal_memset(server_host, 0, SIMPLEPROFILE_CHAR1_LEN);
  osal_memset(server_port,0,SIMPLEPROFILE_CHAR2_LEN);
  osal_memset(ap_ssid,0,SIMPLEPROFILE_CHAR3_LEN);
  osal_memset(ap_pass,0,SIMPLEPROFILE_CHAR4_LEN);
  
  ROLE_Flags=0;
  
  uint8 buf[LIMIT_CHAR_LENGTH];
  osal_memset(buf, 0, LIMIT_CHAR_LENGTH);

  HalFlashRead(0x78, 0, buf, SIMPLEPROFILE_CHAR1_LEN);
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN,
                               buf);
//  server_host=BUILD_UINT32( buf[3], buf[2], buf[1], buf[0] );
  osal_memcpy(server_host,buf,SIMPLEPROFILE_CHAR1_LEN);

  HalFlashRead(0x79, 0, buf, SIMPLEPROFILE_CHAR2_LEN);
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN,
                               buf);
//  server_port=BUILD_UINT16(buf[1],buf[0]);
  osal_memcpy(server_port,buf,SIMPLEPROFILE_CHAR2_LEN);

  HalFlashRead(0x7a, 0, buf, SIMPLEPROFILE_CHAR3_LEN);
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
                               buf);
  osal_memcpy(ap_ssid,buf,SIMPLEPROFILE_CHAR3_LEN);

  HalFlashRead(0x7b, 0, buf, SIMPLEPROFILE_CHAR4_LEN);
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN,
                               buf);
  osal_memcpy(ap_pass,buf,SIMPLEPROFILE_CHAR4_LEN);
  
  
  simpleBLETaskId = task_id;
  //��Ӵ��ڳ�ʼ��������������id
  SerialApp_Init(simpleBLETaskId);

  SerialPrintString("SimpleBLECentral_SerialPrint Start init.\r\n");
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
//  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 123456;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
  SerialPrintString("Ready to Starting\r\n");
  
  
//////////////////////////////////////////////////////////////////////////////////////////////////////  

  masterSlaveSwitch_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {

    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = FALSE; // We set this to FALSE to bypass peripheral.c's auto-start

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

 // Setup the GAP Bond Manager
  
  {
    uint32 passkey = 123456; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }


  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service

  // Proprietary service
  
  SimpleProfile_AddService(GATT_ALL_SERVICES);
  SimpleProfile_RegisterAppCBs( &masterSlaveSwitch_exampleServiceCBs );
  

  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( masterSlaveSwitch_TaskID, START_DEVICE_EVT );
  
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Initialize GATT Client
  VOID GATT_InitClient();
  
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    
}

/*********************************************************************
 * @fn      MasterSlaveSwitch_ProcessEvent
 *
 * @brief   Master Slave Switch Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 MasterSlaveSwitch_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( masterSlaveSwitch_TaskID )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  //////////////////////////////////////////////////////

  if ( events & START_DEVICE_EVT )
  {
  
    if(ROLE_Flags==1){
      VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );
          // Register with bond manager after starting device
      GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB ); 
      SerialPrintString("BLE Stack is running\r\n");
     }
    else
     {
      // Start the Device
      VOID GAPRole_StartDevice( &masterSlaveSwitch_PeripheralCBs );
    // Start Bond Manager
      VOID GAPBondMgr_Register( &masterSlaveSwitch_BondMgrCBs );
     }      
    return ( events ^ START_DEVICE_EVT );
  }
  
 ///////////////////////////////////////////////////////
  
    if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      masterSlaveSwitch_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
//    case KEY_CHANGE:
//      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
//      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
		
        // Set up advertising to stop
         uint8 advertising_enable = FALSE;
         GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );

         GAP_UpdateAdvertisingData( masterSlaveSwitch_TaskID, TRUE, 30, advertData);

        // Set up advertising to start
         advertising_enable = TRUE;
         GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
         gLinkStatus=0;
      }
      break;

    case GAPROLE_ADVERTISING:
      {

        gLinkStatus=0;

      }
      break;

    case GAPROLE_CONNECTED:
      {
          
         gLinkStatus=1;
      }
      break;

    case GAPROLE_WAITING:                 // Disconnected or stopped adv
      
      {gLinkStatus=0;}
      break;
    case GAPROLE_WAITING_AFTER_TIMEOUT:   // Disconnected due to superv. timeout
      {
        gLinkStatus=0;

      }
      break;

    case GAPROLE_ERROR:
      {
      }
      break;

    default:
      {
      }
      break;

  }

  gapPeripheralState = newState;
//  VOID gapPeripheralState;
}



/*********************************************************************
 * @fn      exampleServiceChangeCB
 *
 * @brief   Callback from exampleService indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */

static void exampleServiceChangeCB(uint8 paramID)
{
  uint8 newValue[20];
  uint8 debug_tmp[20];
  osal_memset(newValue, 0, 20);
  osal_memset(debug_tmp, 0, 20);
  
  switch (paramID) {

  case SIMPLEPROFILE_CHAR1:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newValue);
    HalFlashErase(0x78);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF000, newValue, SIMPLEPROFILE_CHAR1_LEN/4);
    HalFlashRead(0x78,0,debug_tmp,SIMPLEPROFILE_CHAR1_LEN);
//    server_host=BUILD_UINT32( newValue[3], newValue[2], newValue[1], newValue[0] );
    osal_memcpy(server_host,newValue,SIMPLEPROFILE_CHAR1_LEN);
    break;
  case SIMPLEPROFILE_CHAR2:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR2, newValue);
    HalFlashErase(0x79);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF200, newValue, SIMPLEPROFILE_CHAR2_LEN);
    HalFlashRead(0x79,0,debug_tmp,SIMPLEPROFILE_CHAR2_LEN);
//    server_port=BUILD_UINT16(newValue[1],newValue[0]);
    osal_memcpy(server_port,newValue,SIMPLEPROFILE_CHAR2_LEN);
    break;
  case SIMPLEPROFILE_CHAR3:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, newValue);
    HalFlashErase(0x7A);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF400, ap_ssid, SIMPLEPROFILE_CHAR3_LEN/4);
    HalFlashRead(0x7A,0,debug_tmp,SIMPLEPROFILE_CHAR3_LEN);
    osal_memcpy(ap_ssid,newValue,SIMPLEPROFILE_CHAR3_LEN);
    
    break;

  case SIMPLEPROFILE_CHAR4:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR4, newValue);
    HalFlashErase(0x7B);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF600, ap_pass, SIMPLEPROFILE_CHAR4_LEN/4);
    HalFlashRead(0x7B,0,debug_tmp,SIMPLEPROFILE_CHAR4_LEN);
    
    osal_memcpy(ap_pass,newValue,SIMPLEPROFILE_CHAR4_LEN);
    break;

  default:
    // should not reach here!
    break;
  }
}


/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 masterSlaveSwitch_TaskID );
}
/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      SerialPrintValue("Read Error", status, 10);SerialPrintString("\r\n");
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];

      SerialPrintValue("Read rsp:", valueRead, 10);SerialPrintString("\r\n");
    }
    
    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      SerialPrintValue( "Write Error", status, 10);SerialPrintString("\r\n");
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      uint8 temp=simpleBLECharVal;    
      SerialPrintValue( "Write sent:", temp, 10);SerialPrintString("\r\n");
    }
    
    simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, masterSlaveSwitch_TaskID );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );
      
      simpleBLEProcedureInProgress = FALSE;
    }
    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;

    
  }    
}

/*********************************************************************
       CENTRAL FUNCTIONS
*********************************************************************/


/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{

}



/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */

static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
 
  uint8 i;

  
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        SerialPrintString("BLE Central: ");
        SerialPrintString((uint8*)bdAddr2Str( pEvent->initDone.devAddr ));SerialPrintString("\r\n");
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
//        // if filtering device discovery results based on service UUID
//        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
//        {
//          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
//                                     pEvent->deviceInfo.pEvtData,
//                                     pEvent->deviceInfo.dataLen ) )         
//          if (pEvent->deviceInfo.eventType == GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
//           {
//    
//               HalLcdWriteStringValue("touch here...", ScanRspName[0] , 16, HAL_LCD_LINE_5);
//
//               temp.Card_Major_H=pEvent->deviceInfo.pEvtData[25];
//               temp.Card_Major_L=pEvent->deviceInfo.pEvtData[26];
//               temp.Table_Minor_H=pEvent->deviceInfo.pEvtData[9];
//               temp.Table_Minor_L=pEvent->deviceInfo.pEvtData[3];
//    
//               HalLcdWriteStringValue("Table_Minor", temp.Table_Minor_L , 16, HAL_LCD_LINE_6);
//            }
//           }
 
         //�����㲥����
//         if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND)
//          {
//            if(pEvent->deviceInfo.pEvtData[9]==0x4A && pEvent->deviceInfo.pEvtData[10]==0xA8 && pEvent->deviceInfo.pEvtData[11]==0x31){
        if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_NONCONN_IND)
          {
            if(pEvent->deviceInfo.pEvtData[9]==0x47 && pEvent->deviceInfo.pEvtData[10]==0xA9 && pEvent->deviceInfo.pEvtData[11]==0x30){
               simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
               
               for(i=6;i>0;i--){
               
                  DataUp[simpleBLEScanRes-1].CardAddr[i-1]=pEvent->deviceInfo.addr[6-i];
                 
               }
               
               DataUp[simpleBLEScanRes-1].Card_Major_H=pEvent->deviceInfo.pEvtData[25];
               DataUp[simpleBLEScanRes-1].Card_Major_L=pEvent->deviceInfo.pEvtData[26];
               DataUp[simpleBLEScanRes-1].Table_Minor_H=pEvent->deviceInfo.pEvtData[27];
               DataUp[simpleBLEScanRes-1].Table_Minor_L=pEvent->deviceInfo.pEvtData[28];
               
            }
          }
         
         //����ɨ���Ӧ����
//         if (pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP)           //to get the name of advertising peripheral after ScanRsp 
//          {   
//    
//            for(i=0;i<(pEvent->deviceInfo.pEvtData[0]-1);i++)
//             {
//                 ScanRspName[i]=(char)pEvent->deviceInfo.pEvtData[2+i];
//             }
//            ScanRspName[(pEvent->deviceInfo.pEvtData[0])-1]='\0';
//
//            LCD_WRITE_STRING( ScanRspName, HAL_LCD_LINE_7 );
//            if(ScanRspName[0]=='T'&&ScanRspName[1]=='A'&&ScanRspName[2]=='B'&&ScanRspName[3]=='L') //only discovery card
//             {  
//                 simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );     //add to device list
//             }
//          }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        SerialPrintValue("Devices Found", simpleBLEScanRes,10);
        SerialPrintString("\r\n");
        
        if ( simpleBLEScanRes > 0 )
        {
          SerialPrintString("<- To Select\r\n");
        }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    

          SerialPrintString("Connected: ");  
          SerialPrintString((uint8*) bdAddr2Str( pEvent->linkCmpl.devAddr ));SerialPrintString("\r\n");
          
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          SerialPrintString("Connect Failed: ");
          SerialPrintValue("Reason:",  pEvent->gap.hdr.status,10);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;
          
        SerialPrintString("Disconnected: ");
        SerialPrintValue("Reason:",  pEvent->linkTerminate.reason,10);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        SerialPrintString("Param Update\r\n");
      }
      break;
      
    default:
      break;
  }
}


/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

//�󶨹����е��������ص�����
/*********************************************************************
 * passcode
 */
static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs )
{
  uint32  passcode;
  passcode = 123456;
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}

//�󶨹����е�״̬����������������ñ�־λ�������벻��ȷʱ����������
/*********************************************************************
 * passcode error
 */
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    gPairStatus = 0;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      gPairStatus = 1;
    }
    else
    {
      if(status ==SMP_PAIRING_FAILED_UNSPECIFIED){
        gPairStatus = 1;
      }else{
        gPairStatus = 0;
      }
    }
    //�ж���Խ�����������ȷ����ֹͣ����
    if(gPairStatus !=1){
      GAPRole_TerminateConnection();
    }
  }
}


/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
//  if ( state == GAPBOND_PAIRING_STATE_STARTED )
//  {
//    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
//  }
//  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
//  {
//    if ( status == SUCCESS )
//    {
//      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
//    }
//    else
//    {
//      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
//    }
//  }
//  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
//  {
//    if ( status == SUCCESS )
//    {
//      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
//    }
//  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
//    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
//    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}




uint8 str2hex(uint8 *hex);
uint8 str2hex(uint8 *str)
{
  uint8 hex[] = "0123456789ABCDEF";
  uint8 i=0,h,l;
  for(i=0;i<16;i++){
    if(hex[i]==str[0])
      h=i;
    if(hex[i]==str[1])
      l=i;
  }
  return (h*16+l);
}
uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len);
uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len)
{
  uint8 i=0;
  while(i<len){
    if(p1[i]!=p2[i])
      return 1;
    i++;
  }
  return 0;
}




//AT        ���ڲ��ԣ�����OK
//AT+ROLE?  ��ȡ��ǰ��ɫ
//AT+SCAN   ɨ��ӻ�
//AT+CON[x] ����ָ���Ĵӻ���xΪ�������Ĵӻ����
//AT+RSSI   ��ȡrssiֵ
//AT+DISCON �Ͽ�����
//AT+WRITE[0xXX]
//AT+GETDATA ��ȡ����
//AT+RESET   ��λ
//AT+CARDNUM ��ȡɨ�赽�Ŀ�����
void CommondHandle(uint8 *pBuffer, uint16 length)
{
  uint8 i=0;
  uint8 j=0;
  unsigned char temp[41];
  osal_memset(temp, 0, 41);
  
  if(length<2)
    return ;
  if(pBuffer[0]!='A' && pBuffer[1]!='T'){
    return ;
  }
  if(length <=4){
    SerialPrintString("OK\r\n");
    return ;
  }
  if(length>=8 && str_cmp(pBuffer+3,"ROLE?",5)==0){
    SerialPrintString("Central\r\n");
    return ;
  }
  if(length>=7 && str_cmp(pBuffer+3,"SCAN",4)==0){
    simpleBLEScanning = TRUE;
    simpleBLEScanRes = 0;
    
    SerialPrintString("Discovering...\r\n");
    
    if(gLinkStatus==0){
      
    P1_0 = 1;
    
    GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );
    
    SerialPrintString("Discovering...\r\n");
    
    GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                   DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                   DEFAULT_DISCOVERY_WHITE_LIST );
    
    DelayMS(50);
    GAPRole_StartDevice( &masterSlaveSwitch_PeripheralCBs );
    GAPBondMgr_Register( &masterSlaveSwitch_BondMgrCBs );
    P1_0 = 0;
    } 
    return ;
  }
  if(length>=8 && str_cmp(pBuffer+3,"SLAVE",5)==0){
    // Start the Device
    GAPRole_StartDevice( &masterSlaveSwitch_PeripheralCBs );
    // Start Bond Manager
    GAPBondMgr_Register( &masterSlaveSwitch_BondMgrCBs );
    SerialPrintString("slave started ok\r\n");
  }
  if(length>=7 && str_cmp(pBuffer+3,"CON",3)==0){
    uint8 tmp=pBuffer[6]-48-1;//���Ӳ���CON���������ֵ���ڵ���1����˼�����ӵ�һ���ӻ� �ڶ����ӻ�������
    if ( simpleBLEState == BLE_STATE_IDLE ){
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        uint8 addrType;
        uint8 *peerAddr;
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[tmp].addr;
        addrType = simpleBLEDevList[tmp].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  

        SerialPrintString("Connecting:");
        SerialPrintString((uint8*)bdAddr2Str( peerAddr));
        SerialPrintString("\r\n");
      }
    }
    return ;
      
  }
  if(length>=7 && str_cmp(pBuffer+3,"RSSI",4)==0){
    // Start or cancel RSSI polling
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
      }
      else
      {
        simpleBLERssi = FALSE;
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );
        
        SerialPrintString("RSSI Cancelled\r\n");
      }
    }
    return ;
  }
  if(length>=9 && str_cmp(pBuffer+3,"DISCON",6)==0){
    if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
      
      SerialPrintString("Disconnecting\r\n");
    }
  }
  //AT+WRITE0xXX
  if(length>=12 && str_cmp(pBuffer+3,"WRITE",5)==0){
    //uint8 val=0;
    simpleBLECharVal=str2hex(pBuffer+10);
    if ( simpleBLEState == BLE_STATE_CONNECTED &&
              simpleBLECharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;

      // Do a write
      attWriteReq_t req;
      
      req.handle = simpleBLECharHdl;
      req.len = 1;
      req.value[0] = simpleBLECharVal;
      req.sig = 0;
      req.cmd = 0;
      status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );         


      
      if ( status == SUCCESS )
      {
        simpleBLEProcedureInProgress = TRUE;
      }
    } 
    return ;
    
  }
  
  if(length>=10 && str_cmp(pBuffer+3,"CARDNUM",7)==0){
    if ( simpleBLEState == BLE_STATE_IDLE ){
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        itoa(simpleBLEScanRes,temp,16);
        SerialPrintString(temp);
        SerialPrintString("\r\n"); 
        osal_memset(temp, 0, 41);    
      }
    }
  }
  
  if(length>=10 && str_cmp(pBuffer+3,"GETDATA",7)==0){
    if ( simpleBLEState == BLE_STATE_IDLE ){
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        
        for(i=0;i<simpleBLEScanRes;i++){
          for(j=0;j<6;j++){
            itoa(DataUp[i].CardAddr[j],&temp[j*2],16);
          }
          itoa(DataUp[i].Card_Major_H,&temp[12],16);
          itoa(DataUp[i].Card_Major_L,&temp[14],16);
          itoa(DataUp[i].Table_Minor_H,&temp[16],16);
          itoa(DataUp[i].Table_Minor_L,&temp[18],16);

//          sbpSerialAppWrite(temp,21);  //tears......................................
          SerialPrintString(temp);         
          SerialPrintString("\r\n");
          osal_memset(temp, 0, 41);
        }
      }
    }
  } 
  
  if(length>=8 && str_cmp(pBuffer+3,"RESET",5)==0) {
    SerialPrintString("BLE reset!!!\r\n");
    HAL_SYSTEM_RESET(); 
  }
  
  if (length >= 7 && str_cmp(pBuffer + 3, "HOST", 4) == 0) {
    for(i=0;i<WIFI_LIMIT_HOST;i++){
      itoa(server_host[i],&temp[i*2],16);
    }
    SerialPrintString(temp);
    SerialPrintString("\r\n");
    osal_memset(temp, 0, 41);
  }
  
  if (length >= 7 && str_cmp(pBuffer + 3, "PORT", 4) == 0) {
    for(i=0;i<WIFI_LIMIT_PORT;i++){
      itoa(server_port[i],&temp[i*2],16);
    }
    SerialPrintString(temp);
    SerialPrintString("\r\n");
    osal_memset(temp, 0, 41);
  }
  
  if (length >= 7 && str_cmp(pBuffer + 3, "SSID", 4) == 0) {
    SerialPrintString(ap_ssid);
    osal_memset(temp, 0, 41);
  }
  
  if (length >= 11 && str_cmp(pBuffer + 3, "PASSWORD", 8) == 0) {
    SerialPrintString(ap_pass);
    osal_memset(temp, 0, 41);
  }
   
}


unsigned char* itoa(uint8 num,unsigned char* str,uint8 radix)
{
unsigned char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
uint8 i=0,j,unum=0;
unum=num;
do{
str[i++]=index[num%radix];
num/=radix;
}while(num);

if(unum<16){
  str[i++]='0';
}
str[i]='\0';
unsigned char temp;
for(j=0;j<=(i-1)/2;j++)
{
  temp=str[j];
  str[j]=str[i-1-j];
  str[i-1-j]=temp;
}
return str;
}

void IO_Init(void)
{
//  P0SEL = 0x00; // Configure Port 0 as GPIO
  P1SEL = 0x00; // Configure Port 1 as GPIO
  P2SEL = 0x00; // Configure Port 2 as GPIO

//  P0DIR = 0xFF; // All port 0 pins (P0.1-P0.7) as output
  P1DIR = 0xFF; // 11010010
  P2DIR = 0xFF; 
  
//  P0 = 0;
  P1 = 0;
  P2 = 0;
}


