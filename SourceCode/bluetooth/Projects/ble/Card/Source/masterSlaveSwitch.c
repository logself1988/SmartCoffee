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
  PROVIDED S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
// Profile文件，这里使用的和SimpleProfile相同，仅仅是修改了部分名字
#include "exampleService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// What is the advertising interval when device is discoverable (units of 625us,
// 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL 3200

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#define DEFAULT_DISCOVERABLE_MODE GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 1600

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY 0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT 1000

#define INVALID_CONNHANDLE 0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN 15

#if defined(PLUS_BROADCASTER)
#define ADV_IN_CONN_WAIT 500 // delay 500 ms
#endif

/***************************** CENTRAL defines ***********************/

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY 1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID TRUE

#define WANTED_SERVICE_UUID 0xFFF0

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES 8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION 520

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD 300           //changed from 500ms to 300ms

/*********************************************************************
 * TYPEDEFS
 */
typedef enum { ROLE_PERIPHERAL = 1, ROLE_CENTRAL = 2 } deviceRole_t;

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
typedef enum {
  BLE_DISC_STATE_IDLE, // Idle
  BLE_DISC_STATE_SVC,  // Service discovery
  BLE_DISC_STATE_CHAR  // Characteristic discovery
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

static uint8 advType2 = GAP_ADRPT_ADV_NONCONN_IND;

static uint8 RSSI_Threshold = 85;
static uint8 ROLE_Flags     = 0;

static uint8 gPairStatus = 0; /*用来管理当前的状态，如果密码不正确，立即取消连接，0表示未配对，1表示已配对*/
static void ProcessPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle,
                              uint8 uiInputs, uint8 uiOutputs);
static void ProcessPairStateCB(uint16 connHandle, uint8 state, uint8 status);

static uint8 masterSlaveSwitch_TaskID; // Task ID for internal task/event processing

static gaprole_States_t gapPeripheralState     = GAPROLE_INIT;
static gapCentralRole_States_t gapCentralState = GAPROLE_CENTRAL_UNITIALIZED;

// jackpe, devices' rssi
static uint8 deviceRSSI[DEFAULT_MAX_SCAN_RES] = {100, 100, 100, 100,
                                                 100, 100, 100, 100};
static uint8 smallestRSSIIndex;
static uint16 tableMinor[DEFAULT_MAX_SCAN_RES] = {0};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] = {
    // complete name
    0x05, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'C', 'A', 'R',
    'D', // jackpe, M is the 2st index.
    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// jackpe
static uint8 advertData[] = {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // in this peripheral
    0x1A, // length of this data 26byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    /*Apple Pre-Amble*/
    0x4C, 0x00, 0x02, 0x15,
    /*Device UUID (16 Bytes)*/
    0x47, 0xA9, 0x30, 0x6E, 0x88, 0xE8, 0x96, 0x78, 0xCA, 0x74, 0xDE, 0x07,
    0x19, 0x8A, 0xB3, 0x9B,

    0x00, 0x05,

    /*Minor Value (2 Bytes)*/
    0x00, 0x00,

    /*Measured Power*/
    0xCD};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "CARD";

// Initial State
static deviceRole_t deviceRole = ROLE_PERIPHERAL;

// Connection handle of active connection
static uint16 connHandle = INVALID_CONNHANDLE;

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
// static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl   = 0;

// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;

// Value to write
static uint8 simpleBLECharVal[SIMPLEPROFILE_CHAR1_LEN] = {0};

// Value read/write toggle
static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void masterSlaveSwitch_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gaprole_States_t newState);
//static void masterSlaveSwitch_HandleKeys(uint8 shift, uint8 keys);

static void simpleBLECentralRssiCB(uint16 connHandle, int8 rssi);
// static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );

static void simpleBLEGATTDiscoveryEvent(gattMsgEvent_t *pMsg);
static void simpleBLECentralEventCB(gapCentralRoleEvent_t *pEvent);
static void simpleBLECentralProcessGATTMsg(gattMsgEvent_t *pMsg);
static void simpleBLECentralStartDiscovery(void);

// static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo(uint8 *pAddr, uint8 addrType);

static void exampleServiceChangeCB(uint8 paramID);
char *bdAddr2Str(uint8 *pAddr);
static void determineRSSI(void);
static void updateAdvData(uint16 minor);

static void IO_Init(void);
uint8 abs(int8 x);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks PERIPHERAL
static gapRolesCBs_t masterSlaveSwitch_PeripheralCBs = {
    peripheralStateNotificationCB, // Profile State Change Callbacks
    NULL // When a valid RSSI is read from controller (not used by application)
};


// GAP Role Callbacks CENTRAL
static const gapCentralRoleCB_t simpleBLERoleCB = {
    simpleBLECentralRssiCB, // RSSI callback
    simpleBLECentralEventCB // Event callback
};

// Example Service Callbacks
static simpleProfileCBs_t masterSlaveSwitch_exampleServiceCBs = {
    exampleServiceChangeCB // Charactersitic value change callback
};

//要实现密码绑定，首先需要实现bongmgr的回调函数。
// GAP Bond Manager Callbacks
static gapBondCBs_t masterSlaveSwitch_BondMgrCBs = {
    ProcessPasscodeCB, // 密码回调
    ProcessPairStateCB // 绑定状态回调
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
        delay function, seems no impact on the osal stack
  */

void DelayMS(uint16 msec)
{
  uint16 i, j;
  for (i = 0; i < msec; i++)
    for (j = 0; j < 536; j++)
      ;
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
void MasterSlaveSwitch_Init(uint8 task_id)
{

  
  IO_Init();

  P1_6 = 1;
  DelayMS(500);
  P1_6 = 0;
  DelayMS(500);
  P1_6 = 1;
  DelayMS(500);

  HCI_EXT_ExtendRfRangeCmd();
//  TXPOWER = 0xB1;
//  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);

  uint8 newChar1[SIMPLEPROFILE_CHAR1_LEN] = {0, 0};

  HalFlashRead(0x78, 0, newChar1, 2);
  advertData[25] = newChar1[0];
  advertData[26] = newChar1[1];
  
  
  //Debug
//  advertData[25] = 0;
//  advertData[26] = 0xA;
  

  HalFlashRead(0x79, 0, newChar1, 1);
  RSSI_Threshold = newChar1[0];

  HalFlashRead(0x7A, 0, newChar1, 1);
  ROLE_Flags = newChar1[0];

//    ROLE_Flags=1;//////////////debug

  if (ROLE_Flags == 1) {

    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advType2);
  }

  masterSlaveSwitch_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {

    // For other hardware platforms, device starts advertising upon
    // initialization
    uint8 initial_advertising_enable =
        FALSE; // We set this to FALSE to bypass peripheral.c's auto-start

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request  = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval  = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval  = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout  = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16),
                         &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8),
                         &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16),
                         &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16),
                         &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16),
                         &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16),
                         &desired_conn_timeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 123456; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8 mitm     = TRUE;
    uint8 ioCap    = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &bonding);
  }

  // jackpe, add charicteristic init
  // Setup the SimpleProfile Characteristic Values
  {
    // uint8 charValue1 = 1;
    // uint8 charValue2 = 2;
    // uint8 charValue3 = 3;
    // uint8 charValue4 = 4;
    uint8 charValue1[SIMPLEPROFILE_CHAR1_LEN] = {0};
    uint8 charValue2                          = 0;
    uint8 charValue3                          = 0;
    uint8 charValue4                          = 0;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN,
                               charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8), &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8), &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8), &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  DevInfo_AddService();                      // Device Information Service

  // Proprietary service

  SimpleProfile_AddService(GATT_ALL_SERVICES);
  SimpleProfile_RegisterAppCBs(&masterSlaveSwitch_exampleServiceCBs);

  // Receive key presses
  RegisterForKeys(masterSlaveSwitch_TaskID);
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd(HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT);

#if defined(DC_DC_P0_7)
  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd(HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7);
#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event(masterSlaveSwitch_TaskID, MSS_START_DEVICE_EVT);

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8),
                                &scanRes);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  P1_6 = 0;
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
uint16 MasterSlaveSwitch_ProcessEvent(uint8 task_id, uint16 events)
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if (events & SYS_EVENT_MSG) {
    uint8 *pMsg;

    if ((pMsg = osal_msg_receive(masterSlaveSwitch_TaskID)) != NULL) {
      masterSlaveSwitch_ProcessOSALMsg((osal_event_hdr_t *)pMsg);

      // Release the OSAL message
      VOID osal_msg_deallocate(pMsg);
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  //////////////////////////////////////////////////////

  if (events & MSS_START_DEVICE_EVT) {

    if (ROLE_Flags == 1) {
      osal_start_timerEx(masterSlaveSwitch_TaskID, MSS_SWITCH_MODE, 20000);
    }
    // Start the Device
    VOID GAPRole_StartDevice(&masterSlaveSwitch_PeripheralCBs);

    // Start Bond Manager
    // jackpe, not need this
    VOID GAPBondMgr_Register(&masterSlaveSwitch_BondMgrCBs);

    return (events ^ MSS_START_DEVICE_EVT);
  }

  ///////////////////////////////////////////////////////

  if (events & MSS_CHANGE_ROLE_EVT) {
    if (deviceRole == ROLE_PERIPHERAL) {

      VOID GAPRole_StartDevice(&masterSlaveSwitch_PeripheralCBs);
      TXPOWER = 0xC1;
      HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);

    } else if (deviceRole == ROLE_CENTRAL) {
      VOID GAPCentralRole_StartDevice((gapCentralRoleCB_t *)&simpleBLERoleCB);
      // Start or stop discovery
      if (gapCentralState != GAPROLE_CENTRAL_CONNECTED) {
        if (!simpleBLEScanning) {
          simpleBLEScanning = TRUE;
          simpleBLEScanRes  = 0;

          //          HalLcdWriteString( "Discovering...", HAL_LCD_LINE_1 );
          //          HalLcdWriteString( "", HAL_LCD_LINE_2 );
          
          TXPOWER = 0xA1;
          HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);

          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        } else {
          GAPCentralRole_CancelDiscovery();
        }
      }
    }

    return (events ^ MSS_CHANGE_ROLE_EVT);
  }

  /////////////////////////////////////////////////////////////////

  if (events & START_DISCOVERY_EVT) {
    simpleBLECentralStartDiscovery();

    return (events ^ START_DISCOVERY_EVT);
  }

  /////////////////////////////////////////////////////////////////

  if (events & DISCOVERY_DONE_EVT) {
    determineRSSI();
    return (events ^ DISCOVERY_DONE_EVT);
  }

  ////////////////////////////////////////////////////////////////

  if (events & MSS_SWITCH_MODE) {
    asm("nop");
    if (deviceRole == ROLE_PERIPHERAL) {
      deviceRole               = ROLE_CENTRAL;
      uint8 adv_enabled_status = FALSE;
      // Disable advertising if active
      if (gapPeripheralState == GAPROLE_ADVERTISING) {
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                             &adv_enabled_status); // To stop advertising
        // Role will be switched on Peripheral callback
      } else if (gapPeripheralState == GAPROLE_CONNECTED) {
        GAPRole_SetParameter(
            GAPROLE_ADVERT_ENABLED, sizeof(uint8),
            &adv_enabled_status); // To avoid auto-reenabling of advertising
        GAPRole_TerminateConnection();
        // Role will be switched on Peripheral callback
      } else {
        osal_set_event(masterSlaveSwitch_TaskID, MSS_CHANGE_ROLE_EVT);
      }
    } else if (deviceRole == ROLE_CENTRAL) {
      deviceRole = ROLE_PERIPHERAL;
      if (gapCentralState == GAPROLE_CENTRAL_INIT_DONE ||
          gapCentralState == GAPROLE_CENTRAL_DISCONNECTED) {
        osal_set_event(masterSlaveSwitch_TaskID, MSS_CHANGE_ROLE_EVT);
      } else {
        // disconnect all
        GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);
        // Expect the Central disconnect callback to switch the role
      }
    }
    return (events ^ MSS_SWITCH_MODE);
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
static void masterSlaveSwitch_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch (pMsg->event) {
  case GATT_MSG_EVENT:
    simpleBLECentralProcessGATTMsg((gattMsgEvent_t *)pMsg);
  default:
    // do nothing
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
static void peripheralStateNotificationCB(gaprole_States_t newState)
{
  switch (newState) {
  case GAPROLE_STARTED: {
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
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                         &advertising_enable);

    GAP_UpdateAdvertisingData(masterSlaveSwitch_TaskID, TRUE, 30, advertData);

    // Set up advertising to start
    advertising_enable = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                         &advertising_enable);
  } break;

  case GAPROLE_ADVERTISING: {

  } break;

  case GAPROLE_CONNECTED: {

  } break;

  case GAPROLE_WAITING:               // Disconnected or stopped adv
  case GAPROLE_WAITING_AFTER_TIMEOUT: // Disconnected due to superv. timeout
  {

    if (deviceRole == ROLE_CENTRAL) {
      // We are going to change role. So do that.
      osal_set_event(masterSlaveSwitch_TaskID, MSS_CHANGE_ROLE_EVT);
    }

  } break;

  case GAPROLE_ERROR: {
  } break;

  default: {
  } break;
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
  uint8 newValue[10];
  uint8 newChar1[4];

  switch (paramID) {

  case SIMPLEPROFILE_CHAR1:
    osal_memset(newValue, 0, 10);
    osal_memset(newChar1, 0, 4);
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newValue);
    advertData[25] = newValue[0];
    advertData[26] = newValue[1];

    HalFlashErase(0x78);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF000, newValue, 1);
    HalFlashRead(0x78, 0, newChar1, 2);
    advertData[25] = newChar1[0];
    advertData[26] = newChar1[1];

    break;
  case SIMPLEPROFILE_CHAR2:
    osal_memset(newValue, 0, 10);
    osal_memset(newChar1, 0, 4);
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR2, newValue);
    RSSI_Threshold = newValue[0];

    HalFlashErase(0x79);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF200, newValue, 1);
    HalFlashRead(0x79, 0, newChar1, 1);
    RSSI_Threshold = newChar1[0];

    break;
  case SIMPLEPROFILE_CHAR3:
    osal_memset(newValue, 0, 10);
    osal_memset(newChar1, 0, 4);
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, newValue);

    HalFlashErase(0x7B);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF600, newValue, 4);
    HalFlashRead(0x7B, 0, newChar1, 4);


    break;

  case SIMPLEPROFILE_CHAR4:
    osal_memset(newValue, 0, 10);
    osal_memset(newChar1, 0, 4);
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR4, newValue);
    ROLE_Flags = newValue[0];
    HalFlashErase(0x7A);
    while (FCTL & 0x80)
      ; // wait for erase to complete
    HalFlashWrite(0xF400, newValue, 1);
    HalFlashRead(0x7A, 0, newChar1, 1);
    ROLE_Flags = newChar1[0];

    if (ROLE_Flags == 1) HAL_SYSTEM_RESET();
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
static void simpleBLECentralStartDiscovery(void)
{
  uint8 uuid[ATT_BT_UUID_SIZE] = {LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                  HI_UINT16(SIMPLEPROFILE_SERV_UUID)};

  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;

  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID(simpleBLEConnHandle, uuid, ATT_BT_UUID_SIZE,
                                masterSlaveSwitch_TaskID);
}
/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg(gattMsgEvent_t *pMsg)
{

  if ((pMsg->method == ATT_READ_RSP) ||
      ((pMsg->method == ATT_ERROR_RSP) &&
       (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ))) {
    if (pMsg->method == ATT_ERROR_RSP) {
      uint8 status = pMsg->msg.errorRsp.errCode;

    } else {
      // After a successful read, display the read value
      uint8 valueRead[SIMPLEPROFILE_CHAR1_LEN] = {pMsg->msg.readRsp.value[0],
                                                  pMsg->msg.readRsp.value[1]};

    }

    simpleBLEProcedureInProgress = FALSE;
  } else if ((pMsg->method == ATT_WRITE_RSP) ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ))) {

    if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP) {
      uint8 status = pMsg->msg.errorRsp.errCode;

    } else {
    }

    simpleBLEProcedureInProgress = FALSE;

  } else if (simpleBLEDiscState != BLE_DISC_STATE_IDLE) {
    simpleBLEGATTDiscoveryEvent(pMsg);
  }
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent(gattMsgEvent_t *pMsg)
{
  attReadByTypeReq_t req;

  if (simpleBLEDiscState == BLE_DISC_STATE_SVC) {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0) {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl =
          pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }

    // If procedure complete
    if ((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->hdr.status == bleProcedureComplete) ||
        (pMsg->method == ATT_ERROR_RSP)) {
      if (simpleBLESvcStartHdl != 0) {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;

        req.startHandle  = simpleBLESvcStartHdl;
        req.endHandle    = simpleBLESvcEndHdl;
        req.type.len     = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID(simpleBLEConnHandle, &req,
                               masterSlaveSwitch_TaskID);
      }
    }
  } else if (simpleBLEDiscState == BLE_DISC_STATE_CHAR) {
    // Characteristic found, store handle
    if (pMsg->method == ATT_READ_BY_TYPE_RSP &&
        pMsg->msg.readByTypeRsp.numPairs > 0) {
      simpleBLECharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.dataList[0],
                                      pMsg->msg.readByTypeRsp.dataList[1]);

      //      HalLcdWriteString( "Simple Svc Found", HAL_LCD_LINE_1 );
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
static void simpleBLECentralRssiCB(uint16 connHandle, int8 rssi) {}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode) {
  case GAP_DEVICE_INIT_DONE_EVENT: {

    gapCentralState = GAPROLE_CENTRAL_INIT_DONE;
  } break;

  case GAP_DEVICE_INFO_EVENT: {
    // static uint8 tempdeviceRSSI[DEFAULT_MAX_SCAN_RES];

    // if filtering device discovery results based on service UUID
    //        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )

    // find adv

    if (pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND) // adv
    {

      if (pEvent->deviceInfo.pEvtData[9] == 0x4A &&              //Should be 4A
          pEvent->deviceInfo.pEvtData[10] == 0xA8 &&             //Should be A8
          pEvent->deviceInfo.pEvtData[11] == 0x31) {             //Should be 31

            if((abs(pEvent->deviceInfo.rssi)) < RSSI_Threshold){

        simpleBLEAddDeviceInfo(pEvent->deviceInfo.addr,
                               pEvent->deviceInfo.addrType);

//        deviceRSSI[simpleBLEScanRes - 1] = 65536 - pEvent->deviceInfo.rssi;
        deviceRSSI[simpleBLEScanRes - 1] = abs(pEvent->deviceInfo.rssi);
        tableMinor[simpleBLEScanRes - 1] =
            ((uint16)pEvent->deviceInfo.pEvtData[27]) << 8;
        tableMinor[simpleBLEScanRes - 1] =
            tableMinor[simpleBLEScanRes - 1] |
            (uint16)pEvent->deviceInfo.pEvtData[28];

                    }
      }
    }

  } break;

  case GAP_DEVICE_DISCOVERY_EVENT: {
    // discovery complete
    simpleBLEScanning = FALSE;

    // if not filtering device discovery results based on service UUID
    if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE) {
      // Copy results
      simpleBLEScanRes = pEvent->discCmpl.numDevs;
      osal_memcpy(simpleBLEDevList, pEvent->discCmpl.pDevList,
                  (sizeof(gapDevRec_t) * pEvent->discCmpl.numDevs));
    }

    // initialize scan index to last device
    simpleBLEScanIdx = simpleBLEScanRes;

    // jackpe, set event DISCOVERY_DONE
    osal_set_event(masterSlaveSwitch_TaskID, DISCOVERY_DONE_EVT);
  } break;

  case GAP_LINK_ESTABLISHED_EVENT: {
    if (pEvent->gap.hdr.status == SUCCESS) {
      simpleBLEProcedureInProgress = TRUE;

      gapCentralState = GAPROLE_CENTRAL_CONNECTED;
      connHandle      = pEvent->linkCmpl.connectionHandle;
      // ghostyu add
      // If service discovery not performed initiate service discovery
      if (simpleBLECharHdl == 0) {
        osal_start_timerEx(masterSlaveSwitch_TaskID, START_DISCOVERY_EVT,
                           DEFAULT_SVC_DISCOVERY_DELAY);
      }
    } else {
      gapCentralState = GAPROLE_CENTRAL_DISCONNECTED;

      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      connHandle         = INVALID_CONNHANDLE;
    }

  } break;

  case GAP_LINK_TERMINATED_EVENT: {
    gapCentralState = GAPROLE_CENTRAL_DISCONNECTED;
    connHandle      = INVALID_CONNHANDLE;
    // simpleBLERssi = FALSE;
    simpleBLEDiscState           = BLE_DISC_STATE_IDLE;
    simpleBLECharHdl             = 0;
    simpleBLEProcedureInProgress = FALSE;


    // Start role switching if applicable
    if (deviceRole == ROLE_PERIPHERAL) {
      osal_set_event(masterSlaveSwitch_TaskID, MSS_CHANGE_ROLE_EVT);
    } else {
      // Do normal Central link terminated tasks. E.g restart scanning.
    }
  } break;

  case GAP_LINK_PARAM_UPDATE_EVENT: {
  } break;

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
static void simpleBLEAddDeviceInfo(uint8 *pAddr, uint8 addrType)
{
  uint8 i;

  // If result count not at max
  if (simpleBLEScanRes < DEFAULT_MAX_SCAN_RES) {
    // Check if device is already in scan results
    for (i = 0; i < simpleBLEScanRes; i++) {
      if (osal_memcmp(pAddr, simpleBLEDevList[i].addr, B_ADDR_LEN)) {
        return;
      }
    }

    // Add addr to scan result list
    osal_memcpy(simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN);
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;

    // Increment scan result count
    simpleBLEScanRes++;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str(uint8 *pAddr)
{
  uint8 i;
  char hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for (i = B_ADDR_LEN; i > 0; i--) {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

#define N_MINORS 9
uint16 minors[N_MINORS] = {0};

static uint16 minors_next()
{
  uint16 ch;
  size_t times = 0;
  size_t i;
  for (i = 0; i < N_MINORS; i++) {
    if (times == 0)
    {
      ch    = minors[i];
      times = 1;
    } else if (ch == minors[i])
      times++;
    else
      times--;
  }
  return ch;
}

static void minors_push(uint16 m)
{
  for (int i = 0; i < N_MINORS - 1; i++) {
    minors[i] = minors[i + 1];
  }
  minors[N_MINORS - 1] = m;
}

static void determineRSSI(void)
{
  uint8 i;
  uint8 smallestRSSI;
  uint16 minor = 0;

  if (simpleBLEScanRes) {
    smallestRSSI      = deviceRSSI[0];
    smallestRSSIIndex = 0;
    for (i = 0; i < simpleBLEScanRes; i = i + 1) {
      if (deviceRSSI[i] < smallestRSSI) {
        smallestRSSI      = deviceRSSI[i];
        smallestRSSIIndex = i;
      }
    }
    minor = tableMinor[smallestRSSIIndex];
  }

  minors_push(minor);
  minor = minors_next();
  updateAdvData(minor);

  deviceRole       = ROLE_PERIPHERAL;
  simpleBLEScanRes = 0;
  if (gapCentralState == GAPROLE_CENTRAL_INIT_DONE ||
      gapCentralState == GAPROLE_CENTRAL_DISCONNECTED) {
    osal_set_event(masterSlaveSwitch_TaskID, MSS_CHANGE_ROLE_EVT);
    osal_start_timerEx(masterSlaveSwitch_TaskID, MSS_SWITCH_MODE, 20000);
    //                HAL_SYSTEM_RESET()
  } else {
    // disconnect all
    GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);
    // Expect the Central disconnect callback to switch the role
  }
}

static void updateAdvData(uint16 minor)
{
  uint8 minor_1 = (uint8)(minor >> 8);
  uint8 minor_0 = (uint8)(minor);

  advertData[28] = minor_0;
  advertData[27] = minor_1;
}

//绑定过程中的密码管理回调函数
/*********************************************************************
 * passcode
 */
static void ProcessPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle,
                              uint8 uiInputs, uint8 uiOutputs)
{
  uint32 passcode;
  passcode = 123456;
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

//绑定过程中的状态管理，在这里可以设置标志位，当密码不正确时不允许连接
/*********************************************************************
 * passcode error
 */
static void ProcessPairStateCB(uint16 connHandle, uint8 state, uint8 status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED) {
    gPairStatus = 0;
  } else if (state == GAPBOND_PAIRING_STATE_COMPLETE) {
    if (status == SUCCESS) {
      gPairStatus = 1;
    } else {
      if (status == SMP_PAIRING_FAILED_UNSPECIFIED) {
        gPairStatus = 1;
      } else {
        gPairStatus = 0;
      }
    }
    //判断配对结果，如果不正确立刻停止连接
    if (gPairStatus != 1) {
      GAPRole_TerminateConnection();
    }
  }
}

void IO_Init(void)
{
  P0SEL = 0x00; // Configure Port 0 as GPIO
  P1SEL = 0x00; // Configure Port 1 as GPIO
  P2SEL = 0x00; // Configure Port 2 as GPIO

  P0DIR = 0xFF; // All port 0 pins (P0.1-P0.7) as output
  P1DIR = 0xFF; // 11010010
  P2DIR = 0xFF; 
  
  P0 = 0;
  P1 = 0;
  P2 = 0;
}

uint8 abs(int8 x)
{return x>=0?x:-x ;}
