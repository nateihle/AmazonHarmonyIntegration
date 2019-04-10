/*******************************************************************************
  Wi-Fi MAC interface functions

  File Name:
    wdrv_mrf24wn_main.h

  Summary:
    Wi-Fi specific MAC function prototypes called by TCP/IP stack.

  Description:
    Wi-Fi specific MAC function prototypes called by TCP/IP stack.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#ifndef _WDRV_MRF24WN_MAIN_H
#define _WDRV_MRF24WN_MAIN_H

#include "system_config.h"

#include "tcpip/tcpip_mac_object.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_common.h"

#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
#error "TCPIP_STACK_USE_EVENT_NOTIFICATION must be defined for Wi-Fi demos"
#endif

#define WDRV_MAX_CLIENT_TABLE_SLOTS 10

/***********************************************************
  Summary:
    Wi-Fi Power-Saving states

  Description:
    Wi-Fi Power-Saving states

    This enumeration identifies Wi-Fi Power-Saving states.
 */
typedef enum {
    WDRV_PS_HIBERNATE,
    WDRV_PS_SLEEP,
    WDRV_PS_ACTIVE
} WDRV_POWER_SAVE_STATES;

/***********************************************************
  Summary:
    Wi-Fi Connection states

  Description:
    Wi-Fi Connection States

    This enumeration identifies Wi-Fi Connection states. See
    WDRV_CLI_ConnectionStateGet().
 */
typedef enum {
    /* No Wi-Fi connection exists */
    WDRV_CSTATE_NOT_CONNECTED               = 1,

    /* Wi-Fi connection in progress */
    WDRV_CSTATE_CONNECTION_IN_PROGRESS      = 2,

    /* Wi-Fi connected */
    WDRV_CSTATE_CONNECTED                   = 3,

    /* Wi-Fi in process of reconnecting */
    WDRV_CSTATE_RECONNECTION_IN_PROGRESS    = 4,

    /* Wi-Fi connection temporarily lost */
    WDRV_CSTATE_CONNECTION_TEMPORARY_LOST   = 5,

    /* Wi-Fi connection permanently lost */
    WDRV_CSTATE_CONNECTION_PERMANENTLY_LOST = 6
} WDRV_CONNECTION_STATES;

typedef enum {
    WDRV_DISCONNECT_REASON_NO_NETWORK_AVAIL      = 0x01,
    WDRV_DISCONNECT_REASON_LOST_LINK             = 0x02,
    WDRV_DISCONNECT_REASON_DISCONNECT_CMD        = 0x03,
    WDRV_DISCONNECT_REASON_BSS_DISCONNECTED      = 0x04,
    WDRV_DISCONNECT_REASON_AUTH_FAILED           = 0x05,
    WDRV_DISCONNECT_REASON_ASSOC_FAILED          = 0x06,
    WDRV_DISCONNECT_REASON_NO_RESOURCES_AVAIL    = 0x07,
    WDRV_DISCONNECT_REASON_CONNECTION_DENIED     = 0x08,
    WDRV_DISCONNECT_REASON_INVALID_PROFILE       = 0x0A,
    WDRV_DISCONNECT_REASON_PROFILE_MISMATCH      = 0x0C,
    WDRV_DISCONNECT_REASON_CONNECTION_EVICTED    = 0x0D
} WDRV_DISCONNECTION_REASON;

typedef struct {
    uint8_t trafficEvents;
    uint16_t trafficEventInfo;
} WDRV_MRF24WN_USREV_DCPT;

// stack internal notification
typedef struct {
    bool                     mrfEnabledEvents; // group enabled notification events
    volatile TCPIP_MAC_EVENT mrfPendingEvents; // group notification events that are set, waiting to be re-acknowledged
    TCPIP_MAC_EventF         mrfNotifyFnc; // group notification handler
    const void              *mrfNotifyParam; // notification parameter
} WDRV_MRF24WN_EVGROUP_DCPT; // event descriptor

typedef struct {
    uint8_t addr[6];
    uint32_t timeStamp;
} MAC_ADDR;

typedef struct {
    MAC_ADDR mac[WDRV_MAX_CLIENT_TABLE_SLOTS];
    uint16_t bitMap;
    uint32_t seqNum;
    uint16_t updated;
} WDRV_CLIENT_CACHE;

typedef struct {
    uint8_t macAddr[6];
    bool isDriverOpen;
    bool updateMacAddressRequired;
    bool isScanDone;
    bool initConn;
    bool isConnReestablished;
    bool isDisconnectRequested;
    uint8_t wpaCipherRetryCnt;
    OSAL_SEM_HANDLE_TYPE disconnectDoneSync;
    WDRV_CLIENT_CACHE clientCache;
} WDRV_MRF24WN_PRIV;

bool WDRV_CONFIG_Load(void);
void WDRV_CONFIG_Save(void);
void WDRV_CONFIG_Delete(void);

bool isLinkUp();
void WDRV_Connect(void);
void WDRV_Disconnect(bool requested);

uint8_t WDRV_ScanStart(void);

void ProceedConnectEventCB(uint32_t connected, uint8_t devID, uint8_t *mac, bool macConn, uint8_t reason);
void WPSDoneCB(void);

bool isMacInitialized(void);

WDRV_CONNECTION_STATES WDRV_ConnectionState_Get(void);

bool isClientCacheUpdated(bool *connected, uint8_t *mac);

bool isEventPending(void);
void WDRV_PendingEventProcess(void);

void WDRV_TrafficEventInit(TCPIP_MAC_EventF eventF, const void *eventParam);
void WDRV_TrafficEventDeinit(void);
void WDRV_TrafficEventReq(uint16_t event, uint16_t eventInfo);
bool WDRV_TrafficEventMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
bool WDRV_TrafficEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
TCPIP_MAC_EVENT WDRV_TrafficEventGet(TCPIP_MAC_HANDLE hMac);
void WDRV_AllEventClear(void);
void WDRV_EventSet(uint8_t event);

TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, const TCPIP_MAC_ADDR *DestMACAddr);

extern WDRV_CONFIG *gp_wdrv_cfg;
extern WDRV_MRF24WN_PRIV g_wdrv_priv;
extern WDRV_SCAN_STATUS g_wdrv_scanStatus;

#endif /* _WDRV_MRF24WN_MAIN_H */

// DOM-IGNORE-END
