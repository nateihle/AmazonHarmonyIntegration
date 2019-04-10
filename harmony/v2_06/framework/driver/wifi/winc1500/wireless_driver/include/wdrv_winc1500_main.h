/*******************************************************************************
  WINC1500 Wireless Driver Internal Functions

  File Name:
    wdrv_winc1500_main.h

  Summary:
    Internal functions for WINC1500 wireless driver.

  Description:
    Internal functions for WINC1500 wireless driver.
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

#ifndef _WDRV_WINC1500_MAIN_H
#define _WDRV_WINC1500_MAIN_H

#include "system_config.h"

#include "tcpip/tcpip_mac_object.h"

#include "driver/wifi/winc1500/include/wdrv_winc1500_common.h"

#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
#error "TCPIP_STACK_USE_EVENT_NOTIFICATION must be defined for Wi-Fi demos"
#endif

#define WDRV_MAX_CLIENT_TABLE_SLOTS 10

typedef struct {
    uint8_t trafficEvents;
    uint16_t trafficEventInfo;
} WDRV_WINC1500_USREV_DCPT;

// stack internal notification
typedef struct {
    bool                     wincEnabledEvents; // group enabled notification events
    volatile TCPIP_MAC_EVENT wincPendingEvents; // group notification events that are set, waiting to be re-acknowledged
    TCPIP_MAC_EventF         wincNotifyFnc; // group notification handler
    const void              *wincNotifyParam; // notification parameter
} WDRV_WINC1500_EVGROUP_DCPT; // event descriptor

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
    bool isDisconnectRequested;
    OSAL_SEM_HANDLE_TYPE disconnectDoneSync;
    WDRV_CLIENT_CACHE clientCache;
    bool isOtaFwUpdateRequested;
    uint8_t fwOtaServerUrl[128];
} WDRV_WINC1500_PRIV;

bool WDRV_CONFIG_Load(void);
void WDRV_CONFIG_Save(void);
void WDRV_CONFIG_Delete(void);

bool isLinkUp();
void WDRV_Connect(void);
void WDRV_Disconnect(void);

uint8_t WDRV_ScanStart(void);

void ConnectEventCB(bool connected, bool isServer, const uint8_t *const client);
void WPSDoneCB(void);

WDRV_CONNECTION_STATES WDRV_ConnectionState_Get(void);

bool isClientCacheUpdated(bool *connected, uint8_t *mac);
void ClientCacheUpdate(bool connected, const uint8_t *const mac);

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

TCPIP_MAC_RES WDRV_WINC1500_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, const TCPIP_MAC_ADDR *DestMACAddr);

extern WDRV_CONFIG *gp_wdrv_cfg;
extern WDRV_WINC1500_PRIV g_wdrv_priv;
extern WDRV_SCAN_STATUS g_wdrv_scanStatus;

#endif /* _WDRV_WINC1500_MAIN_H */

// DOM-IGNORE-END
