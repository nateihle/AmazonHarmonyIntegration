/*******************************************************************************
  WINC1500 Wireless Driver Event Handler

  File Name:
    wdrv_winc1500_events.c

  Summary:
    Event handler for WINC1500 wireless driver.

  Description:
    Event handler for WINC1500 wireless driver.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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

#include "system/int/sys_int.h"

#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_main.h"

volatile static uint8_t s_pendingEventFlags;
static WDRV_WINC1500_EVGROUP_DCPT s_wincGroupDcpt =
{
    TCPIP_MAC_EV_NONE, TCPIP_MAC_EV_NONE, 0
};
static WDRV_WINC1500_USREV_DCPT s_wincUsrEvent; // stack user events

void WDRV_TrafficEventInit(TCPIP_MAC_EventF eventF, const void *eventParam)
{
    s_wincGroupDcpt.wincNotifyFnc = eventF; // set new handler
    s_wincGroupDcpt.wincNotifyParam = eventParam;
    s_wincGroupDcpt.wincEnabledEvents = false;
    s_wincGroupDcpt.wincPendingEvents = 0;

    s_wincUsrEvent.trafficEvents = 0;
    s_wincUsrEvent.trafficEventInfo = 0;
}

void WDRV_TrafficEventDeinit(void)
{
    SYS_INT_SourceDisable(WINC1500_INT_SOURCE);
    SYS_INT_SourceStatusClear(WINC1500_INT_SOURCE);

    s_wincGroupDcpt.wincNotifyFnc = 0;
    s_wincGroupDcpt.wincEnabledEvents = false;
    s_wincGroupDcpt.wincPendingEvents = 0;
}

bool WDRV_TrafficEventMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    if (enable) {
        s_wincGroupDcpt.wincEnabledEvents = true;
        SYS_INT_SourceEnable(WINC1500_INT_SOURCE);
    } else {
        SYS_INT_SourceDisable(WINC1500_INT_SOURCE);
        s_wincGroupDcpt.wincEnabledEvents = false;
    }

    return true;
}

bool WDRV_TrafficEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    if(s_wincGroupDcpt.wincPendingEvents) {
        s_wincGroupDcpt.wincPendingEvents = 0;
        return true;
    } else {
        return false;
    }
}

TCPIP_MAC_EVENT WDRV_TrafficEventGet(TCPIP_MAC_HANDLE hMac)
{
    return s_wincGroupDcpt.wincPendingEvents;
}

void WDRV_TrafficEventReq(uint16_t event, uint16_t eventInfo)
{
    s_wincUsrEvent.trafficEvents = event;
    s_wincGroupDcpt.wincPendingEvents = event; // add this line so event function sees event?
    s_wincUsrEvent.trafficEventInfo = eventInfo;

    // let app know of event
    if (s_wincGroupDcpt.wincNotifyFnc)
        (*s_wincGroupDcpt.wincNotifyFnc)(s_wincGroupDcpt.wincPendingEvents, s_wincGroupDcpt.wincNotifyParam);
}

// called by TCPIP_STACK_Task() if any Wi-Fi event is pending
void WDRV_PendingEventProcess(void)
{
    WDRV_DBG_TRACE_PRINT("wdrv async task\r\n");
}

bool isEventPending(void)
{
    return s_pendingEventFlags;
}

void WDRV_AllEventClear(void)
{
    s_pendingEventFlags = 0x00;
}

// sets an event bit
void WDRV_EventSet(uint8_t event)
{
    s_pendingEventFlags |= event;
}

void WDRV_EventClear(uint8_t event)
{
    s_pendingEventFlags &= ~event;
}

//DOM-IGNORE-END
