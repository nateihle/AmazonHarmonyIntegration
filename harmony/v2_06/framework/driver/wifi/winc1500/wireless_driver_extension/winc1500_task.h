/*******************************************************************************
  WINC1500 Wireless Driver Main Task

  File Name:
    winc1500_task.h

  Summary:
    Entry point of WINC1500 core driver.

  Description:
    Entry point of WINC1500 core driver.
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
//DOM-IGNORE-END

#ifndef _WINC1500_TASK_H
#define _WINC1500_TASK_H

#include "driver/wifi/winc1500/include/wdrv_winc1500_common.h"

#include "driver/wifi/winc1500/wireless_driver_extension/driver/include/m2m_types.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef void *(*GetRxBufFunc)(void);

typedef struct {
    TaskHandle_t handle;
    OSAL_SEM_HANDLE_TYPE scanResultWait;
    OSAL_SEM_HANDLE_TYPE connInfoWait;
    OSAL_SEM_HANDLE_TYPE eventWait;
    volatile bool deinit_in_progress;
} WINC1500_PRIV;

typedef struct {
    bool ethMode;
    GetRxBufFunc get_rx_buf;
    void (*FrameReceived_CB)(uint32_t len, uint8_t const *const frame);
    void (*ConnectEvent_CB)(bool connected, bool isServer, uint8_t const *const client);
    void (*RFReady_CB)(uint8_t const *const addr);
    void (*ScanDone_CB)(uint32_t status);
    void (*InitDone_CB)(void);
    void (*WPSDone_CB)(void);
    uint8_t *fwOtaServerUrl;
} WINC1500_INTF;

typedef struct {
#define CONN_DATA_UNSPECIFIED -1
    uint8_t ssid[WDRV_MAX_SSID_LENGTH + 1];
    int16_t ssidLen;
    int8_t secType;
    uint8_t key[WDRV_MAX_SECURITY_KEY_LENGTH];
    uint8_t keyLen;
    uint8_t keyIdx;
    uint8_t channel;
    int8_t isSoftAP;
} WINC1500_CONNECT_DATA;

#define winc1500_init_completed() do { if (g_winc1500_intf->InitDone_CB) (*g_winc1500_intf->InitDone_CB)(); } while (0);
#define winc1500_rf_ready(mac) do { if (g_winc1500_intf->RFReady_CB) (*g_winc1500_intf->RFReady_CB)(mac); } while (0);
#define winc1500_wps_completed() do { if (g_winc1500_intf->WPSDone_CB) (*g_winc1500_intf->WPSDone_CB)(); } while (0)
#define winc1500_scan_completed(status) do { if (g_winc1500_intf->ScanDone_CB) (*g_winc1500_intf->ScanDone_CB)(status); } while (0);
#define winc1500_eth_data_received(len, frame) do { if (g_winc1500_intf->FrameReceived_CB) (*g_winc1500_intf->FrameReceived_CB)(len, frame); } while (0)
#define winc1500_connect_event_received(connected, isServer, client) do { if (g_winc1500_intf->ConnectEvent_CB) (*g_winc1500_intf->ConnectEvent_CB)(connected, isServer, client); } while (0);
#define winc1500_get_rx_bufer() do { if(g_winc1500_intf->get_rx_buf) (*g_winc1500_intf->get_rx_buf)(); } while (0);
#define winc1500_eth_data_send(frame, len) m2m_wifi_send_ethernet_pkt(frame, len)
#define winc1500_isr(sem) m2m_hif_isr(sem)

void winc1500_task(void *arg);
void winc1500_task_deinit(void);
void winc1500_task_init(WINC1500_INTF *intf);
int8_t winc1500_connect(WINC1500_CONNECT_DATA *data);
void winc1500_scan_result_read(tstrM2mWifiscanResult *result);
void winc1500_wps_info_read(tstrM2MWPSInfo *info);
void winc1500_conn_info_read(tstrM2MConnInfo *info);
sint8 winc1500_init(void);
sint8 winc1500_deinit(void);
void winc1500_fw_update(uint32_t pContext);

extern WINC1500_INTF *g_winc1500_intf;
extern WINC1500_PRIV g_wdrvext_priv;

#ifdef __cplusplus
}
#endif

#endif /* _WINC1500_TASK_H*/
