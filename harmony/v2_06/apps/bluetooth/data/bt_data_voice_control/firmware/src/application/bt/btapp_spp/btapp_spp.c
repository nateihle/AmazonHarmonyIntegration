/*******************************************************************************
    BT SPP Application

  Company:
    Microchip Technology Inc.

  File Name:
    btapp_spp.c

  Summary:
    Contains the functional implementation of SPP application.

  Description:
    This file contains the functional implementation of SPP application.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"
//#include "gfx_resources.h"
#include "btapp_spp.h"
#include "btapp_control.h"
#include "btapp_spp_link.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
//#define BT_MAX_PORTS_SPP 7
static BTAPP_STATUS    mAppStatus = BTAPP_STATUS_IDLE;
static bt_spp_port_t*  mPort = NULL;
static char            mConnected = 0;
static char            mConnecting;
static char            mSending = 0;
static char            mReceiving = 0;
static bt_int          mBytesReceived;
//static bt_uint         mProgConnect = 1;
//static bt_uint         mTimerTick=0;
//static bt_uint         mReConnectTick=0;
//static bt_uint         mNewConnectTick=15;
//static bt_int          mTempTimerFlag =0;
//static bt_bool         mRateChange= BT_FALSE;
//static bt_int          mTempCallbackRate = TEMP_CALLBACK_RATE;
static char            mTxBuffer[BUFFER_SIZE]={0};
static char            mRxBuffer[BUFFER_SIZE]={0};
bt_data_port androidPort[BT_MAX_PORTS_SPP];

#define MIN_SNIFF_INTERVAL       6
#define MAX_SNIFF_INTERVAL       (100 * 16 / 10) // 100 ms / 0.625 ms
#define SNIFF_ATTEMPT_SLOTS      4 //(MAX_SNIFF_INTERVAL >> 1)
#define SNIFF_TIMEOUT            0
#define SNIFF_SUBRATE_LATENCY    (640 * 16 / 10) // 640 ms / 0.625 ms
#define SNIFF_SUBRATE_TIMEOUT    (10 * 1000 * 16 / 10) // 10000 ms / 0.625 ms, 10 s
static const bt_uint mDeviceUUIDs[] =
{
    SDP_CLSID_AUDIO_SINK,
    SDP_CLSID_AV_REMOTE_CONTROL_TARGET,
    SDP_CLSID_ADVANCED_AUDIO_DISTRIBUTION,
    SDP_CLSID_AV_REMOTE_CONTROL,
    SDP_CLSID_AV_REMOTE_CONTROL_CONTROLLER
};

bt_hci_ctrl_listener_t mLinkIdleListener;
bt_hci_ctrl_listener_t mLinkBusyListener;
bt_hci_ctrl_listener_t mScanModeListener;
bt_hci_ctrl_listener_t mConnectRequestListener;
bt_hci_ctrl_listener_t mDisconnectCompleteListener;

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Protoypes
// *****************************************************************************
static void sysStartCallback(bt_bool success, void* param);
//static void sppStateCallback(bt_spp_port_t* port, bt_spp_port_event_e evt, void* param);
static void sppSendCallback(bt_spp_port_t* port, bt_ulong bytesSent,bt_spp_send_status_e result, void* param);
static void initCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void processReceivedData(void);
static void setConnecting(char connecting);
//static void timerCallBack(void);
//static void appTimerCallback(void);

//int btapp_spp_send(const unsigned char* data, unsigned int size);

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Implementation
// *****************************************************************************
/*
static void appTimerCallback(void)
{
   
}

static void timerCallBack(void)
{
    
}
 * */
void btapp_init(void)
{
    mAppStatus = BTAPP_STATUS_INITIALIZED;
}

BTAPP_STATUS btapp_getStatus(void)
{
    return mAppStatus;
}

void btapp_start(void)
{
    bt_sys_init();
    bt_spp_init();
    bt_sys_start(BT_TRUE, BT_TRUE, sdp_db_spp, sdp_db_spp_len, &sysStartCallback, NULL);
}

void btapp_spp_reconnect(bt_uint newConnectTick, bt_uint reConnectTick)
{
    
}

static void sysStartCallback(bt_bool success, void* param)
{
    assert(success);
//    bt_bdaddr_t bdaddr;
//    bt_bool paired;
    if(success)
    {
        mAppStatus = BTAPP_STATUS_STARTED;
        // Enable sniff mode
        bt_hci_write_default_link_policy_settings(4, &initCallback);
    }
}

/*  REPLACED in btapp_control.c
static void sppStateCallback(bt_spp_port_t* port, bt_spp_port_event_e evt, void* param)
{
//    bt_bdaddr_t *mLastDeviceAddress;
    switch (evt)
    {
        case SPP_PORT_EVENT_CONNECTION_FAILED:
            break;
        case SPP_PORT_EVENT_CONNECTED:
            setConnected(1);
            mProgConnect = 0;
            mSending = 0;
            mReceiving = 1;
            bt_spp_receive(mPort, mRxBuffer, sizeof(mRxBuffer), &sppReceiveCallback);
            BT_DISPLAY_STATS.DisplayUpdate = 1;
            BT_DISPLAY_STATS.BlueTooth_Status = Paired_Connected;

            break;
        case SPP_PORT_EVENT_DISCONNECTED:
            setConnected(0);
            mSending = 0;
            mReceiving = 0;
            mProgConnect = 1;
            bttask_pal_setetAddrsAssigned();

            BT_DISPLAY_STATS.DisplayUpdate = 1;
            BT_DISPLAY_STATS.BlueTooth_Status = Paired_NotConnected;

            break;
            
        case SPP_PORT_EVENT_SEND_PROGRESS:
            break;
            
        default:
            break;
    }
}
*/

/*This callback function is used to receive data sent from spp pro app to the bluetooth development kit.*/
//static void sppReceiveCallback(bt_spp_port_t* port, bt_int bytesReceived, void* param)
//{
//    mReceiving = 0;
//    mBytesReceived = bytesReceived;
//    port == androidPort[i].port
//    if (!mSending)
//    {
//            processReceivedData();
//    }
//}

static void sppSendCallback(bt_spp_port_t* port, bt_ulong bytesSent, bt_spp_send_status_e result, void* param)
{
    mSending = 0;
    if (mBytesReceived)
    {
        processReceivedData();
    }

}

//static void display_rxdata(char* data_buff, bt_int data_len)
//{
//
//}


/*This function copies the received data to a buffer and parses the data to perform user defined operations*/
static void processReceivedData(void)
{
//    bt_int i;
    bt_int bytesReceived = mBytesReceived;
    char CommandBuff[BUFFER_SIZE]={0};
//    int Command, Size, Red, Green, Blue;
    mRxBuffer[bytesReceived] = '\0';
    memcpy(mTxBuffer, mRxBuffer, bytesReceived);
    memcpy(CommandBuff, mRxBuffer, (bytesReceived +1))  ;

//    mRxBuffer[bytesReceived] = '\n'; // adds new line
    BT_DISPLAY_STATS.DisplayUpdate = 1;
    BT_DISPLAY_STATS.PROCESS_TEXT = 1;
    memcpy(BT_DISPLAY_STATS.RXBUFFER_DATA, CommandBuff, (bytesReceived +1)); //+1 here to grab the Null
    TakeAction(CommandBuff);
    mSending = 0;
    /*********************************/
    mSending = 1;
    bt_spp_send(mPort, mTxBuffer, bytesReceived, &sppSendCallback);
    mBytesReceived = 0;
    if (!mReceiving)
    {
        mReceiving = 1;
        bt_spp_receive(mPort, mRxBuffer, sizeof(mRxBuffer), &sppReceiveCallback);
    }
}

void setConnected(char connected)
{
    mConnected = connected;

}

static void setConnecting(char connecting)
{
    mConnecting = connecting;
}

void btapp_onButtonDown(bt_uint button, bt_uint repeatCount)
{
    if (button == BTAPP_BUTTON_S1 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 1\r\n");
        mSending = 1;
        btapp_spp_send((char*)mTxBuffer, strlen(mTxBuffer));
//        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
        mSending = 0;
    }
    if (button == BTAPP_BUTTON_S2 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 2\r\n");
        mSending = 1;
        btapp_spp_send(( char*)mTxBuffer, strlen(mTxBuffer));
//        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
        mSending = 0;
    }
    if (button == BTAPP_BUTTON_S3 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 3\r\n");
        mSending = 1;
        btapp_spp_send(( char*)mTxBuffer, strlen(mTxBuffer));
//        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
        mSending = 0;
    }
    if (button == BTAPP_BUTTON_S4 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 4\r\n");
        mSending = 1;
        btapp_spp_send(( char*)mTxBuffer, strlen(mTxBuffer));
//        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
        mSending = 0;
    }
        if (button == BTAPP_BUTTON_S5 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 5\r\n");
        mSending = 1;
        btapp_spp_send(( char*)mTxBuffer, strlen(mTxBuffer));
//        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
        mSending = 0;
    }

}

void btapp_onButtonUp(bt_uint button, bt_uint repeatCount)
{
}

// Bluetooth device name and class
const char* bt_oem_get_device_name(void)
{
//    int i=0;
    static char deviceName[37] =
    {
            BT_CONNECTION_NAME
    };
    bt_bdaddr_t* bdaddr = &HCI_CONTROLLER->bdaddr;
    int len = 18;

    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_m & 0xF000) >> 12);
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_m & 0x0F00) >> 8);
    deviceName[len++] = ':';
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_m & 0x00F0) >> 4);
    deviceName[len++] = *_ulong2str(bdaddr->bd_addr_m & 0x000F);
    deviceName[len++] = ':';
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0xF0000000) >> 28);
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x0F000000) >> 24);
    deviceName[len++] = ':';
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x00F00000) >> 20);
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x000F0000) >> 16);
    deviceName[len++] = ':';
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x0000F000) >> 12);
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x00000F00) >> 8);
    deviceName[len++] = ':';
    deviceName[len++] = *_ulong2str((bdaddr->bd_addr_l & 0x000000F0) >> 4);
    deviceName[len++] = *_ulong2str(bdaddr->bd_addr_l & 0x0000000F);
    deviceName[len] = 0;

    BT_DISPLAY_STATS.BTMACFLG = 1;
    BT_DISPLAY_STATS.DisplayUpdate = 1;
    BT_DISPLAY_STATS.BTDEMONAME = deviceName;

    BT_DISPLAY_STATS.BTMACADD = &deviceName[18];

    return deviceName;
}

//void btapp_aa_setDeviceName(const char* deviceName,char len)
//{
//    btmgr_setDeviceName(deviceName,len);
//    bt_hci_write_local_name(deviceName, &setDeviceNameCallback);
//}

bt_long bt_oem_get_device_class(void)
{
    return COS_INFORMATION |
           COD_MAJOR_COMPUTER | COD_MINOR_COMPUTER_HANDHELD;
}

// Bluetooth PIN code
void bt_oem_get_pin_code(bt_bdaddr_t* bdaddr_remote)
{
    bt_hci_send_pin_code(bdaddr_remote, "0000");
}

#ifndef BT_PASSKEY_ENABLE
// Bluetooth SSP handler customized to bypass passkey
void bt_oem_ssp_callback(SSP_EVENT spp_event, void* event_param, void* init_param) {
    switch (spp_event) {
        case SSP_EVENT_SIMPLE_PAIRING_COMPLETE:
        {
            bt_ssp_simple_pairing_complete* spc = (bt_ssp_simple_pairing_complete*) event_param;
            if (spc->status != HCI_ERR_SUCCESS) {
                // authentication failed.

            } else {
                // all is good. connection set up will proceed.
                BT_DISPLAY_STATS.DisplayUpdate = 1;
                BT_DISPLAY_STATS.BlueTooth_Status = Paired_NotConnected;


            }
        }
            break;

        case SSP_EVENT_USER_CONFIRMATION_REQUEST:
        {
            bt_ssp_user_confirmation_request* user_confirmation = (bt_ssp_user_confirmation_request*) event_param;
            bt_ssp_send_user_confirmation(HCI_ERR_SUCCESS, user_confirmation, NULL);
        }
            break;

        case SSP_EVENT_IO_CAPABILITY_REQUEST:
        {
            bt_ssp_io_capability* io_caps = (bt_ssp_io_capability*) event_param;
            io_caps->io_capability = SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT;
            io_caps->oob_data_present = SSP_OOB_DATA_NOT_PRESENT;
            io_caps->authentication_requirements = SSP_MITM_NOT_REQUIRED_GENERAL_BONDING;
            bt_ssp_set_io_capabilities(HCI_ERR_SUCCESS, io_caps, NULL);
        }
            break;

        default:
            break;
    }
}


static void initCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
    switch (cmd->opcode)
    {
        case HCI_WRITE_DEFAULT_POLICY_SETTINGS:
        {
            cmd = bt_hci_alloc_command(HCI_READ_INQUIRY_RESPONSE_TX_POWER_LEVEL, &initCallback);
            bt_hci_send_cmd(cmd);
            break;
        }

        case HCI_READ_INQUIRY_RESPONSE_TX_POWER_LEVEL:
        {
            bt_int offset = 4;
            bt_byte txPowerLevel;

            bt_hci_get_evt_param_byte(evt, &txPowerLevel, &offset);

            cmd = bt_hci_allocate_write_eir_command(HCI_EIR_FEC_NOT_REQUIRED);
            bt_hci_param_tx_power_level_add(txPowerLevel, cmd);
            bt_hci_param_eir_device_id_add(BTAPP_DID_VENDOR_ID_SOURCE, BTAPP_DID_VENDOR_ID, BTAPP_DID_PRODUCT_ID, BTAPP_DID_VERSION, cmd);
            bt_hci_param_eir_uuid16_add(HCI_EIR_TYPE_UUID16_LIST_COMPLETE, mDeviceUUIDs, sizeof(mDeviceUUIDs) / sizeof(bt_uint), cmd);
            bt_hci_param_eir_local_name_add(bt_oem_get_device_name(), cmd);
            bt_hci_write_eir(cmd, &initCallback);

            break;
        }

        case HCI_WRITE_EXTENDED_INQUIRY_RESPONSE:
        {
            //bt_bdaddr_t bdaddr;
            //bt_bool paired;

            bt_hci_set_incoming_connection_role(HCI_CONN_ROLE_MASTER);

            setConnected(0);
            btapp_control_init();
            btapp_spp_init();
            //paired = btmgr_getLastConnectedDevice(&bdaddr);
//            app_display_PairedStateSet(paired);
//            app_display_Update();
            setConnecting(1);
            mLinkIdleListener.event_id = HCI_EVT_LINK_IS_IDLE;
            mLinkIdleListener.callback.hci_event = &linkIdleListenerCallback;
            bt_hci_ctrl_register_listener(&mLinkIdleListener);

            mLinkBusyListener.event_id = HCI_EVT_LINK_IS_BUSY;
            mLinkBusyListener.callback.hci_event = &linkBusyListenerCallback;
            bt_hci_ctrl_register_listener(&mLinkBusyListener);

            mScanModeListener.event_id = HCI_EVT_COMMAND_COMPLETE;
            mScanModeListener.callback.hci_event = &scanEnableListenerCallback;

            break;
        }
    }
}
static void sniffSubrateCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
}
static void sniffCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
    if (status == HCI_ERR_SUCCESS)
    {
        bt_hci_conn_state_t* hciConn = (bt_hci_conn_state_t*)cmd->callback_param;

        bt_hci_sniff_subrating(
            hciConn,
            SNIFF_SUBRATE_LATENCY, SNIFF_SUBRATE_TIMEOUT,
            SNIFF_SUBRATE_TIMEOUT, &sniffSubrateCallback);
    }
}
void linkIdleListenerCallback(bt_int evcode, void* evt_params, void* cb_param)
{
    bt_hci_conn_state_t* hciConn = (bt_hci_conn_state_t*)evt_params;

    if (hciConn->mode == HCI_POWER_MODE_ACTIVE)
        bt_hci_sniff_mode_ex(hciConn, MIN_SNIFF_INTERVAL, MAX_SNIFF_INTERVAL, SNIFF_ATTEMPT_SLOTS, SNIFF_TIMEOUT, &sniffCallback, hciConn);
}
static void exitSniffCallback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
}

void linkBusyListenerCallback(bt_int evcode, void* evt_params, void* cb_param)
{
    bt_hci_conn_state_t* hciConn = (bt_hci_conn_state_t*)evt_params;

    if (hciConn->mode == HCI_POWER_MODE_SNIFF)
        bt_hci_exit_sniff_mode(hciConn, &exitSniffCallback);

}

void scanEnableListenerCallback(bt_int evcode, void* evt_params, void* cb_param)
{
    bt_int offset = 0;
    bt_byte scanConfig;
    bt_hci_evt_command_complete_t* cmdComplete = (bt_hci_evt_command_complete_t*)evt_params;

    if (cmdComplete->opcode == HCI_WRITE_SCAN_ENABLE)
    {
        bt_hci_get_param_byte(cmdComplete->cmd, &scanConfig, &offset);

//        if (!(scanConfig & HCI_DISCOVERABLE) && mPairing)
//        {
//            mPairing = 0;
//            app_display_Update();
//        }
    }
}






#endif
/*******************************************************************************
 End of File
 */