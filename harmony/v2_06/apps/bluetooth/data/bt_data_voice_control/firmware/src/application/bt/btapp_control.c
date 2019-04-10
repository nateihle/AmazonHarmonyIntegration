//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "btapp_spp_link.h"
#include "btapp_control.h"
#include "msg_handler.h"

#define LED1_MASK    0x01
#define LED2_MASK    0x02
#define LED3_MASK    0x04
#define LED4_MASK    0x08
#define LED5_MASK    0x10
// Link type.
#define BTLINK_TYPE_IAP          0
#define BTLINK_TYPE_SPP          1

// Link state
#define BTLINK_STATE_IDLE        0
#define BTLINK_STATE_CONNECTING  1
#define BTLINK_STATE_CONNECTED   2

// Data session state
#define DATA_SESSION_STATE_CLOSED  0
#define DATA_SESSION_STATE_OPEN    1

#define CONNECT_MAX_RETRIES      5
#define CONNECT_RETRY_INTERVAL   2000

static const bt_uuid_t SERIAL_PORT_SERVICE_CLSID   = { 0x5f9b34fb, 0x80000080, 0x00001000, 0x00001101 };
static char               mLinkType;
static bt_bdaddr_t        mBtAddress;
static bt_byte            mServerChannel;
static bt_byte            mConnectRetryCount;
static void sppStateCallback(bt_spp_port_t* port, bt_spp_port_event_e evt, void* param);
static void retryTimerCallback(void);

char isAndroidConnected = false;
char isAppleConnected = false;


bt_data_port androidPort[BT_MAX_PORTS_SPP];// = {NULL, NULL};
bt_data_port devicePort[BT_MAX_PORTS_SPP];


void btapp_control_init(void)
{
    uint8_t i = 0;
    mConnectRetryCount = 0;
    memset(&mBtAddress, 0, sizeof(mBtAddress));

    for (i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
        androidPort[i].port = bt_spp_allocate(bt_sys_get_l2cap_manager(), &sppStateCallback, NULL);
        bt_spp_set_port_options(androidPort[i].port, SPP_PORT_OPTION_SECURE | SPP_PORT_OPTION_ENCRYPTED);
        androidPort[i].active = false;
        androidPort[i].mConnected = false;
        androidPort[i].mReceiving = 0;
        androidPort[i].mSending = 0;
        androidPort[i].mSendToThisDevice = false;
        androidPort[i].mState = BTLINK_STATE_IDLE;
        androidPort[i].mBtAddress = NULL;
        androidPort[i].bootloaderEnable = false;
        bt_spp_listen(androidPort[i].port, RFCOMM_SERIAL_PORT_CH_1);//Android
    }
}


void btapp_control_connect(bt_bdaddr_t* bdaddr)
{
    return;
}

void btapp_control_onConnected(bt_spp_port_t * port)
{
    int i = 0;
    for (i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
        if(port == androidPort[i].port)
        {
            bt_bdaddr_t* addr = bt_spp_get_remote_address(androidPort[i].port);
            if (addr)
            {
                androidPort[i].mBtAddress = addr;
                androidPort[i].mState = BTLINK_STATE_CONNECTED;
            }
        }
    }
}

void btapp_control_onDisconnected(bt_spp_port_t * port)
{
    int i = 0;
    bool devicesConnected = false;
    for (i = 0 ; i < BT_MAX_PORTS_SPP; i++)
    {
        if(port == androidPort[i].port)
        {
            androidPort[i].mState = BTLINK_STATE_IDLE;
        }
        if (androidPort[i].mState != BTLINK_STATE_IDLE)
        {
            devicesConnected = true;
        }

    }

    if (devicesConnected == false)
    {
        btapp_clearTimer(BTAPP_TIMER_2);
        btapp_clearTimer(BTAPP_TIMER_3);
    }
}

void btapp_control_onDataSessionOpen(void)
{
    return;
}

int connection_mode = RFCOMM_CHANNEL_SPP;
void btapp_control_onDataSessionClose(void)
{
    btapp_clearTimer(BTAPP_TIMER_2);
    btapp_clearTimer(BTAPP_TIMER_3);
}

static void sppStateCallback(bt_spp_port_t* port, 
                             bt_spp_port_event_e evt, 
                             void* param)
{
    switch (evt)
    {
        case SPP_PORT_EVENT_CONNECTION_FAILED:
            if (--mConnectRetryCount)
            {
                btapp_setTimer(BTAPP_TIMER_3, 
                        CONNECT_RETRY_INTERVAL * 
                        (CONNECT_MAX_RETRIES - mConnectRetryCount), 
                        &retryTimerCallback);
            }
            else
            {
                memset(&mBtAddress, 0, sizeof(mBtAddress));
            }
            break;

        case SPP_PORT_EVENT_CONNECTED:
            {
                mLinkType = BTLINK_TYPE_SPP;
                btapp_spp_attach_port(port);
                setConnected(1);

                if(port->server_channel == 1)
                {
                    connection_mode = RFCOMM_CHANNEL_SPP;
                    isAndroidConnected = true;
                }
                msgHandler_SetMode(connection_mode);
                BT_DISPLAY_STATS.DisplayUpdate = 1;
                appData.sppConnected = true;
                BT_DISPLAY_STATS.BlueTooth_Status = Paired_Connected;

            }
            break;

        case SPP_PORT_EVENT_DISCONNECTED:
            if (mLinkType == BTLINK_TYPE_SPP)
            {
                btapp_spp_detach_port(port);
            }
            break;

        case SPP_PORT_EVENT_SEND_PROGRESS:
            break;

        default:
            break;
    }
}
int getConnection_mode()
{
    return connection_mode;
}

void btapp_control_onButtonDown(bt_uint button, bt_uint repeatCount)
{

}


//void btapp_control_onButtonUp(bt_uint button, bt_uint repeatCount)
//{
//    if ( button == BTAPP_BUTTON_S6 )
//    {
//        DelayMs(100);
//        sendspp_data();
//    }
//}

static void retryTimerCallback(void)
{
    int i = 0;
    for (i = 0 ; i < BT_MAX_PORTS_SPP; i++)
    {
        if (!bt_spp_connect(androidPort[i].port, androidPort[i].mBtAddress, mServerChannel))
        {
            memset(&mBtAddress, 0, sizeof(mBtAddress));
        }
    }
}
