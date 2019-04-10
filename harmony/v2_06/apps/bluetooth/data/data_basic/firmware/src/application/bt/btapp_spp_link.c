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



#include "bluetooth/cdbt/bt/bt_std.h"
#include "bluetooth/cdbt/bt/bt_system.h"
#include "bluetooth/cdbt/spp/spp.h"
#include "btapp_control.h"
#include "btapp_spp_link.h"
#include "app.h"
#include "btapp_spp.h"

static void*                  mRxBuffer;
static spp_recv_callback_t    mRxCallback;
static void sppSendCallback(bt_spp_port_t* port, bt_ulong bytes_sent, bt_spp_send_status_e status, void* param);
int temp = 0;



void btapp_spp_init(void)
{
    return;
}

void btapp_spp_attach_port(bt_spp_port_t* port)
{
    int i = 0;
    for(i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
        if(port == androidPort[i].port)
        {
            if(androidPort[i].mConnected == false)
            {
                BT_DISPLAY_STATS.BTPORT +=1;
                BT_DISPLAY_STATS.BTPORTFLAG =1;
                BT_DISPLAY_STATS.DisplayUpdate = 1;
                androidPort[i].mConnected = true;
                androidPort[i].active = false;
                androidPort[i].mReceiving = 0;
                androidPort[i].mSendToThisDevice = false;
                btapp_control_onConnected(androidPort[i].port);
            }
        }
    }
}

void btapp_spp_detach_port(bt_spp_port_t* port)
{
    int i = 0;
    bt_bool devicesConnected = BT_FALSE;
    for (i = 0; i < BT_MAX_PORTS_SPP ; i++)
    {
        if(port == androidPort[i].port)
        {
            BT_DISPLAY_STATS.BTPORT -= 1;
            BT_DISPLAY_STATS.BTPORTFLAG =1;
            BT_DISPLAY_STATS.DisplayUpdate = 1;
            androidPort[i].active = false;
            androidPort[i].mConnected = false;
            androidPort[i].mReceiving = 0;
            androidPort[i].mSendToThisDevice = false;
        }
        devicesConnected |= androidPort[i].mConnected;
    }
    btapp_control_onDisconnected(port);
    if (devicesConnected == BT_FALSE)
    {
        setConnected(0);
        setWaiting(0); // set waiting to be 0
    }
}
/*
void btapp_spp_disconnect(void)
{

}
*/

void sppReceiveCallback(bt_spp_port_t* port, bt_int bytesReceived, void* param)
{
    int i = 0;

    for(i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
        if(port == androidPort[i].port)
        {
            
            androidPort[i].active = false;
            androidPort[i].mReceiving = 0;
            androidPort[i].mSendToThisDevice = true;
        }
    }
    if(mRxCallback != NULL)
    {
        mRxCallback(port, bytesReceived);

        mRxCallback = NULL;
    }
}


void sppSendCallback(bt_spp_port_t* port, bt_ulong bytesSent, bt_spp_send_status_e result, void* param)
{
    
    int i = 0;
    for(i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
        if(port == androidPort[i].port)
        {
            androidPort[i].active = false;
            androidPort[i].mSendToThisDevice = false;
        }
    }
}


int btapp_spp_send(const char* data, unsigned int size)
{
    int i = 0;

    for(i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
    {
//        if(androidPort[i].active == FALSE  && androidPort[i].mSendToThisDevice == TRUE)
        if(androidPort[i].mConnected == true)// && androidPort[i].mSendToThisDevice == TRUE)
        {
            if(bt_spp_send(androidPort[i].port, data, size, &sppSendCallback) == BT_TRUE)
            {
                return BT_TRUE;
            }
        }
    }

    return BT_FALSE;
}

int btapp_spp_recv(const char* data, unsigned int size, spp_recv_callback_t cb)
{
    int i = 0;
    int returnFlag = 0;
    if(data != NULL && size != 0 && cb != NULL)
    {
        for (i = 0 ; i < BT_MAX_PORTS_SPP ; i++)
        {
            if ((androidPort[i].mConnected == true) && (androidPort[i].active == false) && (androidPort[i].mReceiving == 0))
            {
                if(bt_spp_receive(androidPort[i].port, (void*)data, size, &sppReceiveCallback) == BT_TRUE)
                {
                    androidPort[i].active = true;
                    androidPort[i].mReceiving = 1;
                    androidPort[i].mSendToThisDevice = false;
                    mRxBuffer = (unsigned char*)data;
                    mRxCallback = cb;
                    returnFlag = 1;
                }
            }
        }
    }

    if(returnFlag)
        return BT_TRUE;
    else
        return BT_FALSE;
}

