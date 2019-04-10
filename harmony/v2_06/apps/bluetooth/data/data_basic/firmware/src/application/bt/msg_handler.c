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

#include "bluetooth/cdbt/bt/bt_std.h"
#include "app.h"
#include "btapp_device.h"
#include "btapp_control.h"
#include "btapp_spp_link.h"
#include "msg_handler.h"

#define BUFFER_SZ 500
#define CMDBUFSIZE 32

//static bt_bool handle_command(bt_uint len);
static int waiting;
static int _mode;
static char txBuffer[BUFFER_SZ];
static char rxBuffer[BUFFER_SZ];

uint8_t bgColorFlag = 1;
bt_bool Device_Name_flag = BT_FALSE;



/*
static void display_rxdata(unsigned char* data_buff, bt_int data_len)
{
    return;
}
*/

void setWaiting(int v)
{
    waiting = v;
}

void msgHandler_HandleMsg(uint8_t device, void* data, int size)
{
    int remaining, sz;
    unsigned char* ptr;

    if(size > 0)
    {
        remaining = size;
        ptr = data;

        while(remaining != 0)
        {
            //sz = remaining < BUFFER_SZ ? remaining : BUFFER_SZ;
            if(remaining < BUFFER_SZ)
            {
                sz = remaining;
            }
            else
            {
                sz = BUFFER_SZ;
            }
            
            memcpy(rxBuffer, ptr, sz);
            ptr += sz;
            remaining -= sz;

            /* handle spp command */
            /* perform normal echo */
            memcpy(txBuffer, rxBuffer, sz);

            txBuffer[sz - 1] = '\0';



        }
    }
    waiting = 0;
}

int msgHandler_Send(void* data, int size)
{
    return btapp_spp_send(data, size);
}

static void spp_recv_callback(bt_spp_port_t* port, unsigned int size)
{
    

    if(size > 0)
    {
        memcpy(txBuffer, rxBuffer, size);
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.PROCESS_TEXT =1;
        memcpy(BT_DISPLAY_STATS.RXBUFFER_DATA, rxBuffer, (size+1));
        BT_DISPLAY_STATS.RXBUFFER_LENGTH = (size+1);
        TakeAction(rxBuffer);


        /* perform normal SPP echo */
        rxBuffer[size] = '\0';
        memset(txBuffer,0,BUFFER_SZ);

        memcpy(txBuffer, rxBuffer, size);
        memset(rxBuffer,0,size);

        btapp_spp_send(txBuffer, size);

    }
    /* signal task to request another packet */
    waiting = 0;
}


void sppMsgHandlerTask()
{
    if(waiting == 0 && btapp_spp_recv(rxBuffer, BUFFER_SZ,&spp_recv_callback) != 0)
    {
        waiting = 1;
    }
}

void msgHandler_SetMode(int mode)
{
    if(_mode == MSG_MODE_SPP)
        msgHandler_HandleMsg(0, NULL, 0);
    _mode = mode;
}
