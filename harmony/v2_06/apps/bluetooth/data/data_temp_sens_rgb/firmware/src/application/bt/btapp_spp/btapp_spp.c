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
#include <stdio.h>
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
static BTAPP_STATUS    mAppStatus = BTAPP_STATUS_IDLE;
static bt_spp_port_t   *mPort = NULL;
static char            mConnected = 0;
static char            mConnecting;
static char            mSending = 0;
static char            mReceiving = 0;
static bt_int          mBytesReceived;
static bt_uint         mProgConnect = 1;
//static bt_uint         mTimerTick=0;
//static bt_uint         mReConnectTick=0;
//static bt_uint         mNewConnectTick=15;
static bt_int          mTempTimerFlag =0;
static bt_bool         mRateChange= BT_FALSE;
static bt_int          mTempCallbackRate = TEMP_CALLBACK_RATE;
static char            mTxBuffer[BUFFER_SIZE]={0};
static char            mRxBuffer[BUFFER_SIZE]={0};

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Protoypes
// *****************************************************************************
static void sysStartCallback(bt_bool success, void* param);
static void sppStateCallback(bt_spp_port_t* port, bt_spp_port_event_e evt, void* param);
static void sppReceiveCallback(bt_spp_port_t* port, bt_int bytesReceived, void* param);
static void sppSendCallback(bt_spp_port_t* port, bt_ulong bytesSent,
                                    bt_spp_send_status_e result, void* param);
static void processReceivedData(void);
static void setConnected(char connected);
static void setConnecting(char connecting);
//static void timerCallBack(void);
static void appTimerCallback(void);
void SendTemp(void);


// *****************************************************************************
// *****************************************************************************
// Section: Fuction Implementation
// *****************************************************************************
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
    bt_sys_start(BT_TRUE, BT_TRUE, sdp_db_spp, sdp_db_spp_len,
                &sysStartCallback, NULL);
}

void btapp_spp_reconnect(bt_uint newConnectTick, bt_uint reConnectTick)
{
}

void SendTemp(void)
{
    int Temp, BuffSize = 0;
    char __attribute__((unused)) Buffer[BUFFER_SIZE] = {0};
    char Buffer2[100]= {0};
    char TempBuffer[100] = {"253,99"};
    char TestBuffer[100] = {"253,98"};

    Temp = ReadTemp(FAHRENHEIT);
    if (mRateChange == BT_TRUE) 
    {
        strcpy(TempBuffer, "252,");
        mRateChange = BT_FALSE;
    }
    else 
    {
        strcpy(TempBuffer, "253,");
    }

    //NOTE: itoa seems to add some spurious characters at times.
    itoa(Buffer2, Temp, 10);
    TestBuffer[4] = Buffer2[0];
    TestBuffer[5] = Buffer2[1];
    if (Buffer2[3] == '1')
    {
       TestBuffer[6] = '1';
       TestBuffer[7] = 0x00;
    }
    else
    {
       TestBuffer[7] = 0x00;
    }
    //strcat(TestBuffer, Buffer2);
    //strcat(TempBuffer, "\r\n");

    //Buffer[0] = '2';
    //Buffer[1] = '5';
    //Buffer[2] = '3';
    //Buffer[3] = ',';
    //Buffer[4] = '9';
    //Buffer[5] = '9';
    //Buffer[6] = 0x00;

    BuffSize = (strlen(TestBuffer) + 1);
    if (!mSending) 
    {
        if (bt_spp_send(mPort, TestBuffer, BuffSize, &sppSendCallback)) 
        {
            mSending = 1;
        }
    }
    btapp_setTimer(BTAPP_TEMPTIMER, mTempCallbackRate, appTimerCallback);
}

static void sysStartCallback(bt_bool success, void* param)
{
    assert(success);
    //bt_bdaddr_t bdaddr;
    //bt_bool paired;
    mAppStatus = BTAPP_STATUS_STARTED;
    setConnected(0);
    //paired = btmgr_getLastConnectedDevice(&bdaddr);
    mPort = bt_spp_allocate(bt_sys_get_l2cap_manager(), &sppStateCallback, NULL);
    bt_spp_listen(mPort, RFCOMM_SERIAL_PORT_CH_1);
    setConnecting(1);
}

static void sppStateCallback(bt_spp_port_t* port, bt_spp_port_event_e evt, void* param)
{
   
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
            break;
        case SPP_PORT_EVENT_DISCONNECTED:
            setConnected(0);
            mSending = 0;
            mReceiving = 0;
            mProgConnect = 1;
            bttask_pal_setetAddrsAssigned();
            break;
        case SPP_PORT_EVENT_SEND_PROGRESS:
            break;
        case SPP_PORT_EVENT_LOCAL_MODEM_STATUS_CHANGED:
            break;
        case SPP_PORT_EVENT_LOCAL_MODEM_STATUS_CHANGE_FAILED:
            break;
        case SPP_PORT_EVENT_REMOTE_MODEM_STATUS_CHANGED:
            break;

    }
}

/*This callback function is used to receive data sent from spp pro app to the bluetooth development kit.*/
static void sppReceiveCallback(bt_spp_port_t* port, bt_int bytesReceived, void* param)
{
    mReceiving = 0;
    mBytesReceived = bytesReceived;
    if (!mSending)
    {
            processReceivedData();
    }
}

static void sppSendCallback(bt_spp_port_t* port, bt_ulong bytesSent, bt_spp_send_status_e result, void* param)
{

mSending = 0;

}

/* This function copies the received data to a buffer and parses the data to
 * perform user defined operations */
static void processReceivedData(void)
{
    int Command, Size, Red, Green, Blue;
	char CommandBuff[BUFFER_SIZE]={0};
    char CharCommand;
    int TempData;
    char ConversionBuff [BUFFER_SIZE] = {0};
    bt_int bytesReceived = mBytesReceived;

    mRxBuffer[bytesReceived] = '\0';
    memcpy(mTxBuffer, mRxBuffer, bytesReceived);
    memcpy(CommandBuff, mRxBuffer, bytesReceived);
    mSending = 0;
/*******************************************************************************/
    sscanf(CommandBuff, "%d,%d,%d,%d,%d", &Command, &Size, &Red, &Green, &Blue);
/*if command (255,3,R,G,B) is received the led is set according to the hue of RGB*/
        if(Command == RGB_LED_COLOR)
        {
            LEDColorSet( Red,  Green,  Blue);
        }
    /*If command 254 is received the temperature will be sent to terminal emulator once*/
        else if (Command == ONE_TIME_TEMP_CALL)
        {
            TempData = ReadTemp(FAHRENHEIT);
            bytesReceived = (sizeof(mTxBuffer));
            memcpy(mTxBuffer, itoa(ConversionBuff,TempData,10), bytesReceived);
            strcat(mTxBuffer,"\n");
            bt_spp_send(mPort, mTxBuffer, (bytesReceived), &sppSendCallback);
        }
    /*If command 253 is received the temperature will be sent periodically to the terminal emulator */
        else if (Command == CONTINUOUS_TEMP)
        {
            mTempTimerFlag =!mTempTimerFlag;
            if(mTempTimerFlag == 1)
            {

                mSending = 0;
                btapp_setTimer(BTAPP_TEMPTIMER, mTempCallbackRate, appTimerCallback);
            }
         /*If command 253 is received again ,periodic update of temperature will be terminated*/
           else if (mTempTimerFlag == 0)
            {
                /* stop the timer*/
                btapp_clearTimer(BTAPP_TEMPTIMER);
                mRateChange = BT_FALSE;
                mTempCallbackRate = TEMP_CALLBACK_RATE;

            }
        }
    /*If command (252,TIME) is received, the periodic time interval is modified according to TIME . for instance (252,5000) */
        else if (Command == TEMP_CALLBACK_RATE_ADDR)
        {
                /*252,xxx  xxx is the new call back rate in milliseconds */
                mTempCallbackRate = Size;
                mRateChange = BT_TRUE;
        }
        /* For Single Char LED Control */
    /*If characters 'R','G','B','r','g' and 'b' is received the led color is changed accordingly*/
        sscanf(CommandBuff, "%c", &CharCommand);
        if (CharCommand == SINGLE_CHAR_RED50)
        {
            LEDColorSet(127,0,0);
        }
        if (CharCommand == SINGLE_CHAR_RED100)
        {
            LEDColorSet(254,0,0);
        }
        if (CharCommand == SINGLE_CHAR_GREEN50)
        {
            LEDColorSet(0,127,0);
        }
        if (CharCommand == SINGLE_CHAR_GREEN100)
        {
            LEDColorSet(0,254,0);
        }
        if (CharCommand == SINGLE_CHAR_BLUE50)
        {
            LEDColorSet(0,0,127);
        }
        if (CharCommand == SINGLE_CHAR_BLUE100)
        {
            LEDColorSet(0,0,254);
        }
/*******************************************************************************/
    if (!mReceiving)
    {
        mReceiving = 1;
        bt_spp_receive(mPort, mRxBuffer, sizeof(mRxBuffer), &sppReceiveCallback);
    }
}

static void setConnected(char connected)
{
    mConnected = connected;

}

static void setConnecting(char connecting)
{
    mConnecting = connecting;
}

/*
static void timerCallBack(void)
{
    btapp_setTimer(BTAPP_TEMPTIMER, mTempCallbackRate, &SendTemp);

}
*/
void btapp_onButtonDown(bt_uint button, bt_uint repeatCount)
{
    if (button == BTAPP_BUTTON_S1 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 1\r\n");
        mSending = 1;
        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
    }
    if (button == BTAPP_BUTTON_S2 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 2\r\n");
        mSending = 1;
        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
    }
    if (button == BTAPP_BUTTON_S3 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 3\r\n");
        mSending = 1;
        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
    }
    if (button == BTAPP_BUTTON_S4 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 4\r\n");
        mSending = 1;
        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
    }
        if (button == BTAPP_BUTTON_S5 && mConnected && !mSending)
    {
        strcpy(mTxBuffer, "\r\nButton 5\r\n");
        mSending = 1;
        bt_spp_send(mPort, mTxBuffer, strlen(mTxBuffer), &sppSendCallback);
    }

}

void btapp_onButtonUp(bt_uint button, bt_uint repeatCount)
{
}

// Bluetooth device name and class
const char* bt_oem_get_device_name(void)
{
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

    return deviceName;
}

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

#endif

static void appTimerCallback(void)
{
    SendTemp();
}

/*******************************************************************************
 End of File
 */