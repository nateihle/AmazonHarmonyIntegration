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

#ifndef __BTAPP_CONTROL_H_INCLUDED__
#define __BTAPP_CONTROL_H_INCLUDED__


/** BTLINK status code.
*/
typedef enum _btlink_Status
{
    BTLINK_STATUS_SUCCESS,
    BTLINK_STATUS_FAILURE
} btlink_Status;


// void sendspp_data(void);
void btapp_control_init(void);
void clearRepeatTimer(void);
void btapp_aa_setDeviceName(const char* deviceName,char len);
void bttask_setRepeatButton(unsigned char Button);
extern void setConnected(char connected);

// These are the RFCOMM channels on which we are listening for incoming
// connections. They must by in sync with the SDP database.
#define RFCOMM_CHANNEL_SPP       1
#define RFCOMM_CHANNEL_IAP       2

void btapp_control_init(void);

void btapp_control_onButtonDown(bt_uint button, bt_uint repeatCount);
void btapp_control_onButtonUp(bt_uint button, bt_uint repeatCount);
void btapp_control_onDataReceived(const char* data, bt_uint len);
#if defined (BT_AUDIO_SPP)
void btapp_control_onDataSent(btlink_Status status);
#endif
void btapp_control_onConnected(bt_spp_port_t* port);
void btapp_control_onDisconnected(bt_spp_port_t* port);
void btapp_control_onDataSessionOpen(void);
void btapp_control_onDataSessionClose(void);
int getConnection_mode();

#endif // __BTAPP_CONTROL_H_INCLUDED__

