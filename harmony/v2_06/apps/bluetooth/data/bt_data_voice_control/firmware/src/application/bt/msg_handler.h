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


#ifndef SPP_MSG_HANDLER_H
#define SPP_MSG_HANDLER_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define MSG_MODE_NONE   0
#define MSG_MODE_SPP    1
#define MSG_MODE_IAP    2

int msgHandler_Send(void* data, int size);

void sppMsgHandlerTask();
void msgHandler_HandleMsg( uint8_t device, void* data, int size);
void msgHandler_SetMode(int mode);
void btapp_SetSendToDevice ( uint8_t device );
void display_LED(int index,int type);

extern uint8_t bootloaderFlag;
extern bt_data_port androidPort[BT_MAX_PORTS_SPP];
extern bt_data_port devicePort[BT_MAX_PORTS_SPP];
extern char isAndroidConnected;
extern char isAppleConnected;
extern void MCHPBT_CancelReceive ( uint8_t device );
extern int connection_mode;
extern uint8_t ledstatusState[5];
void spp_SendTrackName();

#ifdef	__cplusplus
}
#endif
#endif /* SPP_MSG_HANDLER_H */