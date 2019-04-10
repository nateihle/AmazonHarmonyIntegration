/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    display.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for updating the LCD display.
 *  This is Alpha Code and is subject to change without notice.

*******************************************************************************
// DOM-IGNORE-BEGIN

Copyright (c) 2013-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "driver/can/drv_can.h"
#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

#define HEX 16
#define DEC 10
#define CAN_CHAN CAN_CHANNEL1

void DisplayISR (void);
void _updateLabelWidgetText(laLabelWidget* lbl, const char *data, uint8_t fidx);
void TxScreenUpDate(int Message);

extern GFXU_FontAsset* fontList[2];

char WorkBuffer1[100];
CAN_RX_MSG_BUFFER *RxMessage;

unsigned ID;
unsigned DLC;
/* *****************************************************************************
 End of File
 */
void DisplayTasks(void)
{
//    itoa(WorkBuffer1,DLC, 10); //convert ASCII to base 10
    _updateLabelWidgetText (VersionNumberlbl,SYS_VERSION_STR,0);
}

void DisplayISR (void)
{
    BSP_LED_3On(); // shows visual that code is processing data, for debug only 
    
    uint8_t RxData[9] = {'0'};
    int IndexP = 0;
    
    RxMessage = (CAN_RX_MSG_BUFFER *)PLIB_CAN_ReceivedMessageGet(CAN_ID_1, CAN_CHAN);
    //SYS_PRINT("\n\r RXMessage: %c", RxMessage);
    DLC = RxMessage->msgEID.data_length_code;
    ID = RxMessage->msgSID.sid;
    
    SYS_PRINT("\n\r RxMessage: %x", RxMessage);
    SYS_PRINT("\n\r ID: 0x%x", ID);
    SYS_PRINT("\n\r DLC: %i", DLC);
    
    
    

    while(IndexP < DLC)
    {
         RxData[IndexP] = RxMessage->data[IndexP];
         SYS_PRINT("\n\r RxData[%i]: %x", IndexP, RxData[IndexP]);
         IndexP++;
    }
        
       
    // convert values to strings and send them to the display
    
    itoa(WorkBuffer1,DLC, 10); //convert ASCII to base 10
    _updateLabelWidgetText (CAN_DLC,WorkBuffer1,0);
    
    itoa(WorkBuffer1,ID, HEX);
    _updateLabelWidgetText (CAN_Address,WorkBuffer1,0);
    
     itoa(WorkBuffer1,RxData[0], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_1,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[1], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_2,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[2], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_3,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[3], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_4,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[4], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_5,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[5], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_6,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[6], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_7,WorkBuffer1,0);
    
    itoa(WorkBuffer1,RxData[7], HEX);
    _updateLabelWidgetText (CAN_MESSAGE_8,WorkBuffer1,0);
    

    PLIB_CAN_ChannelUpdate(CAN_ID_1, CAN_CHAN); /* Message processing is done, update the message buffer pointer. */
    BSP_LED_3Off();// shows visual that code is processing data, for debug only  
}

void _updateLabelWidgetText(laLabelWidget* lbl, const char *data, uint8_t fidx)
{
    GFXU_CHAR gchar_data[64];
    laString str;
    int len = 0;
    while(data[len]!='\0')
    {
        gchar_data[len] = data[len];
        len ++;
    }
    if(len ==0)
    {
        return;
    }
    gchar_data[len] = 0x00;
    
    str = laString_CreateFromBuffer(gchar_data, fontList[fidx]);

    laLabelWidget_SetText(lbl, str);
    laString_Destroy(&str);
}

void TxScreenUpDate(int Message)
{
    char WorkBuffer[50] = {'\0'};
    uint8_t TxData[8] = {'\0'};; //Test message to transmit on CAN
    int TxId = 0;
    int TxDlc = 0;
    if (Message == 1) // select message
    {
        TxId = 0x150;
        TxDlc = 4;

        TxData[0] = 15; TxData[1] = 81; TxData[2] = 7; TxData[3] = 10;
        while(DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL0, TxId,TxDlc, &TxData[0]) == false);
    }
    else if (Message == 2)
    {
        TxId = 0x200;
        TxDlc = 8;

        TxData[0] = 12; TxData[1] = 71; TxData[2] = 47; TxData[3] = 14;
        TxData[4] = 100; TxData[5] = 15; TxData[6] = 255; TxData[7] = 80;
        while(DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL0, TxId,TxDlc, &TxData[0]) == false);
        
        
    }

    
    
    
    // update screen
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxDlc, 10); //convert ASCII to base 10
    _updateLabelWidgetText (CAN_TX_DLC,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxId, HEX);
    _updateLabelWidgetText (CAN_TX_ID,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
     itoa(WorkBuffer,TxData[0], HEX);
    _updateLabelWidgetText (TX_DATA_1,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[1], HEX);
    _updateLabelWidgetText (TX_DATA_2,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[2], HEX);
    _updateLabelWidgetText (TX_DATA_3,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[3], HEX);
    _updateLabelWidgetText (TX_DATA_4,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[4], HEX);
    _updateLabelWidgetText (TX_DATA_5,WorkBuffer,0);
    
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[5], HEX);
    _updateLabelWidgetText (TX_DATA_6,WorkBuffer,0);
   
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[6], HEX);
    _updateLabelWidgetText (TX_DATA_7,WorkBuffer,0);
   
    memset(WorkBuffer,'\0',50);
    itoa(WorkBuffer,TxData[7], HEX);
    _updateLabelWidgetText (TX_DATA_8,WorkBuffer,0);
    

}