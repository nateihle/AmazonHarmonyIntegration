/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   Display_tasks.c

  Summary:
    Contains the functional implementation of display task for

  Description:
    This file contains the functional implementation of data parsing functions
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
#include "display.h"

static uint8_t encoderSelectIdx = 0;

static void _updateLabelWidgetByStringID(laLabelWidget* lbl, int strId);

void APP_DisplayTask()
{
    if(appData.encoderSelectIdx != encoderSelectIdx)
    {
        encoderSelectIdx = appData.encoderSelectIdx;
        /* set EncoderList to selected Index */
#ifdef BTADK_CONFIG        
        laListWidget_SetItemSelected(EncoderList, encoderSelectIdx, true);
#elif defined(MEB2_CONFIG)
        laListWheelWidget_SetSelectedItem(EncoderList, encoderSelectIdx);
#endif
    }
}

void APP_DisplayInsertUSB()
{
    _updateLabelWidgetByStringID(PromptTextLabel, string_InsertUSB);
}
void APP_DisplayStartRecord()
{
    _updateLabelWidgetByStringID(PromptTextLabel, string_StartRecord);
}
void APP_DisplayStopRecord()
{
    _updateLabelWidgetByStringID(PromptTextLabel, string_StopRecord);
}
void APP_DisplaySavedFile()
{
    _updateLabelWidgetByStringID(PromptTextLabel, string_SaveFile);
}

void _updateLabelWidgetByStringID(laLabelWidget* lbl, int strId)
{
    laString str;
    str = laString_CreateFromID(strId);
    laLabelWidget_SetText(lbl, str);
    laString_Destroy(&str);
}
