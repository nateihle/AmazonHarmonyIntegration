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

void APP_DisplayInit(GUI_DATA * guiData, AUDIO_GENERATE_STATUS * agStatus)
{
    APP_LED1_OFF();
    APP_LED2_OFF();    
}

//******************************************************************************
// APP_DisplayTask()
//
// GUI_DATA:
//       bool       displayUpdate;
//       GUI_MODE   mode;     MODE_TONE/MODE_CHIRP
//       GUI_SELECT select;   SELECT_F1/SELECT_F2/SELECT_T/SELECT_DUTY
//       int32_t    f1Hz;
//       int32_t    f2Hz;
//       int32_t    timeDeltaMs; //NOTE <0 implies inf. 
//       //int16_t    dutyCycle;
//       bool       onOff;   //Generated output
//
//******************************************************************************
void APP_DisplayTask(GUI_DATA * guiData)
{
    if (guiData->displayUpdate == true)
    {                   
        //Mode Display Change
        if (guiData->onOff)
        {
            APP_LED1_ON(); 
        }
        else
        {
            APP_LED1_OFF();       
        }

        guiData->displayUpdate = false;
        guiData->changeMode= false;
        guiData->changeToneMode= false;

    } //End displayUpdate

} //End APP_DisplayTask()
