/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   display.c

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
#include <string.h>

#include "display.h"
#include "gfx/libaria/libaria_init.h"
#include "gfx/utils/inc/gfxu_string_utils.h"
//#include "gfx/utils/inc/gfxu_font.h"

//******************************************************************************
//
//  WIDGETS AVAILABEL:
//     laScheme           text_label;
//     laScheme           _default;
//     laScheme           track_time;
//     laScheme           image_button;
//     laScheme           filled_circle;
//     laRectangleWidget* RectangleWidget1;
//     laLabelWidget*     DemoNameLabel;
//     laLabelWidget*     ConnectPrompt;
//     laLabelWidget*     MHVersion;
//     laImageWidget*     MCHPLogo;
//     laLabelWidget*     CodecType;
//     laLabelWidget*     LabelWidget1;
//     laRadioButtonWidget* AecEnRadio;
//     laLabelWidget*       AecEnLabel;
//     laLabelWidget*       ConveredLable;
//     laLabelWidget*       EchoLabel;
//     laLabelWidget*       DTLable;
//     laRectangleWidget*   ConvergedLed;
//     laRectangleWidget*   EchoLed;
//     laRectangleWidget*   DTLed;
//
//******************************************************************************

GFX_DISPLAY_STATS displayStats;

//******************************************************************************
// display_init()
//******************************************************************************
void display_init(GFX_DISPLAY_STATS * displayStats)
{
    laString tempStr;  
    char versionStr[16] = "MH V";

    strcat(versionStr,SYS_VERSION_STR);
    tempStr = laString_CreateFromCharBuffer((char *) versionStr, 
                                             &LiberationSans12Italic);
    laLabelWidget_SetText(MHVersion, tempStr);
    laString_Destroy(&tempStr);

    displayStats->usbConnected = false;
    laWidget_SetVisible(&ConnectPrompt->widget, true);

    displayStats->aecEn     = true;
    displayStats->dt        = false;
    displayStats->echo      = false;
    displayStats->converged = false;

    displayStats->displayUpdate = false;
}

//******************************************************************************
// display_tasks()
//
// Description:
//   Updates the display when DISPLAY_STATS.displayUpdate is True.
//******************************************************************************
void display_tasks(GFX_DISPLAY_STATS * displayStats)
{
    if(displayStats->displayUpdate == 1 )
    {
        if (displayStats->usbConnected) 
        {
            laWidget_SetVisible(&ConnectPrompt->widget, false);
        }
        else
        {
            laWidget_SetVisible(&ConnectPrompt->widget, true);
        }

        if (displayStats->converged) 
        {
            //laWidget_SetVisible(&ConvergedLedOn->widget, true);
            //laWidget_SetVisible(&ConvergedLedOff->widget, false);
        }
        else
        {
            //laWidget_SetVisible(&ConvergedLedOn->widget, false);
            //laWidget_SetVisible(&ConvergedLedOff->widget, true);
        }

        if (displayStats->echo) 
        {
            laWidget_SetVisible(&EchoLedOff->widget, false);
            laWidget_SetVisible(&EchoLedOn->widget, true);
        }
        else 
        {
            laWidget_SetVisible(&EchoLedOff->widget, true);
            laWidget_SetVisible(&EchoLedOn->widget, false);
        }

        if (displayStats->dt) 
        {
            laWidget_SetVisible(&DTLedOff->widget, false);
            laWidget_SetVisible(&DTLedOn->widget, true);
        }
        else 
        {
            laWidget_SetVisible(&DTLedOff->widget, true);
            laWidget_SetVisible(&DTLedOn->widget, false);
        }

        //laString tempStr;  
        displayStats->displayUpdate = 0;
    }
} //End display_tasks()