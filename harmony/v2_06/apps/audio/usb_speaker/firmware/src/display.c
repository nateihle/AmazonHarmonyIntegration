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

#include "display.h"
#include "gfx/libaria/libaria_init.h"

GFX_DISPLAY_STATS DISPLAY_STATS;

//******************************************************************************
// display_init()
//******************************************************************************
void display_init(GFX_DISPLAY_STATS * DISPLAY_STATS)
{
    laString tempStr;  
    tempStr = laString_CreateFromCharBuffer((char *) SYS_VERSION_STR, 
                                             &LiberationSans12Italic);
    laLabelWidget_SetText(MHVersion, tempStr);
    laString_Destroy(&tempStr);

}

//******************************************************************************
// display_tasks()
//
// Description:
//   Updates the display when DISPLAY_STATS.DisplayUpdate is True.
//
//   Update the connected icon DISPLAY_STATS.BTPORTFLAG is true.
//
//******************************************************************************
void display_tasks(GFX_DISPLAY_STATS * DISPLAY_STATS)
{
    if(DISPLAY_STATS->DisplayUpdate == 1 )
    {
//      laString tempStr;  

        DISPLAY_STATS->DisplayUpdate = 0;
    }

} //End display_tasks()



