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

#include "app.h"
#include "gfx_resources.h"
#include "gfx/gfx.h"

#define LARGE_FONT      &Arial12pt
#define MEDIUM_FONT     &Arial12pt
#define SMALL_FONT      &Arial12pt
#define RX_DATA_BASE    65
#define TASK_BAR_COLOR GFX_RGBConvert(29, 46, 60)

#define BACKGROUND_COLOR BLACK
#define TEXT_COLOR1     WHITE
#define TEXT_COLOR2     BRIGHTRED


static GFX_INDEX            gfxIndex=0;

void GFX_MENU_DRAW(void)
{

        GFX_TransparentColorEnable(GFX_INDEX_0, TASK_BAR_COLOR);

/*******************************************************************************/
//      Draw Task Bars at Top & Bottom
/*******************************************************************************/
        GFX_ColorSet(gfxIndex, BACKGROUND_COLOR );
        GFX_ScreenClear(gfxIndex);

        GFX_ColorSet(gfxIndex, TASK_BAR_COLOR );

        GFX_GradientColorSet(GFX_INDEX_0, BRIGHTRED, BACKGROUND_COLOR);
        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_GRADIENT_DOWN);
        GFX_RectangleRoundFillDraw(GFX_INDEX_0, 15, 0, 205, 19, 15); // top task bar lowest edge 34 pixels down

//        GFX_GradientColorSet(GFX_INDEX_0, BACKGROUND_COLOR, TASK_BAR_COLOR);
//        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_GRADIENT_UP);
//        GFX_RectangleRoundFillDraw(GFX_INDEX_0, 15, 152, 205, 171, 15);// bottom task bar highest edge 137 pixels

        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_COLOR);
/*******************************************************************************/
//      Text Section
/*******************************************************************************/
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 30, 24, "Harmony USB Headset", 0);
        GFX_TextStringDraw(GFX_INDEX_0, 30, 37, "Sample Frequency: 48KHz", 0);
        GFX_TextStringDraw(GFX_INDEX_0, 30, 50, "USB: v1.1", 0);
       
/*******************************************************************************/
//      Image Icons - Upper Task Bar
/*******************************************************************************/

        /* Mircochip logo*/
        if(GFX_ImageDraw(gfxIndex, 4, 1, (GFX_RESOURCE_HDR *) &MCHP_LOGO) == GFX_STATUS_FAILURE)
            return;
}


void display_tasks(void)
{
    
    
}