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
//#include "gfx_resources.h"
//#include "gfx_hgc_definitions.h"
//remove, for debugging
#include "btapp_spp_link.h"
#include "system_config.h"


//#define LARGE_FONT      &Arial20pt
//#define MEDIUM_FONT     &Arial14pt
//#define SMALL_FONT      &Arial12pt
////#define RX_DATA_BASE    65
//#define TASK_BAR_COLOR GFX_RGBConvert(29, 46, 60)
//#define BACKGROUND_COLOR BLACK
//
//char __attribute__((coherent, aligned(16))) DISPLAY_LINE1[BUFFER_SIZE] ={0};
//char __attribute__((coherent, aligned(16))) DISPLAY_LINE2[BUFFER_SIZE] ={0};
//char __attribute__((coherent, aligned(16))) DISPLAY_LINE3[BUFFER_SIZE] ={0};
//char __attribute__((coherent, aligned(16))) DISPLAY_LINE4[BUFFER_SIZE] ={0};
//char __attribute__((coherent, aligned(16))) Conversion = 0;
//static GFX_INDEX            gfxIndex=0;
////int RX_LINE_POINT = 0;


void display_tasks(void);

void display_tasks(void)
{
//    int Red, Blue, Green;
//    if(BT_DISPLAY_STATS.DisplayUpdate == 1 )
//    {
//        if(BT_DISPLAY_STATS.BlueTooth_Status == NotVisable)
//        {
////            GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &NotDiscoverable);
//        }
//        if(BT_DISPLAY_STATS.BlueTooth_Status == NotPaired_NotConnected)
//        {
//            GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
//            GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &NO_PAIR_NO_CONNECTION);
//        }
//        if(BT_DISPLAY_STATS.BlueTooth_Status == Paired_NotConnected)
//        {
//            GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
//            GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &PAIRED);
//        }
//        if(BT_DISPLAY_STATS.BlueTooth_Status ==  Pared_Connected)
//        {
//            GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
//            GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &CONNECTED);
//        }
//        if(BT_DISPLAY_STATS.VLED_Update == 1)
//        {
//            Red = BT_DISPLAY_STATS.VLED_R;
//            Green = BT_DISPLAY_STATS.VLED_G;
//            Blue = BT_DISPLAY_STATS.VLED_B;
//            BT_DISPLAY_STATS.VLED_Update = 0;
//
//            GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(Red, Green, Blue));
//            GFX_RectangleFillDraw(GFX_INDEX_0, 161, (152 + APP_VIRTUAL_LED_Y_OFFSET), 199, (168 + APP_VIRTUAL_LED_Y_OFFSET));//Vitural RGB LED
//        }
//
//        if (BT_DISPLAY_STATS.BTMACFLG == 1)
//        {
//            GFX_GOL_StaticTextSet( hgcObj.pBroadCastNameObj, (char *) BT_DISPLAY_STATS.BTDEMONAME);
//            GFX_GOL_ObjectStateSet(hgcObj.pBroadCastNameObj,GFX_GOL_STATICTEXT_DRAW_STATE);
//            GFX_GOL_StaticTextSet( hgcObj.pMacIdObj, (char *) BT_DISPLAY_STATS.BTMACADD);
//            GFX_GOL_ObjectStateSet(hgcObj.pMacIdObj, GFX_GOL_STATICTEXT_DRAW_STATE);
//            GFX_GOL_StaticTextSet( hgcObj.pHarmonyVersionObj, (char *) SYS_VERSION_STR);
//            GFX_GOL_ObjectStateSet(hgcObj.pHarmonyVersionObj, GFX_GOL_STATICTEXT_DRAW_STATE);
//
//            BT_DISPLAY_STATS.BTMACFLG = 0;
//        }
////******************************************************************************
//        if(BT_DISPLAY_STATS.VLED1 ==1)
//        {
//            GFX_ColorSet(gfxIndex, BRIGHTGREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 11, (152 + APP_VIRTUAL_LED_Y_OFFSET), 19, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #1
//        }
//        else
//        {
//            GFX_ColorSet(gfxIndex, GREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 11, (152 + APP_VIRTUAL_LED_Y_OFFSET), 19, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #1
//        }
//
//        if(BT_DISPLAY_STATS.VLED2 ==1)
//        {
//            GFX_ColorSet(gfxIndex, BRIGHTGREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 26, (152 + APP_VIRTUAL_LED_Y_OFFSET), 34, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #2
//        }
//        else
//        {
//            GFX_ColorSet(gfxIndex, GREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 26, (152 + APP_VIRTUAL_LED_Y_OFFSET), 34, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #2
//        }
//        if(BT_DISPLAY_STATS.VLED3 ==1)
//        {
//            GFX_ColorSet(gfxIndex, BRIGHTGREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 41, (152 + APP_VIRTUAL_LED_Y_OFFSET), 49, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #3
//        }
//        else
//        {
//            GFX_ColorSet(gfxIndex, GREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 41, (152 + APP_VIRTUAL_LED_Y_OFFSET), 49, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #3
//        }
//        if(BT_DISPLAY_STATS.VLED4 ==1)
//        {
//            GFX_ColorSet(gfxIndex, BRIGHTGREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 56, (152 + APP_VIRTUAL_LED_Y_OFFSET), 64, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #4
//        }
//        else
//        {
//            GFX_ColorSet(gfxIndex, GREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 56, (152 + APP_VIRTUAL_LED_Y_OFFSET), 64, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #4
//        }
//        if(BT_DISPLAY_STATS.VLED5 ==1)
//        {
//            GFX_ColorSet(gfxIndex, BRIGHTGREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 71, (152 + APP_VIRTUAL_LED_Y_OFFSET), 79, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #5
//        }
//        else
//        {
//            GFX_ColorSet(gfxIndex, GREEN);
//            GFX_RectangleFillDraw(GFX_INDEX_0, 71, (152 + APP_VIRTUAL_LED_Y_OFFSET), 79, (168 + APP_VIRTUAL_LED_Y_OFFSET)); //Vitural LED #5
//        }
//
////******************************************************************************
////  Displays all RX text
////******************************************************************************
//        if(BT_DISPLAY_STATS.DISPLAY_ALL == 1)
//        {
//            BT_DISPLAY_STATS.PROCESS_TEXT =1;
//        }
//
//        if(BT_DISPLAY_STATS.PROCESS_TEXT ==1)
//        {
//            //copy memory buffer for Que effect
//            memcpy(DISPLAY_LINE4,DISPLAY_LINE3,BUFFER_SIZE);
//            memcpy(DISPLAY_LINE3,DISPLAY_LINE2,BUFFER_SIZE);
//            memcpy(DISPLAY_LINE2,DISPLAY_LINE1,BUFFER_SIZE);
//            memcpy(DISPLAY_LINE1,BT_DISPLAY_STATS.RXBUFFER_DATA,BUFFER_SIZE);
//
//            GFX_GOL_StaticTextSet( hgcObj.pBuffer1Obj, (char *) &DISPLAY_LINE1);
//            GFX_GOL_ObjectStateSet(hgcObj.pBuffer1Obj,GFX_GOL_STATICTEXT_DRAW_STATE);
//            GFX_GOL_StaticTextSet( hgcObj.pBuffer2Obj, (char *) &DISPLAY_LINE2);
//            GFX_GOL_ObjectStateSet(hgcObj.pBuffer2Obj, GFX_GOL_STATICTEXT_DRAW_STATE);
//            GFX_GOL_StaticTextSet( hgcObj.pBuffer3Obj, (char *) &DISPLAY_LINE3);
//            GFX_GOL_ObjectStateSet(hgcObj.pBuffer3Obj,GFX_GOL_STATICTEXT_DRAW_STATE);
//            GFX_GOL_StaticTextSet( hgcObj.pBuffer4Obj, (char *) &DISPLAY_LINE4);
//            GFX_GOL_ObjectStateSet(hgcObj.pBuffer4Obj, GFX_GOL_STATICTEXT_DRAW_STATE);
//
//            BT_DISPLAY_STATS.PROCESS_TEXT = 0;
//        }
//        if (BT_DISPLAY_STATS.BTPORTFLAG ==1)
//        {
//            itoa(&Conversion,BT_DISPLAY_STATS.BTPORT,10);
//            GFX_GOL_StaticTextSet( hgcObj.pBtConnectedObj, &Conversion);
//            GFX_GOL_ObjectStateSet(hgcObj.pBtConnectedObj, GFX_GOL_STATICTEXT_DRAW_STATE);  
//         }
//      
//
////******************************************************************************
//        BT_DISPLAY_STATS.DisplayUpdate = 0;
//    }

}


