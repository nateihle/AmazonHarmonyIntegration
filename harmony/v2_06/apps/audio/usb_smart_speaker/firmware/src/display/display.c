/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   display_tasks.c

  Summary:
    Contains the functional implementation of display task. 

  Description:
    This file contains the functional implementation of data parsing functions

    The following GFX 2.0 objects are updated:

        laScheme           text_label;
        laScheme           _default;
        laScheme           track_time;
        laScheme           image_button;
        laScheme           filled_circle;
        laScheme           track_info;

        laRectangleWidget* RectangleWidget1;

        laLabelWidget*     DemoName;
        laLabelWidget*     MHVersion;
        laLabelWidget*     BtName;
        laLabelWidget*     BtMacAddr;

        laTextFieldWidget* TextFieldWidget1;

        laImageWidget*     IconConnected;
        laImageWidget*     IconNoConnect;
        laImageWidget*     IconPaired;
        laImageWidget*     ImageWidget1;
   
    Rectangles
        laRectangleWidget* VLED<4-0>          
   
        laRectangleWidget* RGBLed
 

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

_BT_DISPLAY_STATS BT_DISPLAY_STATS;

static GFXU_FontAsset * _lineFontAsset;
char __attribute__((coherent, aligned(16))) DISPLAY_LINE1[BUFFER_SIZE] ={0};
char __attribute__((coherent, aligned(16))) DISPLAY_LINE2[BUFFER_SIZE] ={0};
char __attribute__((coherent, aligned(16))) DISPLAY_LINE3[BUFFER_SIZE] ={0};
char __attribute__((coherent, aligned(16))) DISPLAY_LINE4[BUFFER_SIZE] ={0};
char __attribute__((coherent, aligned(16))) Conversion = 0;
//static GFX_INDEX            gfxIndex=0;
//int RX_LINE_POINT = 0;

char * emptyStrArray = "    ";
laString emptyLineStr;


//******************************************************************************
// display_init()
//******************************************************************************
void display_init(_BT_DISPLAY_STATS * BT_DISPLAY_STATS)
{
    laString tempStr;  
    GFXU_FontAsset * fontAsset;



    BT_DISPLAY_STATS->BTPORT =0;
    BT_DISPLAY_STATS->BTMACFLG = 0;

    fontAsset =  GFXU_StringFontIndexLookup(&stringTable,
                                            string_MHVersion,
                                            language_default);

    tempStr = laString_CreateFromCharBuffer((char *) SYS_VERSION_STR, fontAsset);
    laLabelWidget_SetText(MHVersion, tempStr);
    laString_Destroy(&tempStr);

    //Connected Icon
    laWidget_SetVisible(&IconNoConnect->widget, true);
    laWidget_SetVisible(&IconConnected->widget, false);
    laWidget_SetVisible(&IconPaired->widget, false);

    _lineFontAsset =  GFXU_StringFontIndexLookup(&stringTable,
                                                 string_Blank1,
                                                 language_default);

    emptyLineStr = laString_CreateFromCharBuffer((char *) emptyStrArray, 
                                                 _lineFontAsset);

    //VLED<4-0>
    laWidget_SetScheme((laWidget*)VLED0, &_default);
    laWidget_SetScheme((laWidget*)VLED1, &_default);
    laWidget_SetScheme((laWidget*)VLED2, &_default);
    laWidget_SetScheme((laWidget*)VLED3, &_default);
    laWidget_SetScheme((laWidget*)VLED4, &_default);

    //RGBVLED
    laWidget_SetScheme((laWidget*)VLED4, &RGBLedScheme);

}

//******************************************************************************
// display_tasks()
//
// Description:
//   Updates the display when BT_DISPLAY_STATS.DisplayUpdate is True.
//
//   1. BlueTooth_Status:  
//         NotVisable, NotPaired_NoConnect, Paired_NoConnect
//         Pared_Connected.
//   2. VLED_Update:
//         VLED_<R,G,B>
//   3. BTMACFLG:
//         BTDEMONAME, BTMACADD
//   4  VLED<1,2,3,4,5)
//
//   Update the RX Text Display when BT_DISPLAY_STATS.DISPLAY_ALL is true.
//   --> BT_DISPLAY_STATS.PROCESS_TEXT true.
//
//   Update the connected icon BT_DISPLAY_STATS.BTPORTFLAG is true.
//
//******************************************************************************
void display_tasks(_BT_DISPLAY_STATS * BT_DISPLAY_STATS)
{
    if(BT_DISPLAY_STATS->DisplayUpdate == 1 )
    {
        laString tempStr;  

        if(BT_DISPLAY_STATS->BlueTooth_Status == NotVisable)
        {
            //GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &NotDiscoverable);
            laWidget_SetVisible(&IconNoConnect->widget, true);
            laWidget_SetVisible(&IconPaired->widget, false);
            laWidget_SetVisible(&IconConnected->widget, false);

        }
        if(BT_DISPLAY_STATS->BlueTooth_Status == NotPaired_NotConnected)
        {
            //enable transparency
            //GFX_TransparentColorEnable(GFX_INDEX_0, 
            //                           GFX_RGBConvert(0x1D, 0x2E, 0x3C)); 
            //GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &NO_PAIR_NO_CONNECTION);
            laWidget_SetVisible(&IconNoConnect->widget, true);
            laWidget_SetVisible(&IconPaired->widget, false);
            laWidget_SetVisible(&IconConnected->widget, false);
        }
        if(BT_DISPLAY_STATS->BlueTooth_Status == Paired_NotConnected)
        {
            // enable transparency
            //GFX_TransparentColorEnable(GFX_INDEX_0, 
            //                           GFX_RGBConvert(0x1D, 0x2E, 0x3C)); 
            //GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &PAIRED);
            laWidget_SetVisible(&IconNoConnect->widget, false);
            laWidget_SetVisible(&IconPaired->widget, true);
            laWidget_SetVisible(&IconConnected->widget, false);
        }
        if(BT_DISPLAY_STATS->BlueTooth_Status ==  Paired_Connected)
        {
            //GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
            //GFX_ImageDraw(gfxIndex, 190, 0, (GFX_RESOURCE_HDR *) &CONNECTED);
            laWidget_SetVisible(&IconNoConnect->widget, false);
            laWidget_SetVisible(&IconPaired->widget, false);
            laWidget_SetVisible(&IconConnected->widget, true);
        }
        if(BT_DISPLAY_STATS->VLED_Update == 1)
        {
            uint8_t Red = BT_DISPLAY_STATS->VLED_R;
            uint8_t Green = BT_DISPLAY_STATS->VLED_G;
            uint8_t Blue = BT_DISPLAY_STATS->VLED_B;
            uint32_t Rgb888Color = 0;
            uint32_t Rgb565Color = 0;
            BT_DISPLAY_STATS->VLED_Update = 0;

            //Virtual RGB LED
            //GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(Red, Green, Blue));
            //GFX_RectangleFillDraw(GFX_INDEX_0, 
            //                      161, (152 + APP_VIRTUAL_LED_Y_OFFSET), 
            //                      199, (168 + APP_VIRTUAL_LED_Y_OFFSET));
            //GFX 565
            Rgb888Color = (Red<<16) + (Green<<8) + Blue; 
            Rgb565Color = GFX_ColorConvert(GFX_COLOR_MODE_RGB_888, GFX_COLOR_MODE_RGB_565,
                                           Rgb888Color);
            RGBLedScheme.base= Rgb565Color;
            RGBLedScheme.foreground = Rgb565Color;
            RGBLedScheme.background = Rgb565Color;
            laWidget_SetScheme((laWidget*)RGBLed, &RGBLedScheme);
            laWidget_Invalidate((laWidget*)RGBLed);
        }

        if (BT_DISPLAY_STATS->BTMACFLG == 1)
        {
            //Demo Name
            tempStr = laString_CreateFromCharBuffer(
                            (char *) BT_DISPLAY_STATS->BTDEMONAME, 
                            &LiberationSans12Italic);
            laLabelWidget_SetText(BtName, tempStr);
            laString_Destroy(&tempStr);

            //MAC Address
            tempStr = laString_CreateFromCharBuffer(
                            (char *) BT_DISPLAY_STATS->BTMACADD, 
                            &LiberationSans12);
            laLabelWidget_SetText(BtMacAddr, tempStr);
            laString_Destroy(&tempStr);

            BT_DISPLAY_STATS->BTMACFLG = 0;
        }

        if(BT_DISPLAY_STATS->VLED1 ==1)
        {
            laWidget_SetScheme((laWidget*)VLED0, &filled_circle);
        }
        else
        {
            laWidget_SetScheme((laWidget*)VLED0, &_default);
        }

        if(BT_DISPLAY_STATS->VLED2 ==1)
        {
            laWidget_SetScheme((laWidget*)VLED1, &filled_circle);
        }
        else
        {
            laWidget_SetScheme((laWidget*)VLED1, &_default);
        }
        if(BT_DISPLAY_STATS->VLED3 ==1)
        {
            laWidget_SetScheme((laWidget*)VLED2, &filled_circle);
        }
        else
        {
            laWidget_SetScheme((laWidget*)VLED2, &_default);
        }
        if(BT_DISPLAY_STATS->VLED4 ==1)
        {
            laWidget_SetScheme((laWidget*)VLED3, &filled_circle);
        }
        else
        {
            laWidget_SetScheme((laWidget*)VLED3, &_default);
        }
        if(BT_DISPLAY_STATS->VLED5 ==1)
        {
            //GFX_ColorSet(gfxIndex, BRIGHTGREEN);
            //Virtual LED #5
            //GFX_RectangleFillDraw(GFX_INDEX_0, 71, 
            //                      (152 + APP_VIRTUAL_LED_Y_OFFSET), 79, 
            //                      (168 + APP_VIRTUAL_LED_Y_OFFSET)); 
            laWidget_SetScheme((laWidget*)VLED4, &filled_circle);
        }
        else
        {
            //GFX_ColorSet(gfxIndex, GREEN);
            //Virtual LED #5
            //GFX_RectangleFillDraw(GFX_INDEX_0, 71, 
            //                      (152 + APP_VIRTUAL_LED_Y_OFFSET), 79, 
            //                      (168 + APP_VIRTUAL_LED_Y_OFFSET)); 
            laWidget_SetScheme((laWidget*)VLED4, &_default);
        }

        //----------------------------------------------------------------------
        //  Displays all RX text
        //----------------------------------------------------------------------
        if(BT_DISPLAY_STATS->DISPLAY_ALL == 1)
        {
            BT_DISPLAY_STATS->PROCESS_TEXT =1;
        }

        if(BT_DISPLAY_STATS->PROCESS_TEXT ==1)
        {
            //4 Deep text buffer memory display 
            memcpy(DISPLAY_LINE4, DISPLAY_LINE3,                   BUFFER_SIZE);
            memcpy(DISPLAY_LINE3, DISPLAY_LINE2,                   BUFFER_SIZE);
            memcpy(DISPLAY_LINE2, DISPLAY_LINE1,                   BUFFER_SIZE);
            memcpy(DISPLAY_LINE1, BT_DISPLAY_STATS->RXBUFFER_DATA, BUFFER_SIZE);
            
            //Line4
            if (strlen(DISPLAY_LINE4) != 0)
            {
                tempStr = laString_CreateFromCharBuffer(
                                (char *) DISPLAY_LINE4,
                                _lineFontAsset);
                laLabelWidget_SetText(Line4, tempStr);
                laString_Destroy(&tempStr);
                laWidget_SetVisible(&(Line4->widget), LA_TRUE);
            }
            else
            {
                //laWidget_SetVisible(Line4->widget, LA_FALSE);
                //laLabelWidget_SetText(Line4, emptyLineStr);
            }

            //Line3
            if (strlen(DISPLAY_LINE3) != 0)
            {
                tempStr = laString_CreateFromCharBuffer(
                                (char *) DISPLAY_LINE3,
                                _lineFontAsset);
                laLabelWidget_SetText(Line3, tempStr);
                laString_Destroy(&tempStr);
                laWidget_SetVisible(&(Line3->widget), LA_TRUE);
            }
            else
            {
                //laLabelWidget_SetText(Line3, emptyLineStr);
            }

            //Line2
            if (strlen(DISPLAY_LINE2) != 0)
            {
                tempStr = laString_CreateFromCharBuffer(
                                (char *) DISPLAY_LINE2,
                                _lineFontAsset);
                laLabelWidget_SetText(Line2, tempStr);
                laString_Destroy(&tempStr);
                laWidget_SetVisible(&(Line2->widget), LA_TRUE);
            }
            else
            {
                //laLabelWidget_SetText(Line2, emptyLineStr);
            }

            //Line1
            tempStr = laString_CreateFromCharBuffer(
                            (char *) DISPLAY_LINE1,
                            _lineFontAsset);
            laLabelWidget_SetText(Line1, tempStr);
            laString_Destroy(&tempStr);
            laWidget_SetVisible(&(Line1->widget), LA_TRUE);

            BT_DISPLAY_STATS->PROCESS_TEXT = 0;
        }
        if (BT_DISPLAY_STATS->BTPORTFLAG ==1)
        {
            //itoa(&Conversion,BT_DISPLAY_STATS->BTPORT,10);
            //GFX_GOL_StaticTextSet( hgcObj.pBtConnectedObj, &Conversion);
            //GFX_GOL_ObjectStateSet(hgcObj.pBtConnectedObj, 
            //                       GFX_GOL_STATICTEXT_DRAW_STATE);  
        }
        BT_DISPLAY_STATS->DisplayUpdate = 0;
    }

} //End display_tasks()