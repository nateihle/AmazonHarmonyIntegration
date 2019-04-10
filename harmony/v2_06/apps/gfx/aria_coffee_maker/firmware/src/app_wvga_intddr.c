/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_wvga_intddr.h"
#include "app_splash.h"
#include "gfx/hal/gfx.h"
#include "framework/gfx/gfx_assets.h"
#include "framework/gfx/libaria/libaria_init.h"
#include "gfx/hal/inc/gfx_context.h"
#include <sys/kmem.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

int32_t minx_Left = 0;
int32_t maxx_Left = 0;

int32_t minx_Right = 0;
int32_t maxx_Right = 0;

int32_t miny = 0;
int32_t maxy = 0;

uint32_t* infoTextCacheAddress = NULL;

//These are used to keep track of the flash location of the info text images
//Needed to support dynamic caching
GFXU_ImageAsset text_asset_en;
GFXU_ImageAsset text_asset_fr;
GFXU_ImageAsset text_asset_ger;
GFXU_ImageAsset text_asset_ital;

static void touchDown(laWidget* widget, laInput_TouchDownEvent* evt)
{
    if (appData.moveLeftTrayIn || appData.moveLeftTrayOut || appData.moveRightTrayIn || appData.moveRightTrayOut)
    {
        evt->event.accepted = LA_FALSE;        
    }
    else
    {
        evt->event.accepted = LA_TRUE;
    }
}

static void touchUp(laWidget* widget, laInput_TouchUpEvent* evt)
{
    if (appData.moveLeftTrayIn || appData.moveLeftTrayOut || appData.moveRightTrayIn || appData.moveRightTrayOut)
    {
        evt->event.accepted = LA_FALSE;        
    }
    else
    {
        evt->event.accepted = LA_TRUE;
    }
}

static void touchMovedLeft(laWidget* widget, laInput_TouchMovedEvent* evt)
{
    int32_t dx;
    
    if (evt->x > GFX_ActiveContext()->display_info->rect.width / 2)
    {
        //Ignore touch request that is not in the correct position 
        evt->event.accepted = LA_FALSE;
        return;
    }
    
    dx = evt->x - evt->prevX;
    
    if (widget == (laWidget*)LeftTrayLid || widget == (laWidget*)DragPanel)
    {
		//This is to check if the movement is greater than 30 pixels
		if (abs(dx) > 30)
        {
			//If the movement is significant, we set the flag such that the remainder of the tray movement will
			//be handled by the state machine and the tray would slide in or out fully
			if (dx > 0)
            {
                appData.moveLeftTrayOut = true;            
            }
            else
            {
                appData.moveLeftTrayIn = true;            
            }
        }
    }

	//The movement is amplified to make sure the tray mimics the movement the finger as close as possible.
	//The value is derived from trial and error
	dx *= 5;
    
    if(DragPanel->rect.x + dx < minx_Left)
    {
        laWidget_SetPosition((laWidget*)DragPanel, minx_Left, DragPanel->rect.y);
    }        
    else if(DragPanel->rect.x + dx > maxx_Left)
    {
        laWidget_SetPosition((laWidget*)DragPanel, maxx_Left, DragPanel->rect.y);        
    }
    else
    {
        laWidget_Translate((laWidget*)DragPanel, dx, 0);
    }    
    
    evt->event.accepted = LA_TRUE;
}

static void touchMovedRight(laWidget* widget, laInput_TouchMovedEvent* evt)
{
    int32_t dx;
    
    dx = evt->x - evt->prevX;
    
    if (evt->x < GFX_ActiveContext()->display_info->rect.width / 2)
    {
        //Ignore touch request that is not in the correct position 
        evt->event.accepted = LA_FALSE;
        return;
    }

    if (widget == (laWidget*)RightTrayLid || widget == (laWidget*)DragPanelRight)
    {
		//This is to check if the movement is greater than 30 pixels
		if (abs(dx) > 30)
        {
			//If the movement is significant, we set the flag such that the remainder of the tray movement will
			//be handled by the state machine and the tray would slide in or out fully
			if (dx < 0)
            {
                appData.moveRightTrayOut = true;
            }
            else
            {
                appData.moveRightTrayIn = true;
            }                        
        }        
    }    

	//The movement is amplified to make sure the tray mimics the movement the finger as close as possible.
	//The value is derived from trial and error
	dx *= 5;
    
    if(DragPanelRight->rect.x + dx < minx_Right)
    {
        laWidget_SetPosition((laWidget*)DragPanelRight, minx_Right, DragPanelRight->rect.y);
    }        
    else if(DragPanelRight->rect.x + dx > maxx_Right)
    {
        laWidget_SetPosition((laWidget*)DragPanelRight, maxx_Right, DragPanelRight->rect.y);        
    }
    else
    {
        laWidget_Translate((laWidget*)DragPanelRight, dx, 0);
    }    
    
    evt->event.accepted = LA_TRUE;
}

static void touchMovedUpDown(laWidget* widget, laInput_TouchMovedEvent* evt)
{
    int32_t dy;
    
    dy = evt->y - evt->prevY;
    
	//The movement is amplified to make sure the vertical movement mimics the finger as close as possible.
	//The value is derived from trial and error
	dy *= 3;
    dy /= 2;
    
    if(InfoTextDragPanel->rect.y + dy < miny)
    {
        laWidget_SetPosition((laWidget*)InfoTextDragPanel, InfoTextDragPanel->rect.x, miny);
    }        
    else if(InfoTextDragPanel->rect.y + dy > maxy)
    {
        laWidget_SetPosition((laWidget*)InfoTextDragPanel, InfoTextDragPanel->rect.x, maxy);        
    }
    else
    {
        laWidget_Translate((laWidget*)InfoTextDragPanel, 0, dy);
    }    
    
    evt->event.accepted = LA_TRUE;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_ToggleGPU( void )
{
    appData.isGPUOn = !appData.isGPUOn;
    appData.gpuButtonNeedsUpdate = true;

    if (appData.isGPUOn)
    {
        GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);       
    }
    else
    {
        GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCU);               
    }    
}

void APP_RefreshLanguageImages ( void )
{
    static uint32_t recentLanguage = 0;

    if (laContext_GetActive()->activeScreen->id != 1)
        return;
    
    if (recentLanguage != appData.language)
    {
        laContext_SetStringLanguage(appData.language);
        recentLanguage = appData.language;        
    }

    if(appData.language == language_English)
    {
        laButtonWidget_SetPressedImage(ChangeLanguage, &USA);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &USA);        
    }
    else if(appData.language == language_French)
    {
        laButtonWidget_SetPressedImage(ChangeLanguage, &France);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &France);        
    }
    else if(appData.language == language_Italian)
    {
        laButtonWidget_SetPressedImage(ChangeLanguage, &Italy);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &Italy);        
    }
    else if(appData.language == language_German)
    {
        laButtonWidget_SetPressedImage(ChangeLanguage, &Germany);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &Germany);        
    }    
}

void APP_CycleLanguage ( void )
{
    uint32_t language = laContext_GetStringLanguage();
    
    if(language == language_English)
    {
        appData.language = language_French;        

        info_text_fr.header.dataAddress = text_asset_fr.header.dataAddress;
        info_text_fr.colorMode = text_asset_fr.colorMode;
        info_text_fr.compType = text_asset_fr.compType;
        info_text_fr.flags = text_asset_fr.flags;
        info_text_fr.mask = text_asset_fr.mask;
        info_text_fr.format = text_asset_fr.format;
        GFXU_PreprocessImage(&info_text_fr, (uint32_t)infoTextCacheAddress, GFX_COLOR_MODE_RGBA_8888, true);

        laButtonWidget_SetPressedImage(ChangeLanguage, &France);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &France);        
    }
    else if(language == language_French)
    {
        appData.language = language_Italian;                

        info_text_ital.header.dataAddress = text_asset_ital.header.dataAddress;
        info_text_ital.colorMode = text_asset_ital.colorMode;
        info_text_ital.compType = text_asset_ital.compType;
        info_text_ital.flags = text_asset_ital.flags;
        info_text_ital.mask = text_asset_ital.mask;
        info_text_ital.format = text_asset_ital.format;
        GFXU_PreprocessImage(&info_text_ital, (uint32_t)infoTextCacheAddress, GFX_COLOR_MODE_RGBA_8888, true);

        laButtonWidget_SetPressedImage(ChangeLanguage, &Italy);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &Italy);        
    }
    else if(language == language_Italian)
    {
        appData.language = language_German;                

        info_text_ger.header.dataAddress = text_asset_ger.header.dataAddress;
        info_text_ger.colorMode = text_asset_ger.colorMode;
        info_text_ger.compType = text_asset_ger.compType;
        info_text_ger.flags = text_asset_ger.flags;
        info_text_ger.mask = text_asset_ger.mask;
        info_text_ger.format = text_asset_ger.format;
        GFXU_PreprocessImage(&info_text_ger, (uint32_t)infoTextCacheAddress, GFX_COLOR_MODE_RGBA_8888, true);

        laButtonWidget_SetPressedImage(ChangeLanguage, &Germany);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &Germany);        
    }
    else if(language == language_German)
    {
        appData.language = language_English;                

        info_text_en.header.dataAddress = text_asset_en.header.dataAddress;
        info_text_en.colorMode = text_asset_en.colorMode;
        info_text_en.compType = text_asset_en.compType;
        info_text_en.flags = text_asset_en.flags;
        info_text_en.mask = text_asset_en.mask;
        info_text_en.format = text_asset_en.format;
        GFXU_PreprocessImage(&info_text_en, (uint32_t)infoTextCacheAddress, GFX_COLOR_MODE_RGBA_8888, true);

        laButtonWidget_SetPressedImage(ChangeLanguage, &USA);
        laButtonWidget_SetReleasedImage(ChangeLanguage, &USA);        
    }
}

void APP_TriggerBrewPressed ( void )
{
    int32_t idx = laListWheelWidget_GetSelectedItem(SizeSelect); 
    
    switch(idx)
    {
        case 0:
            BSP_LEDOn(BSP_LED_1);
            break;
        case 1:
            BSP_LEDOn(BSP_LED_2);
            break;
        case 2:
            BSP_LEDOn(BSP_LED_3);
            break;
        case 3:
            BSP_LEDOn(BSP_LED_D6);
            break;
        default:
            break;
    }        
}

void APP_TriggerBrew( void )
{
    BSP_LEDOff(BSP_LED_1);
    BSP_LEDOff(BSP_LED_2);
    BSP_LEDOff(BSP_LED_3);
    BSP_LEDOff(BSP_LED_D6);
}

void APP_CycleBeansBackground ( void )
{
    if (laImageWidget_GetImage(BackgroundImage) == &coffee_beans_1)
    {
        laLabelWidget_SetText(RoastLabel, laString_CreateFromID(string_Medium_Roast));
        laImageWidget_SetImage(BackgroundImage, &coffee_beans_2);        
    }
    else if (laImageWidget_GetImage(BackgroundImage) == &coffee_beans_2)
    {
        laLabelWidget_SetText(RoastLabel, laString_CreateFromID(string_Light_Roast));
        laImageWidget_SetImage(BackgroundImage, &coffee_beans_3);        
    }
    else if (laImageWidget_GetImage(BackgroundImage) == &coffee_beans_3)
    {
        laLabelWidget_SetText(RoastLabel, laString_CreateFromID(string_Dark_Roast));
        laImageWidget_SetImage(BackgroundImage, &coffee_beans_4);        
    }    
    else if (laImageWidget_GetImage(BackgroundImage) == &coffee_beans_4)
    {
        laLabelWidget_SetText(RoastLabel, laString_CreateFromID(string_Medium_Roast));
        laImageWidget_SetImage(BackgroundImage, &coffee_beans_1);        
    }    
}

void APP_GoToSplashState ( void )
{
    appData.state = APP_STATE_SPLASH;    
}

void APP_GoToInfoState( void )
{
    appData.state = APP_STATE_INFO;        
}

void APP_GoToMainState( void )
{
    appData.state = APP_STATE_MAIN;                     
    appData.gpuButtonNeedsUpdate = true;
}


void APP_CycleRightTrayAlpha( void )
{
    static uint32_t alpha = 225;
    int32_t idx = laListWheelWidget_GetSelectedItem(SizeSelect); 

    alpha -= 50;

    if (alpha < 115)
    {        
        alpha = 225;
    }    

    if (alpha == 225)
    {
        laListWheelWidget_RemoveAllItems(SizeSelect);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 0, &Coffee_Small_Hot);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 1, &Coffee_Medium_Hot);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 2, &Coffee_Grande_Hot);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 3, &Coffee_Venti_Hot);

        laButtonWidget_SetPressedImage(CoffeeButton, &temp_control_hot);
        laButtonWidget_SetReleasedImage(CoffeeButton, &temp_control_hot);        
    }
    else if (alpha == 175)
    {
        laListWheelWidget_RemoveAllItems(SizeSelect);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 0, &Coffee_Small_Medium);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 1, &Coffee_Medium_Medium);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 2, &Coffee_Grande_Medium);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 3, &Coffee_Venti_Medium);
        
        laButtonWidget_SetPressedImage(CoffeeButton, &temp_control_source);
        laButtonWidget_SetReleasedImage(CoffeeButton, &temp_control_source);        
    }
    else if (alpha == 125)
    {
        laListWheelWidget_RemoveAllItems(SizeSelect);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 0, &Coffee_Small_Warm);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 1, &Coffee_Medium_Warm);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 2, &Coffee_Grande_Warm);
        laListWheelWidget_AppendItem(SizeSelect);
        laListWheelWidget_SetItemIcon(SizeSelect, 3, &Coffee_Venti_Warm);
        
        laButtonWidget_SetPressedImage(CoffeeButton, &temp_control_warm);
        laButtonWidget_SetReleasedImage(CoffeeButton, &temp_control_warm);
    }
    
    laListWheelWidget_SetSelectedItem(SizeSelect, idx);
    
    if (laWidget_GetX(DragPanelRight) > minx_Right)
    {
        appData.moveRightTrayOut = true;
    }    
}

void APP_CycleLeftTrayAlpha( void )
{
    static uint32_t alpha = 225;
    
    alpha -= 50;
    
    if (alpha < 115)
    {
        alpha = 225;
    }    
    
    if (alpha == 225)
    {
        laButtonWidget_SetPressedImage(TeaButton, &tea_mode_icon_hot);
        laButtonWidget_SetReleasedImage(TeaButton, &tea_mode_icon_hot);        
    }
    else if (alpha == 175)
    {
        laButtonWidget_SetPressedImage(TeaButton, &tea_mode_icon);
        laButtonWidget_SetReleasedImage(TeaButton, &tea_mode_icon);        
    }
    else if (alpha == 125)
    {
        laButtonWidget_SetPressedImage(TeaButton, &tea_mode_icon_warm);
        laButtonWidget_SetReleasedImage(TeaButton, &tea_mode_icon_warm);
    }

    GFX_Set(GFXF_LAYER_ACTIVE, 1);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, alpha);

    GFX_Set(GFXF_LAYER_ACTIVE, 2);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, alpha);    
}

void APP_CycleBrewSize ( void )
{
    int32_t idx = laListWheelWidget_GetSelectedItem(SizeSelect); 
    
    switch(idx)
    {
        case 0:
            laButtonWidget_SetText(BrewButton, laString_CreateFromID(string_BrewEightOunce));            
            break;
        case 1:
            laButtonWidget_SetText(BrewButton, laString_CreateFromID(string_BrewTenOunce));            
            break;
        case 2:
            laButtonWidget_SetText(BrewButton, laString_CreateFromID(string_BrewTwelveOunce));            
            break;
        case 3:
            laButtonWidget_SetText(BrewButton, laString_CreateFromID(string_BrewSixteenOunce));            
            break;
        default:
            break;
    }
    
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_WVGA_INTDDR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.launched = false;
    appData.language = language_English;

    appData.isGPUOn = true;
    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);       
}


bool APP_Preload_Images ( void )
{
    static uint32_t* currentAddress = (uint32_t*)APP_PRELOAD_IMAGE_DDR_ADDRESS;
    GFXU_ImageAsset* imageAssets[33];
    uint32_t imageSize;
    int i = 0;
    static bool cached = false;
    GFX_Bool needsPad = GFX_FALSE;
    
    if (cached == true)
    {
        return cached;
    }
    
    imageAssets[0] = &tray_left_pressed;
    imageAssets[1] = &tray_left;
    imageAssets[2] = &tray_right_pressed;
    imageAssets[3] = &tray_right;

    imageAssets[4] = &USA;
    imageAssets[5] = &Italy;
    imageAssets[6] = &France;
    imageAssets[7] = &Germany;

    imageAssets[8] = &tea_mode_icon;
    imageAssets[9] = &tea_mode_icon_hot;
    imageAssets[10] = &tea_mode_icon_warm;
    
    imageAssets[11] = &temp_control_hot;
    imageAssets[12] = &temp_control_source;
    imageAssets[13] = &temp_control_warm;

    imageAssets[14] = &PlainButton;
    imageAssets[15] = &OvalButton;
    imageAssets[16] = &OvalButtonPressed;

    imageAssets[17] = &coffee_bean_button;
    imageAssets[18] = &question_button;
    imageAssets[19] = &left_right_touch_icon_small;
    
    imageAssets[20] = &Coffee_Grande_Hot;
    imageAssets[21] = &Coffee_Grande_Medium;
    imageAssets[22] = &Coffee_Grande_Warm;

    imageAssets[23] = &Coffee_Medium_Hot;
    imageAssets[24] = &Coffee_Medium_Medium;
    imageAssets[25] = &Coffee_Medium_Warm;

    imageAssets[26] = &Coffee_Small_Hot;
    imageAssets[27] = &Coffee_Small_Medium;
    imageAssets[28] = &Coffee_Small_Warm;
    
    imageAssets[29] = &Coffee_Venti_Hot;
    imageAssets[30] = &Coffee_Venti_Medium;
    imageAssets[31] = &Coffee_Venti_Warm;
    
    imageAssets[32] = &info_text_en;

    text_asset_en = info_text_en;
    text_asset_fr = info_text_fr;
    text_asset_ger = info_text_ger;
    text_asset_ital = info_text_ital;


    for (i = 0; i < 33; ++i)
    {       
        imageSize = imageAssets[i]->width * imageAssets[i]->height * 4;
        needsPad = GFX_FALSE;
        
        //Padding calculation
        if (i >= 4 && i <= 7)
        {
            imageSize = 65536;
            needsPad = GFX_TRUE;
        }
        
        if (i == 32)
		{
            infoTextCacheAddress = currentAddress;   
            imageSize = 2097152;
            needsPad = GFX_TRUE;
		}

        GFXU_PreprocessImage(imageAssets[i], (uint32_t)currentAddress, GFX_COLOR_MODE_RGBA_8888, needsPad);
        currentAddress += imageSize;
    }
    
    cached = true;
    return cached;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_WVGA_INTDDR_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
               
            if (appInitialized)
            {
                appData.state = APP_STATE_PRELOAD_IMAGES;
            }
            break;
        }

        case APP_STATE_PRELOAD_IMAGES:
        {
            APP_Preload_Images();
            appData.state = APP_STATE_SPLASH;
            break;
        }
        
        case APP_STATE_SPLASH:
        {            
           if (laContext_GetActive()->activeScreen->id != 0)
             break;
           
            if (APP_IsMultiSplashScreenComplete())
            {
                if (!appData.launched)
                {
                    appData.state = APP_STATE_MAIN;
                    appData.launched = true;
                    appData.gpuButtonNeedsUpdate = true;                    

                    laContext_SetActiveScreen(1);
                    laContext_SetStringLanguage(appData.language);
                }
                else
                {
                    appData.state = APP_STATE_INFO;
                    laContext_SetActiveScreen(2);
                }
            }        
            break;
        }
        
        case APP_STATE_MAIN:
        {
            // Do not continue to trigger any redraw if any layer hasn't been completely drawn
            if (laContext_IsDrawing())
                break;
                
            if (laContext_GetActive()->activeScreen->id != 1)
                break;

            if (DragPanel != NULL && LeftTrayLid != NULL
                    && DragPanelRight != NULL && RightTrayLid != NULL)
            {
                APP_RefreshLanguageImages();
                
                if (appData.gpuButtonNeedsUpdate == true)
                {                    
                    if (appData.isGPUOn == true)
                    {
                        if (GPUButtonLabel != NULL)
                        {
                            laLabelWidget_SetText(GPUButtonLabel, laString_CreateFromID(string_GPU_On));            
                        }        
                    }
                    else
                    {
                        if (GPUButtonLabel != NULL)
                        {
                            laLabelWidget_SetText(GPUButtonLabel, laString_CreateFromID(string_GPU_Off));            
                        }        
                    }
                    appData.gpuButtonNeedsUpdate = false;                    
                }
                
                minx_Left = PanelWidget2->rect.width * -1;
                maxx_Left = 0;
                minx_Right = GFX_ActiveContext()->display_info->rect.width - DragPanelRight->rect.width;
                maxx_Right = GFX_ActiveContext()->display_info->rect.width - RightTrayLid->widget.rect.width;                

                laWidget_OverrideTouchMovedEvent((laWidget*)LeftTrayLid, &touchMovedLeft);
                laWidget_OverrideTouchMovedEvent((laWidget*)RightTrayLid, &touchMovedRight);

                laWidget_OverrideTouchDownEvent((laWidget*)DragPanel, &touchDown);
                laWidget_OverrideTouchMovedEvent((laWidget*)DragPanel, &touchMovedLeft);
                laWidget_OverrideTouchUpEvent((laWidget*)DragPanel, &touchUp);                

                laWidget_OverrideTouchDownEvent((laWidget*)DragPanelRight, &touchDown);
                laWidget_OverrideTouchMovedEvent((laWidget*)DragPanelRight, &touchMovedRight);
                laWidget_OverrideTouchUpEvent((laWidget*)DragPanelRight, &touchUp);
              
                if (appData.moveRightTrayOut)
                {
                    if (laWidget_GetX(DragPanelRight) > (minx_Right + 5))
                    {
                        laWidget_Translate((laWidget*)DragPanelRight, (minx_Right - laWidget_GetX(DragPanelRight))*3/4, 0);
                    }
                    else
                    {
                        laWidget_SetPosition((laWidget*)DragPanelRight, minx_Right, DragPanelRight->rect.y);        
                        appData.moveRightTrayOut = false;
                    }
                }
                
                if (appData.moveRightTrayIn)
                {
                    if (laWidget_GetX(DragPanelRight) < (maxx_Right - 5))
                    {
                        laWidget_Translate((laWidget*)DragPanelRight, (maxx_Right - laWidget_GetX(DragPanelRight))*3/4, 0);
                    }
                    else
                    {
                        laWidget_SetPosition((laWidget*)DragPanelRight, maxx_Right, DragPanelRight->rect.y);        
                        appData.moveRightTrayIn = false;
                    }
                }

                if (appData.moveLeftTrayOut)
                {
                    if (laWidget_GetX(DragPanel) < (maxx_Left - 5))
                    {
                        laWidget_Translate((laWidget*)DragPanel, (maxx_Left - laWidget_GetX(DragPanel))*3/4, 0);
                    }
                    else
                    {
                        laWidget_SetPosition((laWidget*)DragPanel, maxx_Left, DragPanel->rect.y);        
                        appData.moveLeftTrayOut = false;
                    }
                }

                if (appData.moveLeftTrayIn)
                {
                    if (laWidget_GetX(DragPanel) > (minx_Left + 5))
                    {
                        laWidget_Translate((laWidget*)DragPanel, (minx_Left - laWidget_GetX(DragPanel))*3/4, 0);
                    }
                    else
                    {
                        laWidget_SetPosition((laWidget*)DragPanel, minx_Left, DragPanel->rect.y);        
                        appData.moveLeftTrayIn = false;
                    }
                }                
            }
            break;
        }
        
        case APP_STATE_INFO:
        {
             // Do not continue to trigger any redraw if any layer hasn't been completely drawn
            if (laContext_IsDrawing())
                break;

            if (laContext_GetActive()->activeScreen->id != 2)
                break;
            
            if (InfoTextDragPanel != NULL)
            {
                miny = 0 - InfoTextDragPanel->rect.height;
                maxy = 0;

                laWidget_OverrideTouchDownEvent((laWidget*)InfoTextDragPanel, &touchDown);
                laWidget_OverrideTouchMovedEvent((laWidget*)InfoTextDragPanel, &touchMovedUpDown);
                laWidget_OverrideTouchUpEvent((laWidget*)InfoTextDragPanel, &touchUp);                
            }
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
