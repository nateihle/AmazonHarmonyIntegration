/* **************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_control_single_layer.c

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
 ************************************************************************** */

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

/* **************************************************************************
   Section: Included Files                                                    
 ************************************************************************** */


/* This section lists the other files that are included in this file.
 */

#include "gfx/hal/inc/gfx_context.h"
#include "framework/gfx/libaria/libaria_init.h"

#include "app.h"
#include "app_control_common.h"

#define DEFAULT_BYTES_PER_PIXEL  2
#define NUM_ROWS_BUFFERED 128
#define CONTROL_LAYER_ALPHA_MAX 200
#define CONTROL_LAYER_ALPHA_DEC 5
#define START_HIDE_CONTROL_MIN_FRAMES 10

#define VIDEO_LAYER_ID      0
#define MESSAGE_LAYER_ID    1
#define CONTROL_LAYER_ID    2

static uint8_t __attribute__((coherent, aligned(16))) readBuffer[
                                480 * 
                                NUM_ROWS_BUFFERED * 
                                DEFAULT_BYTES_PER_PIXEL];

/* **************************************************************************
  Section: File Scope or Global Data                                         
 ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

int appControlShowControlPanel(APP_CONTROL * appControl)
{
    
    if (appControl->playbackScreenID == PlayBackScreen_ID)
    {
        appControl->playbackScreenHeight =
                appControl->gfxContext->display_info->rect.height;
    }
    
    // Reduce height by size of draw surface
    laWidget_SetHeight((laWidget*) appControl->touchPanelWidget,
            appControl->playbackScreenHeight);
    laWidget_Invalidate((laWidget*) appControl->touchPanelWidget);
    
    laWidget_SetVisible((laWidget *) appControl->fpsPanel, appControl->showMetrics);
    
    //Show the control panel
    laWidget_SetVisible((laWidget *) appControl->controlPanel, LA_TRUE);
    
    //Let App know Control Panel is enabled
    appControl->playBackControlState = APP_PLAYBACK_CONTROL_VISIBLE;
    
    GFX_Set(GFXF_LAYER_ACTIVE, CONTROL_LAYER_ID);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, CONTROL_LAYER_ALPHA_MAX, GFX_FALSE);
    
    return 0;
    
}

APP_HIDE_CONTROL_STATUS appControlAutoHideControlPanel(APP_CONTROL * appControl)
{
    static int controlPanelLayerAlpha = CONTROL_LAYER_ALPHA_MAX;
    static int lastFrame;
    APP_HIDE_CONTROL_STATUS status = APP_HIDE_CONTROL_IN_PROGRESS;
    
    switch (appControl->playBackControlState)
    {
        case APP_PLAYBACK_CONTROL_HIDING:
        {
            appControl->playBackControlState = APP_PLAYBACK_CONTROL_HIDING;

            //Start hiding only after some frames
            if (appControl->frameNum > START_HIDE_CONTROL_MIN_FRAMES)
            {
                //Frame has changed, hide the controls a bit
                if (lastFrame != appControl->frameNum) 
                {
                    controlPanelLayerAlpha -= CONTROL_LAYER_ALPHA_DEC;
                    if (controlPanelLayerAlpha <= 20) 
                    {
                        appControl->playBackControlState = APP_PLAYBACK_CONTROL_HIDDEN;
                    } 
                    else 
                    {
                        GFX_Set(GFXF_LAYER_ACTIVE, CONTROL_LAYER_ID);
                        GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, controlPanelLayerAlpha);
                    }

                    lastFrame = appControl->frameNum;
                }
            }
            
            break;
        }
        case APP_PLAYBACK_CONTROL_VISIBLE:
        {
            appControl->playbackScreenHeight = 
                    appControl->gfxContext->display_info->rect.height;
            appControl->playBackControlState = APP_PLAYBACK_CONTROL_HIDING;
            
            lastFrame = appControl->frameNum;
            
            controlPanelLayerAlpha = CONTROL_LAYER_ALPHA_MAX;
            GFX_Set(GFXF_LAYER_ACTIVE, CONTROL_LAYER_ID);
            GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, controlPanelLayerAlpha, GFX_FALSE);

            break;
        }
        case APP_PLAYBACK_CONTROL_HIDDEN:
        {
            controlPanelLayerAlpha = 0;
            appControlHideControlPanel(appControl);
            break;
        }
        default:
            break;
    }
    
    if (controlPanelLayerAlpha == 0)
    {
        status = APP_HIDE_CONTROL_DONE;
    }
    
    return status;
}

APP_HIDE_CONTROL_STATUS appControlHideControlPanel(APP_CONTROL * appControl)
{
    appControl->playbackScreenHeight =
            appControl->gfxContext->display_info->rect.height;
    
    // Set draw Surface Area size to full
    laWidget_SetHeight((laWidget*) appControl->touchPanelWidget,
                        appControl->playbackScreenHeight);
    laWidget_Invalidate((laWidget*) appControl->touchPanelWidget);
    
    //Hide the control panel
    laWidget_SetVisible((laWidget *) appControl->controlPanel, LA_FALSE);
    
    //Hide the metrics panel
    laWidget_SetVisible((laWidget *) appControl->fpsPanel, LA_FALSE);
    
    //Let App know Control Panel is disabled
    appControl->playBackControlState = APP_PLAYBACK_CONTROL_HIDDEN;
    
    GFX_Set(GFXF_LAYER_ACTIVE, CONTROL_LAYER_ID);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, 0, GFX_FALSE);
    
    return APP_HIDE_CONTROL_DONE;
}

// Config-specific playback initializations
int appControlPlaybackInit(APP_CONTROL * appControl)
{
    //Set video frame (base) to RGB565
    GFX_Set(GFXF_LAYER_ACTIVE, VIDEO_LAYER_ID);
    GFX_Set(GFXF_COLOR_MODE, GFX_COLOR_MODE_RGB_565);   
    
    //Set Control Panel layer to RGBA8888
    GFX_Set(GFXF_LAYER_ACTIVE, MESSAGE_LAYER_ID);
    GFX_Set(GFXF_COLOR_MODE, GFX_COLOR_MODE_RGBA_8888);
    
    GFX_Set(GFXF_LAYER_ACTIVE, CONTROL_LAYER_ID);
    GFX_Set(GFXF_COLOR_MODE, GFX_COLOR_MODE_RGBA_8888);
    
    //BlackOut the video layer framebuffer
    memset(appControl->videoFrameBuffer, 0x0,
           appControl->gfxContext->display_info->rect.width * 
           appControl->gfxContext->display_info->rect.height *
           appControl->bytesPerPixel);
    
    return 0;
}

// Config-specific application initializations
int appControlInit(APP_CONTROL * appControl)
{
    appControl->videoFrameBuffer = 
            appControl->gfxContext->layer.layers[0].buffers[0].pb.pixels;
    
    appControl->bytesPerPixel = DEFAULT_BYTES_PER_PIXEL;
    appControl->linesBuffered = NUM_ROWS_BUFFERED;
    
    appControl->fileDataBuffer = (uint8_t *) readBuffer;

    return 0;
}

int appControlShowMessage(APP_CONTROL * appControl, unsigned int stringID)
{
    laWidget_SetVisible((laWidget*) appControl->playbackMessageLabel,
                        LA_TRUE);
    
    laLabelWidget_SetText(appControl->playbackMessageLabel,
                          laString_CreateFromID(stringID));
    
    GFX_Set(GFXF_LAYER_ACTIVE, MESSAGE_LAYER_ID);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, 255, GFX_FALSE);
    
    return 0;
}

int appControlHideMessage(APP_CONTROL * appControl)
{
    GFX_Set(GFXF_LAYER_ACTIVE, MESSAGE_LAYER_ID);
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, 0, GFX_FALSE);
    
    return 0;
}


/* *****************************************************************************
 End of File
 */