/***************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_process_events_main.c

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
 ***************************************************************************/

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

/****************************************************************************
Section: Included Files                                                    
 ***************************************************************************/

#include "gfx/hal/inc/gfx_context.h"
#include "framework/gfx/libaria/libaria_init.h"

#include "app.h"
#include "app_splash.h"
#include "app_control_common.h"


/***************************************************************************
  Section: File Scope or Global Data                                        
 ***************************************************************************/

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* **************************************************************************
  Section: Local Functions                                                   
 ***************************************************************************/

/*  A brief description of a section can be given directly below the section
    banner.
 */

int processEventsMainScreen(APP_DATA * appData, APP_CONTROL * appControl) 
{
    //Process internal events
    if (APP_CHECK_EVENT(appData->appInternalEvent, APP_EVENT_INT_NO_MEDIA))
    {
        laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
        APP_ClearInternalEvent(APP_EVENT_INT_NO_MEDIA);
    }
    else if (APP_CHECK_EVENT(appData->appInternalEvent, APP_EVENT_INT_STATE_SHOW_SCREEN))
    {
        laContext_SetActiveScreen(MainScreen_ID);
        APP_SetInternalEvent(APP_EVENT_INT_STATE_INIT);
        
        APP_ClearInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
    }
    else if (APP_CHECK_EVENT(appData->appInternalEvent, APP_EVENT_INT_STATE_INIT))
    {
        //Nothing to do
        APP_ClearInternalEvent(APP_EVENT_INT_STATE_INIT);
    }
    
    //Process user events
    if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_START_USB_PLAY))
    {
        if (appControl->usbDeviceConnected == 0)
        {
            laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
        }
        else
        {
            SYS_FS_CurrentDriveSet("/mnt/usb");
            appData->state = APP_STATE_PLAYING;
            APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
        }
        APP_ClearUserEvent(APP_EVENT_USER_START_USB_PLAY);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_START_SD_PLAY))
    {
        if (appControl->sdcardDeviceConnected == 0)
        {
            laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
        }
        else
        {
            SYS_FS_CurrentDriveSet("/mnt/sdcard");
            appData->state = APP_STATE_PLAYING;
            APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
        }

        APP_ClearUserEvent(APP_EVENT_USER_START_SD_PLAY);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_OPEN_SETTINGS))
    {
        // Go to Settings page/state, set event to initialize settings values
        appData->state = APP_STATE_SETTINGS;
        APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
        
        APP_ClearUserEvent(APP_EVENT_USER_OPEN_SETTINGS);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_OPEN_HELP))
    {
        appData->state = APP_STATE_HELP;
        APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
        
        APP_ClearUserEvent(APP_EVENT_USER_OPEN_HELP);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_USB_INSERTED))
    {
        laWidget_SetVisible((laWidget *) PlayUSBButton, LA_TRUE);
        laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_FALSE);
        
        APP_ClearUserEvent(APP_EVENT_USER_USB_INSERTED);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SD_INSERTED))
    {
        laWidget_SetVisible((laWidget *) PlaySDButton, LA_TRUE);
        laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_FALSE);
        
        APP_ClearUserEvent(APP_EVENT_USER_SD_INSERTED);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_USB_REMOVED))
    {
        laWidget_SetVisible((laWidget *) PlayUSBButton, LA_FALSE);

        if ((appControl->usbDeviceConnected == 0) && 
            (appControl->sdcardDeviceConnected == 0))
        {
            laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
        }

        APP_ClearUserEvent(APP_EVENT_USER_USB_REMOVED);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SD_REMOVED))
    {
        laWidget_SetVisible((laWidget *) PlaySDButton, LA_FALSE);

        if ((appControl->usbDeviceConnected == 0) && 
            (appControl->sdcardDeviceConnected == 0))
        {
            laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
        }

        APP_ClearUserEvent(APP_EVENT_USER_SD_REMOVED);
    }
    
    return 0;
}

int processEventsSettingsScreen(APP_DATA * appData, APP_CONTROL * appControl) 
{
    
    //Process internal events
    if (APP_CHECK_EVENT(appData->appInternalEvent, APP_EVENT_INT_STATE_SHOW_SCREEN))
    {
        laContext_SetActiveScreen(SettingsScreen_ID);
        APP_SetInternalEvent(APP_EVENT_INT_STATE_INIT);
        
        APP_ClearInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
    }
    else if (APP_CHECK_EVENT(appData->appInternalEvent, APP_EVENT_INT_STATE_INIT))
    {
        //Nothing to do
        APP_ClearInternalEvent(APP_EVENT_INT_STATE_INIT);
    }

    //Process user events
    if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_SET_FPS))
    {
        switch(laListWidget_GetFirstSelectedItem(PlaySpeedListWidget))
        {
            case PLAYBACK_SPEED_MAX:
            {
                appControl->frameRateSetting = PLAYBACK_FPS_MAX;
                break;                    
            }
            case PLAYBACK_SPEED_FAST:
            {
                appControl->frameRateSetting = PLAYBACK_FPS_FAST;
                break;                    
            }
            case PLAYBACK_SPEED_SLOW:
            {
                appControl->frameRateSetting = PLAYBACK_FPS_SLOW;
                break;                    
            }
            case PLAYBACK_SPEED_NORMAL: //No break, fall through.
            default:
            {
                appControl->frameRateSetting = PLAYBACK_FPS_NORMAL;
                break;
            }
        }
        
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_SET_FPS);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_SET_RESOLUTION))
    {
        appControl->selectedVideoRes = 
                    laListWidget_GetFirstSelectedItem(ResolutionListWidget);
            
        switch(appControl->selectedVideoRes)
        {
            case RES_LIST_IDX_320x240:
            {
                sprintf(appControl->fileName, VIDEO_320x240_FILE_NAME);
                appControl->frameWidth = 320;
                appControl->frameHeight = 240;
                break;
            }
            case RES_LIST_IDX_320x180:
            {
                sprintf(appControl->fileName, VIDEO_320x180_FILE_NAME);
                appControl->frameWidth = 320;
                appControl->frameHeight = 180;
                break;
            }
            case RES_LIST_IDX_800x480:
            {
                sprintf(appControl->fileName, VIDEO_800x480_FILE_NAME);
                appControl->frameWidth = 800;
                appControl->frameHeight = 480;
                break;
            }
            case RES_LIST_IDX_480x272: //No break, fall through
            default:
            {
                sprintf(appControl->fileName, VIDEO_480x272_FILE_NAME);
                appControl->frameWidth = 480;
                appControl->frameHeight = 272;
                break;
            }
        }

        if (appControl->maxVideoRes == appControl->selectedVideoRes)
        {
            appControl->playbackScreenID = PlayBackScreen_ID;
        }
        else
        {
            appControl->playbackScreenID = PlayBackScreenSmall_ID;
        }
        
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_SET_RESOLUTION);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_SET_HORZ_ALIGN))
    {
        appControl->horzAlign 
            = laListWidget_GetFirstSelectedItem(HorzAlignListWidget);
        //Clear the event
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_SET_HORZ_ALIGN);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_SET_VERT_ALIGN))
    {
        appControl->vertAlign 
            = laListWidget_GetFirstSelectedItem(VertAlignListWidget);
        //Clear the event
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_SET_VERT_ALIGN);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_SHOW_FRAME_RATE))
    {
        appControl->showMetrics = true;
            
        laCheckBoxWidget_SetChecked(CheckBoxWidget13, LA_TRUE);
        //Clear the event
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_SHOW_FRAME_RATE);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_SETTINGS_HIDE_FRAME_RATE))
    {
        appControl->showMetrics = false;
            
        laCheckBoxWidget_SetChecked(CheckBoxWidget13, LA_FALSE);
        //Clear the event
        APP_ClearUserEvent(APP_EVENT_USER_SETTINGS_HIDE_FRAME_RATE);
    }
    else if (APP_CHECK_EVENT(appData->appUserEvent, APP_EVENT_USER_BACK_TO_MAIN_MENU))
    {
        appData->state = APP_STATE_MENU;
        
        APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
        
        //Clear the event
        APP_ClearUserEvent(APP_EVENT_USER_BACK_TO_MAIN_MENU);
    }
    
    return 0;
}


/* *****************************************************************************
 End of File
 */
