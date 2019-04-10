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

#include "gfx/hal/inc/gfx_context.h"
#include "gfx/libaria/inc/libaria_widget.h"
#include "gfx/libaria/inc/libaria_widget_label.h"

#include "app.h"
#include "app_splash.h"
#include "app_control_common.h"
#include "gfx/libaria/inc/libaria_widget_button.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define DEFAULT_TARGET_FRAMES_PER_SEC       PLAYBACK_FPS_NORMAL
#define FRAMERATE_UPDATE_PERIOD_MS          1000
#define USB_ID      1
#define SDCARD_ID   2
#define PLAYBACK_FACTOR_MAX 64

#define BMP_HEADER_SIZE_BYTES 0x42
#define GET_TICKS() __builtin_mfc0(9, 0)

#define UPDATE_FPS_TICKS_DELTA_SECS 1
#define SYSTEM_TICK_FREQUENCY (SYS_CLK_FREQ/2)

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
APP_CONTROL appControl;
static SYS_TMR_HANDLE handleTimerFramePlayer;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
static inline void updateFPS(unsigned int secs) 
{
    static int prevFrameCount = 0;
    static char charBuff[10];
    laString str;
    
    appControl.frameRate = (appControl.frameCount - prevFrameCount)/ secs;
    prevFrameCount = appControl.frameCount;
    
    sprintf(charBuff, "%3u fps", appControl.frameRate);
    str = laString_CreateFromCharBuffer(charBuff,
            GFXU_StringFontIndexLookup(&stringTable, string_nums, 0));

    laLabelWidget_SetText(appControl.fpsLabel, str);
    laString_Destroy(&str);
}

static inline void updateBandwidth(unsigned int secs) 
{
    static int prevBytesCount = 0;
    static char charBuff[20];
    laString str;
    
    appControl.bandWidthKBPS = (appControl.bytesCount - prevBytesCount) / (1000 * secs);
    prevBytesCount = appControl.bytesCount;

    sprintf(charBuff, "%4u kBps", appControl.bandWidthKBPS);
    str = laString_CreateFromCharBuffer(charBuff,
            GFXU_StringFontIndexLookup(&stringTable, string_nums, 0));

    laLabelWidget_SetText(appControl.bandwidthLabel, str);
    laString_Destroy(&str);
}


static void FramePlayerTimer_Callback ( uintptr_t context, uint32_t currTick)
{
    
    static unsigned int thisTimerTicks = 0;
    static unsigned long lastSystemTimerTicks = 0;
    unsigned long systemTimerTicks;
    unsigned long secs;
    thisTimerTicks++;
    
    systemTimerTicks = GET_TICKS();

    if ((appControl.playBackControlState != APP_PLAYBACK_CONTROL_HIDDEN) &&
        ((systemTimerTicks - lastSystemTimerTicks) >= SYSTEM_TICK_FREQUENCY))
    {
        if (appControl.showMetrics)
        {
            secs = ((systemTimerTicks - lastSystemTimerTicks)/SYSTEM_TICK_FREQUENCY);
            updateBandwidth(secs);
            updateFPS(secs);
        }

        if (appControl.playBackControlState == APP_PLAYBACK_CONTROL_VISIBLE)
        {
            APP_SetInternalEvent(APP_EVENT_INT_PROGRESS_UPDATE);
        }

        lastSystemTimerTicks = systemTimerTicks;
        
        thisTimerTicks = 0;
    }
    
    appControl.frameNum += appControl.frameInc;
}

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event,
                                                 void* eventData,
                                                 uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event,
                           void* eventData,
                           uintptr_t context)
{
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
        {
            if(strcmp((const char *)eventData, 
               SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0) == 0)
            {
                appControl.usbDeviceConnected = 1;
                APP_SetUserEvent(APP_EVENT_USER_USB_INSERTED);
            }
            else if(strcmp((const char *)eventData, 
               SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0) == 0)
            {
                appControl.sdcardDeviceConnected = 1;
                APP_SetUserEvent(APP_EVENT_USER_SD_INSERTED);
                
            }
            
            break;
        }    
        case SYS_FS_EVENT_UNMOUNT:
        {
            if(strcmp((const char *)eventData,
               SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0) == 0)
            {
                appControl.usbDeviceConnected = 0;
                APP_SetUserEvent(APP_EVENT_USER_USB_REMOVED);
            }
            else if(strcmp((const char *)eventData,
               SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0) == 0)
            {
                appControl.sdcardDeviceConnected = 0;
                APP_SetUserEvent(APP_EVENT_USER_SD_REMOVED);
            }
            
            break;
        }
        default:
            break;
    }
}

static int PausePlayback(void)
{
    if (appControl.mediaPlaying == 0)
        return -1;
    
    SYS_TMR_ObjectDelete(handleTimerFramePlayer);
    appControl.mediaPlaying = 0;
    
    return 0;
}

static int StartPlayback(void)
{
    if (appControl.mediaPlaying == 1)
        return -1;

    SYS_TMR_ObjectDelete(handleTimerFramePlayer);
    handleTimerFramePlayer = SYS_TMR_CallbackPeriodic(
            appControl.playbackUpdatePeriodMS,
            1,
            FramePlayerTimer_Callback);
    
    appControlHideMessage(&appControl);
    
    appControl.mediaPlaying = 1;
    
    return 0;
}

void APP_SetUserEvent(APP_USER_EVENTS event)
{
    appData.appUserEvent |= (1 << event);
}

void APP_SetInternalEvent(APP_INTERNAL_EVENTS event)
{
    appData.appInternalEvent |= (1 << event);
}

void APP_ClearUserEvent(APP_USER_EVENTS event)
{
    appData.appUserEvent &= ~(1 << event);
}

void APP_ClearInternalEvent(APP_INTERNAL_EVENTS event)
{
    appData.appInternalEvent &= ~(1 << event);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


static void DrawWidgetTouchDown(laWidget* widget, laInput_TouchDownEvent* evt);


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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.appUserEvent = 0;
    appData.appInternalEvent = 0;
    appData.appTimerEvent = 0;
    
    SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, USB_ID);
    USB_HOST_EventHandlerSet(&APP_USBHostEventHandler, 0);
    USB_HOST_BusEnable(0);
}

int readRawImageToFrame(uint8_t * frameBuff,
                        SYS_FS_HANDLE fd,
                        unsigned int frameNum,
                         GFX_Context * gfxContext)
{   
    int retval = -1;
    int chunkAbsRow, chunkRelRow, numRowsInChunk, screenRow;
    uint8_t * src_address, * fb_address;
    
    if (fd == SYS_FS_HANDLE_INVALID)
    {
        return -1;
    }
        
    numRowsInChunk = (appControl.linesBuffered <= appControl.frameHeight) ? 
                    appControl.linesBuffered : appControl.frameHeight;
    
    for (chunkAbsRow = 0; chunkAbsRow < appControl.frameHeight; )        
    {
        src_address = (void*) ((appControl.frameSizeBytes * frameNum) + 
                              (chunkAbsRow * appControl.frameWidth * appControl.bytesPerPixel));
        
        retval = SYS_FS_FileSeek(fd, 
                        (uint32_t) src_address, 
                        SYS_FS_SEEK_SET);
        if (retval < 0)
        {
            return -1;
        }

        retval = SYS_FS_FileRead(fd, 
                                 appControl.fileDataBuffer, 
                                 appControl.frameWidth * appControl.bytesPerPixel * numRowsInChunk);
        if (retval <= 0)
        {
            return -1;
        }
        
        appControl.bytesCount += retval;
        
        for (chunkRelRow = 0; chunkRelRow < numRowsInChunk; chunkRelRow++)
        {
            screenRow = appControl.vertOffset + chunkAbsRow + chunkRelRow;
            
            if ((appControl.playBackControlState == APP_PLAYBACK_CONTROL_VISIBLE)&& 
                (screenRow > appControl.playbackScreenHeight))
                continue;
            
            if (appControl.interlace)
            {
                if ((screenRow & 0x1) == (appControl.frameCount & 0x1)) 
                    continue;
            }
            
            fb_address = frameBuff + 
                    (screenRow * gfxContext->display_info->rect.width * appControl.bytesPerPixel) + 
                    appControl.horzOffset * appControl.bytesPerPixel;

            memcpy(fb_address, 
                   appControl.fileDataBuffer + (chunkRelRow * appControl.frameWidth * appControl.bytesPerPixel), 
                   appControl.frameWidth * appControl.bytesPerPixel);
        }
        
        numRowsInChunk = (appControl.frameHeight - (chunkAbsRow + chunkRelRow) >= appControl.linesBuffered) ? 
                         appControl.linesBuffered : 
                         (appControl.frameHeight - (chunkAbsRow + chunkRelRow));        
           
        chunkAbsRow = chunkAbsRow + chunkRelRow;
        
    }
    
    return retval;
}
 
  int readRawImageToFullFrame(uint8_t * frame,
                                    SYS_FS_HANDLE fd,
                                    unsigned int frameNum,
                                    GFX_Context * gfxContext)
{   
    int retval;
    unsigned int numRows;

     
    if (fd == SYS_FS_HANDLE_INVALID)
    {
        return -1;
    }
    
    numRows =  appControl.playbackScreenHeight;
    
    retval = SYS_FS_FileSeek(fd, (appControl.frameSizeBytes * frameNum), SYS_FS_SEEK_SET);
    if (retval < 0)
    {
        return -1;
    }
    
    retval = SYS_FS_FileRead(fd, 
                             frame, 
                             appControl.frameWidth * appControl.bytesPerPixel * numRows);

    appControl.bytesCount += (retval > 0) ? retval : 0;
    
    return retval;
}

int APP_InitializeMainMenuScreen(void) 
{
    if (appControl.sdcardDeviceConnected == 1) 
    {
        laWidget_SetVisible((laWidget *) PlaySDButton, LA_TRUE);
    } 
    else 
    {
        laWidget_SetVisible((laWidget *) PlaySDButton, LA_FALSE);
    }

    if (appControl.usbDeviceConnected == 1) 
    {
        laWidget_SetVisible((laWidget *) PlayUSBButton, LA_TRUE);
    } 
    else 
    {
        laWidget_SetVisible((laWidget *) PlayUSBButton, LA_FALSE);
    }
    
    if ((appControl.usbDeviceConnected == 0) && 
        (appControl.sdcardDeviceConnected == 0))
    {
        laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_TRUE);
    }
    else
    {
        laWidget_SetVisible((laWidget *) NoMediaLabelWidget, LA_FALSE);
    }
    
    return 0;
}

  
int APP_InitializeSettingsScreen(void) 
{
    //Read FPS value from appData, then set list value
    switch (appControl.frameRateSetting) 
    {
        case PLAYBACK_FPS_MAX:
        {
            laListWidget_SetItemSelected(PlaySpeedListWidget,
                    PLAYBACK_SPEED_MAX,
                    LA_TRUE);
            break;
        }
        case PLAYBACK_FPS_FAST:
        {
            laListWidget_SetItemSelected(PlaySpeedListWidget,
                    PLAYBACK_SPEED_FAST,
                    LA_TRUE);
            break;
        }
        case PLAYBACK_FPS_SLOW:
        {
            laListWidget_SetItemSelected(PlaySpeedListWidget,
                    PLAYBACK_SPEED_SLOW,
                    LA_TRUE);
            break;
        }
        case PLAYBACK_FPS_NORMAL: //No break, fall through.
        default:
        {
            laListWidget_SetItemSelected(PlaySpeedListWidget,
                    PLAYBACK_SPEED_NORMAL,
                    LA_TRUE);
            break;
        }
    }

    //Read Resolution setting and select radio button
    laListWidget_SetItemSelected
                (ResolutionListWidget, appControl.selectedVideoRes, LA_TRUE);
    

    laButtonWidget_SetPressed(ShowFrameRateButtonWidget, appControl.showMetrics);
    laCheckBoxWidget_SetChecked(CheckBoxWidget13, appControl.showMetrics);

    laListWidget_SetItemSelected
            (VertAlignListWidget, appControl.vertAlign, LA_TRUE);
    laListWidget_SetItemSelected
            (HorzAlignListWidget, appControl.horzAlign, LA_TRUE);
    
    return 0;
}

void APP_InitializePlaybackScreen(void)
{
    if (appControl.playbackScreenID == PlayBackScreenSmall_ID) 
    {
        appControl.playPauseButton = PauseButtonWidget2;
        appControl.controlPanel = RightPanelWidget;
        appControl.ffButton = FFButtonWidget2;
        appControl.rwButton = RWButtonWidget2;
        appControl.fpsLabel = FrameRateLabelWidget2;
        appControl.fpsPanel = MetricsPanelWidget2;
        appControl.bandwidthLabel = BandWidthLabelWidget2;
        appControl.playbackRateLabel = PlaybackMultiplierLabelWidget2;
        appControl.playbackMessageLabel = PlaybackMessageLabel2;
        appControl.touchPanelWidget = TouchPanelWidget2;
        appControl.playbackScreenHeight =
                appControl.gfxContext->display_info->rect.height;
        appControl.sliderControl = SliderControlSmall;

        //Align the video
        switch (appControl.vertAlign) 
        {
            case APP_VIDEO_TOP:
            {
                appControl.vertOffset = 0;
                break;
            }
            case APP_VIDEO_BOTTOM:
            {
                appControl.vertOffset =
                        (appControl.gfxContext->display_info->rect.height -
                        appControl.frameHeight);
                break;
            }
            case APP_VIDEO_CENTER:
            default:
            {
                appControl.vertOffset =
                        (appControl.gfxContext->display_info->rect.height -
                        appControl.frameHeight) / 2;
                break;
            }
        }
        switch (appControl.horzAlign) 
        {
            case APP_VIDEO_LEFT:
            {
                appControl.horzOffset = 0;

                laWidget_SetX((laWidget *) appControl.controlPanel,
                        appControl.gfxContext->display_info->rect.width -
                        laWidget_GetWidth((laWidget *) appControl.controlPanel));
                break;
            }
            case APP_VIDEO_RIGHT:
            {
                appControl.horzOffset =
                        (appControl.gfxContext->display_info->rect.width -
                        appControl.frameWidth);

                laWidget_SetX((laWidget *) appControl.controlPanel, 0);
                break;
            }
            case APP_VIDEO_CENTER:
            default:
            {
                appControl.horzOffset =
                        (appControl.gfxContext->display_info->rect.width -
                        appControl.frameWidth) / 2;

                break;
            }
        }
    } 
    else 
    {
        appControl.playPauseButton = PlayPauseButtonWidget;
        appControl.ffButton = FFButtonWidget;
        appControl.rwButton = RewindButtonWidget;
        appControl.controlPanel = PlayBackControlPanel;
        appControl.fpsLabel = frameRateLabelWidget;
        appControl.fpsPanel = MetricsPanelWidget1;
        appControl.bandwidthLabel = BandWidthLabelWidget;
        appControl.playbackRateLabel = PlaybackMultiplierLabelWidget;
        appControl.playbackMessageLabel = PlaybackMessageLabel;
        appControl.touchPanelWidget = TouchPanelWidget;
        appControl.playbackScreenHeight =
                appControl.gfxContext->display_info->rect.height;
        appControl.sliderControl = SliderControlFull;

        appControl.horzOffset = 0;
        appControl.vertOffset = 0;
        appControl.horzAlign = APP_VIDEO_CENTER;
        appControl.vertAlign = APP_VIDEO_CENTER;
    }
}
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    static int prevFrameNum = 0;
    int retval;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        case APP_STATE_PLAYING:
        {
            //Process internal events
            if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_STATE_INIT))
            {
                    appControl.frameSizeBytes = appControl.frameHeight * 
                                        appControl.frameWidth * 
                                        appControl.bytesPerPixel;
            
                    // Initialize playback settings
                    appControl.playbackUpdatePeriodMS = (1000/appControl.frameRateSetting);
                    appControl.frameNum = DEFAULT_START_FRAME_NUM;
                    appControl.frameInc = DEFAULT_FRAME_INC;
                    appControl.interlace = 0;
                    appControl.frameCount = 0;
                    appControl.bytesCount = 0;
                    
                    laWidget_OverrideTouchDownEvent(
                        (laWidget*)appControl.touchPanelWidget, 
                        &DrawWidgetTouchDown);
                    
                    appControl.fileHandle = SYS_FS_FileOpen
                            (appControl.fileName, (SYS_FS_FILE_OPEN_READ));
                    if (appControl.fileHandle <= 0)
                    {
                        //Failed to Open file, show something
                        appControlShowMessage(&appControl, string_FileNotFound);
                        APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_END);
                        
                        break;
                    }
                    
                    //Get file information
                    SYS_FS_FileStat(appControl.fileName, &appControl.fileStat);
                    if (appControl.fileStat.fsize <= 0)
                    {
                        //File size is zero? Abort playback
                        appControlShowMessage(&appControl, string_FileNotFound);
                        APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_END);
                        
                        break;
                    }
                    
                    laWidget_SetVisible((laWidget *) appControl.playbackRateLabel, LA_FALSE);
                    
                    appControlPlaybackInit(&appControl);
                    
                    appControlShowControlPanel(&appControl);
                    
                    APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS);
                    
                    APP_ClearInternalEvent(APP_EVENT_INT_STATE_INIT);
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_STATE_SHOW_SCREEN))
            {
                if (appControl.maxVideoRes == appControl.selectedVideoRes)
                {
                    appControl.playbackScreenID = PlayBackScreen_ID;
                }
                else
                {
                    appControl.playbackScreenID = PlayBackScreenSmall_ID;
                }
                    
                laContext_SetActiveScreen(appControl.playbackScreenID);
                
                APP_SetInternalEvent(APP_EVENT_INT_STATE_INIT);
                
                APP_ClearInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PLAYBACK_SHOW_CONTROLS))
            {
                appControlShowControlPanel(&appControl);
                    
                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_SHOW_CONTROLS);

            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PLAYBACK_HIDE_CONTROLS))
            {
                APP_HIDE_CONTROL_STATUS retval;

                retval = appControlHideControlPanel(&appControl);
                if (retval == APP_HIDE_CONTROL_DONE_BLACKOUT_NEEDED)
                {
                    APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN);
                } 

                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS);
                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_HIDE_CONTROLS);
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN))
            {
                laWidget_SetBackgroundType(
                            (laWidget*)appControl.touchPanelWidget, 
                            LA_WIDGET_BACKGROUND_FILL);
                    
               
                APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_CLEAR_BLACKOUT_SCREEN);
                
                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN);
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PLAYBACK_CLEAR_BLACKOUT_SCREEN))
            {
                laWidget_SetBackgroundType(
                            (laWidget*)appControl.touchPanelWidget, 
                            LA_WIDGET_BACKGROUND_NONE);
                
                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_CLEAR_BLACKOUT_SCREEN);
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS))
            {
                APP_HIDE_CONTROL_STATUS retval;

                StartPlayback();
                
                appControl.frameInc = DEFAULT_FRAME_INC;
                
                APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS);

                retval = appControlAutoHideControlPanel(&appControl);
                if (retval == APP_HIDE_CONTROL_IN_PROGRESS)
                {
                    //If there is a pending user event, show the controls
                    if (appData.appUserEvent != 0)
                    {
                        APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_SHOW_CONTROLS);
                    }
                    else
                    {
                        //Continue hiding the control panel until it's disabled
                        APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS);
                    }
                }
                else if (retval == APP_HIDE_CONTROL_DONE_BLACKOUT_NEEDED)
                {
                    APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN);
                    
                    APP_ClearInternalEvent(APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS);
                }
            }
            else if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_PROGRESS_UPDATE))
            {
                unsigned long progress;

                //Update progress bar slider
                progress = ((appControl.frameSizeBytes / 10) * appControl.frameNum) /
                    (appControl.fileStat.fsize / 1000);
                laSliderWidget_SetSliderValue(appControl.sliderControl, progress);
                
                //Clear any user playback seek event
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_SEEK);
                
                APP_ClearInternalEvent(APP_EVENT_INT_PROGRESS_UPDATE);
            }
            
            //Process user events
            if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_PLAY))
            {
                StartPlayback();
                
                laButtonWidget_SetReleasedImage(appControl.ffButton, 
                                          &FFButton);
                laButtonWidget_SetPressedImage(appControl.ffButton, 
                                          &FFButton);

                laButtonWidget_SetReleasedImage(appControl.rwButton, 
                                          &RewindButton);
                laButtonWidget_SetPressedImage(appControl.rwButton, 
                                          &RewindButton);

                laWidget_SetVisible((laWidget *) appControl.playbackRateLabel, LA_FALSE);

                appControl.frameInc = DEFAULT_FRAME_INC;
                
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PLAY);
            }                    
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_PAUSE))
            {
                PausePlayback();

                appControl.frameInc = DEFAULT_FRAME_INC;
                
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PAUSE);
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_FF))
            {
                static char charBuff[10];
                laString str;

                //Pause Playback
                PausePlayback();

                laButtonWidget_SetPressed(appControl.playPauseButton, 
                                          LA_FALSE);
                
                //Clear any play or pause event
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PAUSE);
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PLAY);

                laButtonWidget_SetReleasedImage(appControl.rwButton, 
                                          &RewindButton);
                laButtonWidget_SetPressedImage(appControl.rwButton, 
                                          &RewindButton);

                laButtonWidget_SetReleasedImage(appControl.ffButton, 
                                          &FFButton_Active);
                laButtonWidget_SetPressedImage(appControl.ffButton, 
                                          &FFButton_Active);

                //Decrease frame increments
                if (appControl.frameInc < 0)
                {
                    appControl.frameInc = 1;
                }

                appControl.frameInc *= 2;
                if (appControl.frameInc > PLAYBACK_FACTOR_MAX)
                {
                    appControl.frameInc = 2;
                }

                sprintf(charBuff, "x%2u", appControl.frameInc);
                str = laString_CreateFromCharBuffer(charBuff,
                    GFXU_StringFontIndexLookup(&stringTable, string_nums, 0));

                laWidget_SetVisible((laWidget *) appControl.playbackRateLabel, LA_TRUE);
                laLabelWidget_SetText(appControl.playbackRateLabel, str);
                laString_Destroy(&str);

                if (appControl.mediaPlaying == 0)
                {
                    StartPlayback();
                }                

                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_FF);
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_RW))
            {
                static char charBuff[10];
                laString str;

                //Pause Playback
                PausePlayback();

                laButtonWidget_SetPressed(appControl.playPauseButton, 
                                          LA_FALSE);
                
                //Clear any play or pause event
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PAUSE);
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_PLAY);

                laButtonWidget_SetReleasedImage(appControl.ffButton, 
                                          &FFButton);
                laButtonWidget_SetPressedImage(appControl.ffButton, 
                                          &FFButton);

                laButtonWidget_SetReleasedImage(appControl.rwButton, 
                                          &RewindButton_Active);
                laButtonWidget_SetPressedImage(appControl.rwButton, 
                                          &RewindButton_Active);

                //Decrease frame increments
                if (appControl.frameInc >= 0)
                {
                    appControl.frameInc = -1;
                }

                appControl.frameInc *= 2;
                if (appControl.frameInc < -(PLAYBACK_FACTOR_MAX))
                {
                    appControl.frameInc = -2;
                }

                sprintf(charBuff, "x%2u", -appControl.frameInc);
                str = laString_CreateFromCharBuffer(charBuff,
                    GFXU_StringFontIndexLookup(&stringTable, string_nums, 0));

                laWidget_SetVisible((laWidget *) appControl.playbackRateLabel, LA_TRUE);
                laLabelWidget_SetText(appControl.playbackRateLabel, str);
                laString_Destroy(&str);

                StartPlayback();

                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_RW);
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_STOP))
            {
                PausePlayback();

                appControl.frameNum = DEFAULT_START_FRAME_NUM;

                appData.state = APP_STATE_MENU;
                APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN); 

                SYS_FS_FileClose(appControl.fileHandle);
                
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_STOP);

                break;
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_RESTART))
            {
                PausePlayback();

                appControl.frameNum = DEFAULT_START_FRAME_NUM;
                
                laButtonWidget_SetPressed(appControl.playPauseButton, 
                    LA_TRUE);

                APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_PLAY); 
                
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_RESTART);

                break;
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_END))
            {
                appControl.frameNum = DEFAULT_START_FRAME_NUM;
                
                if (appControl.loopPlayback == true)
                {
                    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_RESTART);
                }
                else
                {
                    PausePlayback();

                    laButtonWidget_SetPressed(appControl.playPauseButton, 
                                              LA_FALSE);

                    laButtonWidget_SetReleasedImage(appControl.ffButton, 
                                              &FFButton);
                    laButtonWidget_SetPressedImage(appControl.ffButton, 
                                              &FFButton);

                    laButtonWidget_SetReleasedImage(appControl.rwButton, 
                                              &RewindButton);
                    laButtonWidget_SetPressedImage(appControl.rwButton, 
                                              &RewindButton);

                    laWidget_SetVisible((laWidget *) appControl.playbackRateLabel, LA_FALSE);


                    //Set the play button to 'paused'
                    laButtonWidget_SetPressed(appControl.playPauseButton, 
                                              LA_FALSE);

                    APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN);
                }
                
                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_END);
            }
            else if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_PLAYBACK_SEEK))
            {
                bool wasMediaPlaying = appControl.mediaPlaying;
                uint32_t sliderVal;

                PausePlayback();

                sliderVal = laSliderWidget_GetSliderValue(appControl.sliderControl);

                appControl.frameNum = sliderVal * 
                        (appControl.fileStat.fsize / 1000) / 
                        (appControl.frameSizeBytes / 10);

                if (wasMediaPlaying)
                {
                    StartPlayback();
                }
                else
                {
                    if (appControl.frameWidth == appControl.gfxContext->display_info->rect.width)
                    {
                        retval = readRawImageToFullFrame(appControl.videoFrameBuffer,
                                        appControl.fileHandle,
                                        appControl.frameNum,
                                        appControl.gfxContext);
                    }
                    else
                    {
                        retval = readRawImageToFrame(appControl.videoFrameBuffer,
                                        appControl.fileHandle,
                                        appControl.frameNum,
                                        appControl.gfxContext);
                    }

                    appControlHideMessage(&appControl);
                }

                APP_ClearUserEvent(APP_EVENT_USER_PLAYBACK_SEEK);

                break;

            }
            
            if (appControl.mediaPlaying == 1) 
            {
                //Do best case render
                if ((appControl.frameNum  > 0) && (appControl.frameNum != prevFrameNum))
                {
                    if (appControl.frameWidth == appControl.gfxContext->display_info->rect.width)
                    {
                        retval = readRawImageToFullFrame(appControl.videoFrameBuffer,
                                        appControl.fileHandle,
                                        appControl.frameNum,
                                        appControl.gfxContext);
                    }
                    else
                    {
                        retval = readRawImageToFrame(appControl.videoFrameBuffer,
                                        appControl.fileHandle,
                                        appControl.frameNum,
                                        appControl.gfxContext);
                    }
                    
                    
                    if (retval <= 0)
                    {
                        //Failed to read first frame, error reading file!
                        if (appControl.frameCount == 0)
                        {
                            appControlShowMessage(&appControl, string_FileNotFound);
                        }
                        else
                        {
                            appControlShowMessage(&appControl, string_TheEnd);
                        }
                        
                        APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_END);
                        prevFrameNum = 0;
                    }
                    //Update frame count only if frames have changed
                    else if(appControl.frameNum != prevFrameNum)
                    {
                        prevFrameNum = appControl.frameNum;                    
                        appControl.frameCount++;
                    }
                }
                else if (appControl.frameNum < 0)
                {
                    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_END);
                }
            }
            
            break;
        }
        case APP_STATE_SETTINGS:
        {
            retval = processEventsSettingsScreen(&appData, &appControl);
            break;
        }
        case APP_STATE_HELP:
        {
            //Process internal events
            if (APP_CHECK_EVENT(appData.appInternalEvent, APP_EVENT_INT_STATE_SHOW_SCREEN))
            {
                    laContext_SetActiveScreen(HelpScreen_ID);
                    
                    APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
                    
                    APP_ClearInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
            }
            
            //Process User events
            if (APP_CHECK_EVENT(appData.appUserEvent, APP_EVENT_USER_BACK_TO_MAIN_MENU))
            {
                appData.state = APP_STATE_MENU;
                
                APP_SetInternalEvent(APP_EVENT_INT_STATE_SHOW_SCREEN);
                
                APP_ClearUserEvent(APP_EVENT_USER_BACK_TO_MAIN_MENU);
            }
            
            break;
                    
        }
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (appInitialized)
            {
                appControl.gfxContext = GFX_ActiveContext();
                
                //config-specific app Control settings
                appControlInit(&appControl);
                
                appData.state = APP_STATE_SPLASH;
                appControl.frameRateSetting = DEFAULT_TARGET_FRAMES_PER_SEC;
                
                appControl.fileHandle = 0;
                appControl.frameNum = DEFAULT_START_FRAME_NUM;
                appControl.frameInc = DEFAULT_FRAME_INC;
                appControl.frameSizeBytes = 0;
                appControl.usbDeviceConnected = 0;
                appControl.sdcardDeviceConnected = 0;
                appControl.playBackControlState = APP_PLAYBACK_CONTROL_HIDDEN;
                appControl.mediaPlaying = 0;
                appControl.loopPlayback = true;

                // Initialize playback settings
                appControl.horzOffset = 0;
                appControl.vertOffset = 0;
                appControl.horzAlign = APP_VIDEO_CENTER;
                appControl.vertAlign = APP_VIDEO_CENTER;
                
                //Set default video file settings
                //Determine max video resolution
                if ((appControl.gfxContext->display_info->rect.width == 800) &&
                    (appControl.gfxContext->display_info->rect.height == 480))
                {
                    appControl.maxVideoRes = RES_LIST_IDX_800x480;
                    sprintf(appControl.fileName, VIDEO_800x480_FILE_NAME);
                    appControl.frameHeight = 480;
                    appControl.frameWidth = 800;
                }
                else if 
                    ((appControl.gfxContext->display_info->rect.width == 480) &&
                    (appControl.gfxContext->display_info->rect.height == 272))
                {
                    appControl.maxVideoRes = RES_LIST_IDX_480x272;
                    sprintf(appControl.fileName, VIDEO_480x272_FILE_NAME);
                    appControl.frameHeight = 272;
                    appControl.frameWidth = 480;
                }
                else
                {
                    printf(appControl.fileName, VIDEO_DEFAULT_FILE_NAME);
                    appControl.frameHeight = DEFAULT_IMAGE_HEIGHT;
                    appControl.frameWidth = DEFAULT_IMAGE_WIDTH;
                }

                //Set the currently selected res to the max res
                appControl.selectedVideoRes = appControl.maxVideoRes;
                
                appControl.frameSizeBytes = appControl.frameHeight * 
                                         appControl.frameWidth * 
                                         appControl.bytesPerPixel;

                appControl.showMetrics = 0;
            }
            break;
        }

        case APP_STATE_SPLASH:
        {            
            if (APP_IsSplashScreenComplete())
            {
                appData.state = APP_STATE_MENU;
                laContext_SetActiveScreen(MainScreen_ID);
            }
        
            break;
        }
        case APP_STATE_MENU:
        {
            retval = processEventsMainScreen(&appData, &appControl);
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

static void DrawWidgetTouchDown(laWidget* widget, laInput_TouchDownEvent* evt)
{
    if (appControl.playBackControlState == APP_PLAYBACK_CONTROL_HIDDEN)
    {   
        APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_SHOW_CONTROLS);
    }
    else
    {
        APP_SetInternalEvent(APP_EVENT_INT_PLAYBACK_HIDE_CONTROLS);
    }
    evt->event.accepted = LA_TRUE;
}

 

/*******************************************************************************
 End of File
 */
