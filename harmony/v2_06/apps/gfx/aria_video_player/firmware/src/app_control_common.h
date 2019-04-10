/*******************************************************************************
  MPLAB Harmony Application Splash Screen Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_control.h

  Summary:
    This header file provides prototypes and definitions for the application 
    splash screen.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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

#ifndef _APP_CONTROL_H    /* Guard against multiple inclusion */
#define _APP_CONTROL_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
#define TICK_FREQUENCY_HZ           100000000
#define DEFAULT_START_FRAME_NUM     1
#define DEFAULT_FRAME_INC           1

#define VIDEO_DEFAULT_FILE_NAME VIDEO_480x272_FILE_NAME
#define DEFAULT_IMAGE_WIDTH   480
#define DEFAULT_IMAGE_HEIGHT  272
    
#define PLAYBACK_CONTROL_HEIGHT 60


#define VIDEO_320x180_FILE_NAME "video0.rgb"
#define VIDEO_320x240_FILE_NAME "video1.rgb"
#define VIDEO_480x272_FILE_NAME "video2.rgb"    
#define VIDEO_800x480_FILE_NAME "video3.rgb"
    
//The indices should match the entries in the List Widget in the Settings screen
typedef enum
{
    RES_LIST_IDX_320x180 = 0,
    RES_LIST_IDX_320x240,            
    RES_LIST_IDX_480x272,            
    RES_LIST_IDX_800x480,
} VIDEO_RES_LIST_INDEX;

//The order of the enum here must match 
//the order of the entries in the list widget
typedef enum
{
    PLAYBACK_SPEED_SLOW = 0,
    PLAYBACK_SPEED_NORMAL = 1,
    PLAYBACK_SPEED_FAST = 2,
    PLAYBACK_SPEED_MAX = 3,
} VIDEO_SPEED_INDEX;

typedef enum
{
    PLAYBACK_FPS_NORMAL = 15,
    PLAYBACK_FPS_FAST = 24,
    PLAYBACK_FPS_SLOW = 8,
    PLAYBACK_FPS_MAX = 80,
} VIDEO_SPEED_FPS;

typedef enum
{
    APP_VIDEO_CENTER = 0,
    APP_VIDEO_LEFT = 1,
    APP_VIDEO_RIGHT = 2,
    APP_VIDEO_TOP = 1,
    APP_VIDEO_BOTTOM = 2,
} VIDEO_ALIGNMENT;

typedef enum
{
    APP_PLAYBACK_CONTROL_VISIBLE,
    APP_PLAYBACK_CONTROL_HIDDEN,
    APP_PLAYBACK_CONTROL_HIDING,
} PLAYBACK_CONTROLS_STATE;

typedef enum
{
    APP_HIDE_CONTROL_DONE,
    APP_HIDE_CONTROL_DONE_BLACKOUT_NEEDED,
    APP_HIDE_CONTROL_IN_PROGRESS,
} APP_HIDE_CONTROL_STATUS;

typedef struct
{
    bool usbDeviceConnected;
    bool sdcardDeviceConnected;
    PLAYBACK_CONTROLS_STATE playBackControlState;
    bool mediaPlaying;
    bool loopPlayback;
    SYS_FS_HANDLE fileHandle;
    SYS_FS_FSTAT fileStat;
    char fileName[50];
    volatile int frameNum;
    int frameInc;
    unsigned int frameCount;
    unsigned int bytesCount;
    unsigned int frameRate;
    unsigned int bandWidthKBPS;
    uint8_t * fileDataBuffer;
    GFX_Context * gfxContext;
    uint8_t * videoFrameBuffer;
    
    // Video Frame properties
    unsigned int frameHeight;
    unsigned int frameWidth;
    unsigned int bytesPerPixel;
    unsigned int frameSizeBytes;
    
    // Playback properties
    bool interlace;
    unsigned int frameRateSetting;
    unsigned int playbackUpdatePeriodMS;
    uint32_t horzOffset;
    uint32_t vertOffset;
    
    //Playback Screen settings
    unsigned int playbackScreenID;
    VIDEO_RES_LIST_INDEX maxVideoRes;
    VIDEO_RES_LIST_INDEX selectedVideoRes;
    VIDEO_ALIGNMENT vertAlign;
    VIDEO_ALIGNMENT horzAlign;
    bool showMetrics;
    unsigned int playbackScreenHeight;
    unsigned int linesBuffered;
    
    //Playback Controls
    laButtonWidget* playPauseButton;
    laButtonWidget* ffButton;
    laButtonWidget* rwButton;
    laWidget* controlPanel;
    laWidget* fpsPanel;
    laLabelWidget* fpsLabel;
    laLabelWidget* bandwidthLabel;
    laLabelWidget* playbackRateLabel;
    laLabelWidget* playbackMessageLabel;
    laWidget * touchPanelWidget;
    laSliderWidget * sliderControl;
    
} APP_CONTROL;

int appControlShowControlPanel(APP_CONTROL * appControl);
APP_HIDE_CONTROL_STATUS appControlAutoHideControlPanel(APP_CONTROL * appControl);
APP_HIDE_CONTROL_STATUS appControlHideControlPanel(APP_CONTROL * appControl);
int appControlPlaybackInit(APP_CONTROL * appControl);
int appControlShowMessage(APP_CONTROL * appControl, unsigned int stringID);
int appControlHideMessage(APP_CONTROL * appControl);
int appControlInit(APP_CONTROL * appControl);

int processEventsMainScreen(APP_DATA * appData, APP_CONTROL * appControl);
int processEventsSettingsScreen(APP_DATA * appData, APP_CONTROL * appControl);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _APP_CONTROL_H */

/* *****************************************************************************
 End of File
 */
