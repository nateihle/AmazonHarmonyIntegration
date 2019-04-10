/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_display_task.c

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
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "app_display_task.h"
#include "app_sdcard_audio_task.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
#define VOL_STEP_SIZE               20
#define TRACKLISTBOX_PAGE_SIZE      6

typedef enum
{
    VOL_DIRECTION_UP = 0,
    VOL_DIRECTION_DOWN,
}VOL_DIRECTION;

/*  A brief description of a section can be given directly below the section
    banner.
 */
APP_DISPLAY_DATA AppDisplayData;

/*
 * Forward declarations.
 */
static int8_t *_APP_DISPLAY_GetFileName(const int8_t *path);
static void _APP_DISPLAY_HandleMuteOn(void);
static void _APP_DISPLAY_HandleMuteOff(void);
static uint8_t _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION volDir);
static void _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE streamSrc);
   
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_DISPAY_Initialize(void)
{  
    AppDisplayData.streamSource = APP_DISPLAY_STREAMING_SOURCE_UNCHANGED;
    
    AppDisplayData.volMuteSource = APP_DISPLAY_VOLUME_MUTE_SRC_SDCARD;    
    
    AppDisplayData.volumeMute = APP_DISPLAY_VOLUME_MUTE_UNCHANGED;   
    
    AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_OFF;
    
    AppDisplayData.isListPopulated = false;
    
    AppDisplayData.numListItems = 0;
    
    AppDisplayData.scrollBarPosition = 0;
}

///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_Tasks(void)
{ 
    uint8_t codecVolumeLevel;    
    uint8_t item_index = 0;
                   
    if(true == APP_SDCARD_AUDIO_isSonglistAvailable() && false == AppDisplayData.isListPopulated)
    {
        uint16_t index;
        static GFXU_CHAR strBuffer[64] = {0};
        static laString gfxString = {0};        
        APP_SDCARD_AUDIO_CARD_FILE_PATH* pTablePointer;
        
        pTablePointer = APP_SDCARD_AUDIO_GetFileTablePointer();
        AppDisplayData.numListItems = APP_SDCARD_AUDIO_FileCountGet();

        for (index = 0; index < AppDisplayData.numListItems; index++)
        {
            strcpy((char*)strBuffer, (const char*)_APP_DISPLAY_GetFileName(pTablePointer[index].path));
            gfxString = laString_CreateFromCharBuffer((const char*)strBuffer, &Arial14pt);
            item_index = laListWidget_AppendItem((laListWidget *)TrackListBox);
            laListWidget_SetItemText((laListWidget *)TrackListBox, item_index, gfxString);            
        } 

        AppDisplayData.currentTrackId =  APP_SDCARD_AUDIO_CurrentTrackIdGet();            

        laWidget_SetEnabled((laWidget *)TrackListBox, true);        
        laListWidget_SetSelectionMode(TrackListBox, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
        laListWidget_SetItemSelected((laListWidget *)TrackListBox, AppDisplayData.currentTrackId, LA_TRUE);  
        if (AppDisplayData.numListItems > TRACKLISTBOX_PAGE_SIZE)
        {
            laListWidget_SetItemVisible((laListWidget *)TrackListBox, 1);        
        }
                                                 
        AppDisplayData.scrollBarPosition = laProgressBarWidget_GetValue((laProgressBarWidget*)VolumeBar);
        codecVolumeLevel = (AppDisplayData.scrollBarPosition * (APP_DISPLAY_VOLUME_MAX))/100; 
        APP_SDCARD_AUDIO_VolumeSet(codecVolumeLevel);                                

        /* the above call is not required as the widget is already invalidated and hence will re-draw itself*/
        AppDisplayData.isListPopulated = true;
    }    
       
    if(AppDisplayData.streamSource != APP_DISPLAY_STREAMING_SOURCE_UNCHANGED)
    {
        if(AppDisplayData.streamSource == APP_DISPLAY_STREAMING_SOURCE_CHANGED_SDCARD)
        {
            _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_USB);
            APP_USB_AUDIO_SuspendStreaming(); 
            APP_SDCARD_AUDIO_Initialize();      
            laWidget_SetEnabled((laWidget *)TrackListBox, true);

            APP_StreamSourceSet(APP_STREAMING_SOURCE_SDCARD);
        }
        else if(AppDisplayData.streamSource == APP_DISPLAY_STREAMING_SOURCE_CHANGED_USB)
        {
            _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_SDCARD);
            APP_SDCARD_AUDIO_SuspendStreaming();
            APP_USB_AUDIO_AttachDevice();  
            //clear the list box when stream source is USB
            laListWidget_RemoveAllItems((laListWidget *)TrackListBox);
            //disable the list box and make it in-visible
            laWidget_SetEnabled((laWidget *)TrackListBox, false);            
            
            APP_StreamSourceSet(APP_STREAMING_SOURCE_USB);
        }
        else
        {
            
        }                      
        
        AppDisplayData.isListPopulated = false;
        AppDisplayData.streamSource = APP_DISPLAY_STREAMING_SOURCE_UNCHANGED;       
    }
           
    //check if the state of volume slider or the mute button is changed
    if(AppDisplayData.volumeMute != APP_DISPLAY_VOLUME_MUTE_UNCHANGED)
    {
        if(AppDisplayData.volMuteSource == APP_DISPLAY_VOLUME_MUTE_SRC_SDCARD)
        {
            if(APP_DISPLAY_VOLUME_MUTE_ON == AppDisplayData.volumeMute)
            {
                APP_SDCARD_AUDIO_MuteOn();
                _APP_DISPLAY_HandleMuteOn();                
                AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_ON;
            }
            else if(APP_DISPLAY_VOLUME_MUTE_OFF == AppDisplayData.volumeMute)
            {
                APP_SDCARD_AUDIO_MuteOff(); 
                _APP_DISPLAY_HandleMuteOff();
                AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_OFF;
            }
            else if(APP_DISPLAY_VOLUME_MUTE_INCREASE == AppDisplayData.volumeMute)
            {                        
                _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_SDCARD);
                codecVolumeLevel = _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION_UP);  
                APP_SDCARD_AUDIO_VolumeSet(codecVolumeLevel);
            }
            else if (APP_DISPLAY_VOLUME_MUTE_DECREASE == AppDisplayData.volumeMute)
            {        
                _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_SDCARD);
                codecVolumeLevel = _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION_DOWN);  
                APP_SDCARD_AUDIO_VolumeSet(codecVolumeLevel);
            }
            else
            {
                ;
            }       
        }
        else if(AppDisplayData.volMuteSource == APP_DISPLAY_VOLUME_MUTE_SRC_USB)
        {
            if(APP_DISPLAY_VOLUME_MUTE_ON == AppDisplayData.volumeMute)
            {
                APP_USB_AUDIO_MuteOn();
                _APP_DISPLAY_HandleMuteOn();               
                AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_ON;
            }
            else if(APP_DISPLAY_VOLUME_MUTE_OFF == AppDisplayData.volumeMute)
            {
                APP_USB_AUDIO_MuteOff();
                _APP_DISPLAY_HandleMuteOff();
                AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_OFF;
            }
            else if(APP_DISPLAY_VOLUME_MUTE_INCREASE == AppDisplayData.volumeMute)
            {         
                _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_USB);
                codecVolumeLevel = _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION_UP);  
                APP_USB_AUDIO_VolumeSet(codecVolumeLevel);
            }  
            else if(APP_DISPLAY_VOLUME_MUTE_DECREASE == AppDisplayData.volumeMute)
            {         
                _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE_USB);
                codecVolumeLevel = _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION_DOWN); 
                APP_USB_AUDIO_VolumeSet(codecVolumeLevel);
            }  
            else
            {
                ;
            }
        }
        else
        {
            ;
        }        
        AppDisplayData.volumeMute = APP_DISPLAY_VOLUME_MUTE_UNCHANGED;                   
    }   

    if(AppDisplayData.isListPopulated == true)
    {
        if(AppDisplayData.currentTrackId != APP_SDCARD_AUDIO_CurrentTrackIdGet())
        {
            AppDisplayData.currentTrackId = APP_SDCARD_AUDIO_CurrentTrackIdGet();                        
            
            if (AppDisplayData.numListItems > TRACKLISTBOX_PAGE_SIZE)
            {
                if ( AppDisplayData.currentTrackId >= (TRACKLISTBOX_PAGE_SIZE-1))
                {
                    laListWidget_SetItemVisible((laListWidget *)TrackListBox, AppDisplayData.currentTrackId);
                }
                else
                {
                    laListWidget_SetItemVisible((laListWidget *)TrackListBox, 1);
                }
            }
            
            laListWidget_SetItemSelected((laListWidget *)TrackListBox, AppDisplayData.currentTrackId, LA_TRUE);
        }
    }    
}

///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_StreamSourceSet(APP_DISPLAY_STREAMING_SOURCE streamSrc)
{
    AppDisplayData.streamSource = streamSrc;
}

///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_VolumeMuteSourceSet(APP_DISPLAY_VOLUME_MUTE_SRC volMuteSrc)
{
    AppDisplayData.volMuteSource = volMuteSrc;
}


///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_VolumeMuteChange(APP_DISPLAY_VOLUME_MUTE volMuteChange)
{
    AppDisplayData.volumeMute = volMuteChange;    
}


///////////////////////////////////////////////////////////////////////////////
static int8_t *_APP_DISPLAY_GetFileName(const int8_t *path)
{
    int8_t *filename = (int8_t*)strrchr((const char*)path, '/');
    if (filename == NULL)
        filename = (int8_t *)path;
    else
        filename++;
    return filename;
}


///////////////////////////////////////////////////////////////////////////////
static void _APP_DISPLAY_HandleMuteOn(void)
{       
    laProgressBarWidget_SetValue((laProgressBarWidget*)VolumeBar, 0);                
}

///////////////////////////////////////////////////////////////////////////////
static void _APP_DISPLAY_HandleMuteOff(void)
{   
    laProgressBarWidget_SetValue((laProgressBarWidget*)VolumeBar, AppDisplayData.scrollBarPosition);            
}

void APP_DISPLAY_CheckAndUnmute(APP_STREAMING_SOURCE streamSrc)
{
    if (APP_DISPLAY_VOLUME_MUTE_OFF == AppDisplayData.muteButtonState)
    {
        if (APP_STREAMING_SOURCE_SDCARD == streamSrc)
        {
            APP_SDCARD_AUDIO_MuteOff();         
        }
        else
        {
            APP_USB_AUDIO_MuteOff();
        }
    }
}

static void _APP_DISPLAY_Unmute(APP_STREAMING_SOURCE streamSrc)
{
    if (APP_STREAMING_SOURCE_SDCARD == streamSrc)
    {
        //Turn off mute as the volume button is changed              
        APP_SDCARD_AUDIO_MuteOff(); 
        laButtonWidget_SetPressed((laButtonWidget*)MuteButton, true);
        AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_OFF;        
    }
    else
    {        
        //Turn off mute as the volume button is changed              
        APP_USB_AUDIO_MuteOff(); 
        laButtonWidget_SetPressed((laButtonWidget*)MuteButton, true);
        AppDisplayData.muteButtonState = APP_DISPLAY_VOLUME_MUTE_OFF;        
    }    
}

///////////////////////////////////////////////////////////////////////////////
static uint8_t _APP_DISPLAY_HandleVolumeIncreaseDecrease(VOL_DIRECTION volDir)
{                
    AppDisplayData.scrollBarPosition = laProgressBarWidget_GetValue((laProgressBarWidget*)VolumeBar);
    
    if (VOL_DIRECTION_UP == volDir)
    {        
        if ((AppDisplayData.scrollBarPosition + VOL_STEP_SIZE) <= 100)
        {
            AppDisplayData.scrollBarPosition += VOL_STEP_SIZE;
        }
    }
    else
    {
        if (AppDisplayData.scrollBarPosition >= VOL_STEP_SIZE)
        {
            AppDisplayData.scrollBarPosition -= VOL_STEP_SIZE;
        }
    }        
    
    laProgressBarWidget_SetValue((laProgressBarWidget*)VolumeBar, AppDisplayData.scrollBarPosition);    
        
    return (AppDisplayData.scrollBarPosition * (APP_DISPLAY_VOLUME_MAX))/100;     
}




/* *****************************************************************************
 End of File
 */
