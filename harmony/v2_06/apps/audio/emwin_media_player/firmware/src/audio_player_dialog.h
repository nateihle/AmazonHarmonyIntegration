/*******************************************************************************
  Universal Audio Decoders Application Demo

  Company:
    Microchip Technology Inc.

  File Name:
    audio_player_dialog.h

  Summary:
    This header file provides prototypes and data type definitions for the 
    audio player dialog.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by other tasks and some of them 
    are only used internally by the application.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef AUDIO_PLAYER_DIALOG_H
#define	AUDIO_PLAYER_DIALOG_H

#include "DIALOG.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h> 
#include "app_display_task.h"

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END   
    
typedef bool (*AUDIO_PLAYER_GUI_EVENTS_NOTIFY)(DISP_CMD cmd);    
    
typedef enum
{
    PLAY_BUTTON_STATES_PAUSE = 0,
    PLAY_BUTTON_STATES_PLAY,
}PLAY_BUTTON_STATES;

typedef enum
{
    REPEAT_BUTTON_STATES_REPEAT_OFF = 0,
    REPEAT_BUTTON_STATES_REPEAT_ON,
    REPEAT_BUTTON_STATES_REPEAT_SINGLE_TRACK,
    REPEAT_BUTTON_STATES_MAX,
}REPEAT_BUTTON_STATES;

typedef enum
{
    SHUFFLE_BUTTON_STATES_SHUFFLE_OFF = 0,
    SHUFFLE_BUTTON_STATES_SHUFFLE_ON,
}SHUFFLE_BUTTON_STATES;

typedef enum
{
    SWITCH_BUTTON_STATES_SWITCH_OFF = 0,
    SWITCH_BUTTON_STATES_SWITCH_ON,
}SWITCH_BUTTON_STATES;

typedef enum
{
    SWITCH_BUTTON_TYPE_TWO_WAY = 0,
    SWITCH_BUTTON_TYPE_ONE_WAY,
}SWITCH_BUTTON_TYPE;

typedef enum
{
    MUTE_BUTTON_STATES_MUTE_OFF = 0,
    MUTE_BUTTON_STATES_MUTE_ON,
}MUTE_BUTTON_STATES;

typedef enum
{
    SETTINGS_BUTTON_STATES_HIDE = 0,
    SETTINGS_BUTTON_STATES_SHOW,
}SETTINGS_BUTTON_STATES;

void deleteAllListBoxItems(void);
void addListBoxItem(const char* pItemString);
void initListBox(const char* const pErrorMsg);
uint32_t getListBoxSelectedItem(void);
void setCurrentTrackText(void);
void clrCurrentTrackText(void);
uint32_t getSliderValue(void);
uint32_t getTrackSeekValue(void);
uint32_t getMaxSliderValue(void);
void volMuteButtonControl(bool isOn);
void setListBoxSelectedItem(uint32_t selectedItem);
void setSliderValue(uint32_t sliderVal);
void updateProgressBar(uint8_t progBarVal);
void updateTotalTrackTime(const uint8_t* const pStr);
void updateElapsedTrackTime(const uint8_t* const pStr);
void updatePlayButtonState(PLAY_BUTTON_STATES playButtonState);
bool getUSBModeSwitchState(void);
void setUSBModeSwitchState(SWITCH_BUTTON_STATES switchState);
uint32_t APP_SDCARD_AUDIO_GetElapsedTrackTimeInBytes(void);
uint32_t APP_SDCARD_AUDIO_GetTotalTrackTimeInBytes(void);
bool isGuiScreenInitialized(void);
void registerGuiEventsHandler(AUDIO_PLAYER_GUI_EVENTS_NOTIFY evHandler);

#ifdef __cplusplus
}
#endif

#endif

