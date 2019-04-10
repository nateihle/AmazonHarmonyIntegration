/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_display_task.h

  Summary:
    This header file provides prototypes and definitions for the display task.

  Description:
    This header file provides prototypes and definitions for the display task.
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
//DOM-IGNORE-END

#ifndef APP_DISPLAY_H
#define	APP_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "app_freq_spectrum_task.h"
#include "app_queue.h"


#ifdef	__cplusplus
extern "C" {
#endif
    
#define APP_DISPLAY_QUEUE_SIZE                      10
#define APP_DISPLAY_MAX_TONES                       3  
    
typedef enum
{
    APP_GRAPH_CHANGE_TYPE_INC = 0,
    APP_GRAPH_CHANGE_TYPE_DEC,
            
}APP_GRAPH_CHANGE_TYPE;    

typedef enum
{
    APP_DISPLAY_ACTIVE_SCREEN_MAIN = 0,
    APP_DISPLAY_ACTIVE_SCREEN_TIME_DOMAIN,
            
}APP_DISPLAY_ACTIVE_SCREEN;
    
typedef enum
{
    APP_DISPLAY_EVENT_TYPE_VOL_CHANGE_REQ = 0,
    APP_DISPLAY_EVENT_TYPE_MAX,
            
}APP_DISPLAY_EVENT_TYPE;
    
typedef void (*APP_DISPLAY_EVENT_CALLBACK) (APP_DISPLAY_EVENT_TYPE eventType, const void* const pEventData);    
    
typedef enum
{
    APP_DISPLAY_CMD_UPDATE_FREQ_SPECTRUM = 0,    
    APP_DISPLAY_CMD_UI_EVENT_MIC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_TONE_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_F1_RADIO_BTN_SEL,
    APP_DISPLAY_CMD_UI_EVENT_F2_RADIO_BTN_SEL,
    APP_DISPLAY_CMD_UI_EVENT_F3_RADIO_BTN_SEL,
    APP_DISPLAY_CMD_UI_EVENT_INC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_DEC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_CLR_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_HZ_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_KHZ_BTN_PRESSED,   
    APP_DISPLAY_CMD_UI_EVENT_DBFS_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_PLAY_BTN_PRESSED,    
    APP_DISPLAY_CMD_UI_EVENT_HANN_WIN_RADIO_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_BLACKMAN_WIN_RADIO_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_VOLUME_SLIDER_CHANGED,
    APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_INC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_DEC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_INC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_DEC_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_GRID_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_TIME_DOMAIN_SCREEN_BTN_PRESSED,
    APP_DISPLAY_CMD_UI_EVENT_MAIN_SCREEN_BTN_PRESSED,
    APP_DISPLAY_CMD_MAX,
}APP_DISPLAY_CMD;

typedef enum
{
    APP_DISPLAY_UI_ELEMENTS_F1_RADIO_BTN = 0,
    APP_DISPLAY_UI_ELEMENTS_F2_RADIO_BTN,
    APP_DISPLAY_UI_ELEMENTS_F3_RADIO_BTN,
    APP_DISPLAY_UI_ELEMENTS_MIC_BTN,
    APP_DISPLAY_UI_ELEMENTS_TONE_BTN,
    APP_DISPLAY_UI_ELEMENTS_INC_BTN,
    APP_DISPLAY_UI_ELEMENTS_DEC_BTN,
    APP_DISPLAY_UI_ELEMENTS_CLR_BTN,
    APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN,
    APP_DISPLAY_UI_ELEMENTS_UNIT_KHZ_BTN,    
    APP_DISPLAY_UI_ELEMENTS_UNIT_DBFS_BTN,
    APP_DISPLAY_UI_ELEMENTS_PLAY_BTN,
    APP_DISPLAY_UI_ELEMENTS_F1_LABEL,
    APP_DISPLAY_UI_ELEMENTS_F2_LABEL,
    APP_DISPLAY_UI_ELEMENTS_F3_LABEL,    
    APP_DISPLAY_UI_ELEMENTS_HANN_WIN_BTN,
    APP_DISPLAY_UI_ELEMENTS_BLACKMAN_WIN_BTN,
    APP_DISPLAY_UI_ELEMENTS_MAIN_SCREEN_BTN,
    APP_DISPLAY_UI_ELEMENTS_TIME_DOMAIN_SCREEN_BTN,
    APP_DISPLAY_UI_ELEMENTS_MAX,
}APP_DISPLAY_UI_ELEMENTS;

typedef enum
{
    PLAY_BUTTON_STATE_PLAYING = 0,
    PLAY_BUTTON_STATE_STOPPED,
}PLAY_BUTTON_STATE;

typedef enum
{
    GRID_BUTTON_STATE_OFF = 0,
    GRID_BUTTON_STATE_ON,
}GRID_BUTTON_STATE;

typedef struct
{
    uint32_t                    freqValue;    
    int32_t                     dBFSValue;
    bool                        isActive;
}APP_DISPLAY_SIG_PARAMS;

typedef struct
{
    uint32_t                    lowFreqLimitInHz;
    uint32_t                    highFreqLimitInHz;
    int32_t                     lowAmpLimitIndBFS;
    int32_t                     highAmpLimitIndBFS;
}APP_DISPLAY_SIG_LIMITS;



typedef struct
{
    uint8_t                         cmdQueue[APP_DISPLAY_QUEUE_SIZE];
    MSG_QUEUE_HANDLE                queueHandle;
    uint8_t                         mode;
    PLAY_BUTTON_STATE               playButtonState;    
    GRID_BUTTON_STATE               gridButtonState;
    uint8_t                         activeSigSelected;
    uint8_t                         activeUnitSelected;
    WINDOW_TYPE                     activeWindowSelected;
    APP_DISPLAY_SIG_LIMITS          sigLimits;
    APP_DISPLAY_SIG_PARAMS          toneParams[APP_DISPLAY_MAX_TONES];
    APP_DISPLAY_EVENT_CALLBACK      eventsCallback;    
    APP_DISPLAY_ACTIVE_SCREEN       activeScreen;
}APP_DISPLAY_DATA;
    
void APP_DISPLAY_TaskInitialize(void);
void APP_DISPLAY_Task(void);
bool APP_DISPLAY_RegisterCallback(APP_DISPLAY_EVENT_CALLBACK evHandler);
bool APP_DISPLAY_AddCommand(APP_DISPLAY_CMD cmd);


#ifdef	__cplusplus
}
#endif

#endif	/* APP_DISPLAY_H */

