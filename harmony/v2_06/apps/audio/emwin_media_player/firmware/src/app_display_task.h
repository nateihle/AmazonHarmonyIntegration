/*******************************************************************************
  Universal Audio Decoders Application Demo

  Company:
    Microchip Technology Inc.

  File Name:
    app_display_task.h

  Summary:
    This header file provides prototypes and data type definitions for the 
    display task.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by other tasks and some of them 
    are only used internally by the application
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

#ifndef APP_DISPLAY_TASK_H
#define	APP_DISPLAY_TASK_H


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define MSG_BUFFER_LEN      20

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END
    
    
    
#define ENTER_CRITICAL(currentInterruptState)      Nop()
#define EXIT_CRITICAL(previousInterruptState)      Nop()

typedef struct
{
    uint16_t inPtr;
    uint16_t outPtr;
    uint16_t maxLen;
    uint8_t* pBuffer;
}MSG_QUEUE;

typedef enum
{
    DISP_CMD_POPULATE_TRACKLIST_REQ = 0,
    DISP_CMD_CLEAR_TRACKLIST_REQ,
    DISP_CMD_TRACK_CHANGED_EVT,
    DISP_CMD_TRACK_CHANGE_REQ,
    DISP_CMD_VOLUME_CHANGED_EVT,
    DISP_CMD_VOLUME_CHANGE_REQ,
    DISP_CMD_MUTE_ON_EVT,
    DISP_CMD_MUTE_OFF_EVT,
    DISP_CMD_MUTE_OFF_REQ,
    DISP_CMD_SHUFFLE_ON_EVT,
    DISP_CMD_SHUFFLE_OFF_EVT,
    DISP_CMD_LOOP_TRACKLIST_EVT,
    DISP_CMD_UNLOOP_TRACKLIST_EVT,
    DISP_CMD_LOOP_SINGLE_TRACK_EVT,
    DISP_CMD_UPDATE_PROGBAR_IN_PER_REQ,
    DISP_CMD_UPDATE_TOTAL_TRACK_TIME_REQ,
    DISP_CMD_UPDATE_ELAPSED_TRACK_TIME_REQ,
    DISP_CMD_PLAYER_PLAY_EVT,
    DISP_CMD_PLAYER_PAUSE_EVT,
    DISP_CMD_PLAYER_PAUSE_REQ,
    DISP_CMD_TRACK_SEEK_EVT,
    DISP_CMD_USB_MODE_CHANGE_EVT,
    DISP_CMD_USB_MODE_ON_REQ,
    DISP_CMD_SDCARD_MODE_ON_REQ,
    DISP_CMD_USB_MEDIA_FS_MOUNT_ERROR,
    DISP_CMD_SDCARD_MEDIA_FS_MOUNT_ERROR,
    DISP_CMD_MAX,
}DISP_CMD;

typedef struct
{
    uint8_t msgBuffer[MSG_BUFFER_LEN];
    MSG_QUEUE msgQueue;
}DISPLAY_TASK_DATA;

void APP_DISPLAY_Initialize(void);
void APP_DISPLAY_Tasks(void);

#include "audio_player_dialog.h"


#ifdef __cplusplus
}
#endif

#endif

