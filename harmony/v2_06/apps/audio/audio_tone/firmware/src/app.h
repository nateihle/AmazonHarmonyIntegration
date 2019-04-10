/*******************************************************************************
  AUDIO TONE CONTROL DEMO 

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

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
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/* Standard Includes */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* PIC32 related includes */
#include <p32xxxx.h>
#include <xc.h>

/* Harmony related Include/s */
#include "system/system.h"
#include "system/debug/sys_debug.h"
//#include "system_definitions.h"

//APP: Hardware specific configuration
#include "driver/tmr/drv_tmr.h"
#include "bsp.h"    // was "bsp_config.h"
#include "system_config.h"

#include "app_config.h" //Includes system_definitions.h

//APP: Error and assert support 
//#include "app_error.h"

//NOTE: Included in app_config.h
//APP: Audio Generate API/Audio Data Queue 
#include "AudioGenerateFx.h"
#include "AudioPlayBufferQueue.h"

#ifdef HAS_VOLUME_CTRL
//APP: Volume control API
#include "volume_mx.h"
#endif
    
//APP: Audio Codec API
#include "audio_codec.h"

//APP: Display
//#include "display_task.h"

//APP: Button ICN interrupt handler
#include "buttons.h"


#if 0
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************


/* BT module reset duration */

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
// Application States
//
//Summary:
//  Application states enumeration
//
//Description:
//  This enumeration defines the valid application states.  These states
//  determine the behavior of the application at various times.
//
//******************************************************************************
typedef enum
{
   //APP_Tasks: Application defined states
   APP_STATE_CODEC_OPEN,
   APP_STATE_CODEC_SET_BUFFER_HANDLER,
   APP_STATE_CODEC_ADD_BUFFER,
   APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE,
   APP_STATE_CODEC_BUFFER_COMPLETE,
   APP_STATE_BUTTON_TASK_RUN,
   APP_STATE_DISPLAY
} APP_STATES;

#if 0
//******************************************************************************
//Application USART client for BT
//
//Summary:
//  Application USART client for BT.
//
//Description:
//  This object holds the BT USART's client handle, read and write buffer handle
//  created and the context
//******************************************************************************
typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle;
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject;
    size_t bufferSize;

} APP_CODEC_CLIENT;
#endif

typedef struct
{
    DRV_HANDLE handle;
    DRV_I2S_BUFFER_HANDLE writeBufHandle;
    DRV_I2S_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject;
    size_t bufferSize;

} APP_I2S_CLIENT;


//******************************************************************************
//
// AUDIO_GENERATE_STATUS
//
// Keeps track of tone, parameters and buffer for current values and next 
// initialization.
//
// NOTE: Used to reinit the generation.
//
//******************************************************************************
typedef struct _AUDIO_GENERATE_STATUS
{
   bool            processingBuffer;
   AUDIO_GEN_TYPE  currentToneType;
   AUDIO_GEN_TYPE  nextToneType;
   AUDIO_GEN_PARAM currentToneParam;
   AUDIO_GEN_PARAM nextToneParam;
   bool            changeToneType;
   bool            onOff;
   int32_t         fMinHz;
   int32_t         fMaxHz;
} AUDIO_GENERATE_STATUS;


#if 0
//TODO: use this queue. not the PLAY Queue
typedef struct{
    int8_t buffer[DECODER_MAX_OUTPUT_BUFFER_SIZE];
    bool inUse;
    bool decoded;
    DRV_CODEC_BUFFER_HANDLE writeHandler;
    size_t bufferSize;
}AUDIO_QUEUEBUFFER;
#endif


//TODO: This goes in display_tasks.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Change the agStatus depending on mode:
//  1) MODE_TONE_CONTINUOUS:
//     --> ON, SELECT_F1
//     --> allow real-time adjustment fHz
//     --> ON/OFF SW4
//  2) MODE_TONE:
//     --> OFF, SELECT_T
//     --> Adjust T/F1/Duty Cycle
//     --> One shot/repeated with SW4
//  3) MODE_CHIRP
//     --> OFF, SELECT_T
//     --> Adjust T/F1/F2/Duty Cycle
//     --> One shot/Repeated with SW4
//  5) MODE_UWN, MODE_GWN, MODE_PN
//     --> ON, SELECT_T
//     --> Adjust T/Duty Cycle
//     --> One shot/Repeated with SW4
//
//  TIME_DURATION -- SELECT_T and long press SW3 give inf.
//                   SELECT_T and long press SW5 
//                   SELECT_T and short press SW3/SW5 increase/decrease 10Hz
//
//  DUTY_CYCLE    -- SELECT_DUTY and long press SW5 gives 100%
//                   SELECT_DUTY and long press SW3 gives on_shot
//                   SELECT_DUTY and short press SW3/SW5 increase/decrease 5%
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//GUI Signal Generation Modes
//--> setting mode/not generation output mode
typedef enum _GUI_MODE
{
   MODE_TONE=0,
   MODE_CHIRP, 
   //MODE_UWN,  
   //MODE_GWN, 
   //MODE_PN, 
   GUI_NUMMODES
} GUI_MODE;

//GUI Parameter value selects
typedef enum _GUI_SELECT
{
   SELECT_F1 = 0,
   SELECT_F2 = 1,
   SELECT_T  = 2,
   //SELECT_DUTY,
   GUI_NUMSEL = 3
} GUI_SELECT;

//GUI state data
typedef struct _GUI_DATA
{
   bool       displayUpdate;
   GUI_MODE   mode;
   GUI_SELECT select;
   int32_t    f1Hz;
   int32_t    f2Hz;
   int32_t    timeDeltaMs;     //NOTE <0 implies inf. 
   int32_t    durationSamples; //NOTE <0 implies inf. 
   int16_t    progress;
   //int16_t    dutyCycle;
   bool       onOff;   //Generated output
   bool       changeToneMode;
   bool       changeMode;
   int8_t     lastButton;   
   int8_t     volume;           // in percent, for devices w/o analog pot
} GUI_DATA;

//******************************************************************************
// Application Data
//
// Summary:
//   Holds application data
//
// Description:
//   This structure holds the application's data.
//
// Remarks:
//   Application strings and buffers are be defined outside this structure.
//
//******************************************************************************
typedef struct _APP_DATA
{
    //APP_Tasks: current state 
    APP_STATES state;

    //APP: SPI-I2S client handle
    AUDIO_CODEC_DATA codecData; //Client and State
    
    //APP: Button Repeat Timer Handle
    DRV_HANDLE repeatTmrHandle;

    //Audio generated output
    AUDIO_GENERATE_STATUS agStatus;

    GUI_DATA guiData;

    //Audio Decode Source Output.
    //AUDIO_QUEUEBUFFER audioBuffer[AUDIO_QUEUEBUFFER_NUMBER];

} APP_DATA;

//APP_DisplayTask: Display Control 
typedef struct _DISPLAY_STATE
{
    int DisplayUpdate;
    int VLED_Update;
    int VLED1;
    int VLED2;
    int VLED3;
    int VLED4;
    int VLED5;
    int VLED_R;
    int VLED_G;
    int VLED_B;
    int32_t frequency;
    int16_t amplitude;
} DISPLAY_STATE;
 
int8_t  mRepeatButton;
int32_t mRepeatCount;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
//  void APP_Initialize ( void )
//
//Summary:
//   MPLAB Harmony application initialization routine.
//
// Description:
//   This function initializes the Harmony application.  It places the 
//   application in its initial state and prepares it to run so that its 
//   APP_Tasks function can be called.
//
// Precondition:
//   All other system initialization routines should be called before calling
//   this routine (in "SYS_Initialize").
//
// Parameters:
//   None.
//
// Returns:
//   None.
//
// Example:
//   <code>
//    APP_Initialize();
//    </code>
//
// Remarks:
//   This routine must be called from the SYS_Initialize function.
//
//******************************************************************************
void APP_Initialize (void);


//******************************************************************************
// Function:
//   void APP_Tasks ( void )
//
// Summary:
//   MPLAB Harmony Demo application tasks function
//
// Description:
//   This routine is the Harmony Demo application's tasks function.  It
//   defines the application's state machine and core logic.
//
// Precondition:
//   The system and application initialization ("SYS_Initialize") should be
//   called before calling this.
//
// Parameters:
//   None.
//
// Returns:
//   None.
//
// Example:
//   <code>
//   APP_Tasks();
//   </code>
//
// Remarks:
//   This routine must be called from SYS_Tasks() routine.
//
//*******************************************************************************
void APP_Tasks ( void );


//*******************************************************************************
// 
//  APP_BufferEventHandler()
//
// Description:
//   This is the Event Handler for CODEC Tx and Rx Complete Events.
//
// Precondition:
//   The system and application initialization ("SYS_Initialize") should be
//   called before calling this.
//
// Parameters:
//   None.
//
// Returns:
//   None.
//
// Example:
//   <code>
//   </code>
//
// Remarks:
// 
//*******************************************************************************
//void APP_BufferEventHandler(DRV_CODEC_BUFFER_EVENT event, 
//                            DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context);


//APP: Audio Output (SPI/I2S D/A Amplifier module)
//void AUDIO_CODEC_Initialize(void);

//APP: API to CODEC Event Handler
void APP_CodecTxBufferComplete(void);
void APP_CodecCommandClear(void);

//APP_ButtonTask:  Prototypes of functions in btad_buttons.c
// NOTE: Buttons are a specific implementation for the App
//         (i.e cannot be separate object class)
void APP_ButtonInit(void);
void APP_ButtonTask(void);
void APP_OnButtonEvent(APP_DATA * appData, 
                       uint8_t button, 
                       bool bButtonClosed, 
                       int32_t repeatCount);
void buttons_handleInterrupt();

//APP Repeat Timer - used for button timing.
void APP_PeriodicTimerInit();
void APP_handlePeriodicTimerSignal(uintptr_t context, uint32_t alarmCount);
void APP_PeriodicTimer_Task();

//TODO put with AudioGenerateFx Class -- NOT APP. CAL
//AudioGenerateFx
void APP_audioGenSetNextToneType(AUDIO_GENERATE_STATUS * ags, 
                                 AUDIO_GEN_TYPE toneType,
                                 AUDIO_GEN_PARAM nextAgParam,
                                 bool onOff);
void APP_audioGenSetFrequency(AUDIO_GENERATE * ag, int32_t freq);
int32_t APP_audioGenFrequencyGet(AUDIO_GENERATE * ag);

void APP_DisplayTask(GUI_DATA * guiData);
void APP_DisplayInit(GUI_DATA * guiData, AUDIO_GENERATE_STATUS * agStatus);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
extern APP_DATA appData;

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _APP_HEADER_H */
/*******************************************************************************
 End of File
*/
