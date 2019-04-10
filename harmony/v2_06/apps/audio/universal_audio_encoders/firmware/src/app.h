/*******************************************************************************
  MPLAB Harmony Application Header File

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

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "encoder.h"
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/


typedef enum
{
	APP_STATE_BUS_ENABLE = 0,
    APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE,
    APP_STATE_WAIT_FOR_DEVICE_ATTACH,
    APP_STATE_AUDIO_CODEC_OPEN,
    APP_STATE_CODEC_SET_BUFFER_HANDLER,
    APP_STATE_START_RECORD,
            APP_STATE_PREPARE_ENCODING,
    APP_SUBMIT_INITIAL_CODEC_READ_REQUEST,
    APP_INIT_ENCODER,
    APP_PROCESS_DATA,
    APP_STATE_DEVICE_CONNECTED,
    APP_STATE_MOUNT_DISK,
    APP_STATE_UNMOUNT_DISK,
    APP_STATE_OPEN_FILE,
    APP_STATE_WRITE_TO_FILE,
    APP_STATE_CLOSE_FILE,
    APP_STATE_CLOSE_ENCODER,
    APP_STATE_CONSTRUCT_AUDIO_FILE_HEADER,
    APP_STATE_IDLE,
    APP_STATE_ERROR
} APP_STATES;

typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE bufHandle1;
    DRV_CODEC_BUFFER_HANDLE bufHandle2;    
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *bufferObject1;
    uint8_t *bufferObject2;
    uint32_t bufferSize;
    bool isCodecReadComplete1;
    bool isCodecReadComplete2;
} APP_CODEC_CLIENT;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
 /* The application's current state */
  
    /* SYS_FS File handle for 1st file */
    SYS_FS_HANDLE fileHandle;

    /* Application's current state */
    APP_STATES state;

    /* Number of bytes written */
    uint32_t nBytesWritten;

    /* Number of bytes read */
    uint32_t nBytesRead;

    bool deviceIsConnected;
    
    APP_CODEC_CLIENT codecClientRead;
    
    /* Microphone recording is in progress */
    bool record;

    /* Runtime encoder type*/
    AUDIO_FILE_FORMAT runTimeAudioType;
    
    /* Encoder selected index from input system */
    /* 0: PCM, 1: ADPCM, 2: OPUS, 3: SPEEX */
    uint8_t encoderSelectIdx; 
} APP_DATA;
extern APP_DATA appData;
#define APP_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16))) 
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

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

/*******************************************************************************
  Function:
    void APP_StartRecord ( void )

  Summary:
    Set Microphone recording to start 

  Description:
    Set Microphone recording to start

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_StartRecord();
void APP_StopRecord();

void APP_ListWheelItemChanged(uint8_t idx);
void APP_IncrementEncoderSelectionIndex();
void APP_DecrementEncoderSelectionIndex();

void APP_ButtonInit(void);
void APP_ButtonTask(void);

void APP_ButtonsHandleInterrupt(unsigned int newButtonState);
#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

