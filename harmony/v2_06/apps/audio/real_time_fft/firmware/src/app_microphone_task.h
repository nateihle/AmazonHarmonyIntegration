/*******************************************************************************
  Audio Microphone storage

  Company:
    Microchip Technology Inc.

  File Name:
    app_microphone_task.h

  Summary:
    This header file provides prototypes and definitions for the application

  Description:
    This header file provides prototypes and definitions for the application
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

#ifndef _APP_MICROPHONE_TASK_HEADER_H
#define _APP_MICROPHONE_TASK_HEADER_H

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

#include "system_config.h"
#include "system_definitions.h"
#include "app_queue.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define APP_MICROPHONE_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(4)))
#define APP_MICROPHONE_REQ_QUEUE_SIZE                               10   
#define APP_MICROPHONE_MAX_EVENT_LISTNERS                           2    

typedef enum
{
    APP_MICROPHONE_EVENT_DATA_AVAIL = 0,
    APP_MICROPHONE_EVENT_STOPPED,
}APP_MICROPHONE_EVENT;
    
 typedef void (*APP_MICROPHONE_EVENT_CALLBACK)(APP_MICROPHONE_EVENT event, const void* const pEventData);
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{                        
    APP_MICROPHONE_TASK_STATES_INIT = 0,
    APP_MICROPHONE_TASK_STATES_CODEC_OPEN,
    APP_MICROPHONE_TASK_STATES_CODEC_SET_BUFFER_HANDLER,    
    APP_MICROPHONE_TASK_STATES_IDLE,            
    APP_MICROPHONE_TASK_STATES_ERROR,

} APP_MICROPHONE_TASK_STATES;


typedef enum
{   
    APP_MICROPHONE_REQ_START = 0,
    APP_MICROPHONE_REQ_STOP,        
    APP_MICROPHONE_REQ_MICROPHONE_DATA_AVAIL,
    APP_MICROPHONE_REQ_NO_BUFFER_TO_PLAY,
    APP_MICROPHONE_REQ_MAX,
}APP_MICROPHONE_REQ;

// *****************************************************************************
/* Application client

  Summary:
    Application client

  Description:
    This object holds the client handle, read and write buffer handle and event handlers
    created and the context
*/
typedef struct
{    
    DRV_HANDLE                      handle;
    DRV_CODEC_BUFFER_HANDLE         bufHandle;    
    DRV_CODEC_BUFFER_EVENT_HANDLER  bufferEventHandler;
    uintptr_t                       context;        
    size_t                          bufferSize;            
} APP_MICROPHONE_CODEC;


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
    volatile APP_MICROPHONE_TASK_STATES     state;    
    bool                                    isTaskRun;
    APP_MICROPHONE_CODEC                    codecClient;                
    uint32_t                                codecInIndex;
    uint32_t                                codecOutIndex;   
    uint8_t                                 nEventListners;
    APP_MICROPHONE_EVENT_CALLBACK           eventsCallback[APP_MICROPHONE_MAX_EVENT_LISTNERS];    
    uint8_t                                 cmdQueue[APP_MICROPHONE_REQ_QUEUE_SIZE];
    MSG_QUEUE_HANDLE                        queueHandle;        
} APP_MICROPHONE_TASK_DATA;

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
    void APP_MIC_INPUT_TaskInitialize ( void )

  Summary:
     Application initialization routine

  Description:
    This routine initializes the application.

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
void APP_MICROPHONE_TaskInitialize (void);

/*******************************************************************************
  Function:
    void APP_MIC_INPUT_Tasks ( void )

  Summary:
    Application tasks function

  Description:
    This routine is the application's tasks function.  It
    implements the application's state machine and core logic.

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
void APP_MICROPHONE_Tasks ( void );


/*******************************************************************************
  Function:
    void APP_MIC_INPUT_BufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context );

  Summary:
    Event Handler for Buffer complete event.

  Description:
    This is the Event Handler for HEadphone DMA data transfer event complete.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
 None
*/
bool APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ cmd);
bool APP_MICROPHONE_RegisterCallback(APP_MICROPHONE_EVENT_CALLBACK evHandler);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************


#ifdef __cplusplus
}
#endif

#endif /* _APP_MICROPHONE_TASK_HEADER_H */
/*******************************************************************************
 End of File
*/
