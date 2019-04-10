/*******************************************************************************
  Sample Application Header

  File Name:
    app.h

  Summary:
    Sample application definitions and prototypes

  Description:
    This file contains the sample application's definitions and prototypes.
 ******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
 ******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_definitions.h"
#include "system_config.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *************************************************;jgbpynfgobyfgl****************************
extern SYS_MODULE_OBJ    usartModule;

// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
#define APP_NO_OF_BYTES_TO_READ 1
#define APP_NUM_LINES           7
#define APP_BUFFER_SIZE         60
#define APP_UART_BAUDRATE       9600
#define APP_USR_ESC_KEY         0x1B
#define APP_USR_RETURN_KEY      0x0D
#define APP_USART_DRIVER_INDEX  DRV_USART_INDEX_0

/* Driver and App. Contexts */
#define APP_DRV_CONTEXT         1
#define APP_USR_CONTEXT         2

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
    /* In this state, the application opens the driver. */
    APP_DRV_OPEN,

    /* In this state driver will be ready to accept buffer */
    APP_DRV_READY,

    /* In this state driver prints demo banner message */
    APP_DRV_MSG_WRITE,

    /* In this state driver waits for the buffer event to get complete. */
    APP_WAIT_FOR_DONE,

    /* In this state, Demo App. accepts data from user*/
    APP_USR_MSG_READ,

    /* In this state, Demo App. writes user data to output terminal */
    APP_USR_MSG_WRITE,

    /* In this state, Demo App. prints banner message and exits the App. */
    APP_DRV_DEMO_COMPLETE,

    /* The application does nothing in the idle state. */
    APP_IDLE,

    /* This state indicates an error has occurred. */
    APP_ERROR,
            
} APP_STATES;


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
    /* Application current state */
    APP_STATES currentState;

    /* Application Previous state */
    APP_STATES prevState;

    /* USART buffer for display */
    char __attribute__ ((aligned (32))) buffer[APP_BUFFER_SIZE];

    /* Data Size in the USART Buffer */
    uint32_t bufferSize;

    /* Demo Application Message Indices */
    uint32_t usrMsgIndex;
    
    /* Cleint Status */
    DRV_USART_CLIENT_STATUS usartStatus;

    /* USART driver handle */
    DRV_HANDLE usartHandle;

    /* Handle returned by USART for buffer submitted */
    DRV_USART_BUFFER_HANDLE usartBufferHandle;

    /* Handle returned by USART for buffer event */
    DRV_USART_BUFFER_EVENT  usartBufferEvent;

    /* Flag to indicate the driver message is been processed */
    bool drvBufferEventComplete;

    /* Flag to indicate the user message is been processed */
    bool usrBufferEventComplete;

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     Sample application's initialization routine

  Description:
    This routine initializes sample application's state machine.
    
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
    Demo application tasks function

  Description:
    This routine is the demo application's tasks function.  It
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

void APP_Tasks ( void );
/*******************************************************************************
  Function:
    void App_UpdateTaskState ( uint32_t appState )

  Summary:
    Updates the application's state

  Description:
    This routine updates the demo app. state. This routine is used to switch
    the driver between read and write modes.

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
    UpdateTaskState();

  Remarks:
    This routine must be called after every successful data transfer to update
    the application's next state.
 */

void App_GetNextTaskState ( uint32_t appState );

#endif /* _APP_HEADER_H */
/*******************************************************************************
 End of File
*/

