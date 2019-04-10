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

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define MAX_NUM_OF_BYTES        64

#define MAX_NUM_OF_BYTES_IN_BUF (MAX_NUM_OF_BYTES + 4)
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
    APP_STATE_INIT,

    /* Check if the driver is opened and ready  */
    APP_STATE_CHECK_DRVR_STATE,

    /* Application let UART1 and UART2 transmit data inbetween  */
    APP_STATE_TRANSFER_DATA_BTWN_USART1_USART2,

    /* Application: USART1 gets data from USART2 */
    APP_STATE_USART1_RX,

    /* Application: USART1 transmitts data to USART2. */
    APP_STATE_USART1_TX,

    /* Application to wait (non blocking) for the USART1 to complete the Tx*/
    APP_STATE_USART1_WAIT_FOR_TX_COMPLETION,

    /* Application to wait (non blocking) for the USART2 to complete the Tx*/
    APP_STATE_USART2_WAIT_FOR_TX_COMPLETION,

    /* Application to wait (non blocking)for the USART1 to receive back the transmitted data*/
    APP_STATE_USART1_WAIT_TO_RX_BACK_DATA,

    /*Application verifies the received back data at UART1*/
    APP_STATE_USART1_VERIFY_RXED_BACK_DATA,

    /* Application: USART2 Transmitts the received data to USART1. */
    APP_STATE_USART2_TX,

    /* Application: USART2 Rx data from USRAT1 */
    APP_STATE_USART2_RX,

    /* Application verifies the write and read data. */
    APP_STATE_VERIFY_LOOPBACK_DATA,

    /* USART1 Application State machine completed its intended tasks. Go to Idle*/
    APP_STATE_USART1_IDLE,

    /* USART2 Application State machine completed its intended tasks. Go to Idle*/
    APP_STATE_USART2_IDLE,

    /* Application error state */
    APP_STATE_ERROR,

    APP_STATE_RELEASE_DRVR_RESOURCES,

    /* In this state, application is in IDLE state after verifying the received data. */
    APP_STATE_IDLE,

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

    /* Current state of Application */
    APP_STATES  state;

    /* Application current state of USART1 */
    APP_STATES  usart1State;

    /* Application current state of USART2 */
    APP_STATES  usart2State;

    /* UART1 Driver Handle  */
    DRV_HANDLE  drvUsart1Handle;

	/* UART2 Driver Handle  */
    DRV_HANDLE  drvUsart2Handle;

    /* Write buffer handle */
    DRV_USART_BUFFER_HANDLE   drvUsart1TxBufHandle;

    /* Read buffer handle */
    DRV_USART_BUFFER_HANDLE   drvUsart1RxBufHandle;

    /* Write buffer handle */
    DRV_USART_BUFFER_HANDLE   drvUsart2TxBufHandle;

    /* Read buffer handle */
   DRV_USART_BUFFER_HANDLE   drvUsart2RxBufHandle;

    /* UART1 TX buffer  */
    uint8_t  drvUsart1TxBuffer[MAX_NUM_OF_BYTES_IN_BUF];

    /* UART1 RX buffer  */
    uint8_t  drvUsart1RxBuffer[MAX_NUM_OF_BYTES_IN_BUF];

    /* UART2 TX buffer  */
    uint8_t  drvUsart2TxBuffer[MAX_NUM_OF_BYTES_IN_BUF];

    /* UART2 RX buffer  */
    uint8_t  drvUsart2RxBuffer[MAX_NUM_OF_BYTES_IN_BUF];


} APP_DATA;


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

void APP_Tasks ( void );


#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

