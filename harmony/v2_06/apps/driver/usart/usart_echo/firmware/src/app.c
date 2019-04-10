/*******************************************************************************
  USART Driver Demo Application Soure File.

  File Name:
    app.c

  Summary:
    USART driver echo demo application.

  Description:
    This file contains the USART driver echo demo application's implementation.
 ******************************************************************************/


// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
/* USART Driver Demo Banner Message */
char appMsg[APP_NUM_LINES][APP_BUFFER_SIZE] = {
        {"*******************************************************\r\n"},
        {"Welcome to Microchip USART Driver Demo Application.\r\n"},
        {"Press any character, the character will be echoed back.\r\n"},
        {"Press 'ESC' key to exit the Demo Application.\r\n"},
        {"*******************************************************\r\n"},
        {"\n"},
        {"\r\n\r\n****** End of USART Driver Demo Application. *****"},};

/* User Application Data Structure */
APP_DATA appData;

extern SYSTEM_OBJECTS sysObj;

#define min(a,b) (((a) < (b)) ? (a) : (b))
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data.

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
*/
// *****************************************************************************
// *****************************************************************************
// Section: Application Data Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize( void )
{
    /* Set the App. previous state to its initial state. */
    appData.prevState         = APP_DRV_OPEN;
    /* Set the App. current state to its initial state. */
    appData.currentState      = APP_DRV_OPEN;
    /* Initialise buffer Size */
    appData.bufferSize        = 0;
    /* Demo App. message index */
    appData.usrMsgIndex       = 0;
    /* Set the USART Clent Statud to error */
    appData.usartStatus       = DRV_USART_CLIENT_STATUS_ERROR;
    /* Set the USART buffer handler to invalid */
    appData.usartBufferHandle = DRV_HANDLE_INVALID;
    /* Set the USART handler to invalid */
    appData.usartHandle       = DRV_HANDLE_INVALID;
    /* Set the USART buffer event to invalid */
    appData.usartBufferEvent  = DRV_USART_BUFFER_EVENT_ERROR;
    /* Set the initial state of event flags for driver messages */
    appData.drvBufferEventComplete = false;
    /* Set the initial state of event flags for user messages */
    appData.usrBufferEventComplete = false;
    /* Clear Application Buffer */
    strcpy(appData.buffer, "");
}


void APP_BufferEventHandler(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context )
{
    switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if(context == APP_DRV_CONTEXT)
            {
                /* Update buffer event status */
                appData.drvBufferEventComplete = true;
            }
            else if (context == APP_USR_CONTEXT)
            {
                /* if user has pressed 'ESC' key, exit the app */
                if(appData.buffer[0] == APP_USR_ESC_KEY)
                {
                    appData.currentState = APP_DRV_DEMO_COMPLETE;
                    appData.usrBufferEventComplete = false;
                }
                /* if user has pressed 'enter' key, add newline character */
                else if(appData.buffer[0] == APP_USR_RETURN_KEY)
                {
                    strcpy(appData.buffer, "\r\n");
                    /* Update Buffer Size */
                    appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                    /* Update buffer event status */
                    appData.usrBufferEventComplete = true;
                }
                else
                {
                    /* Update buffer event status */
                    appData.usrBufferEventComplete = true;
                }
            }
        }
            break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
            break;

        /* Buffer event has aborted */
        case DRV_USART_BUFFER_EVENT_ABORT:
            break;
    }
}


/*******************************************************************************
  Function:
    void APP_Tasks( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks( void )
{
    /* Check the Application State*/
    switch ( appData.currentState )
    {
        /* Open USART Driver and set the Buffer Event Handling */
        case APP_DRV_OPEN:
        {
            /* Open an instance of USART driver */
            appData.usartHandle = DRV_USART_Open(APP_USART_DRIVER_INDEX,
                         (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING));

            /* Check the USART driver handler */
            if (appData.usartHandle != DRV_HANDLE_INVALID )
            {
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                       APP_BufferEventHandler, APP_DRV_CONTEXT);
                /* Set the app state to Ready */
                appData.prevState    = APP_DRV_OPEN;
                appData.currentState = APP_DRV_READY;
            }
            else
            {
                /* If handler is not ready, set the App. state to error */
                appData.currentState = APP_ERROR;
            }
        }
        break;

        /* When the driver is ready, add Data Buffer to queue */
        case APP_DRV_READY:
        {
            /* Get the USART driver status */
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                /* To begin with, Send the newline character to buffer */
                strcpy(appData.buffer, "\r\n");
                /* Update Buffer Size */
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                strlen(appData.buffer));
                /* Submit buffer to USART */
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize );

                /* check buffer handler */
                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Terminate the Demo, if add to buffer is failed */
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    /* If handle is valid, Set the app state to next state */
                    appData.prevState    = APP_DRV_READY;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
            else
            {
                /* If client is not ready, set App. state to error */
                appData.currentState = APP_ERROR;
            }
        }
        break;

        case APP_WAIT_FOR_DONE:
        {
            /* check if the driver buffer event is complete or not */
            if(appData.drvBufferEventComplete)
            {
                appData.drvBufferEventComplete = false;
                /* Get next Demo App. state */
                App_GetNextTaskState(appData.prevState);
            }
            /* check if the user buffer event is complete or not */
            else if(appData.usrBufferEventComplete)
            {
                appData.usrBufferEventComplete = false;
                /* Get next Demo App. state */
                App_GetNextTaskState(appData.prevState);
            }
        }
        break;

        case APP_DRV_MSG_WRITE:
        {
            /* Get the USART Driver Status */
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, appMsg[appData.usrMsgIndex] );
                /* Update Buffer Size */
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                /* Submit buffer to USART */
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the App. state to error */
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    /* Set the App. state to wait for done */
                    appData.prevState    = APP_DRV_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_USR_MSG_READ:
        {
            /* Get the USART driver status */
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );
            /* Check if the client is ready or not */
            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, appMsg[appData.usrMsgIndex] );
                /* Update Buffer Size */
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                /* Submit buffer to USART */
                DRV_USART_BufferAddRead( appData.usartHandle,
                                         &(appData.usartBufferHandle),
                                         appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the app state to invalid */
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    /* Set the App. state to wait for done */
                    appData.prevState    = APP_USR_MSG_READ;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_USR_MSG_WRITE:
        {
            /* Get the USART driver status */
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                /* Submit buffer to USART */
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the App. state to error */
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    /* Set the App. state to wait for done */
                    appData.prevState    = APP_USR_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_DRV_DEMO_COMPLETE:
        {
            /* Get the USART driver status */
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {

                appData.usrMsgIndex++;
                /* Get the exit message from the user data buffer */
                strcpy( appData.buffer, appMsg[appData.usrMsgIndex] );
                /* Update Buffer Size */
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                   strlen(appData.buffer));
                /* Submit buffer to USART */
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the app state to invalid */
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    /* Set the app state to Wait for done */
                    appData.prevState    = APP_DRV_DEMO_COMPLETE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_IDLE:
        {
            /* Close USART Driver */
            DRV_USART_Close( appData.usartHandle );
            /* Deinitialize the driver */
            DRV_USART_Deinitialize( sysObj.drvUsart0 );
            /* The appliction comes here when the demo has completed
             * successfully. Switch on LED D5. */
            BSP_LEDOn(APP_LED_SUCCESS);
        }
        break;

        case APP_ERROR:
        {
            /* The appliction comes here when the demo
             * has failed. Switch on the LED D9.*/
            BSP_LEDOn(APP_LED_FAILURE);
        }
        break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void App_UpdateTaskState( void )

  Remarks:
    See prototype in app.h.
 */

void App_GetNextTaskState(uint32_t appState)
{
    switch ( appState )
    {
        case APP_DRV_READY:
            /* Set the next Demo App. State */
            appData.currentState = APP_DRV_MSG_WRITE;
            break;

        case APP_DRV_MSG_WRITE:
        {
            if(appData.usrMsgIndex < (APP_NUM_LINES - 2))
            {
                /* Get the next message from App. banner */
                appData.usrMsgIndex++;
                /* Set the next Demo App. State */
                appData.currentState = APP_DRV_MSG_WRITE;
            }
            else
            {
                /* Insert newline character, before accepting data from user */
                strcpy(appData.buffer, appMsg[appData.usrMsgIndex]);
                appData.bufferSize = APP_NO_OF_BYTES_TO_READ;
                /* Set the buffer event for user data */
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                  APP_BufferEventHandler, APP_USR_CONTEXT);
                /* Set the next Demo App. State */
                appData.currentState = APP_USR_MSG_READ;
            }
        }
        break;

        case APP_USR_MSG_READ:
            /* Set the next Demo App. State */
            appData.currentState = APP_USR_MSG_WRITE;
            break;

        case APP_USR_MSG_WRITE:
            /* Set the next Demo App. State */
            appData.currentState = APP_USR_MSG_READ;
            break;

        case APP_DRV_DEMO_COMPLETE:
            /* Set the next Demo App. State */
            appData.currentState = APP_IDLE;
            break;
    }
}

/*******************************************************************************
 End of File
*/

