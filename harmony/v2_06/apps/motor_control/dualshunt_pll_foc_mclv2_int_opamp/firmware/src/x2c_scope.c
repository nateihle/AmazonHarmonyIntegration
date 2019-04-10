/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    x2c_scope.c

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "x2c_scope.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

X2C_SCOPE_DATA x2c_scopeData;


DRV_HANDLE X2C_SCOPE_UART_HANDLE;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(sendSerial, receiveSerial, isReceiveDataAvailable, isSendReady);
    X2CScope_Initialise();
   
}


void sendSerial(uint8_t data)
{
    PLIB_USART_TransmitterByteSend(X2C_SCOPE_UART_MODULE_ID, data);

     
}

uint8_t receiveSerial()
{
    if((PLIB_USART_ReceiverFramingErrorHasOccurred(X2C_SCOPE_UART_MODULE_ID))\
      |(PLIB_USART_ReceiverParityErrorHasOccurred(X2C_SCOPE_UART_MODULE_ID))\
      |(PLIB_USART_ReceiverOverrunHasOccurred(X2C_SCOPE_UART_MODULE_ID)))
     {
       PLIB_USART_ReceiverOverrunErrorClear(X2C_SCOPE_UART_MODULE_ID);

        return ((uint8_t)0);   
     }
   return (PLIB_USART_ReceiverByteReceive(X2C_SCOPE_UART_MODULE_ID));
   
}

uint8_t isReceiveDataAvailable()
{
     return (PLIB_USART_ReceiverDataIsAvailable(X2C_SCOPE_UART_MODULE_ID));
     
}

uint8_t isSendReady()
{
    return(PLIB_USART_TransmitterIsEmpty(X2C_SCOPE_UART_MODULE_ID));
    
}




/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void X2C_SCOPE_Initialize ( void )

  Remarks:
    See prototype in x2c_scope.h.
 */

void X2C_SCOPE_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    x2c_scopeData.state = X2C_SCOPE_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void X2C_SCOPE_Tasks ( void )

  Remarks:
    See prototype in x2c_scope.h.
 */

void X2C_SCOPE_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( x2c_scopeData.state )
    {
        /* Application's initial state. */
        case X2C_SCOPE_STATE_INIT:
        {
            bool appInitialized = true;
       
			X2CScope_Init();
        
            if (appInitialized)
            {
            
                x2c_scopeData.state = X2C_SCOPE_STATE_SERVICE_TASKS;
            }
            break;
        }

        case X2C_SCOPE_STATE_SERVICE_TASKS:
        {
			X2CScope_Communicate(); //Communicate with X2C Scope Plugin
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
