/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"


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

APP_DATA appData;

uint8_t gDataRxedAtUsart1 = false;
uint8_t gDataRxedAtUsart2 = false;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************


void APP_BufferEventHandlerUsart2(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context );

void APP_BufferEventHandlerUsart1(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context );

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*Application State machine for the USART1 Tx and Rx task*/
void APP_USART1_Task( void );

/*Application State machine for the USART2 Tx and Rx task*/
void APP_USART2_Task( void );


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{

    /* APP state task Init */
    appData.state = APP_STATE_INIT	;
    appData.usart1State = APP_STATE_USART1_RX;
    appData.usart2State = APP_STATE_USART2_RX;
    
    appData.drvUsart1Handle= DRV_HANDLE_INVALID;
    appData.drvUsart2Handle= DRV_HANDLE_INVALID;
		
	
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    uint8_t count=0;
    
    switch(appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            if(appData.drvUsart1Handle == DRV_HANDLE_INVALID)
            {
		/* Open the USART Driver for USART1 Client  */
               appData.drvUsart1Handle = DRV_USART_Open( DRV_USART_INDEX_0,
                                                     DRV_IO_INTENT_READWRITE );
		   
            }
            if(appData.drvUsart2Handle == DRV_HANDLE_INVALID)
            {
            	/* Open the USART Driver for USART2 Client  */
		appData.drvUsart2Handle = DRV_USART_Open( DRV_USART_INDEX_1,
							 DRV_IO_INTENT_READWRITE );
            }
		appData.state = APP_STATE_CHECK_DRVR_STATE;
         }break;

         case APP_STATE_CHECK_DRVR_STATE:
         {
                     /* Check the USART1 driver handler */
            if (appData.drvUsart1Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to Ready */
                appData.state   = APP_STATE_INIT;
		return;
            }
	/* Check the USART2 driver handler */
            if (appData.drvUsart2Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to Ready */
                appData.state   = APP_STATE_INIT;
		return;
            }

            DRV_USART_BufferEventHandlerSet(appData.drvUsart1Handle,APP_BufferEventHandlerUsart1,(uintptr_t)1);
	    DRV_USART_BufferEventHandlerSet(appData.drvUsart2Handle,APP_BufferEventHandlerUsart2,(uintptr_t)2);

	    appData.state = APP_STATE_TRANSFER_DATA_BTWN_USART1_USART2;
    
	}break;
			
        case APP_STATE_TRANSFER_DATA_BTWN_USART1_USART2:
        {

                /* Start the Application data transfer between UART1 and UART2*/
                APP_USART1_Task();
                APP_USART2_Task();


        }break;

        case APP_STATE_VERIFY_LOOPBACK_DATA:
        {
            //glow the LED.

            for(count = 0; count < MAX_NUM_OF_BYTES; count ++)
            {
                    if(appData.drvUsart1RxBuffer[count] == appData.drvUsart1TxBuffer[count])
                    {
                            count+=1;
                    }
                    else
                    {
                            appData.state = APP_STATE_ERROR;
                            break;
                    }
            }
            /* Check the count reached number of data in first page*/
            if(count != MAX_NUM_OF_BYTES)
            {
                    /* Test failed*/
                    BSP_LEDOn(BSP_LED_1);

                    appData.state = APP_STATE_ERROR;
            }
            else
            {
                    /* The appliction comes here when the demo has completed, Test Passed
                    Provide LED indication */

                    BSP_LEDOn(BSP_LED_3);
            }

    /* Move the app state */
            appData.state = APP_STATE_IDLE;

        }break;
                
	case APP_STATE_IDLE:
        {
            BSP_LEDOn(BSP_LED_2);
            Nop();
        }break;
		
        case APP_STATE_ERROR:
        {
                //Add the condition for Test failed
                BSP_LEDOn(BSP_LED_1);

        }break;

        default:
                break;

    }
}

/******************************************************************************
  Function:
    void APP_USART1_Tasks ( void )

 Precondition: APP_Tasks() should have been called.

  Remarks:
    This task state machine registers the transmit and receive functionality
    of the USART1 to the USART driver.
 */

void APP_USART1_Task( void )
{
    uint8_t num_of_bytes=0;

    switch(appData.usart1State)
    {

        case APP_STATE_USART1_RX:
        {
            /*USART1 RX*/

            /* Add application Rx buffer to Usart1Driver Rx buffer and wait*/
            DRV_USART_BufferAddRead(appData.drvUsart1Handle,&(appData.drvUsart1RxBufHandle),
            (uint8_t*)&appData.drvUsart1RxBuffer[0],MAX_NUM_OF_BYTES);

            appData.usart1State = APP_STATE_USART1_TX;

        }break;

        case APP_STATE_USART1_TX:
        {

            /*USART1 TX*/

            /* Populate the application Tx buffer for Tx over USART1*/
            for(num_of_bytes = 0; num_of_bytes <= MAX_NUM_OF_BYTES; num_of_bytes++)
            {
                appData.drvUsart1TxBuffer[num_of_bytes] = num_of_bytes;
            }


            /* Add application transmit buffer to Usart1 Tx buffer and transmit*/
            DRV_USART_BufferAddWrite(appData.drvUsart1Handle,&(appData.drvUsart1TxBufHandle),
            (uint8_t*)&appData.drvUsart1TxBuffer[0],MAX_NUM_OF_BYTES);

            appData.usart1State = APP_STATE_USART1_WAIT_FOR_TX_COMPLETION;

        }break;

        case APP_STATE_USART1_WAIT_FOR_TX_COMPLETION:
        {

          Nop();

        }break;

        case APP_STATE_USART1_WAIT_TO_RX_BACK_DATA:
        {

            /* Application state to wait for Rx back Data */
            if(gDataRxedAtUsart1 == true)
            {
                appData.usart1State = APP_STATE_USART1_IDLE;
                gDataRxedAtUsart1 = false;

            }

        }break;

        case APP_STATE_USART1_IDLE:
        {

            /* Wait till State machine is Resetted by external event like Button press.*/

            Nop();

            break;
        }        

        default:
        break;
    }
}

/******************************************************************************
  Function:
    void APP_USART2_Tasks ( void )

 Precondition: APP_Tasks() should have been called.

  Remarks:
    This task state machine registers the transmit and receive functionality
    of the USART2 to the USART driver.
 */

void APP_USART2_Task( void )
{
    switch(appData.usart2State)
    {
        case APP_STATE_USART2_RX:
        {
            /*USART2 RX*/
            /* Add application Rx buffer to Usart1Driver Rx buffer and wait*/
            DRV_USART_BufferAddRead(appData.drvUsart2Handle,&(appData.drvUsart2RxBufHandle),
            (uint8_t*)&appData.drvUsart2RxBuffer[0],MAX_NUM_OF_BYTES);

            appData.usart2State = APP_STATE_USART2_TX;

        }break;

        case APP_STATE_USART2_TX:
        {
            /*USART2 TX*/
            if(gDataRxedAtUsart2 == true) /* If data is received, then only trannsmit it back.*/
            {
                /* Add the received data on USART2 to the write buffer and transmit back to USART1*/
                DRV_USART_BufferAddWrite(appData.drvUsart2Handle,&(appData.drvUsart2TxBufHandle),
                (uint8_t *)&appData.drvUsart2RxBuffer[0], MAX_NUM_OF_BYTES);

                gDataRxedAtUsart2=false;
                appData.usart2State = APP_STATE_USART2_WAIT_FOR_TX_COMPLETION;

             }
        }break;

        case APP_STATE_USART2_WAIT_FOR_TX_COMPLETION:
        {
            Nop();
        }break;

        case APP_STATE_USART2_IDLE:
        {
            /* Wait till State machine is Resetted by external event like Button press.*/
            Nop();
            break;
        }

        default:
        break;
    }
	
}

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************
/******************************************************************************
  Function:
    void APP_BufferEventHandlerUsart1 ( DRV_USART_BUFFER_EVENT,
                DRV_USART_BUFFER_HANDLE,  uintptr_t );

   Remarks:
    This routine is callback function for the USART1 buffer events. Driver
    uses the application Tx or Rx buffer reference along with the client handle
    to notify the buffer events to the application. This is USART1 buffer
    event handler. This call back function to be registerd with the driver
    using DRV_USART_BufferAddRead(); or DRV_USART_BufferAddWrite();
 */

void APP_BufferEventHandlerUsart1(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context )
{
    switch(buffEvent)
    {	
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {

            if((context == 1) &&(appData.usart1State == APP_STATE_USART1_WAIT_FOR_TX_COMPLETION))
            {

                    appData.usart1State = APP_STATE_USART1_WAIT_TO_RX_BACK_DATA;

            }
            else if((context == 1) &&(appData.usart1State == APP_STATE_USART1_WAIT_TO_RX_BACK_DATA))
            {

                    gDataRxedAtUsart1 = true;
                    appData.state = APP_STATE_VERIFY_LOOPBACK_DATA;

            }
			
        }
        break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
            break;

       default:
            break;
    }
}

/******************************************************************************
  Function:
    void APP_BufferEventHandlerUsart2 ( DRV_USART_BUFFER_EVENT,
                DRV_USART_BUFFER_HANDLE,  uintptr_t );

   Remarks:
    This routine is callback function for the USART2 buffer events. Driver
    uses the application Tx or Rx buffer reference along with the client handle
    to notify the buffer events to the application. This is USART1 buffer
    event handler. This call back function to be registerd with the driver
    using DRV_USART_BufferAddRead(); or DRV_USART_BufferAddWrite();
 */

void APP_BufferEventHandlerUsart2(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context )
{
    switch(buffEvent)
    {	
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
			
            if((context == 2) &&(appData.usart2State == APP_STATE_USART2_TX))
            {
                    gDataRxedAtUsart2 = true;
            }
            else if((context == 2) &&(appData.usart2State == APP_STATE_USART2_WAIT_FOR_TX_COMPLETION))
            {
                    appData.usart2State = APP_STATE_USART2_IDLE;
            }
   
        }
        break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
            break;

       default:
            break;
    }
}

/*******************************************************************************
 End of File
 */

