/*******************************************************************************
  MPLAB Harmony SPI Looback Example

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    MPLAB Harmony SPI loopback application logic

  Description:
    This file contains the MPLAB Harmony SPI loopback application logic.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#include "system/system.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Macros
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
*/
APP_DATA appData;

/* SPI2 Driver RX buffer  */
SPI_DATA_TYPE __attribute__ ((coherent)) drvSPI2RXbuffer[MAX_NUM_OF_BYTES_IN_BUF] = {0};

/* SPI1 Driver TX buffer  */
SPI_DATA_TYPE __attribute__ ((coherent)) drvSPI1TXbuffer[MAX_NUM_OF_BYTES_IN_BUF] = {0};

/* Driver's Buffer context

  Summary:
    Holds drivers buffer reference for the Buffer event handler call back.

  Description:
    This structure holds the driver's buffer reference.

  Remarks:
    None
*/
context gBufferContext;


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************




// *****************************************************************************
// *****************************************************************************
// Section: File Scope Definitions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
*/

void APP_Initialize ( void )
{
    /* APP state task Init */
    appData.state = APP_STATE_INIT;
    appData.masterState = APP_STATE_SPI1_MASTER_WR;
    appData.slaveState = APP_STATE_SPI2_SLAVE_RD;

    appData.drvSPI1Handle= DRV_HANDLE_INVALID;
    appData.drvSPI2Handle= DRV_HANDLE_INVALID;
}

/*******************************************************************************
  Function:
    void APP_Tasks( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks( void )
{
uint32_t count=0;
    
    switch(appData.state)
    {
        case APP_STATE_INIT:
        {
            /* Open the SPI1 Driver */
            if(appData.drvSPI1Handle == DRV_HANDLE_INVALID)
            {
                appData.drvSPI1Handle = DRV_SPI_Open( DRV_SPI_INDEX_0,DRV_IO_INTENT_READWRITE );
            }
            /* Open the SPI Driver */
            if(appData.drvSPI2Handle == DRV_HANDLE_INVALID)
            {
                appData.drvSPI2Handle = DRV_SPI_Open( DRV_SPI_INDEX_1, DRV_IO_INTENT_READWRITE );
            }
            if((appData.drvSPI1Handle != DRV_HANDLE_INVALID) && (appData.drvSPI2Handle != DRV_HANDLE_INVALID ))
            {
                appData.state = APP_STATE_CHECK_DRVR_STATE;
            }
        }break;

        case APP_STATE_CHECK_DRVR_STATE:
        {
            /* Check the SPI1 driver handler */
            if (appData.drvSPI1Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to initialization */
                appData.state   = APP_STATE_INIT;
                return;
            }
            /* Check the SPI2 driver handler */
            if (appData.drvSPI2Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to initialization */
                appData.state   = APP_STATE_INIT;
                return;
            }
            /* Driver instances are now "open". Start transmitting data between Master and Slave*/
            appData.state = APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE;
        }break;

        /* Start the Application data transfer between SP1 (Master) and SPI2 (Slave)*/
        case APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE:
        {
            /* SPI Driver Client 1. Run the Slave Task*/
            APP_SPI_SLAVE_Task(); 	    	
	    /*SPI Driver Client 2.  Run the Master Task*/
            APP_SPI_MASTER_Task();
           
			
		}break;

        case APP_STATE_VERIFY_DATA:
        {
            /* Glow the Result LED */
            for(count = 0; count < MAX_NUM_OF_BYTES; count++)
            {
                if(drvSPI1TXbuffer[count] != drvSPI2RXbuffer[count])
                {

                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }
            /* Check the count reached number of data in first page*/
            if(count != MAX_NUM_OF_BYTES)
            {
                /* Test failed*/
                appData.state = APP_STATE_ERROR;
            }
            else
            {
                /* Test Passed. The application comes here when the demo is successful */
                appData.state = APP_STATE_SUCCESS;
            }

            
        }break;

        case APP_STATE_SUCCESS:
        {
			BSP_LEDOn(BSP_LED_3);	
            Nop();
        }break;
        
        case APP_STATE_ERROR:
        {
			BSP_LEDOn(BSP_LED_2);	
            Nop();
        }break;
        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_SPI_MASTER_Task( void )

  Remarks:
    See prototype in app.h.
 */


void APP_SPI_MASTER_Task(void)
{

uint32_t num_of_bytes=0;

    switch(appData.masterState)
    {
         /*SPI1 TX*/
        case APP_STATE_SPI1_MASTER_WR:
        {
	     /* Populate the application Tx buffer for MASTER(SPI1) to TX*/
            for(num_of_bytes = 0; num_of_bytes <= (MAX_NUM_OF_BYTES- 1); num_of_bytes++)
            {
                drvSPI1TXbuffer[num_of_bytes] = num_of_bytes;
            }

            /* Populate the buffer context with Tx Buffer ptr for the Buffer Event Handler */
            gBufferContext.SPI1TxBufferRef = &drvSPI1TXbuffer[0];
            appData.masterState = APP_STATE_SPI1_WAIT_FOR_WR_COMPLETION;
			
            /* Add Master transmit buffer to SPI1 Tx buffer and transmit*/
            appData.drvSPI1TxBufHandle = DRV_SPI_BufferAddWrite(appData.drvSPI1Handle,(SPI_DATA_TYPE *)&drvSPI1TXbuffer[0],
            MAX_NUM_OF_BYTES ,APP_BufferEventHandlerSPI1,gBufferContext.SPI1TxBufferRef);
		
	}break;

        case APP_STATE_SPI1_WAIT_FOR_WR_COMPLETION:
        {
            /* Wait till the SPI1 write to slave is over */
            Nop();
        }break;

        case APP_STATE_SPI1_MASTER_IDLE:
        {
            /* Demo is over*/
            Nop();
        }break;

        default:
        break;
    }
}				


/*******************************************************************************
  Function:
    void APP_SPI_SLAVE_Task( void )

  Remarks:
    See prototype in app.h.
 */

void APP_SPI_SLAVE_Task(void)
{

    switch(appData.slaveState)
    {
        /*SPI2 RX*/
        case APP_STATE_SPI2_SLAVE_RD:
        {
            /* Populate the buffer context with Rx Buffer ptr for the Buffer Event Handler */
            gBufferContext.SPI2RxBufferRef = &drvSPI2RXbuffer[0];
		
            appData.slaveState = APP_STATE_SPI2_WAIT_FOR_RD_COMPLETION;
			
            /* Slave to read/receive the data sent from Master (SPI1) */
            appData.drvSPI2RxBufHandle = DRV_SPI_BufferAddRead( appData.drvSPI2Handle,
            (SPI_DATA_TYPE *)&drvSPI2RXbuffer[0], MAX_NUM_OF_BYTES,APP_BufferEventHandlerSPI2,gBufferContext.SPI2RxBufferRef);
 
        }break;

        case APP_STATE_SPI2_WAIT_FOR_RD_COMPLETION:
        {
            /* Wait for the Slave to receive the expected data*/
            Nop();
        }break;

        case APP_STATE_SPI2_SLAVE_IDLE:
        {
            /* Demo is over*/
            Nop();
        }break;

        default:
            break;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

void APP_BufferEventHandlerSPI1(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{
    switch(buffEvent)
    {	
        /* Buffer event is completed successfully. Data is transmitted */
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {
            if((context == gBufferContext.SPI1TxBufferRef) &&(appData.masterState == APP_STATE_SPI1_WAIT_FOR_WR_COMPLETION))
            {
                appData.masterState = APP_STATE_SPI1_MASTER_IDLE;
            }
	}break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {    /* Test failed*/
                BSP_LEDOn(BSP_LED_2);
        }break;

        default:
            break;
    }
}


void APP_BufferEventHandlerSPI2(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{

    switch(buffEvent)
    {	
        /* Buffer event is completed successfully. Data is received */
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {			
            if((context == gBufferContext.SPI2RxBufferRef) &&(appData.slaveState == APP_STATE_SPI2_WAIT_FOR_RD_COMPLETION))
            {
                appData.slaveState = APP_STATE_SPI2_SLAVE_IDLE;
		appData.state = APP_STATE_VERIFY_DATA;
            }
        }
        break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {
            /* Test failed*/
               BSP_LEDOn(BSP_LED_2);
        }break;

       default:
            break;
    }
}



/*******************************************************************************
 End of File
 */
