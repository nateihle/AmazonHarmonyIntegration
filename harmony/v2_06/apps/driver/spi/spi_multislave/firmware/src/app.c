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

/* Master TX buffer 1 */
SPI_DATA_TYPE __attribute__ ((coherent)) drvMasterTXbuffer1[MAX_NUM_OF_BYTES_IN_BUF];

/* Master TX buffer 2 */
SPI_DATA_TYPE __attribute__ ((coherent)) drvMasterTXbuffer2[MAX_NUM_OF_BYTES_IN_BUF];

/* Slave 1 RX buffer  */
SPI_DATA_TYPE __attribute__ ((coherent)) drvSlave1RXbuffer[MAX_NUM_OF_BYTES_IN_BUF];

/* Slave 2 RX buffer  */
SPI_DATA_TYPE __attribute__ ((coherent)) drvSlave2RXbuffer[MAX_NUM_OF_BYTES_IN_BUF];

context gBufferContext;

/* Client configure data */
DRV_SPI_CLIENT_DATA cfgObj={
 .baudRate = 0,
 .operationStarting = APP_BufferEventHandlerMaster,
 .operationEnded = NULL,
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

void APP_BufferEventHandlerMaster(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{
    switch(buffEvent)
    {
        
        case DRV_SPI_BUFFER_EVENT_PROCESSING:
        {
            /* If First transfer is requested, then activate Salve 1 SS line */
            if((context == gBufferContext.MasterTxBufferRef1) &&(appData.masterState == APP_STATE_MASTER_WAIT_FOR_SLAVE_1_WR_COMPLETION))
            {
                APP_SPI_CS_SELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
            }
            /* Else if Second transfer is requested, then activate Salve 2 SS line */
            else if((context == gBufferContext.MasterTxBufferRef2) &&(appData.masterState == APP_STATE_MASTER_WAIT_FOR_SLAVE_2_WR_COMPLETION))
            {
                APP_SPI_CS_SELECT(SPI_SLAVE_2_CS_PORT_ID,SPI_SLAVE_2_CS_PORT_PIN);
            }
	    }break;

        /* Buffer event is completed successfully. Data is transmitted */
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {
            if((context == gBufferContext.MasterTxBufferRef1) &&(appData.masterState == APP_STATE_MASTER_WAIT_FOR_SLAVE_1_WR_COMPLETION))
            {
                /* Deactivate Slave 1*/
                APP_SPI_CS_DESELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
            }
            else if((context == gBufferContext.MasterTxBufferRef2) &&(appData.masterState == APP_STATE_MASTER_WAIT_FOR_SLAVE_2_WR_COMPLETION))
            {
                /* Deactivate Slave 2*/
                APP_SPI_CS_DESELECT(SPI_SLAVE_2_CS_PORT_ID,SPI_SLAVE_2_CS_PORT_PIN);
                appData.masterState = APP_STATE_MASTER_IDLE;
            }
	    }break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {    
            /* Transfer failed*/
            BSP_LEDOn(BSP_LED_1);
        }break;

        default:
            break;
    }
}


void APP_BufferEventHandlerSlave1(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{

    switch(buffEvent)
    {	
        /* Buffer event is completed successfully. Data is received */
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {			
            if((context == gBufferContext.Slave1RxBufferRef) &&(appData.slaveState == APP_STATE_SLAVE_1_WAIT_FOR_RD_COMPLETION))
            {
                appData.state = APP_STATE_START_TRANSFER_2;
            }
        }
        break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {
            /* Test failed*/
               BSP_LEDOn(BSP_LED_1);
        }break;

       default:
            break;
    }
}

void APP_BufferEventHandlerSlave2(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{

    switch(buffEvent)
    {	
        /* Buffer event is completed successfully. Data is received */
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {			
            if((context == gBufferContext.Slave2RxBufferRef) &&(appData.slaveState == APP_STATE_SLAVE_2_WAIT_FOR_RD_COMPLETION))
            {
                appData.slaveState = APP_STATE_SLAVES_IDLE;
		        appData.state = APP_STATE_VERIFY_SLAVES_DATA;
            }
        }
        break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {
            /* Test failed*/
               BSP_LEDOn(BSP_LED_1);
        }break;

       default:
            break;
    }
}



// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_SPI_MASTER_Task( void )

  Remarks:
    See prototype in app.h.
 */


void APP_SPI_MASTER_Task(void)
{
    appData.masterCfgData = &cfgObj;

uint8_t num_of_bytes=0;

    switch(appData.masterState)
    {
         /*Master TX to Slave 1*/
        case APP_STATE_MASTER_WR_SLAVE_1:
        {
            DRV_SPI_ClientConfigure(appData.drvMasterHandle, appData.masterCfgData);
	     /* Populate the application Tx buffer for MASTER to TX on Slave 1.
            Buffer holds values from 0 to 63 */
            for(num_of_bytes = 0; num_of_bytes <= MAX_NUM_OF_BYTES; num_of_bytes++)
            {
                drvMasterTXbuffer1[num_of_bytes] = num_of_bytes;
            }

            /* Populate the buffer context with Tx Buffer ptr for the Buffer Event Handler */
            gBufferContext.MasterTxBufferRef1 = &drvMasterTXbuffer1[0];
            appData.masterState = APP_STATE_MASTER_WAIT_FOR_SLAVE_1_WR_COMPLETION;
			
            /* Add Master transmit buffer to Master Tx buffer and transmit*/
            appData.drvMasterTxBufHandle = DRV_SPI_BufferAddWrite(appData.drvMasterHandle,(SPI_DATA_TYPE *)&drvMasterTXbuffer1[0],
            MAX_NUM_OF_BYTES ,APP_BufferEventHandlerMaster,gBufferContext.MasterTxBufferRef1);
		
	    }break;

        case APP_STATE_MASTER_WAIT_FOR_SLAVE_1_WR_COMPLETION:
        {
            /* Wait till the Master write to slave 1 is over */
            Nop();
        }break;
        
         /*Master TX to Slave 2*/
        case APP_STATE_MASTER_WR_SLAVE_2:
        {
	        /* Populate the application Tx buffer for MASTER to TX on Slave 2.
               Buffer holds values from 64 to 1*/
            for(num_of_bytes = 0; num_of_bytes <= MAX_NUM_OF_BYTES; num_of_bytes++)
            {
                drvMasterTXbuffer2[num_of_bytes] = (MAX_NUM_OF_BYTES-num_of_bytes);
            }

            /* Populate the buffer context with Tx Buffer ptr for the Buffer Event Handler */
            gBufferContext.MasterTxBufferRef2 = &drvMasterTXbuffer2[0];
            appData.masterState = APP_STATE_MASTER_WAIT_FOR_SLAVE_2_WR_COMPLETION;
			
            /* Add Master transmit buffer to Master Tx buffer and transmit*/
            appData.drvMasterTxBufHandle = DRV_SPI_BufferAddWrite(appData.drvMasterHandle,(SPI_DATA_TYPE *)&drvMasterTXbuffer2[0],
            MAX_NUM_OF_BYTES ,APP_BufferEventHandlerMaster,gBufferContext.MasterTxBufferRef2);
		
	    }break;

        case APP_STATE_MASTER_WAIT_FOR_SLAVE_2_WR_COMPLETION:
        {
            /* Wait till the Master write to slave 2 is over */
            Nop();
        }break;

        case APP_STATE_MASTER_IDLE:
        {
            /* Master work is done*/
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
        /*Slave 1 RX*/
        case APP_STATE_SLAVE_1_RD:
        {
            /* Populate the buffer context with Rx Buffer ptr for the Buffer Event Handler */
            gBufferContext.Slave1RxBufferRef = &drvSlave1RXbuffer[0];
		
            appData.slaveState = APP_STATE_SLAVE_1_WAIT_FOR_RD_COMPLETION;
			
            /* Slave 1 reads the data sent from Master (SPI1) */
            appData.drvSlave1RxBufHandle = DRV_SPI_BufferAddRead( appData.drvSlave1Handle,
            (SPI_DATA_TYPE *)&drvSlave1RXbuffer[0], MAX_NUM_OF_BYTES,APP_BufferEventHandlerSlave1,gBufferContext.Slave1RxBufferRef);
 
        }break;

        case APP_STATE_SLAVE_1_WAIT_FOR_RD_COMPLETION:
        {
            /* Wait for the Slave 1 to receive the expected data*/
            Nop();
        }break;
        
        /*Slave 2 RX*/
        case APP_STATE_SLAVE_2_RD:
        {
            /* Populate the buffer context with Rx Buffer ptr for the Buffer Event Handler */
            gBufferContext.Slave2RxBufferRef = &drvSlave2RXbuffer[0];
		
            appData.slaveState = APP_STATE_SLAVE_2_WAIT_FOR_RD_COMPLETION;
			
            /* Slave 2 reads the data sent from Master (SPI1) */
            appData.drvSlave2RxBufHandle = DRV_SPI_BufferAddRead( appData.drvSlave2Handle,
            (SPI_DATA_TYPE *)&drvSlave2RXbuffer[0], MAX_NUM_OF_BYTES,APP_BufferEventHandlerSlave2,gBufferContext.Slave2RxBufferRef);
 
        }break;

        case APP_STATE_SLAVE_2_WAIT_FOR_RD_COMPLETION:
        {
            /* Wait for the Slave 2 to receive the expected data*/
            Nop();
        }break;

        case APP_STATE_SLAVES_IDLE:
        {
            /* Slaves work is done*/
            Nop();
        }break;

        default:
            break;
    }
}

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
    /* Tasks initial states */
    appData.state = APP_STATE_INIT;
    appData.masterState = APP_STATE_MASTER_IDLE;
    appData.slaveState = APP_STATE_SLAVES_IDLE;

    /* Initialize the driver handles */
    appData.drvMasterHandle= DRV_HANDLE_INVALID;
    appData.drvSlave1Handle= DRV_HANDLE_INVALID;
    appData.drvSlave2Handle= DRV_HANDLE_INVALID;
    
    /* Keep both SS lines inactive */
    APP_SPI_CS_DESELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
    APP_SPI_CS_DESELECT(SPI_SLAVE_2_CS_PORT_ID,SPI_SLAVE_2_CS_PORT_PIN);
    
    /* Initiate the LED states */
    BSP_LEDOn(BSP_LED_1);
    BSP_LEDOff(BSP_LED_2);
    BSP_LEDOff(BSP_LED_3);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
uint8_t count1,count2=0;
    
    switch(appData.state)
    {
        case APP_STATE_INIT:
        {
            if((appData.drvMasterHandle != DRV_HANDLE_INVALID) && (appData.drvSlave1Handle != DRV_HANDLE_INVALID )&& (appData.drvSlave2Handle != DRV_HANDLE_INVALID ))
            {
                /* All SPI instances are properly opened , so start first transfer*/
                appData.state = APP_STATE_START_TRANSFER_1;
            }
            /* Open the SPI1 Driver */
            else if(appData.drvMasterHandle == DRV_HANDLE_INVALID)
            {
                appData.drvMasterHandle = DRV_SPI_Open( DRV_SPI_INDEX_0,DRV_IO_INTENT_WRITE );
            }
            /* Open the SPI2 Driver */
            else if(appData.drvSlave1Handle == DRV_HANDLE_INVALID)
            {
                appData.drvSlave1Handle = DRV_SPI_Open( DRV_SPI_INDEX_1, DRV_IO_INTENT_READ );
            }
            /* Open the SPI3 Driver */
            else if(appData.drvSlave2Handle == DRV_HANDLE_INVALID)
            {
                appData.drvSlave2Handle = DRV_SPI_Open( DRV_SPI_INDEX_2, DRV_IO_INTENT_READ );
            }
        }break;

        case APP_STATE_START_TRANSFER_1:
        {
            /* Initiate data transfer from SPI1 to SPI2 */
            appData.masterState = APP_STATE_MASTER_WR_SLAVE_1;
            appData.slaveState = APP_STATE_SLAVE_1_RD;
            appData.state = APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE;
           
		}break;

        case APP_STATE_START_TRANSFER_2:
        {
            /* Initiate data transfer from SPI1 to SPI3 */
            appData.masterState = APP_STATE_MASTER_WR_SLAVE_2;
            appData.slaveState = APP_STATE_SLAVE_2_RD;
            appData.state = APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE;
        }break;

        case APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE:
        {
            /* Run the Slave Task*/
            APP_SPI_SLAVE_Task(); 	    	
	        /* Run the Master Task*/
            APP_SPI_MASTER_Task();
           
		}break;
        
        case APP_STATE_VERIFY_SLAVES_DATA:
        {
            /* Verify the first transfer */
            for(count1 = 0; count1 < MAX_NUM_OF_BYTES; count1++)
            {
                if(drvMasterTXbuffer1[count1] != drvSlave1RXbuffer[count1])
                {
                    break;
                }
            }
            if(count1 == MAX_NUM_OF_BYTES)
            {
                /* Transfer 1 is successful*/
                BSP_LEDOn(BSP_LED_2);
            }

            /* Verify the second transfer */
            for(count2 = 0; count2 < MAX_NUM_OF_BYTES; count2++)
            {
                if(drvMasterTXbuffer2[count2] != drvSlave2RXbuffer[count2])
                {
                    break;
                }
            }            
            if(count2 == MAX_NUM_OF_BYTES)
            {
                /* Transfer 2 is successful*/
                BSP_LEDOn(BSP_LED_3);
            }
            
            /* Error check */    
            if(count1 != MAX_NUM_OF_BYTES || count2 != MAX_NUM_OF_BYTES)
            {
                /* Transfer 1 or Transfer 2 or both received data doesn't match*/
                BSP_LEDOn(BSP_LED_1);
            }
            else
            {
                BSP_LEDOff(BSP_LED_1);
            }

            /* Put Application in to the Idle state */
            appData.state = APP_STATE_IDLE;
        }break;

        case APP_STATE_IDLE:
        {	
            Nop();
        }break;

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
