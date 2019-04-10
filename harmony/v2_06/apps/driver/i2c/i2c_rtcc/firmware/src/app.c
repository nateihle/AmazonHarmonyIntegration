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

/*******************************************************************************
                Application Information and Set-up

 * This demo illustrates the an I2C module that acts as a Master talking to 
 * to an external slave device. 
 * The external slave device used is a Real Time Clock Calendar (RTCC) chip.
 * The data from the Master is written into an SRAM location in the RTCC chip.
 * 
 * The I2C Master writes and reads data from I2C Slave and RTCC chip. The RTCC
 * chip sends back the data that was written to it by the I2C Master. The data
 * written by the I2C Master is stored in the SRAM of RTCC.
 *  
 * This demo uses uses a Buffer model of I2C transfer. The APIs are matched to
 * the buffer static implementation of I2C driver. The operation to write, read
 * and read-after-write is demonstrated. The read-after-write allows 
 * random access of a register in a slave device, in this case RTCC.
 * 
 * Refer to Applications Help > Driver Demonstrations > I2C Driver Demonstrations 
                              > Demonstrations > i2c_rtcc_loopback
 *  for more information on this demo
  *******************************************************************************/


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

static uint32_t ReadCoreTimer(void);

static uint32_t ReadCoreTimer()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return(timer);
}


#define GetSystemClock() (SYS_CLK_FREQ)
#define us_SCALE   (GetSystemClock()/2000000)
#define ms_SCALE   (GetSystemClock()/2000)

/* Address of slave devices */
#define RTCC_SLAVE_ADDRESS              0xDE
#define MEM_LOC_PAGE_1_START            0x33

#define PIC32_SLAVE_ADDRESS             0x60

#define NUMBER_OF_UNKNOWN_BYTES_TO_SLAVE    255

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

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t operationStatus;

uint8_t deviceAddressSlave1;

uint8_t deviceAddressSlave2;

uint8_t deviceAddressPIC32 = PIC32_SLAVE_ADDRESS;

uint8_t numOfBytes;

uint8_t indexRegByteSize;

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_1[] = "\x45""1M";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_2[] = "MSTER_2_SLAVE";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_3[] = "\x25""3RTCCSLAVE";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_4[] = "\x25";

/* I2C driver RX Buffer */
uint8_t         RXbuffer_4[20];

/* I2C driver RX Buffer */
uint8_t         RXbuffer_5[20];

/* I2C Driver TX buffer  */
uint8_t         SlaveTxbuffer[] = "FROMSLAVE";

/* I2C Driver TX buffer  */
uint8_t         SlaveRxbuffer[20];

DRV_I2C_BUFFER_EVENT appBufferEvent[MAX_NUMBER_OF_BUFFERS];

uint32_t  appNumberOfBytesTransferred[MAX_NUMBER_OF_BUFFERS];


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* callback to Master indicating a change in Buffer Status of transmit 
    OR receive buffer */

void I2C_MasterBufferStatusCallback (   DRV_I2C_BUFFER_EVENT event,
                                        DRV_I2C_BUFFER_HANDLE bufferHandle,
                                        uintptr_t context);

/* callback to slave after a read or write */

void I2C_SlaveCbAfterByteReadorWrite (  DRV_I2C_BUFFER_EVENT event,
                                        DRV_I2C_BUFFER_HANDLE bufferHandle,
                                        uintptr_t context);



// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

DRV_I2C_BUFFER_EVENT APP_Check_Transfer_Status(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle);

uint32_t    APP_Number_Of_Bytes_Transferred(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle);

void DelayMs(unsigned long int msDelay );

void DelayMs(unsigned long int msDelay );

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/* State Machine for Master Write */
bool APP_Write_Tasks(void);

/* State Machine for Master Read */
bool APP_Read_Tasks(void);


typedef enum{
    
        TxRx_OPEN = 0,
        TxRx_TO_EXTERNAL_SLAVE_1,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1,
        TxRx_TO_EXTERNAL_SLAVE_2,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2,
        TxRx_TO_EXTERNAL_SLAVE_3,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3,
        TxRx_TO_EXTERNAL_SLAVE_4,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4,
        TxRx_TO_EXTERNAL_SLAVE_5,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5,
        TxRx_STATUS_CHECK,
        TxRx_COMPLETED

}APP_EEPROM_WR_STATES;

static APP_EEPROM_WR_STATES appWriteState = TxRx_OPEN;

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
              appData.state = APP_WRITE_DATA;

            break;
        }
        case APP_WRITE_DATA:
        {
            
            /* completes write task to the slave */
            if(APP_Write_Tasks())
            {
                appData.state = APP_STATE_IDLE;
            }
            break;
        }
        case APP_STATE_IDLE:
        {
            Nop();
            break;
        }
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

    DRV_I2C_BUFFER_EVENT Transaction;
    unsigned int ErrorDetection = 0;

bool APP_Write_Tasks(void)
{
    switch (appWriteState)
    {
        case TxRx_OPEN:
        {
            /* Open the I2C Driver for Master */
            appData.drvI2CHandle_Master = DRV_I2C_Open( DRV_I2C_INDEX_0,
                                                     DRV_IO_INTENT_WRITE );

            if(appData.drvI2CHandle_Master != DRV_HANDLE_INVALID)
            {
                ErrorDetection = 0;
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_1;
//              DRV_I2C_BufferEventHandlerSet(appData.drvI2CHandle_Master, I2C_MasterBufferStatusCallback, operationStatus );
            }
            else
            {
                appData.state = APP_STATE_ERROR;
            }

            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_1:
        {    
            /* Number of bytes to transfer */
            numOfBytes = 3;

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 1 to RTCC */

            if ( (appData.appI2CWriteBufferHandle[0] == DRV_I2C_BUFFER_HANDLE_INVALID) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.appI2CWriteBufferHandle[0] = DRV_I2C_Transmit (appData.drvI2CHandle_Master,
                                                                            deviceAddressSlave1,
                                                                            &TXbuffer_1[0], numOfBytes, NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1:
        {
            Transaction = APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[0]);
            if  ( Transaction == DRV_I2C_BUFFER_EVENT_COMPLETE)
            {
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_2;
            }
            else if  (Transaction == DRV_I2C_BUFFER_EVENT_ERROR)
            {
                ErrorDetection = 1;
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_2;
            }
            
            break;
        }        
        case TxRx_TO_EXTERNAL_SLAVE_2:
        {
                                  
            /* Write Transaction - 2 to I2C2-Slave */
            
            deviceAddressSlave2 = RTCC_SLAVE_ADDRESS;
            
            if ( (appData.appI2CWriteBufferHandle[1] == DRV_I2C_BUFFER_HANDLE_INVALID) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_COMPLETE) || 
                         (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.appI2CWriteBufferHandle[1] = DRV_I2C_Transmit (appData.drvI2CHandle_Master,
                                                                            deviceAddressSlave2,
                                                                            &TXbuffer_2[0], (sizeof(TXbuffer_2)-1), NULL);
            } 
                        
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2;
            break;
        }
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2:
        {
            Transaction = APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[1]);
            if  ( Transaction == DRV_I2C_BUFFER_EVENT_COMPLETE)
            {
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_3;
            }
            else if  (Transaction == DRV_I2C_BUFFER_EVENT_ERROR)
            {
                ErrorDetection = 1;
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_3;
            }
            break;            
        }
        case TxRx_TO_EXTERNAL_SLAVE_3:
        {    

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 1 to RTCC */

            if ( (appData.appI2CWriteBufferHandle[2] == DRV_I2C_BUFFER_HANDLE_INVALID) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                         (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.appI2CWriteBufferHandle[2] = DRV_I2C_Transmit (appData.drvI2CHandle_Master,
                                                                            deviceAddressSlave1,
                                                                            &TXbuffer_3[0], (sizeof(TXbuffer_3)-1), NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3:
        {
            Transaction = APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[2]);
            if  ( Transaction == DRV_I2C_BUFFER_EVENT_COMPLETE)
            {
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_4;
            }
            else if  (Transaction == DRV_I2C_BUFFER_EVENT_ERROR)
            {
                ErrorDetection = 1;
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_4;
            }
            
            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_4:
        {    
            
            /* Number of bytes of index register address */
            indexRegByteSize = 1;
            
            /* Number of bytes to read- this is from previous transaction 
             * -2 takes account of register address which occupies byte 
             * position 0 and NULL at end of array */
            numOfBytes = (sizeof(TXbuffer_3)-2);

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 1 to RTCC */

            if ( (appData.appI2CWriteBufferHandle[3] == DRV_I2C_BUFFER_HANDLE_INVALID) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_COMPLETE) || 
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.appI2CWriteBufferHandle[3] = DRV_I2C_TransmitThenReceive (appData.drvI2CHandle_Master,
                                                                            deviceAddressSlave1,
                                                                            &TXbuffer_4[0], indexRegByteSize, 
                                                                            &RXbuffer_4[0], numOfBytes, NULL );
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4:
        {
            Transaction = APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[3]);
            if  ( Transaction == DRV_I2C_BUFFER_EVENT_COMPLETE)
            {
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_5;
            }
            else if  (Transaction == DRV_I2C_BUFFER_EVENT_ERROR)
            {
                ErrorDetection = 1;
                appWriteState =TxRx_TO_EXTERNAL_SLAVE_5;
            }
            
            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_5:
        {    
            /* Number of bytes to transfer */
            numOfBytes = ((sizeof(SlaveTxbuffer)) - 1);

            /* address of PIC32 slave */
            deviceAddressSlave2 = RTCC_SLAVE_ADDRESS;

            /* Read Transaction to PIC32 Slave */

            if ( (appData.appI2CWriteBufferHandle[4] == DRV_I2C_BUFFER_HANDLE_INVALID) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master,  appData.appI2CWriteBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_COMPLETE) || 
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_ERROR)  )
            {
                appData.appI2CWriteBufferHandle[4] = DRV_I2C_Receive(appData.drvI2CHandle_Master,
                                                                            deviceAddressSlave2,
                                                                            &RXbuffer_5[0], numOfBytes, NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5:
        {
            Transaction = APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.appI2CWriteBufferHandle[4]);
            if  ( Transaction == DRV_I2C_BUFFER_EVENT_COMPLETE)
            {
                appWriteState = TxRx_STATUS_CHECK;
            }
            else if  (Transaction == DRV_I2C_BUFFER_EVENT_ERROR)
            {
                ErrorDetection = 1;
                appWriteState = TxRx_STATUS_CHECK;
            }
            
            break;
        }  
        case TxRx_STATUS_CHECK:
        {
            
// Only defined for dynamic driver.            
            #ifdef DRV_I2C_CLIENTS_NUMBER            
extern const DRV_I2C_INIT drvI2C0InitData;
            DRV_I2C_Close( appData.drvI2CHandle_Master );
            DRV_I2C_Deinitialize (sysObj.drvI2C0);
            #endif
            
            DelayMs(800);
            #ifdef DRV_I2C_CLIENTS_NUMBER            
            sysObj.drvI2C0 = DRV_I2C_Initialize(DRV_I2C_INDEX_0, (SYS_MODULE_INIT *)&drvI2C0InitData);
            #endif
            LED_SUCCESS_STAGE1Toggle();
            if(ErrorDetection == 1)
            {
                LED_SUCCESS_STAGE2Toggle();
            }
            
            /* to run the application only once,  
             * set next state to TxRx_COMPLETED */
//            appWriteState = TxRx_COMPLETED;
            
            /* to run the application in a continuous loop,  
             * set next state to TxRx_OPEN */
            #ifdef DRV_I2C_CLIENTS_NUMBER            
            appWriteState = TxRx_OPEN;
            #else
            appWriteState = TxRx_TO_EXTERNAL_SLAVE_1;
            #endif
            break;
        }
        case TxRx_COMPLETED:
        {
            return true;
            break;
        }
    }
    
    return false;
}

//****************************************************************************/
//  Function: APP_Check_Transfer_Status
//
//  Returns the status of Buffer operation from the driver; The application
//  can probe this function to see if the status of a particular I2C
//  transfer is in its execution
//****************************************************************************/
DRV_I2C_BUFFER_EVENT APP_Check_Transfer_Status(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle)
{
    return (DRV_I2C_TransferStatusGet (appData.drvI2CHandle_Master, drvBufferHandle));
}

//****************************************************************************/
//  Function: APP_Check_Transfer_Status
//
//  Returns the number of bytes that was transferred in a particular I2C   
//  transaction. The application can identify the transfer using the 
//  appropriate driver handle
//****************************************************************************/

uint32_t APP_Number_Of_Bytes_Transferred(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle)
{
    return (DRV_I2C_BytesTransferred (appData.drvI2CHandle_Master,drvBufferHandle));
}


//****************************************************************************/
//  Function: APP_SlaveDataforMaster
//
//  Callback function from DRV_I2C_Tasks when operating as a SLAVE. When address
//  match is received by the SLAVE, this callback is executed and buffer event
//  depends on the R/W bit. The R/W = 0, BufferAddRead is called implying
//  slave is going to read data send from Master. If R/W = 1, BufferAddWrite
//  implying Slave is going to send data to Master
//****************************************************************************/

void APP_SlaveDataforMaster(DRV_I2C_BUFFER_EVENT event, void * context)
{
    switch (event)
    {
        case DRV_I2C_BUFFER_SLAVE_READ_REQUESTED:
            deviceAddressPIC32 = PIC32_SLAVE_ADDRESS;
            
            appData.appI2CSlaveWriteHandle = DRV_I2C_Receive( appData.drvI2CHandle_Slave,
                    deviceAddressPIC32,
                    &SlaveRxbuffer[0], NUMBER_OF_UNKNOWN_BYTES_TO_SLAVE, NULL );

            break;
        case DRV_I2C_BUFFER_SLAVE_WRITE_REQUESTED:
            deviceAddressPIC32 = PIC32_SLAVE_ADDRESS;
            
            appData.appI2CSlaveReadHandle = DRV_I2C_Transmit ( appData.drvI2CHandle_Slave,
                    deviceAddressPIC32,
                    &SlaveTxbuffer, NUMBER_OF_UNKNOWN_BYTES_TO_SLAVE, NULL );
            break;
        default:
            break;
    }

}

/***********************************************************
 *   Millisecond Delay function using the Count register
 *   in coprocessor 0 in the MIPS core.
 *   When running 200 MHz, CoreTimer frequency is 100 MHz
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 10ns
 *   1 ms = N x CoreTimer_period;
 *   To count 1ms, N = 100000 counts of CoreTimer
 *   1 ms = 10 ns * 100000 = 10e6 ns = 1 ms
 *   When running 80 MHz, CoreTimer frequency is 40 MHz 
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 25ns
 *   To count 1ms, N = 40000 counts of CoreTimer
 *   1ms = 25 ns * 40000 = 10e6 ns = 1 ms
 *   ms_SCALE = (GetSystemClock()/2000) @ 200 MHz = 200e6/2e3 = 100e3 = 100000
 *   ms_SCLAE = (GetSystemClock()/2000) @ = 80e6/2e3 = 40e3 = 40000 
 */
 
void DelayMs(unsigned long int msDelay )
{
      register unsigned int startCntms = ReadCoreTimer();
      register unsigned int waitCntms = msDelay * ms_SCALE;
 
      while( ReadCoreTimer() - startCntms < waitCntms );
}

/***********************************************************
 *   Microsecond Delay function using the Count register
 *   in coprocessor 0 in the MIPS core.
 *   When running 200 MHz, CoreTimer frequency is 100 MHz
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 10ns
 *   1 us = N x CoreTimer_period;
 *   To count 1us, N = 100 counts of CoreTimer
 *   1 us = 10 ns * 100 = 1000 ns  = 1us
 *   When running 80 MHz, CoreTimer frequency is 40 MHz 
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 25ns
 *   To count 1us, N = 40 counts of CoreTimer
 *   1us = 25 ns * 40 = 1000 ns = 1 us
 *   us_SCALE = (GetSystemClock()/2000) @ 200 MHz = 200e6/2e6 = 100 
 *   us_SCLAE = (GetSystemClock()/2000) @ 80 MHz = 80e6/2e6 = 40 
 */
 
void DelayUs(unsigned long int usDelay )
{
      register unsigned int startCnt = ReadCoreTimer();
      register unsigned int waitCnt = usDelay * us_SCALE;
 
      while( ReadCoreTimer() - startCnt < waitCnt );
}
 

/*******************************************************************************
 End of File
 */
