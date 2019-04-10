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
 * another I2C module acting as a slave and also to an external slave device. 
 * The external slave device used is a Real Time Clock Calendar (RTCC) chip.
 * The data from the Master is written into an SRAM location in the RTCC chip.
 * 
 * The I2C Master writes and reads data from I2C Slave and RTCC chip. The RTCC
 * chip sends back the data that was written to it by the I2C Master. The data
 * written by the I2C Master is stored in the SRAM of RTCC.
 * In case of PIC32MZ-EC and PIC32MZ-EF, I2C-2 is the Master and I2C-1 
 * is the slave. For PIC32MX795 device, I2C-2 is the Master and I2C-1 is the 
 * Slave.
 *  
 * This demo uses uses a Buffer model of I2C transfer. The APIs are matched to
 * the dynamic implementation of I2C driver. The operation to write, read
 * and read-after-write is demonstrated. The read-after-write allows 
 * random access of a register in a slave device, in this case RTCC.
 * 
 * Refer to Applications > Examples > Peripheral Library > I2C Peripheral Examples
 * for more information on this demo 
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

DRV_I2C_BUFFER_EVENT i2cOpStatus;

uint8_t operationStatus;

uint8_t deviceAddressSlave1;

uint8_t deviceAddressSlave2;

uint8_t deviceAddressPIC32 = PIC32_SLAVE_ADDRESS;

uint8_t numOfBytes;

uint8_t indexRegByteSize;

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_1[] = "\x45""1M";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_2[] = "2Buffer";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_3[] = "\x25""3RTCCSLAVE";

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_4[] = "\x25";

/* I2C driver RX Buffer */
uint8_t         RXbuffer_4[20];

/* I2C driver RX Buffer */
uint8_t         RXbuffer_5[20];

/* I2C Driver TX buffer  */
uint8_t         TXbuffer_6[] = "\x25";

/* I2C driver RX Buffer */
uint8_t         RXbuffer_6[20];

/* I2C Driver TX buffer  */
uint8_t         SlaveTxbuffer[] = "FROMSLAVE";

/* I2C Driver TX buffer  */
uint8_t         SlaveRxbuffer[20];

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
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void I2CMasterOpStatusCb ( DRV_I2C_BUFFER_EVENT event,
                           DRV_I2C_BUFFER_HANDLE bufferHandle,
                           uintptr_t context);

void I2CSlaveOpStatusCb  ( DRV_I2C_BUFFER_EVENT event,
                           DRV_I2C_BUFFER_HANDLE bufferHandle,
                           uintptr_t context);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

DRV_I2C_BUFFER_EVENT APP_Check_Transfer_Status(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle);

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
    
        TxRx_TO_EXTERNAL_SLAVE_1 = 0,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1,
        TxRx_TO_EXTERNAL_SLAVE_2,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2,
        TxRx_TO_EXTERNAL_SLAVE_3,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3,
        TxRx_TO_EXTERNAL_SLAVE_4,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4,
        TxRx_TO_EXTERNAL_SLAVE_5,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5,
        TxRx_TO_EXTERNAL_SLAVE_6,
        TxRx_EXTERNAL_SLAVE_STATUS_CHECK_6,
        TxRx_STATUS_CHECK,
        TxRx_COMPLETED

}APP_EEPROM_WR_STATES;

static APP_EEPROM_WR_STATES appWriteState = TxRx_TO_EXTERNAL_SLAVE_1;

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
            appData.drvI2CHandle_Master = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
            
            /* event-handler set up receive callback from DRV_I2C_Tasks */
            DRV_I2C_BufferEventHandlerSet(appData.drvI2CHandle_Master, I2CMasterOpStatusCb, i2cOpStatus );
            
            if(appData.drvI2CHandle_Master != (DRV_HANDLE) NULL)
            {
                appData.state = APP_WRITE_READ_DATA;
            }
            else
            {
                appData.state = APP_STATE_ERROR;
            }
            appData.drvI2CHandle_Slave  = DRV_I2C_Open(DRV_I2C_INDEX_1, DRV_IO_INTENT_READWRITE);
            
            if(appData.drvI2CHandle_Slave != (DRV_HANDLE) NULL)
            {
                appData.state = APP_WRITE_READ_DATA;
            }
            else
            {
                appData.state = APP_STATE_ERROR;
            }
            
            break;
        }
        case APP_WRITE_READ_DATA:
        {
            /* completes write task to the slave */
            if(APP_Write_Tasks())
            {
                appData.state = APP_STATE_IDLE;
            }
            break;

        }
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

bool APP_Write_Tasks(void)
{
    switch (appWriteState)
    {
        case TxRx_TO_EXTERNAL_SLAVE_1:
        {    

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 1 to RTCC */

            if ( (appData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) || 
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Transmit (  appData.drvI2CHandle_Master,
                                                                        deviceAddressSlave1,
                                                                        &TXbuffer_1[0], 
                                                                        (sizeof(TXbuffer_1)-1), 
                                                                        NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE ) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                    
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_2; 
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_1;
            
            break;
        }        
        case TxRx_TO_EXTERNAL_SLAVE_2:
        {
                                  
            /* Write Transaction - 2 to I2C2-Slave */
            
            deviceAddressSlave2 = PIC32_SLAVE_ADDRESS;
            
            if ( (appData.drvI2CTxRxBufferHandle[1] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_COMPLETE) || 
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[1] = DRV_I2C_Transmit (  appData.drvI2CHandle_Master,
                                                                        deviceAddressSlave2,
                                                                        &TXbuffer_2[0], 
                                                                        (sizeof(TXbuffer_2)-1), 
                                                                        NULL);
            } 
                        
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2;
            break;
        }
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[1]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_3;
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_2;
             
            break;            
        }
        case TxRx_TO_EXTERNAL_SLAVE_3:
        {    

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 3 to RTCC */

            if ( (appData.drvI2CTxRxBufferHandle[2] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[2] = DRV_I2C_Transmit (  appData.drvI2CHandle_Master,
                                                                        deviceAddressSlave1,
                                                                        &TXbuffer_3[0], 
                                                                        (sizeof(TXbuffer_3)-1), 
                                                                        NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[2]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_4;
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_3;

            
            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_4:
        {    

            deviceAddressSlave1 = RTCC_SLAVE_ADDRESS;

            /* Write Transaction - 4 to RTCC */

            if ( (appData.drvI2CTxRxBufferHandle[3] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[3] = DRV_I2C_Transmit (  appData.drvI2CHandle_Master,
                                                                        deviceAddressSlave1,
                                                                        &TXbuffer_4[0], 
                                                                        (sizeof(TXbuffer_4)-1), 
                                                                        NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[3]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_5;
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_4;
             break;
            
            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_5:
        {    
            /* Number of bytes to transfer */
            numOfBytes = ((sizeof(SlaveTxbuffer)) - 1);
            

            /* address of PIC32 slave */
            deviceAddressSlave2 = PIC32_SLAVE_ADDRESS;

            /* Read Transaction to PIC32 Slave */

            if ( (appData.drvI2CTxRxBufferHandle[4] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[4] = DRV_I2C_Receive (   appData.drvI2CHandle_Master,
                                                                        deviceAddressSlave2,
                                                                        &RXbuffer_5[0], 
                                                                        numOfBytes, 
                                                                        NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[4]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                appWriteState = TxRx_TO_EXTERNAL_SLAVE_6;
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_5;
            
            break;
        }
        case TxRx_TO_EXTERNAL_SLAVE_6:
        {    
            /* Number of bytes to read during a WriteRead Operation -            
             * -2 takes account of register address which is 1st byte of 
             * transmitted data and the NULL that at end of array */
            numOfBytes = ((sizeof(TXbuffer_3)) - 2);
            

            /* address of PIC32 slave */
            deviceAddressSlave2 = RTCC_SLAVE_ADDRESS;

            /* Read Transaction to PIC32 Slave */

            if ( (appData.drvI2CTxRxBufferHandle[5] == (DRV_I2C_BUFFER_HANDLE) NULL) || 
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[5]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                        (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[5]) == DRV_I2C_BUFFER_EVENT_ERROR) )
            {
                appData.drvI2CTxRxBufferHandle[5] = DRV_I2C_TransmitThenReceive (   appData.drvI2CHandle_Master,
                                                                                    deviceAddressSlave2,
                                                                                    &TXbuffer_6[0], 
                                                                                    (sizeof(TXbuffer_6)-1),
                                                                                    &RXbuffer_6[0], 
                                                                                    numOfBytes, 
                                                                                    NULL);
            }
            
            appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_6;
            
            break;
        }   
        case TxRx_EXTERNAL_SLAVE_STATUS_CHECK_6:
        {
            if ( (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[5]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(appData.drvI2CHandle_Master, appData.drvI2CTxRxBufferHandle[5]) == DRV_I2C_BUFFER_EVENT_ERROR) )
                appWriteState = TxRx_STATUS_CHECK;
            else
                appWriteState = TxRx_EXTERNAL_SLAVE_STATUS_CHECK_6;
            
            break;
        }  
        case TxRx_STATUS_CHECK:
        {
            DelayMs(300);
            
            LED_SUCCESS_STAGE1Toggle(); 
            LED_SUCCESS_STAGE2Toggle(); 
                                                
            /* to run the application only once,  
             * set next state to TxRx_COMPLETED */
//            appWriteState = TxRx_COMPLETED;
            
            /* to run the application in a continuous loop,  
             * set next state to TxRx_TO_EXTERNAL_SLAVE_1 */
            appWriteState = TxRx_TO_EXTERNAL_SLAVE_1;
            
            
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

void slaveRxorTXNotification(DRV_I2C_BUFFER_EVENT event, void * context)
{
        switch (event)
    {
        case DRV_I2C_BUFFER_SLAVE_READ_REQUESTED:
            
            appData.drvI2CSlaveReadHandle = DRV_I2C_Receive (   appData.drvI2CHandle_Slave, 
                                                                deviceAddressPIC32,
                                                                &SlaveRxbuffer[0],
                                                                20,
                                                                NULL);
            Nop();
            break;
        case DRV_I2C_BUFFER_SLAVE_WRITE_REQUESTED:                        
            appData.drvI2CSlaveWriteHandle = DRV_I2C_Transmit   (   appData.drvI2CHandle_Slave, 
                                                                    deviceAddressPIC32,
                                                                    &SlaveTxbuffer[0],
                                                                    (sizeof(SlaveTxbuffer)-1),
                                                                    NULL);
            Nop();
            break;
        default:
            break;
    }
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
    return (DRV_I2C_TransferStatusGet  (appData.drvI2CHandle_Master, drvBufferHandle));
}

//****************************************************************************/
//  Function: Master Callback Function 
//
//  Callback from DRV_I2C_Tasks when I2C is configured in Master mode     
//****************************************************************************/

void I2CMasterOpStatusCb ( DRV_I2C_BUFFER_EVENT event,
                           DRV_I2C_BUFFER_HANDLE bufferHandle,
                           uintptr_t context)
{
    static uint32_t successCount = 0;
    
    switch (event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            successCount++;
            Nop();
            break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            successCount--;
            break;
        default:
            break;
    }
}

//****************************************************************************/
//  Function: Slave Callback Function 
//
//  Callback from DRV_I2C_Tasks when I2C is configured in Slave mode     
//****************************************************************************/

void I2CSlaveOpStatusCb ( DRV_I2C_BUFFER_EVENT event,
                          DRV_I2C_BUFFER_HANDLE bufferHandle,
                          uintptr_t context)
{
    Nop();
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
