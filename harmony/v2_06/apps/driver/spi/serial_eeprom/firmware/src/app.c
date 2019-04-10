/*******************************************************************************
  MPLAB Harmony Application 
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    Application Template

  Description:
    This file contains the application logic.
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
/* EEPROM OP CODES*/
#define EEPROM_WRITE_STATUS_OP_CODE     1
#define EEPROM_WRITE_COMMAND_OP_CODE    2
#define EEPROM_READ_COMMAND_OP_CODE     3
#define EEPROM_READ_STATUS_OP_CODE      5
#define EEPROM_WRITE_ENABLE_OP_CODE     6

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

uint8_t getEEPROMdata[MAX_NUM_OF_BYTES] = {0};

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Definitions
// *****************************************************************************
// *****************************************************************************

typedef enum{

    WR_SEND_WRITE_STATUS_CMD = 0,
    WR_WAIT_FOR_STATUS_REPLY,
    WR_SEND_WREN_CMD,
    WR_ENABLE_STATUS_CHECK,
    WR_SEND_WRITE_CMD,
    WR_WAIT_FOR_WRITE_COMPLETE,
    WR_COMPLETED

}APP_EEPROM_WR_STATES;

typedef enum{

    RD_SEND_STATUS_CODE_CMD = 0,
    RD_WAIT_FOR_STATUS_CMD_REPLY,
    RD_GET_STATUS_DATA,
    RD_BUSY_STATUS_CHECK,
    RD_SEND_READ_CMD,
    RD_WAIT_FOR_READ_CMD_REPLY,
    RD_GET_DATA,
    RD_WAIT_FOR_DATA,
    RD_COMPLETE

}APP_EEPROM_RD_STATES;

uint8_t APP_EEPROM_Write_Tasks(void);
uint8_t APP_EEPROM_Read_Tasks(void);
uint8_t APP_EEPROM_Check_Transfer_Status(DRV_SPI_BUFFER_HANDLE drvBufferHandle);
void APP_EEPROM_Write( SPI_DATA_TYPE *txbuffer, uint32_t num_of_bytes);
void APP_EEPROM_Read( SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes );

/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData = 
{
    //Initialize appData structure. 

};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/


APP_DRV_OBJECTS appDrvObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
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
    
    /* Deselect the Chip Select Lines*/
    APP_SPI_CS_DESELECT();

    /* APP state task Init */
    appData.state = APP_STATE_INIT;
}

/********************************************************
 * Application switch press routine
 ********************************************************/



/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    uint8_t count;

    switch(appData.state)
    {
        case APP_STATE_INIT:
        {
            /* Open the SPI Driver */
            appData.drvSPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                                     DRV_IO_INTENT_READWRITE );

            if(appData.drvSPIHandle != DRV_HANDLE_INVALID)
            {
                appData.state = APP_STATE_EEPROM_WRITE;
            }
            else
            {
                appData.state = APP_STATE_ERROR;
            }
            break;
        }
        case APP_STATE_EEPROM_WRITE:
        {
            /* EEPROM write task completes when it writes first page in EEPROM*/
            if(APP_EEPROM_Write_Tasks())
            {
                appData.state = APP_STATE_EEPROM_READ;
            }
            break;
        }
        case APP_STATE_EEPROM_READ:
        {
           /* EEPROM read task completes when it reads first page in EEPROM*/
            if(APP_EEPROM_Read_Tasks())
           {
               appData.state = APP_STATE_EEPROM_VERIFY;
           }
           break;
        }
        case APP_STATE_EEPROM_VERIFY:
        {
            /* Verify write and read data from the EEPROM*/
            for(count =0; count < MAX_NUM_OF_BYTES; )
            {
                /* Odd address location */
                if(count%2)
                {
                    /* Odd address should have 0x55*/
                    if(!(getEEPROMdata[count++] == 0x55))
                    {
                        break;
                    }
                }
                else /* Even address locations*/
                {
                    /* Even address should have 0xAA*/
                    if(!(getEEPROMdata[count++] == 0xAA))
                    {
                        break;
                    }
                }
            }

            /* Check the count reached number of data in first page*/
            if(count != MAX_NUM_OF_BYTES)
            {
                /* Test failed*/
                BSP_LEDOn(BSP_LED_9);
            }
            else
            {
                /* Test Passed*/
                BSP_LEDOn(BSP_LED_10);
            }

            /* Move the app state */
            appData.state = APP_STATE_IDLE;

            break;
        }
        case APP_STATE_IDLE:
        {
            Nop();
            break;
        }
        case APP_STATE_ERROR:
        {
            /* Test failed*/
            BSP_LEDOn(BSP_LED_9);
            break;
        }
        default:
            break;
    }
} 

void APP_EEPROM_Read( SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes )
{
    /* Add the buffer pointer to read the data from EEPROM */
    appData.drvSPIRDBUFHandle = DRV_SPI_BufferAddRead( appData.drvSPIHandle,
                    (SPI_DATA_TYPE *)&appData.drvSPIRXbuffer[0], num_of_bytes, 0, 0);
}

uint8_t APP_EEPROM_Read_Tasks(void)
{
    static APP_EEPROM_RD_STATES readstate = RD_SEND_STATUS_CODE_CMD;
    uint32_t num_of_bytes;
    uint32_t count;

    switch(readstate)
    {
        /* Send read status commmand to check EEPROM is busy for not! */
        case RD_SEND_STATUS_CODE_CMD:
        {
            /* Assert the CS Line */
            APP_SPI_CS_SELECT();

            /* Add the EEPROM Read STATUS OP Code to the buffer*/
            appData.drvSPITXbuffer[0] = EEPROM_READ_STATUS_OP_CODE;
            appData.drvSPITXbuffer[1] = 0; /* Dummy byte */

            /* Number of bytes to transfer */
            num_of_bytes = 2;

            /* Add to the write buffer to transmit*/
            APP_EEPROM_Write(&appData.drvSPITXbuffer[0], num_of_bytes);
            readstate = RD_WAIT_FOR_STATUS_CMD_REPLY;

            break;
        }
        case RD_WAIT_FOR_STATUS_CMD_REPLY:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIWRBUFHandle))
            {
                /* Transfer Status Success*/
                readstate = RD_GET_STATUS_DATA;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                readstate = RD_WAIT_FOR_STATUS_CMD_REPLY;
            }
            break;
        }
        case RD_GET_STATUS_DATA:
        {
            /* Add the buffer to get the data from EEPROM*/
            num_of_bytes = 1;

            APP_EEPROM_Read(&appData.drvSPIRXbuffer[0], num_of_bytes);
            readstate = RD_BUSY_STATUS_CHECK;

            break;
        }
        case RD_BUSY_STATUS_CHECK:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIRDBUFHandle))
            {
                /* Deassert the CS Line*/
                APP_SPI_CS_DESELECT();

                //Check if Write in Progress (WIP) is true
                if(appData.drvSPIRXbuffer[0] & 0x01)
                {
                    /* Re-Send status command again to check the busy stautus */
                    readstate = RD_SEND_STATUS_CODE_CMD;
                }
                else
                {
                    /* Transmit read command to read data from EEPROM memory */
                    readstate = RD_SEND_READ_CMD;
                }
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                readstate = RD_BUSY_STATUS_CHECK;
            }
            break;
        }
        case RD_SEND_READ_CMD:
        {
            /* Add read command op code & 16-bit beginning address to buffer. 
             * This sequence picks the data byte from the selected location and 
             * holds it in the shift register of the SPI interface of the EEPROM */
            appData.drvSPITXbuffer[0] = EEPROM_READ_COMMAND_OP_CODE;
            appData.drvSPITXbuffer[1] = 0; /* Address - LSB */
            appData.drvSPITXbuffer[2] = 0; /* Address - MSB */
            /* A dummy byte is needed to push that byte of data out of shift register*/
            appData.drvSPITXbuffer[3] = 0; /* Dummy byte */

            /* Number of bytes to transfer */
            num_of_bytes = 4;

            /* Assert the CS line */
            APP_SPI_CS_SELECT();

            /* Add to the write buffer and transmit */
            APP_EEPROM_Write(&appData.drvSPITXbuffer[0], num_of_bytes);
            readstate = RD_WAIT_FOR_READ_CMD_REPLY;

            break;
        }
        case RD_WAIT_FOR_READ_CMD_REPLY:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIWRBUFHandle))
            {
                /*  Get the data from EEPROM */
                readstate = RD_GET_DATA;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                readstate = RD_WAIT_FOR_READ_CMD_REPLY;
            }
            break;
        }
        case RD_GET_DATA:
        {
            /* Add the buffer pointer to read the data*/
            APP_EEPROM_Read(&appData.drvSPIRXbuffer[0], (MAX_NUM_OF_BYTES));
            readstate = RD_WAIT_FOR_DATA;

            break;
        }
        case RD_WAIT_FOR_DATA:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIRDBUFHandle))
            {
                /* Deassert the CS Line */
                APP_SPI_CS_DESELECT();

                readstate = RD_COMPLETE;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                readstate = RD_WAIT_FOR_DATA;
            }
            break;
        }
        case RD_COMPLETE:
        {
            /* Copy the received data to the local buffer */
            for(count = 0; count < MAX_NUM_OF_BYTES;)
            {
                getEEPROMdata[count] = appData.drvSPIRXbuffer[count];
                count++;
            }

            /* return done to app task */
            return true;
            break;
        }
        default:
            break;
    }

    return false;
}

uint8_t APP_EEPROM_Write_Tasks(void)
{
    static APP_EEPROM_WR_STATES writestate = WR_SEND_WRITE_STATUS_CMD;
    uint32_t num_of_bytes, loop, dataCount;

    switch(writestate)
    {
        case WR_SEND_WRITE_STATUS_CMD:
        {
            /* Assert CS Line */
            APP_SPI_CS_SELECT();

            /* Add Write status op code, data to the buffer */
            appData.drvSPITXbuffer[0] = EEPROM_WRITE_STATUS_OP_CODE;
            appData.drvSPITXbuffer[1] = 0x08; /* EEPROM BP1 = 1, BP0 = 0 */
            appData.drvSPITXbuffer[2] = 0; /* Dummy byte */

            /* Number of bytes to transfer */
            num_of_bytes = 3;

            /* Add to the write buffer and transmit*/
            APP_EEPROM_Write(&appData.drvSPITXbuffer[0], num_of_bytes);
            writestate = WR_WAIT_FOR_STATUS_REPLY;

            break;
        }
        case WR_WAIT_FOR_STATUS_REPLY:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIWRBUFHandle))
            {
                /* Deassert CS Line */
                APP_SPI_CS_DESELECT();

                writestate = WR_SEND_WREN_CMD;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                writestate = WR_WAIT_FOR_STATUS_REPLY;
            }
            break;
        }
        case WR_SEND_WREN_CMD:
        {
            /* Assert CS Line */
            APP_SPI_CS_SELECT();

            /* Add Write Enable op code to the buffer */
            appData.drvSPITXbuffer[0] = EEPROM_WRITE_ENABLE_OP_CODE;
            appData.drvSPITXbuffer[1] = 0; /* Dummy byte */

            /* Number of bytes to transfer */
            num_of_bytes = 2;

            /* Add to the write buffer and transmit*/
            APP_EEPROM_Write(&appData.drvSPITXbuffer[0], num_of_bytes);
            writestate = WR_ENABLE_STATUS_CHECK;

            break;
        }
        case WR_ENABLE_STATUS_CHECK:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIWRBUFHandle))
            {
                /* Deassert CS Line */
                APP_SPI_CS_DESELECT();

                writestate = WR_SEND_WRITE_CMD;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                writestate = WR_ENABLE_STATUS_CHECK;
            }
            break;
        }
        case WR_SEND_WRITE_CMD:
        {
            APP_SPI_CS_SELECT();

            appData.drvSPITXbuffer[0] = EEPROM_WRITE_COMMAND_OP_CODE;
            appData.drvSPITXbuffer[1] = 0; /* Address - MSB */
            appData.drvSPITXbuffer[2] = 0; /* Address - LSB */
            appData.drvSPITXbuffer[3] = 0; /* Dummy byte */

            dataCount = 4;

            /* Add the data to the buffer */
            for(loop =0; loop < MAX_NUM_OF_BYTES; )
            {
                if(loop%2)
                {
                    appData.drvSPITXbuffer[dataCount++] = 0x55;
                }
                else
                {
                    appData.drvSPITXbuffer[dataCount++] = 0xAA;
                }

                loop++;
            }
            /* Number of bytes to transfer */
            num_of_bytes = dataCount; //opcode + address + data

            /* Add the write buffer and transmit*/
            APP_EEPROM_Write(&appData.drvSPITXbuffer[0], num_of_bytes);
            writestate = WR_WAIT_FOR_WRITE_COMPLETE;

            break;
        }
        case WR_WAIT_FOR_WRITE_COMPLETE:
        {
            /* Check if the transfer status is success or not */
            if(APP_EEPROM_Check_Transfer_Status(appData.drvSPIWRBUFHandle))
            {
                APP_SPI_CS_DESELECT();

                writestate = WR_COMPLETED;
            }
            else
            {
                /* Transfer Status - Wait untill staus becomes true*/
                writestate = WR_WAIT_FOR_WRITE_COMPLETE;
            }
            break;
        }
        case WR_COMPLETED:
        {
            /* return done to app task */
            return true;
            break;
        }
        default:
            break;
    }
    return false;
}

void APP_EEPROM_Write( SPI_DATA_TYPE * txbuffer, uint32_t num_of_bytes)
{
    /* Add the buffer to transmit */
    appData.drvSPIWRBUFHandle = DRV_SPI_BufferAddWrite(appData.drvSPIHandle,
                     (SPI_DATA_TYPE *)&appData.drvSPITXbuffer[0], num_of_bytes, 0, 0);
}

uint8_t APP_EEPROM_Check_Transfer_Status(DRV_SPI_BUFFER_HANDLE drvBufferHandle)
{
    if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus (drvBufferHandle))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*******************************************************************************
 End of File
 */

