/*******************************************************************************
 Data Stream usb Source File

  File Name:
    usb.c

  Summary:
 Data Stream USART source

  Description:
    This file contains source code necessary for the data stream interface.
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
#include "bootloader/src/bootloader.h"
#include "../nvm.h"
#include "peripheral/nvm/plib_nvm.h"
#include "system/common/sys_module.h"
#include "bootloader/src/datastream.h"
//#include "peripheral/ports/plib_ports.h"
#include "peripheral/dma/plib_dma.h"
#include "system/clk/sys_clk.h"
#include "usb/usb_host.h"
#include "usb/usb_host_msd.h"
#include "usb/usb_host_scsi.h"
#include "system/int/sys_int.h"
#include "system/tmr/sys_tmr.h"

typedef enum {
    REC_FLASHED = 0,
    REC_NOT_FOUND,
    REC_FOUND_BUT_NOT_FLASHED
} T_REC_STATUS;

typedef struct
{
    uint8_t *start;
    uint8_t len;
    T_REC_STATUS status;
}T_REC;

typedef struct
{
	uint8_t RecDataLen;
	uint32_t Address;
	uint8_t RecType;
	uint8_t* Data;
	uint8_t CheckSum;
	uint32_t ExtSegAddress;
	uint32_t ExtLinAddress;
}T_HEX_RECORD;

T_REC record;

extern BOOTLOADER_DATA bootloaderData __attribute__((coherent, aligned(16)));

USB_HOST_EVENT_RESPONSE Bootloader_BufferEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context);

/********************************************************************
* Function:     ConvertAsciiToHex()
*
* Precondition:
*
* Input:        ASCII buffer and hex buffer.
*
* Output:
*
* Side Effects: No return from here.
*
* Overview:     Converts ASCII to Hex.
*
*
* Note:         None.
********************************************************************/
void ConvertAsciiToHex(uint8_t* asciiRec, uint8_t* hexRec)
{
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t hex;


    while((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66))
    {
            // Check if the ASCII values are in alpha numeric range.

            if(asciiRec[i] < 0x3A)
            {
                    // Numerical representation in ASCII found.
                    hex = asciiRec[i] & 0x0F;
            }
            else
            {
                    // Alphabetical value.
                    hex = 0x09 + (asciiRec[i] & 0x0F);
            }

            // Following logic converts 2 bytes of ASCII to 1 byte of hex.
            k = i%2;

            if(k)
            {
                    hexRec[i>>1] |= hex;

            }
            else
            {
                    hexRec[i>>1] = (hex << 4) & 0xF0;
            }
            i++;
    }

}

void Bootloader_ProcessBuffer( BOOTLOADER_DATA *handle )
{
    uint32_t i;
    uint32_t recCount = 0;
    size_t numBytes = handle->bufferSize;

    for(i = 0; i < (numBytes + handle->cmdBufferLength); i ++)
    {
            // This state machine seperates-out the valid hex records from the read 512 bytes.
        switch(record.status)
        {
            case REC_FLASHED:
            case REC_NOT_FOUND:
                if(handle->data->buffer[i] == ':')
                {
                        // We have a record found in the 512 bytes of data in the buffer.
                    record.start = &handle->data->buffer[i];
                    record.len = 0;
                    record.status = REC_FOUND_BUT_NOT_FLASHED;
                }
                break;
            case REC_FOUND_BUT_NOT_FLASHED:
                if((handle->data->buffer[i] == 0x0A) || (handle->data->buffer[i] == 0xFF))
                {
                        // We have got a complete record. (0x0A is new line feed and 0xFF is End of file)
                    // Start the hex conversion from element
                    // 1. This will discard the ':' which is
                    // the start of the hex record.
                    ConvertAsciiToHex(&record.start[1],&record.start[1]);
                    APP_ProgramHexRecord(&record.start[1], record.len >> 1);
                    recCount++;
                    record.status = REC_FLASHED;
                }
                break;
        }
        // Move to next byte in the buffer.
        record.len ++;
    }

    if(record.status == REC_FOUND_BUT_NOT_FLASHED)
    {
            // We still have a half read record in the buffer. The next half part of the record is read
            // when we read 512 bytes of data from the next file read.
        memcpy(handle->data->buffer, record.start, record.len);
        handle->cmdBufferLength = record.len;
        record.status = REC_NOT_FOUND;
    }
    else
    {
        handle->cmdBufferLength = 0;
    }
}

/*******************************************************
 * USB HOST MSD Layer Events - Application Event Handler
 *******************************************************/
void Bootloader_USBHostMSDEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch ( event)
    {
        case SYS_FS_EVENT_MOUNT:
            bootloaderData.currentState =  BOOTLOADER_DEVICE_CONNECTED;
            break;

        case SYS_FS_EVENT_UNMOUNT:
            bootloaderData.currentState = BOOTLOADER_UNMOUNT_DISK;
            break;

        default:
            break;
    }
}


/*******************************************************
 * USB HOST Layer Events - Host Event Handler
 *******************************************************/
USB_HOST_EVENT_RESPONSE Bootloader_BufferEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
    return USB_HOST_EVENT_RESPONSE_NONE;
}

void DATASTREAM_BufferEventHandlerSet
(
    const DRV_HANDLE hClient,
    const void * eventHandler,
    const uintptr_t context
)
{
    /* This means host operation is enabled. We can
    * move on to the next state */
    SYS_FS_EventHandlerSet(Bootloader_USBHostMSDEventHandler, (uintptr_t)NULL);
    USB_HOST_EventHandlerSet(Bootloader_BufferEventHandler, 0);
           
}

DRV_HANDLE DATASTREAM_Open(const DRV_IO_INTENT ioIntent)
{
    USB_HOST_BusEnable(0);
    
    return 0;
}

DRV_CLIENT_STATUS DATASTREAM_ClientStatus(DRV_HANDLE handle)
{
    if(USB_HOST_BusIsEnabled(0))
        return DRV_CLIENT_STATUS_READY;
    else
        return DRV_CLIENT_STATUS_BUSY;
}

int DATASTREAM_Data_Read(uintptr_t * const bufferHandle, unsigned char* buffer, const int maxsize)
{
    return(SYS_FS_FileRead( bootloaderData.fileHandle, (void *)buffer, maxsize ));
}

int DATASTREAM_Data_Write(uintptr_t * const bufferHandle, unsigned char* buffer, const int bufsize)
{
   //No writing performed for USB Host
    return(0);
}

void DATASTREAM_Close(void)
{
    //Disable Interrupt sources so bootloader application runs without issues
    SYS_INT_SourceDisable(DRV_TMR_INTERRUPT_SOURCE_IDX0);
    SYS_INT_VectorPrioritySet(DRV_TMR_INTERRUPT_VECTOR_IDX0, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(DRV_TMR_INTERRUPT_VECTOR_IDX0, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_SourceDisable(INT_SOURCE_USB_1);
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);
#if defined(_USB_DMA_VECTOR)
    SYS_INT_SourceDisable(INT_SOURCE_USB_1_DMA);
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1_DMA, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1_DMA, INT_SUBPRIORITY_LEVEL0);
#endif
        
    PLIB_TMR_Stop(DRV_TMR_PERIPHERAL_ID_IDX0);
}
