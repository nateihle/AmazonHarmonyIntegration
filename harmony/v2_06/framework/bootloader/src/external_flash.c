/*******************************************************************************
  MPLAB Harmony Bootloader Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    nvm.c

  Summary:
    This file contains the source code for handling NVM controllers.

  Description:
    This file contains the source code for the NVM handling functions for PIC32MX
	and MZ devices.
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

#include "nvm.h"
#include "system_config.h"
#include "peripheral/nvm/plib_nvm.h"
#include "system/devcon/sys_devcon.h"
#include <sys/kmem.h>

#include "driver/spi_flash/sst25vf016b/drv_sst25vf016b.h"
#include "driver/spi_flash/sst25vf016b/src/drv_sst25vf016b_local.h"

#include "system_definitions.h"

void SPIPut(unsigned int channel, unsigned char data);

DRV_SST25VF016B_BLOCK_COMMAND_HANDLE  commandHandle1;
DRV_HANDLE          sstOpenHandle1 = DRV_HANDLE_INVALID;
extern DRV_SST25VF016B_BUFFER_OBJ gDrvSST25VF016BBufferObj[DRV_SST25VF016B_QUEUE_DEPTH_COMBINED];

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

void APP_FlashErase( void )
{

     /* Enable CS Line */
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF016B_CHIP_SELECT_PORT_CHANNEL_IDX0, DRV_SST25VF016B_CHIP_SELECT_PORT_BIT_POS_IDX0);
    SPIPut(DRV_SPI_SPI_ID_IDX0, SST25VF016B_WREN_OP_CODE);
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX0);
    
     /* Disable CS Line */
    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF016B_CHIP_SELECT_PORT_CHANNEL_IDX0, DRV_SST25VF016B_CHIP_SELECT_PORT_BIT_POS_IDX0);

    /* Enable CS Line */
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF016B_CHIP_SELECT_PORT_CHANNEL_IDX0, DRV_SST25VF016B_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SPIPut(DRV_SPI_SPI_ID_IDX0, 0x60);
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX0);

     /* Disable CS Line */
    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF016B_CHIP_SELECT_PORT_CHANNEL_IDX0, DRV_SST25VF016B_CHIP_SELECT_PORT_BIT_POS_IDX0);

}

char APP_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen)
{
    static T_HEX_RECORD HexRecordSt;
    static uint32_t _Len = 0;
    uint8_t Checksum = 0;
    uint32_t i;
    static uint32_t nextRecStartPt = 0;
    static uint8_t state =0;
    
    if(state == 0)
    {    
        nextRecStartPt = 0;
        _Len = totalLen;
        state = 1;        
    }
    
    while(_Len >= 5) // A hex record must be atleast 5 bytes. (1 Data Len byte + 1 rec type byte+ 2 address bytes + 1 crc)
    {
       
        if(gDrvSST25VF016BBufferObj[0].inUse == true)
           return(HEX_REC_PGM_ERROR);     
        
        //HexRecord = &HexRecord[nextRecStartPt];
        HexRecordSt.RecDataLen = HexRecord[nextRecStartPt];
        HexRecordSt.RecType = HexRecord[nextRecStartPt+3];
        HexRecordSt.Data = &HexRecord[nextRecStartPt+4];

        // Hex Record checksum check.
        Checksum = 0;
        for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
        {
            Checksum += HexRecord[nextRecStartPt+i];
        }

        if(Checksum != 0)
        {
            return (0);//HEX_REC_CRC_ERROR;
        }
        else
        {
            // Hex record checksum OK.
            switch(HexRecordSt.RecType)
            {
                case DATA_RECORD:  //Record Type 00, data record.
                    HexRecordSt.Address = (HexRecord[1]<<8) + HexRecord[2];

                    // Derive the address.
                    HexRecordSt.Address = HexRecordSt.Address + HexRecordSt.ExtLinAddress + HexRecordSt.ExtSegAddress;
                                         
                    DRV_SST25VF016B_BlockWrite
                    (
                        sstOpenHandle1,
                        &commandHandle1,
                        HexRecordSt.Data,
                        HexRecordSt.Address,
                        HexRecordSt.RecDataLen
                    ); 
                    
                    break;

                case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
                    HexRecordSt.ExtSegAddress >>= (HexRecordSt.Data[0]<<12) + (HexRecordSt.Data[1]<<4);
                    
                    // Reset linear address.
                    HexRecordSt.ExtLinAddress = 0;
                    break;

                case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
                    HexRecordSt.ExtLinAddress = (HexRecordSt.Data[0]<<24) + (HexRecordSt.Data[1]<<16);

                    // Reset segment address.
                    HexRecordSt.ExtSegAddress = 0;
                    break;

                case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;
                default:
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;
            }
            
        //Determine next record starting point.
        nextRecStartPt += HexRecordSt.RecDataLen + 5;

        // Decrement total hex record length by length of current record.
        _Len -= HexRecordSt.RecDataLen + 5;// _Len - nextRecStartPt;

        }
    }//while(1)

    state = 0;

    if ( (HexRecordSt.RecType == DATA_RECORD) || (HexRecordSt.RecType == EXT_SEG_ADRS_RECORD)
            || (HexRecordSt.RecType == EXT_LIN_ADRS_RECORD) || (HexRecordSt.RecType == END_OF_FILE_RECORD) )
    {
        return (HEX_REC_NORMAL);
    }
    else
    {
        return (HEX_REC_UNKNOW_TYPE);
    }

}

/*****************************************************************************
 * void SPIPut(unsigned int channel, unsigned char data)
 *****************************************************************************/
void SPIPut(unsigned int channel, unsigned char data)
{
                       
        // Wait for free buffer
        while(!PLIB_SPI_TransmitBufferIsEmpty(channel));

        PLIB_SPI_BufferWrite(channel, data);
        
        // Wait for data uint8_t
        while(!PLIB_SPI_ReceiverBufferIsFull(channel));
        return;

          
}
