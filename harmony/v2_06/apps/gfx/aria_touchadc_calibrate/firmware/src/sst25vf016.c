/*****************************************************************************
 *
 * Basic access to SPI Flash SST25VF016 and M25P80
 *
 *****************************************************************************
 * FileName:        SST25VF016.c
 * Dependencies:    SST25VF016.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30 V3.00, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright © 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 01/07/09	   ...
 * 03/08/11    - Modified header file dependencies
 *             - Abstracted out SPI component 
 * 05/11/11    - Updated this file to support both SST25VF016 and M25P80 families
 *             - Although the file name is still specific for SST25VF016
 * 02/29/12    - Updated this file for specific M25P80 families usage.
 *****************************************************************************/

#include "spi.h"
#include "sst25vf016.h"
#include "xc.h"

/************************************************************************
* SST25 Commands                                                       
************************************************************************/
#define SST25_CMD_READ  (unsigned)0x03
#define SST25_CMD_WRITE (unsigned)0x02
#define SST25_CMD_WREN  (unsigned)0x06
#define SST25_CMD_RDSR  (unsigned)0x05

#define USE_SST25VF016
#if defined(USE_SST25VF016)
  #define SST25_CMD_EWSR  (unsigned)0x50
  #define SST25_CMD_SER   (unsigned)0x20
  #define SST25_CMD_ERASE (unsigned)0x60
#else
  #error "EWSR and SER commands not defined for the selected Serial flash."
#endif	

#define SST25_CMD_WRSR  (unsigned)0x01  // Write Status Register 

/************************************************************************
* Function: SST25Init                                                  
*                                                                       
* Overview: this function setup SPI and IOs connected to SST25
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST25Init(void)
{
    SST25ResetWriteProtection();
}

void SST25WriteByte(uint8_t data, uint32_t address)
{
    SST25WriteEnable();

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_WRITE);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x00ff0000UL) >> 16);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x0000ff00UL) >> 8);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x000000ffUL));
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, data);
    SPIGet(SST25_SPI_CHANNEL);

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy());
}

uint8_t SST25ReadByte(uint32_t address)
{
    uint8_t    temp;

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_READ);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x00ff0000UL) >> 16);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x0000ff00UL) >> 8);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x000000ffUL));
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, 0);
    temp = SPIGet(SST25_SPI_CHANNEL);

    SST25CSHigh();

    return (temp);
}

void SST25WriteWord(uint16_t data, uint32_t address)
{
    SST25WriteByte(data & 0x000000ffUL,  address);
    SST25WriteByte((data & 0x0000ff00UL) >> 8, address + 1);
}

uint16_t SST25ReadWord(uint32_t address)
{
    uint16_t res;

    res = SST25ReadByte(address + 1) << 8 | SST25ReadByte(address);

    return (res);
}

void SST25WriteEnable(void)
{
    SST25CSLow();
    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_WREN);
    SPIGet(SST25_SPI_CHANNEL);
    SST25CSHigh();

}

uint8_t SST25IsWriteBusy(void)
{
    uint8_t    temp;

    SST25CSLow();
    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_RDSR);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, 0);
    temp = SPIGet(SST25_SPI_CHANNEL);
    SST25CSHigh();

    return (temp & 0x01);
}

uint8_t SST25WriteArray(uint32_t address, uint8_t *pData, uint16_t nCount)
{
    uint32_t   addr;
    uint8_t    *pD;
    uint16_t    counter;

    addr = address;
    pD = pData;

    // WRITE
    for(counter = 0; counter < nCount; counter++)
    {
        SST25WriteByte(*pD++, addr++);
    }

    // VERIFY
    for(counter = 0; counter < nCount; counter++)
    {
        if(*pData != SST25ReadByte(address))
            return (0);
        pData++;
        address++;
    }

    return (1);
}

void SST25ReadArray(uint32_t address, uint8_t *pData, uint16_t nCount)
{

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_READ);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x00ff0000UL) >> 16);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x0000ff00UL) >> 8);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x000000ffUL));
    SPIGet(SST25_SPI_CHANNEL);

    while(nCount--)
    {
        SPIPut(SST25_SPI_CHANNEL, 0);
        *pData++ = SPIGet(SST25_SPI_CHANNEL);
    }

    SST25CSHigh();

}

void SST25ChipErase(void)
{


#if defined(USE_SST25VF016)
    SST25WriteEnable();
#endif

    SST25CSLow();

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_ERASE);
    SPIGet(SST25_SPI_CHANNEL);

    SST25CSHigh();
    
    // Wait for write end
    while(SST25IsWriteBusy());
}

void SST25ResetWriteProtection(void)
{

    SST25CSLow();
    
    // send write enable command
    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_EWSR);
    SPIGet(SST25_SPI_CHANNEL);
	
    SST25CSHigh();

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_WRSR);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, 0);
    SPIGet(SST25_SPI_CHANNEL);

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy());
}

void SST25SectorErase(uint32_t address)
{
    SST25WriteEnable();

    SST25CSLow();

    SPIPut(SST25_SPI_CHANNEL, SST25_CMD_SER);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x00ff0000UL) >> 16);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x0000ff00UL) >> 8);
    SPIGet(SST25_SPI_CHANNEL);

    SPIPut(SST25_SPI_CHANNEL, (address & 0x000000ffUL));
    SPIGet(SST25_SPI_CHANNEL);

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy());
}


