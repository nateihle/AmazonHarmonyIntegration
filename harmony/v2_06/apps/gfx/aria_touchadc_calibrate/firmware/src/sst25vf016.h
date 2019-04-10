/*****************************************************************************
 *
 * Basic access to SPI Flash SST25VF016 and M25P80
 *
 *****************************************************************************
 * FileName:        SST25VF016.h
 * Dependencies:    Graphics.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30, MPLAB C32
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
 * Date         Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 01/07/09	   	...
 * 03/08/11     Modified header file dependencies
 * 05/11/11     - Updated this file to support both SST25VF016 and M25P80 families
 *              - Although the file name is still specific for SST25VF016
 *****************************************************************************/
#ifndef _SST25VF016_H
#define _SST25VF016_H

#include "system_config.h"

#if defined (MEB_BOARD)
    #define SPI_CHANNEL_2_ENABLE
    #define SST25_SPI_CHANNEL 2
    #define SST25_CS_TRIS       TRISGbits.TRISG9
    #define SST25_CS_LAT        LATGbits.LATG9
    #define SPI_FLASH_CHANNEL   CPLD_SPI2
#elif defined (GFX_PICTAIL_V3E)
    #define SPI_CHANNEL_2_ENABLE
    #define SST25_SPI_CHANNEL 2
    #define SST25_CS_TRIS   TRISDbits.TRISD1
    #define SST25_CS_LAT    LATDbits.LATD1

    #define SST25_SCK_TRIS  TRISGbits.TRISG6
    #define SST25_SDO_TRIS  TRISGbits.TRISG8
    #define SST25_SDI_TRIS  TRISGbits.TRISG7
#elif defined (GFX_PICTAIL_LCC)
    #define SST25_CS_TRIS   TRISDbits.TRISD1
    #define SST25_CS_LAT    LATDbits.LATD1

    #define SST25_SCK_TRIS  TRISGbits.TRISG6
    #define SST25_SDO_TRIS  TRISGbits.TRISG8
    #define SST25_SDI_TRIS  TRISGbits.TRISG7
    #define SPI_CHANNEL_1_ENABLE
    #define SST25_SPI_CHANNEL 1
#elif defined (GFX_PICTAIL_V3)
    #define SST25_CS_TRIS   TRISDbits.TRISD1
    #define SST25_CS_LAT    LATDbits.LATD1

    #define SST25_SCK_TRIS  TRISGbits.TRISG6
    #define SST25_SDO_TRIS  TRISGbits.TRISG8
    #define SST25_SDI_TRIS  TRISGbits.TRISG7
    #define SPI_CHANNEL_2_ENABLE
    #define SST25_SPI_CHANNEL 2
#endif

/************************************************************************
 * Section:  Includes                                                       
 ************************************************************************/
//    #include "spi.h"
//#include "GenericTypeDefs.h"

/************************************************************************
* Macro: SST25CSLow()                                                   
*                                                                       
* Preconditions: CS IO must be configured as output
*                                                                       
* Overview: this macro pulls down CS line                    
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none                                                         
*                                                                       
************************************************************************/
    #define SST25CSLow()    SST25_CS_LAT = 0;

/************************************************************************
* Macro: SST25CSHigh()
*                                                                       
* Preconditions: CS IO must be configured as output
*                                                                       
* Overview: this macro set CS line to high level
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
    #define SST25CSHigh()   SST25_CS_LAT = 1;

/************************************************************************
* Function: SST25Init()
*                                                                       
* Overview: this function setups SPI and IOs connected to SST25
*                                                                       
* Input: none
*                                                                       
* Output: none  
*                                                                       
************************************************************************/
void    SST25Init(void);

/************************************************************************
* Function: uint8_t SST25IsWriteBusy(void)  
*                                                                       
* Overview: this function reads status register and chek BUSY bit for write operation
*                                                                       
* Input: none                                                          
*                                                                       
* Output: non zero if busy
*                                                                       
************************************************************************/
uint8_t    SST25IsWriteBusy(void);

/************************************************************************
* Function: void SST25WriteByte(uint8_t data, uint32_t address)                                           
*                                                                       
* Overview: this function writes a byte to the address specified
*                                                                       
* Input: byte to be written and address
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void    SST25WriteByte(uint8_t data, uint32_t address);

/************************************************************************
* Function: uint8_t SST25ReadByte(uint32_t address)       
*                                                                       
* Overview: this function reads a byte from the address specified
*                                                                       
* Input: address          
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
uint8_t    SST25ReadByte(uint32_t address);

/************************************************************************
* Function: void SST25WriteWord(WODR data, uint32_t address)                                           
*                                                                       
* Overview: this function writes a 16-bit word to the address specified
*                                                                       
* Input: data to be written and address
*                                                                       
* Output: none                                                         
*                                                                       
************************************************************************/
void    SST25WriteWord(uint16_t data, uint32_t address);

/************************************************************************
* Function: uint16_t SST25ReadWord(uint32_t address)             
*                                                                       
* Overview: this function reads a 16-bit word from the address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
uint16_t    SST25ReadWord(uint32_t address);

/************************************************************************
* Function: SST25WriteEnable()                                       
*
* Overview: this function allows writing into SST25. Must be called
*           before every write/erase command
*
* Input: none
*            
* Output: none
*
************************************************************************/
void    SST25WriteEnable(void);

/************************************************************************
* Function: uint8_t SST25WriteArray(uint32_t address, uint8_t* pData, nCount)
*                                                                       
* Overview: this function writes data array at the address specified
*                                                                       
* Input: flash memory address, pointer to the data array, data number
*                                                                       
* Output: return 1 if the operation was successfull
*                                                                     
************************************************************************/
uint8_t    SST25WriteArray(uint32_t address, uint8_t *pData, uint16_t nCount);

/************************************************************************
* Function: void SST25ReadArray(uint32_t address, uint8_t* pData, nCount)
*                                                                       
* Overview: this function reads  data into buffer specified
*                                                                       
* Input: flash memory address, pointer to the buffer, data number
*                                                                       
************************************************************************/
void    SST25ReadArray(uint32_t address, uint8_t *pData, uint16_t nCount);

/************************************************************************
* Function: void SST25ChipErase(void)
*                                                                       
* Overview: chip erase
*                                                                       
* Input: none
*                                                                       
************************************************************************/
void    SST25ChipErase(void);

/************************************************************************
* Function: void SST25ResetWriteProtection()
*                                                                       
* Overview: this function reset write protection bits
*                                                                       
* Input: none                                                     
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void    SST25ResetWriteProtection(void);

/************************************************************************
* Function: void SST25SectorErase(uint32_t address)                                           
*                                                                       
* Overview: this function erases a 4Kb sector
*                                                                       
* Input: address within sector to be erased
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void    SST25SectorErase(uint32_t address);
#endif //_SST25VF016_H
