/*******************************************************************************
  IPF SPI Flash Driver memory protection Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf_prot.c

  Summary:
    Source code for the SPI flash driver memory protection supporting function
	implementation.

  Description:
    This file contains the Source code for the SPI flash driver memory 
	protection supporting function implementation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/spi_flash/pic32wk_ipf/src/drv_ipf_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

extern uint8_t gDrvBlockProtReg[];

uint8_t DRV_IPF_GetBlockProtectBitPosition(uintptr_t memAddress, DRV_IPF_PROT_MODE protMode)
{
	uint8_t bitPosition = 0;

	if((memAddress >= 0x10000) && (memAddress <= 0x1EFFFF))
	{
		if(protMode & DRV_IPF_READ_PROTECT)
			return 0xFF;

		bitPosition = (uint8_t)((uint32_t)memAddress / (uint32_t) 0x10000 - 1);
	}
	else if((memAddress >= 0x00000) && (memAddress <= 0x07FFF))
	{
		bitPosition = (uint8_t)32 + (uint8_t)((memAddress & 0xE000) >> 12);

		if(protMode & DRV_IPF_READ_PROTECT)
			bitPosition += 1;
	}
	else if((memAddress >= 0x1F8000) && (memAddress <= 0x1FFFFF))
	{
		bitPosition = (uint8_t)40 + (uint8_t)((memAddress & 0x6000) >> 12);

		if(protMode & DRV_IPF_READ_PROTECT)
			bitPosition += 1;
	}
	else if((memAddress >= 0x8000) && (memAddress <= 0xFFFF))
	{
		if(protMode & DRV_IPF_READ_PROTECT)
			return 0xFF;
		
		bitPosition = 30;
	}
	else if((memAddress >= 0x1F0000) && (memAddress <= 0x1F7FFF))
	{
		if(protMode & DRV_IPF_READ_PROTECT)
			return 0xFF;

		bitPosition = 31;
	}	
    else
    {
        return 0xFF;
    }
                          
	return bitPosition;	
}

void DRV_IPF_SetBitInArray(uint8_t arr[], uint8_t bitPosition)
{
	uint8_t ArrIndex = bitPosition / 8;
	uint8_t ArrBitPos = bitPosition % 8;
	
	arr[ArrIndex] = arr[ArrIndex] | (1 << ArrBitPos);
}

void DRV_IPF_ClearBitInArray(uint8_t arr[], uint8_t bitPosition)
{
	uint8_t ArrIndex = bitPosition / 8;
	uint8_t ArrBitPos = bitPosition % 8;
	
	arr[ArrIndex] = arr[ArrIndex] & ~(1 << ArrBitPos);
}

bool DRV_IPF_CheckBitInArray(uint8_t arr[], uint8_t bitPosition)
{
	uint8_t ArrIndex = bitPosition / 8;
	uint8_t ArrBitPos = bitPosition % 8;
	
	return (arr[ArrIndex] & (1 << ArrBitPos)) ? true : false;
}

void DRV_IPF_BPRBitSet(uint8_t bitPosition, DRV_IPF_PROT_MODE protMode)
{
	DRV_IPF_SetBitInArray(gDrvBlockProtReg, bitPosition);

    /* Checking if both Write and read protect is enabled */
    if(protMode && (protMode & (protMode - 1))) 
    {
        DRV_IPF_SetBitInArray(gDrvBlockProtReg, bitPosition - 1);        
    }
}


void DRV_IPF_BPRBitClear(uint8_t bitPosition, DRV_IPF_PROT_MODE protMode)
{	
	DRV_IPF_ClearBitInArray(gDrvBlockProtReg, bitPosition);

    /* Checking if both Write and read protect to be removed */
    if(protMode && (protMode & (protMode - 1))) 
    {
            DRV_IPF_ClearBitInArray(gDrvBlockProtReg, bitPosition - 1);        
    }
}   