/*******************************************************************************
  Class B Library Implementation File

  Summary:
    This file contains the implementation for 
    the Class B Safety Software Library Flash CRC16 calculation
    for PIC32 MCUs.
    
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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
// DOM-IGNORE-END
#include "classb/classb.h"


/*********************************************************************
 * Function:        unsigned int Flash_CRC16(unsigned char* pBuff, 
                                             unsigned int crcPoly, 
                                             unsigned int crcReg, 
                                             unsigned int bSize)
                                             
  Summary:
    Calculates a CRC for a buffer.

  Remarks:
    Refer to classb.h for usage information.
    
 ********************************************************************/
unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize)
{
	while(bSize--)
	{
		int		ix;
		unsigned char b=*pBuff++;
		int		mask=0x80;
		
		for(ix=0; ix<8; ix++)
		{
			int	fback=(crcReg&FLASH_CRC16_MSB)!=0;  // see if there's transport from the MSb of the LFSR
            
			crcReg<<=1;                             // shift the LFSR
            
            // feed the data MSb into LFSR LSb
			if((b&mask)!=0)
			{
				crcReg|=1;  // set the LSb of the LFSR
			}
           
            // apply the polynomial xor
            // if there's transport out of the LFSR MSb 
			if(fback)
			{
				crcReg^=crcPoly;
			}
            
			mask>>=1;               // move to the next bit in incoming data
		}
	}

	return crcReg&FLASH_CRC16_MASK;    
}


