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

/*******************************************************************************
  Function:
    unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)

  Summary:
    The Flash CRC16 test implements the periodic modified checksum
    H.2.19.3.1 as defined by the IEC 60730 standard.

Remarks:
    Refer to classb.h for usage information.
    
  *****************************************************************************/

unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)
{
    return Flash_CRC16((unsigned char*)startAddress, crcPoly, crcSeed, endAddress-startAddress+1);
}

  

