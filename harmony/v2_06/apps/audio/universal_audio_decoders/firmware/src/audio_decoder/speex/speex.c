/*******************************************************************************
  USB Host MSD Demo .

  Company:
    Microchip Technology Inc.

  File Name:
    speex.c

  Summary:
   Contains the functional implementation of speex decoder.

  Description:
   This file contains the functional implementation of speex decoder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
#include "speex.h"



SPEEX_ERROR_MSG SPEEX_Initialize(uintptr_t spx_file_handle)
{
    return SPEEX_GENERAL_ERROR;
}

SPEEX_ERROR_MSG SPEEX_Decoder(uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written)
{
   return SPEEX_GENERAL_ERROR;
}

int32_t SPEEX_DiskRead(uint8_t *inBuff){
    return -1;
}


void SPEEX_Cleanup(){
}


bool isSPEEXdecoder_enabled()
{
    return false;
}

int32_t SPEEX_GetSamplingRate(){
    return 0;
}
uint8_t SPEEX_GetChannels(){
    return 0;
}
int32_t SPEEX_GetBitrate(){
    return 0;
}
