/*******************************************************************************
  Universal Audio Decoders Demo

  Company:
    Microchip Technology Inc.

  File Name:
    adpcm.c

  Summary:
   Contains the functional implementation of ADPCM decoder.

  Description:
   This file contains the functional implementation of ADPCM decoder.
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
#include "adpcm.h"

int ADPCM_HdrGetFormat (void)
{
    return 0;

}
int ADPCM_HdrGetNumOfChan(void)
{
    return 0;
}
uint8_t ADPCM_GetChannels(){
    return ADPCM_HdrGetNumOfChan();
}
int ADPCM_HdrGetSamplesPerSec(void)
{
    return 0;
}
int ADPCM_HdrGetBlockAlign(void)
{
   return 0;
}
int ADPCM_HdrGetBitsPerSample(void)
{
    return 0;

}
int ADPCM_HdrGetBytesPerSec(void)
{
    return 0;

}
int ADPCM_HdrGetDataLen(void)
{
    return (int) 0;
}

unsigned int ADPCM_HdrGetFileSize(void)
{
    return (unsigned int) 0;
}


void ADPCM_Initialize(uint8_t *input)
{
}

bool ADPCM_Decoder(uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written)
{
    return false;
}

bool isADPCMdecoder_enabled(){
    return false;
}