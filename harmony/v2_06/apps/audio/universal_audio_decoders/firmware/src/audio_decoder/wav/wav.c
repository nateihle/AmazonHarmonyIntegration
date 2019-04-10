/*******************************************************************************
  Universal Audio Decoders Demo

  Company:
    Microchip Technology Inc.

  File Name:
    wav.c

  Summary:
   Contains the functional implementation of wav decoder.

  Description:
   This file contains the functional implementation of wav decoder.
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
#include "wav.h"

int WAV_HdrGetFormat (void)
{
    return (int) 0;

}
int WAV_HdrGetNumOfChan(void)
{
    return (int) 0;

}
int WAV_HdrGetSamplesPerSec(void)
{
    return (int) 0;
}
int WAV_HdrGetBlockAlign(void)
{
   return (int) 0;
}
int WAV_HdrGetBitsPerSample(void)
{
    return (int) 0;

}
int WAV_HdrGetBytesPerSec(void)
{
    return (int) 0;

}
int WAV_HdrGetDataLen(void)
{
    return (int) 0;
}

unsigned int WAV_HdrGetFileSize(void)
{
    return (unsigned int) 0;
}

void WAV_Initialize(uint8_t *input)
{
}
uint8_t WAV_GetChannels(){
    return 0;
}
bool WAV_Decoder(uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written)
{   
    return false;
}

bool isWAVdecoder_enabled(){
    return false;
}

uint32_t WAV_UpdatePlaytime(){
    return 0;
}

uint32_t WAV_GetAudioSize(){
    return 0;
}