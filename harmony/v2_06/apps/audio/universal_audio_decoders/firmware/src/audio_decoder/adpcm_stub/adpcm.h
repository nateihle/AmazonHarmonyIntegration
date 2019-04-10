/*******************************************************************************
  Universal Audio Decoders Demo

  Company:
    Microchip Technology Inc.

  File Name:
    adpcm.h

  Summary:
    Contains the adpcm decoder specific definitions and function prototypes.

  Description:
    This file contains the adpcm decoder specific definitions and function
    prototypes.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

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

#include <stdint.h>
#include <stdbool.h>

#ifndef ADPCM_H

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END


#define	ADPCM_H

#define ADPCM_HEADER_SIZE 44
//#define WAV_HEAP_SIZE  1024
#define ADPCM_INPUT_BUFFER_SIZE        512//(1024)
    // ADPCM compression ratio is 4:1
#define ADPCM_OUTPUT_BUFFER_SIZE       (4*ADPCM_INPUT_BUFFER_SIZE)
    

typedef struct {
    int format;
    int filesize;
    int filetype;
    int frmtchunk_marker;
    int dataLen;
    short type_frmt;
    short numOfChan;
    int samplesPerSec;
    int bytesPerSec;
    short blockAlign;
    short bitsPerSample;
    int *extra;
    unsigned int extralen;        
} ADPCMHEADER; // which is the

void ADPCM_Initialize(uint8_t *input);
bool ADPCM_Decoder(uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written);

int ADPCM_HdrGetFormat (void);
int ADPCM_HdrGetNumOfChan(void);
uint8_t ADPCM_GetChannels();
int ADPCM_HdrGetSamplesPerSec(void);
int ADPCM_HdrGetBlockAlign(void);
int ADPCM_HdrGetBitsPerSample(void);
int ADPCM_HdrGetBytesPerSec(void);
int ADPCM_HdrGetDataLen(void);
unsigned int ADPCM_HdrGetFileSize(void);
bool isADPCMdecoder_enabled();

#ifdef __cplusplus
}
#endif

#endif	/* ADPCM_H */

