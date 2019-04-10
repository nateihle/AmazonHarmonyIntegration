
/*******************************************************************************
MP3 Decoder Library Interface File

  Company:
    Microchip Technology Inc.

  File Name:
    mp3.h

  Summary:
    MP3 Decoder support API.

  Description:
    This header file consists of support function declarations.

*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.
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
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END
#include "system/fs/sys_fs.h"
    
#define MP3_IN_FRAME_SIZE       6144//1538
#define MP3_OUT_FRAME_SIZE      1152 * 4
#define MP3_STATE_SIZE          16876//11024

#define MP3_BYTES_PER_SAMPLE    2
#define MP3_BITS_PER_SAMPLE     MP3_BYTES_PER_SAMPLE * 8
    
    
typedef bool (*DecoderEventHandlerCB)(uint32_t event, uint32_t data);
    
bool isMP3decoder_enabled();
bool MP3_Initialize ( void *heap, uint16_t size, SYS_FS_HANDLE mp3Filehandle);
void MP3_RegisterDecoderEventHandlerCallback(DecoderEventHandlerCB fptr);
bool MP3_Decode ( uint8_t *input, uint16_t inSize, uint16_t *read, uint8_t *output, uint16_t *written );
uint32_t MP3_UpdatePlaytime();
uint32_t MP3_GetAudioSize();
uint8_t MP3_GetChannels();
#ifdef __cplusplus
}
#endif