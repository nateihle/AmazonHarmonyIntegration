/*******************************************************************************
  Audio Codec Interface

  Company:
    Microchip Technology Inc.

  File Name:
    audio_codec.h

  Summary:
    Audio Codec Interface 

  Description:
    This file describes the audio codec interface routines. 
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#ifndef AUDIO_CODEC_H
#define AUDIO_CODEC_H

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* CODEC states

  Summary:
    CODEC states enumeration

  Description:
    This enumeration defines the valid codec states.  These states
    determine the behavior of the codec at various times.
*/

typedef enum
{
    AUDIO_CODEC_OPEN,

    AUDIO_CODEC_SET_BUFFER_HANDLER,

    AUDIO_CODEC_ADD_BUFFER,

    AUDIO_CODEC_WAIT_FOR_BUFFER_COMPLETE,

    AUDIO_CODEC_BUFFER_COMPLETE,

    AUDIO_CODEC_WAIT

} AUDIO_CODEC_STATES;

typedef enum{
    CODEC_COMMAND_NONE,
    CODEC_COMMAND_SAMPLING_RATE_SET
    
}CODEC_COMMAND;

typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle;
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    DRV_CODEC_COMMAND_EVENT_HANDLER commandHandler;
    CODEC_COMMAND currentCommand;
    uintptr_t context;
    uint8_t *txbufferObject;
//    size_t bufferSize;

} AUDIO_CODEC_CLIENT;



// *****************************************************************************
/* CODEC Data

  Summary:
    Holds codec data

  Description:
    This structure holds the codec's data.

  Remarks:
   
*/

typedef struct
{
    /* Application's current state*/
    AUDIO_CODEC_STATES state;

    /* USART client handle */
    AUDIO_CODEC_CLIENT codecClient;
    
    DATA_LENGTH     dl;
    SAMPLE_LENGTH   sl;

} AUDIO_CODEC_DATA;
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    void AUDIO_CODEC_Initialize ( void )

  Summary:
     Codec initialization routine

  Description:
    This routine initializes the Codec.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "APP_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    AUDIO_CODEC_Initialize();
    </code>

  Remarks:
    This routine must be called from the APP_Initialize function.
*/
 void AUDIO_CODEC_Initialize (void);


/*******************************************************************************
  Function:
    void APP_CODECBufferEventHandler((DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )

  Summary:
    Buffer Event Handler for Codec task.

  Description:
    This is the Event Handler for Codec Tx and Rx Complete Events.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    
*/
void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context );

void APP_CODECCommandEventHandler(uintptr_t context);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
void Audio_Codec_SetAudioFormat(DATA_LENGTH dl, SAMPLE_LENGTH sl);
bool Audio_Codec_Addbuffer(int8_t *, size_t);
bool Audio_Codec_Open(void);
void Audio_Codec_SetBufferHandler(void);
void Audio_Codec_SetCommandCallback(void);
void Audio_Codec_Close(void);
extern AUDIO_CODEC_DATA CodecData;

#ifdef __cplusplus
}
#endif

#endif