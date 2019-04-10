//******************************************************************************
// AudioGenerateFx.h
// 
// Description:
//
//    Generate audio functions to a buffer queue.
//
// Arguments:
//   [in]/[out]  AUDIO_PLAY_BUFFER * audioPromptBuffer - buffer to fill
//
// returns: BOOL "end of prompt reached"
//
//******************************************************************************

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _AUDIOGENERATEFX_H
#define _AUDIOGENERATEFX_H

#include "GenericTypeDefs.h"
#include "AudioPlayBufferQueue.h"
#include "AudioGenerateFx.h"

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

//******************************************************************************
//AUDIO_GEN_TYPE
//******************************************************************************
typedef enum _AUDIO_GEN_TYPE
{
    AUDIO_GEN_SILENCE,
    AUDIO_GEN_TONE,
    AUDIO_GEN_CHIRP,
    AUDIO_GEN_UWN,
    AUDIO_GEN_GWN,
    AUDIO_GEN_PINK,
    NUM_AUDIO_GEN_TYPE
} AUDIO_GEN_TYPE;

//******************************************************************************
//AUDIO_GEN_PARAM
//******************************************************************************
typedef struct _AUDIO_GEN_PARAM
{
    uint32_t       sampleRate;
    //uint16_t       ampExpFS; //Tone ampExpFS
    uint16_t       fHz1; //tone or chirp1
    uint16_t       fHz2; //chirp2
    int32_t        timeDeltaMs; //chirp  
    uint32_t       sigma; //noise power
} AUDIO_GEN_PARAM;

//******************************************************************************
//AUDIO_GENERATE
//******************************************************************************
typedef struct _AUDIO_GENERATE
{
    AUDIO_PLAY_BUFFER * audioBuffer;
    int8_t           type;  
    AUDIO_GEN_PARAM  param;

    //Generate state
    //uint16_t         ampExpFS;            //Current ampExpFS
    int32_t          audioSampleCnt;      //Current sample
    int32_t          durationSamples;     //Length of generate
    BOOL             infDuration;         //Inf duration
    int32_t          fHz;                 //Current frequency
    int32_t          f0Hz;                 //Current frequency
    int32_t          fMaxHz;              //Highest possible generated frequency
    int32_t          fMinHz;              //Lowest possible generated frequency
    int32_t          samplesPerCycle;     //#samples per cycle @ fHz
    double           sampleTimeSec;       //Sample Time in Sec
    double           deltaHz;             //Range of chirp Hz 
    double           deltaHzPerSample;    //fraction of chirp per sample time.
    double           cycleDurationFrac;   //Fraction of duration for current
    int32_t          cyclesPerBuffer;     //Number of cycles that fit 
    int32_t          chirpDelHzPerCycle;  //Chirp Freq Change param each cycle
    int32_t          chirpDelHzPerBuffer; //Chirp Freq Change param each cycle
    
} AUDIO_GENERATE;

//******************************************************************************
// Function prototypes
//******************************************************************************

//Audio Signal Generate Constructor
void AudioGenerateInit(AUDIO_GENERATE * ag, 
                       AUDIO_GEN_TYPE   type, 
                       AUDIO_GEN_PARAM  param);

//Audio Signal Generate Process
BOOL AudioGenerateFx(AUDIO_GENERATE * ag, AUDIO_PLAY_BUFFER * audioBuffer); 

//Accessors
//void    AudioGenerateSetAmpExp(AUDIO_GENERATE * ag, uint16_t ampExpFS);
int32_t AudioGenerateFrequencyGet(AUDIO_GENERATE * ag);
void    AudioGenerateFrequencySet(AUDIO_GENERATE * ag, uint32_t freq);
int32_t AudioGenerateFminHz(AUDIO_GENERATE *);
int32_t AudioGenerateFmaxHz(AUDIO_GENERATE *);

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif
