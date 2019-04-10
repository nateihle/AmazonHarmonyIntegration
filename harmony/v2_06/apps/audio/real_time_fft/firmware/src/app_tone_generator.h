/*******************************************************************************
  Audio Microphone storage

  Company:
    Microchip Technology Inc.

  File Name:
    app_tone_generator.h

  Summary:
    This header file provides prototypes and definitions for the application

  Description:
    This header file provides prototypes and definitions for the application
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef APP_TONE_GENERATOR_H
#define	APP_TONE_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define MAX_DURATION_IN_SECS                        300    
#define MAX_TONES                                    3
#define SAMPLING_FREQUENCY                          48000
#define MAX_SAMPLES_PER_CHANNEL                     4096
#define SIGNAL_FS_VALUE                             32768    
#define MAX_FREQ_LIMIT                              20000
#define MIN_FREQ_LIMIT                              0    
#define MAX_DBFS_LIMIT                              0    
#define MIN_DBFS_LIMIT                             -72

#define APP_TONE_MAX_EVENT_LISTNERS                  2
    
typedef enum
{
    APP_TONE_GENERATOR_EVENT_TYPE_SIG_READY = 0,
    APP_TONE_GENERATOR_EVENT_TYPE_SIG_DURATION_ELAPSED,
    APP_TONE_GENERATOR_EVENT_TYPE_MAX,
}APP_TONE_GENERATOR_EVENT_TYPE;
    
typedef void (*APP_TONE_GENERATOR_EVENT_CALLBACK) (APP_TONE_GENERATOR_EVENT_TYPE eventType, const void* const pEventData);
    
typedef enum
{
    APP_TONE_GENERATOR_STATE_WAIT_FOR_REQUEST = 0,
    APP_TONE_GENERATOR_STATE_GEN_SIGNAL,
    APP_TONE_GENERATOR_STATE_GEN_SIGNAL1,
    APP_TONE_GENERATOR_STATE_NOTIFY_SIG_READY,
    APP_TONE_GENERATOR_STATE_MAX,
}APP_TONE_GENERATOR_STATE;

typedef struct
{
	uint32_t                            amplitude;
	float                               toneFreqHz;
	float                               degreePerStep;	
}APP_TONE_PARAMS;

typedef struct
{
    uint8_t                             nTones;
    uint32_t                            samplingFreq;
    uint32_t                            duration;
    uint32_t                            elapsedDuration;
    APP_TONE_PARAMS                     toneParams[MAX_TONES];
}APP_TONE_CONFIG;

typedef struct
{
    APP_TONE_GENERATOR_STATE            state;
    bool                                isToneGenRequested;
    bool                                isDataRequested;
    APP_TONE_CONFIG                     toneConfig;  
    uint32_t                            sigSample;
    int16_t                             mixedSignal[MAX_SAMPLES_PER_CHANNEL];
    APP_TONE_GENERATOR_EVENT_CALLBACK   eventListners[APP_TONE_MAX_EVENT_LISTNERS];    
}APP_TONE_TASK_DATA;

void APP_TONE_TaskInitialize(void);
void APP_TONE_Task(void);

void APP_TONE_ReStartSignal(void);
void APP_TONE_SetSamplingFreq(uint32_t samplingFreq);
uint32_t APP_TONE_GetSamplingFreq(void);
void APP_TONE_SetSignalFreq(uint8_t sigIndex, uint32_t freqHz);
void APP_TONE_SetSignalAmplitude(uint8_t sigIndex, int32_t amplitudeIndBFS);
void APP_TONE_StartSignalGeneration(uint32_t duration);
void APP_TONE_GenSignalData(void);
bool APP_TONE_RegisterCallback(APP_TONE_GENERATOR_EVENT_CALLBACK evHandler);
bool APP_TONE_UnRegisterCallback(APP_TONE_GENERATOR_EVENT_CALLBACK evHandler);
int16_t* APP_TONE_GetMixedSignal(void);
uint32_t APP_TONE_GetNumSamples(void);
uint32_t APP_TONE_GetFullScaleSignalValue(void);
void APP_TONE_GetFreqRangeInHz(uint32_t* const pLowFreqLim, uint32_t* const pHighFreqLim);
void APP_TONE_GetAmplitudeRagneInDBFS(int32_t* const pLowLim, int32_t* const pHighLim);

#ifdef	__cplusplus
}
#endif

#endif	/* APP_TONE_GENERATOR_H */

