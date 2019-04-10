/*******************************************************************************
  Audio Microphone storage

  Company:
    Microchip Technology Inc.

  File Name:
    app_freq_spectrum_task.h

  Summary:
    This header file provides prototypes and definitions for the frequency 
    spectrum application.

  Description:
    This header file provides prototypes and definitions for the frequency 
    spectrum application.
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

#ifndef APP_FREQ_SPECTRUM_H
#define	APP_FREQ_SPECTRUM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define MAX_FREQ_SPECTRUM_BANDS     24
#define NUM_POINT_FFT               4096
#define RECT_WINDOW_COMPN           (float)1.0
#define HANN_WINDOW_COMPN           (float)1.63
#define BLACKMAN_WINDOW_COMPN       (float)1.97
#define HAMMING_WINDOW_COMPN        (float)1.59
    
#define FREQ_SPECTRUM_MAX_LISTNERS  1    
    
typedef enum
{
    FREQ_SPECTRUM_EVENT_TYPE_FFT_RESULTS_READY = 0,
}FREQ_SPECTRUM_EVENT_TYPE;

typedef void (*WINDOW_INIT_FUNC)(int16_t* outVector, int N);
typedef void (*WINDOW_FUNC)(int16_t* outVector, int16_t* inVector, int N);
typedef void (*FFT_SPECTRUM_EVENT_CALLBACK) (FREQ_SPECTRUM_EVENT_TYPE eventType, const void* const pEventData);

typedef enum
{
    SAMPLING_FREQ_48_KHZ = 0,
    SAMPLING_FREQ_44_1_KHZ,
    SAMPLING_FREQ_MAX,
}SAMPLING_FREQ;

typedef struct
{
    uint16_t            centerFreq;
    uint16_t            sampleLow;
	uint16_t            sampleHigh;
}BAND_SAMPLE_VALUES;

typedef struct
{	
	BAND_SAMPLE_VALUES   band[MAX_FREQ_SPECTRUM_BANDS];
}OCTAVE_TABLE;

typedef struct
{
	float Re;
	float Im;
}complex;

typedef enum
{
    AUDIO_CHANNEL_LEFT = 0,
    AUDIO_CHANNEL_RIGHT,
}AUDIO_CHANNEL;

typedef enum
{
    WINDOW_TYPE_RECT = 0,
    WINDOW_TYPE_HANN,
    WINDOW_TYPE_BLACKMAN,
    WINDOW_TYPE_HAMMING,
    WINDOW_TYPE_MAX,
}WINDOW_TYPE;

typedef struct
{
    WINDOW_INIT_FUNC    init;
    WINDOW_FUNC         func;
    float               compFactor;
    bool                isWindowComputed;
}WINDOW_FUNCTION;

typedef enum
{
    FREQ_SPECTRUM_STATE_INIT = 0,
    FREQ_SPECTRUM_STATE_WAIT_FOR_INPUT,
    FREQ_SPECTRUM_STATE_WINDOW_INPUT,
    FREQ_SPECTRUM_STATE_PREPARE_INPUT,
    FREQ_SPECTRUM_STATE_CALC_FFT,
    FREQ_SPECTRUM_STATE_CALC_POWERS,
    FREQ_SPECTRUM_STATE_FFT_DONE,
}FREQ_SPECTRUM_STATE;

typedef struct
{
    WINDOW_TYPE                     selectedWinType;
    AUDIO_CHANNEL                   channel;
}FFT_CONFIG;

typedef enum
{
    AUDIO_TYPE_STEREO = 0,
    AUDIO_TYPE_MONO,
}AUDIO_TYPE;

typedef struct
{
    AUDIO_TYPE                      audioType;
    SAMPLING_FREQ                   samplingFreq;
}AUDIO_INPUT_CONFIG;

typedef struct
{
    FREQ_SPECTRUM_STATE             state;
    bool                            isAudioSamplesAvailable;
    volatile const int16_t*         pAudioData;
    AUDIO_INPUT_CONFIG              audioInputConfig;
    FFT_CONFIG                      fftConfig;
    uint8_t                         spectrumValue[MAX_FREQ_SPECTRUM_BANDS];
    FFT_SPECTRUM_EVENT_CALLBACK     eventListners[FREQ_SPECTRUM_MAX_LISTNERS];
    int32_t                         lowDBFSLimit;
    int32_t                         highDBFSLimit;
    uint32_t                        signalFSValue;
}APP_FREQ_SPECTRUM_TASK_DATA;


/*----------------------------------------------------------------------------*/

void APP_FREQ_SPECTRUM_TaskInitialize(void);
void APP_FREQ_SPECTRUM_Task(void);
bool APP_FREQ_SPECTRUM_RegisterCallback(FFT_SPECTRUM_EVENT_CALLBACK evHandler);
bool APP_FREQ_SPECTRUM_UnRegisterCallback(FFT_SPECTRUM_EVENT_CALLBACK evHandler);
void APP_FREQ_SPECTRUM_ConfigInputAudio(AUDIO_TYPE audioType, SAMPLING_FREQ sampFreq);
uint8_t APP_FREQ_SPECTRUM_GetSpectrumValueAtFreq(uint8_t freq);
void APP_FREQ_SPECTRUM_SetWindowType(WINDOW_TYPE winType);



#ifdef	__cplusplus
}
#endif

#endif	/* APP_FREQ_SPECTRUM_H */

