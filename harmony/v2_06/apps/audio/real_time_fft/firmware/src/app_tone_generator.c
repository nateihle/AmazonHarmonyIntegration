/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_tone_generator.c

  Summary:
    Contains the functional implementation of the tone generator application 
    task.

  Description:
    This file contains the functional implementation of the tone generator
    application. The tone generator application generates tone based on the 
    request received from the speaker application.
 *******************************************************************************/

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_tone_generator.h"
#include <math.h>

static APP_TONE_TASK_DATA   appToneGeneratorData;


static uint32_t APP_TONE_GetdBFSToFS(int32_t dBFSValue)
{
    float value = (float)dBFSValue/20.0;
    
    value = powf((float)10.0, (float)value);
    
    return (SIGNAL_FS_VALUE * value);
    
}
int16_t* APP_TONE_GetMixedSignal(void)
{
    return appToneGeneratorData.mixedSignal;
}

uint32_t APP_TONE_GetNumSamples(void)
{
    return MAX_SAMPLES_PER_CHANNEL;
}

void APP_TONE_GetFreqRangeInHz(
    uint32_t* const pLowFreqLim, 
    uint32_t* const pHighFreqLim
)
{
    *pHighFreqLim = (uint32_t)MAX_FREQ_LIMIT;
    *pLowFreqLim  = (uint32_t)MIN_FREQ_LIMIT;
}

void APP_TONE_GetAmplitudeRagneInDBFS(
    int32_t* const pLowLim, 
    int32_t* const pHighLim
)
{
    *pLowLim = (int32_t)MIN_DBFS_LIMIT;
    *pHighLim = (int32_t)MAX_DBFS_LIMIT;
}

uint32_t APP_TONE_GetFullScaleSignalValue(void)
{
    return SIGNAL_FS_VALUE;
}

void APP_TONE_ReStartSignal(void)
{
    //appToneGeneratorData.sigSample = 0;
    appToneGeneratorData.toneConfig.elapsedDuration = 0;
}

void APP_TONE_SetSamplingFreq(uint32_t samplingFreq)
{
    appToneGeneratorData.toneConfig.samplingFreq = samplingFreq;
}

uint32_t APP_TONE_GetSamplingFreq(void)
{
    return appToneGeneratorData.toneConfig.samplingFreq;
}

void APP_TONE_SetSignalFreq(uint8_t sigIndex, uint32_t freqHz)
{
    if (sigIndex < appToneGeneratorData.toneConfig.nTones)
    {
        appToneGeneratorData.toneConfig.toneParams[sigIndex].toneFreqHz = freqHz;
    }    
}

void APP_TONE_SetSignalAmplitude(uint8_t sigIndex, int32_t amplitudeIndBFS)
{
    uint32_t amplitude;
    
    if (sigIndex < appToneGeneratorData.toneConfig.nTones)
    {
        amplitude = APP_TONE_GetdBFSToFS(amplitudeIndBFS);
        appToneGeneratorData.toneConfig.toneParams[sigIndex].amplitude = amplitude;
    }
}

void APP_TONE_StartSignalGeneration(uint32_t duration)
{
    if (duration <= MAX_DURATION_IN_SECS)
    {
        appToneGeneratorData.isToneGenRequested = true;
        appToneGeneratorData.toneConfig.duration = duration;
        appToneGeneratorData.toneConfig.elapsedDuration = 0;
        appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_WAIT_FOR_REQUEST;
    }
}

void APP_TONE_GenSignalData(void)
{
    appToneGeneratorData.isDataRequested = true;
}

static bool APP_TONE_IsSigGenRequested(void)
{
    bool isRequested = false;
    
    if (true == appToneGeneratorData.isToneGenRequested) 
    {
        appToneGeneratorData.isToneGenRequested = false;
        isRequested = true;
    }
    
    return isRequested;
}

static bool APP_TONE_IsDataRequested(void)
{
    bool isRequested = false;
    
    if (true == appToneGeneratorData.isDataRequested) 
    {
        appToneGeneratorData.isDataRequested = false;
        isRequested = true;
    }
    
    return isRequested;
}

static void APP_TONE_GenerateMixedSignal(void)
{
	uint32_t nTones = 0;
    uint32_t k = 0;
    long double sampleVal = 0.0;   	
    APP_TONE_CONFIG* const pSigConfig = &appToneGeneratorData.toneConfig;

	for (nTones = 0; nTones < pSigConfig->nTones; nTones++)
	{
		pSigConfig->toneParams[nTones].degreePerStep = 
            (2.0 * (float)M_PI * (pSigConfig->toneParams[nTones].toneFreqHz/(float)pSigConfig->samplingFreq));		
	}
    
    for (k = 0; k < MAX_SAMPLES_PER_CHANNEL; k++, appToneGeneratorData.sigSample++)
    {
        for (nTones = 0; nTones < pSigConfig->nTones; nTones++)
        {				
			sampleVal += ((long double)pSigConfig->toneParams[nTones].amplitude * 
                sinl((long double)appToneGeneratorData.sigSample * (long double)pSigConfig->toneParams[nTones].degreePerStep));                        
		}
        
		appToneGeneratorData.mixedSignal[k] = sampleVal;                
		sampleVal = 0.0;        
	}
}

static void APP_TONE_InitEvListners(void)
{
    uint8_t i;
    
    for (i = 0; i < APP_TONE_MAX_EVENT_LISTNERS; i++)
    {
        appToneGeneratorData.eventListners[i] = NULL;
    }
}

bool APP_TONE_RegisterCallback(APP_TONE_GENERATOR_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    uint8_t i;
    
    if (evHandler)
    {
        for (i = 0; i < APP_TONE_MAX_EVENT_LISTNERS; i++)
        {
            if (NULL == appToneGeneratorData.eventListners[i])
            {
                appToneGeneratorData.eventListners[i] = evHandler;
                isSuccess = true;
                break;
            }
        }
    }           
    
    return isSuccess;
}

bool APP_TONE_UnRegisterCallback(APP_TONE_GENERATOR_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    uint8_t i;
    
    if (evHandler)
    {
        for (i = 0; i < APP_TONE_MAX_EVENT_LISTNERS; i++)
        {
            if (appToneGeneratorData.eventListners[i] == evHandler)
            {
                appToneGeneratorData.eventListners[i] = NULL;
                isSuccess = true;
                break;
            }
        }
    }
    
    return isSuccess;
}

void APP_TONE_NotifyListners(
    APP_TONE_GENERATOR_EVENT_TYPE event,
    const void* const pEventData
)
{
    uint8_t i;
    
    for (i = 0; i < APP_TONE_MAX_EVENT_LISTNERS; i++)
    {
        if (appToneGeneratorData.eventListners[i])
        {
            appToneGeneratorData.eventListners[i](event, pEventData);
        }
    }    
}

void APP_TONE_TaskInitialize(void)
{    
    appToneGeneratorData.toneConfig.nTones = 3;
    appToneGeneratorData.toneConfig.samplingFreq = SAMPLING_FREQUENCY;
    appToneGeneratorData.toneConfig.duration = MAX_DURATION_IN_SECS;
    appToneGeneratorData.toneConfig.elapsedDuration = 0;
    
    appToneGeneratorData.toneConfig.toneParams[0].amplitude = 8000;
    appToneGeneratorData.toneConfig.toneParams[0].toneFreqHz = 12000;
    
    appToneGeneratorData.toneConfig.toneParams[1].amplitude = 8000;
    appToneGeneratorData.toneConfig.toneParams[1].toneFreqHz = 400;
    
    appToneGeneratorData.toneConfig.toneParams[2].amplitude = 8000;
    appToneGeneratorData.toneConfig.toneParams[2].toneFreqHz = 5000;
    
    appToneGeneratorData.isToneGenRequested = false;
    appToneGeneratorData.isDataRequested = false;
    appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_WAIT_FOR_REQUEST;
    
    APP_TONE_InitEvListners();
    
    
}

void APP_TONE_Task(void)
{
    switch(appToneGeneratorData.state)
    {
        case APP_TONE_GENERATOR_STATE_WAIT_FOR_REQUEST:
            if (true == APP_TONE_IsSigGenRequested())
            {                
                appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_GEN_SIGNAL;
            }
            break;
        case APP_TONE_GENERATOR_STATE_GEN_SIGNAL:
            if (true == APP_TONE_IsDataRequested())
            {
                if (appToneGeneratorData.toneConfig.elapsedDuration < 
                        (uint32_t)(((float)appToneGeneratorData.toneConfig.samplingFreq/MAX_SAMPLES_PER_CHANNEL)*(float)appToneGeneratorData.toneConfig.duration))
                {                 
                    APP_TONE_GenerateMixedSignal();                 
                    appToneGeneratorData.toneConfig.elapsedDuration++;
                    appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_NOTIFY_SIG_READY;
                }
                else
                {
                    APP_TONE_NotifyListners(
                        APP_TONE_GENERATOR_EVENT_TYPE_SIG_DURATION_ELAPSED, 
                        appToneGeneratorData.mixedSignal
                    );
                                        
                    appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_WAIT_FOR_REQUEST;
                }
            }
            break;
        case APP_TONE_GENERATOR_STATE_NOTIFY_SIG_READY:                    
            
            APP_TONE_NotifyListners(
                APP_TONE_GENERATOR_EVENT_TYPE_SIG_READY, 
                appToneGeneratorData.mixedSignal
            );

            appToneGeneratorData.state = APP_TONE_GENERATOR_STATE_GEN_SIGNAL;
                        
            break;
        default:
            
            break;
    }
}