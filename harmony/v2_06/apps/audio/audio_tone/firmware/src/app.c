/*******************************************************************************
  AUDIO TONE DEMO  
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Created By: c16825 (CAL)

  Date: 08/11/2015

  Summary:
    Contains the functional implementation of this demo application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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


// Note: 565 background color used in app schemes (3,11,7) is 24,44,56 in 
// Windows 888 space

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

//Generated include
#include "app.h"

//******************************************************************************
// Application Include Files
//******************************************************************************
#include "app_config.h"  //Non generated data

#include <string.h> //memcpy
#include <math.h>   //sin
#include <stdlib.h> //malloc

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

//Initial Audio Generator Parameters
AUDIO_GEN_PARAM agParamInit = {.sampleRate  = 48000,  
                             //.timeDeltaMs = 1000,   //1 sec
                               .timeDeltaMs = -1,     //inf
                               //.ampExpFS    = 0,      //Tone ampExpFS
                               .fHz1        = 1000,    //tone or chirp1
                               .fHz2        = 2000,   //chirp2
                               .sigma       = 0 };    //noise power

//Initial GUI Display Settings (should match the audio generation parameters)
GUI_DATA guiDataInit = { .displayUpdate = true,
                         .mode        = MODE_TONE,
                         .select      = SELECT_F1,
                         .f1Hz        = 1000,
                         .f2Hz        = 2000,
                         .timeDeltaMs = -1,  //inf   
                         .durationSamples = -1, //inf
                         .progress    = -1,  //No progress bar
                         .onOff       = false,      // start off with tone off
                         .changeToneMode = true };


//******************************************************************************
// Audio Play Buffer Queue implementation
//******************************************************************************

// TODO: Put generation, queue and buffering into the appData object,
//       rather than globals.  Use dynamic allocation of buffer objects. CAL 

//Application globals
AUDIO_PLAY_BUFFER audioPlayBuffer[NUM_PLAY_BUFFERS];
AUDIO_PLAY_BUFFER * currAudioPlayBuffer = NULL;

PlayBufferQueue       queue;  //Buffer queue
AUDIO_GENERATE           ag;  //Instance class to generate a signal

//GENERATED: Global data for APP_Tasks polling loop:  MHC Harmony 1.0.5
APP_DATA appData;


//******************************************************************************
//******************************************************************************
//  APP File Scope Function Prototypes
//******************************************************************************
//******************************************************************************

//******************************************************************************
//******************************************************************************
//  APP Function Implementatation
//******************************************************************************
//******************************************************************************

// TODO: Abstract this periodic timer.  
//       Also add a one-shot repeatable timer. CAL
//******************************************************************************
// APP Periodic Timer - Handler
//
// NOTE: Used to check the pushbutton every 100ms interval. 
//
//******************************************************************************
uint32_t count = 0;

void APP_handlePeriodicTimerSignal(uintptr_t context, uint32_t alarmCount)
{
    //Only counts how long the button is pressed down.
    //NOTE: Should have exception check.
    if (mRepeatButton != INVALID_BUTTON)
    {
        mRepeatCount++;
        APP_OnButtonEvent(&appData, mRepeatButton, true, mRepeatCount);
    }
}

//******************************************************************************
// APP Periodic Timer - Init 100ms timer 
//******************************************************************************
void APP_PeriodicTimerInit()
{
/* Open the repeat timer driver */
    if (SYS_STATUS_READY == DRV_TMR_Status(sysObj.drvTmr2))
    {
        appData.repeatTmrHandle = DRV_TMR_Open(DRV_TMR_INDEX_2,
                                               DRV_IO_INTENT_EXCLUSIVE);
        if(DRV_HANDLE_INVALID == appData.repeatTmrHandle ||
                (DRV_HANDLE) NULL == appData.repeatTmrHandle)
        {
           SYS_DEBUG(0, "Timer DRV_TMR_Open Error");
        }
        else
        {
            DRV_TMR_AlarmRegister (appData.repeatTmrHandle,
                                   APP_REPEAT_TIMER_PERIOD, true,                    
                                   (uintptr_t)0, 
                                   &APP_handlePeriodicTimerSignal);
            DRV_TMR_Start (appData.repeatTmrHandle);
        }
    }
    else
    {
//        SYS_DEBUG(0, "Timer Driver Not Ready");
    }
}


//******************************************************************************
// APP_Initialize() - Application Initialize. 
//
// NOTE: Called from the SYS_Initialized() function.
//******************************************************************************
void APP_Initialize (void)
{
    //APP: Audio Output
    AUDIO_CODEC_Initialize(&(appData.codecData));
    
    //APP: Push-Buttons
    APP_ButtonInit();
    APP_PeriodicTimerInit();
    
    //APP: Tone Generation/Play buffer queue
    PlayBufferQueueInit(&queue, audioPlayBuffer);

    /* MHC GENERATED: Place the App state machine in its initial state. */
    //appData.state = APP_STATE_INIT;
    
    //APP_Tasks: Initial State
    appData.state = APP_STATE_CODEC_OPEN;

    //Audio Generation
    appData.agStatus.processingBuffer  = false;
    appData.agStatus.currentToneType = AUDIO_GEN_SILENCE;
    appData.agStatus.nextToneType    = AUDIO_GEN_SILENCE;
    appData.agStatus.changeToneType  = false;
    appData.agStatus.onOff  = false;

    //Display
    appData.guiData = guiDataInit;

    //ADC Volume Control - bt_audio_dk
#ifdef HAS_VOLUME_CTRL   
    APP_VolumeInitialize(); //Poll volume control value.
#endif    
    APP_LED1_ON();      // indicate in mode 1

} //End APP_Init()


//******************************************************************************
//
//  APP_audioGenSetNextToneType() - Change the tone type, without changing
//                                  any other parameter.
//
//  NOTE: Used to change generator output for next play buffer.
//
//******************************************************************************
void APP_audioGenSetNextToneType(AUDIO_GENERATE_STATUS * agStatus, 
                                 AUDIO_GEN_TYPE toneType, 
                                 AUDIO_GEN_PARAM nextAgParam,
                                 bool onOff)
{
   //Values to use for the next agStatus->changeToneType.
   agStatus->onOff = onOff;
   agStatus->nextToneType   = toneType;
   agStatus->nextToneParam  = nextAgParam;
   agStatus->changeToneType = true;
}


//******************************************************************************
// APP_Tasks() - Application tasks routine. 
//
// NOTE: Implements the application state machine.
//
//******************************************************************************
void APP_Tasks(void )
{
    AUDIO_GENERATE_STATUS * agStatus = &(appData.agStatus);
    GUI_DATA * guiData = &(appData.guiData);
    
    //Check for button push (SW1 ... SW5) UP/DOWN Changes
    APP_ButtonTask();
    
    //Poll volume control to set CODEC output volume
#ifdef HAS_VOLUME_CTRL
    APP_VolumeTasks();
#endif    
    //GFX_Status (gfxObject) == SYS_STATUS_READY
//    if (hgcObj.screenState == HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen)
    if (appData.state >= APP_STATE_CODEC_SET_BUFFER_HANDLER)
    {   
      APP_DisplayTask(guiData);     
    }
    
    //Tone Task
    switch (appData.state)
    {

        //Initial state
        case APP_STATE_CODEC_OPEN:
        {
            BOOL audioGenEnd;
            if ( Audio_Codec_Open(&(appData.codecData)) )
            {
                //Set Sampling Rate from CODEC driver value
                agParamInit.sampleRate = 
                     Audio_Codec_SamplingRateGet(&(appData.codecData));

                //Initialize audio generation status
                agStatus->currentToneType  = AUDIO_GEN_TONE;
                agStatus->nextToneType     = AUDIO_GEN_TONE;
                agStatus->currentToneParam = agParamInit;
                agStatus->nextToneParam    = agParamInit;
                agStatus->changeToneType   = false;
                AudioGenerateInit(&ag, AUDIO_GEN_TONE, agParamInit);

                agStatus->fMinHz = AudioGenerateFminHz(&ag);
                agStatus->fMaxHz = AudioGenerateFmaxHz(&ag);
                agStatus->onOff = false;
#ifndef HAS_VOLUME_CTRL
                appData.guiData.volume = (100*(int16_t)Audio_Codec_VolumeGet(&(appData.codecData)))/255;
#endif                
                APP_DisplayInit(guiData, agStatus);             

                do
                {
                    //Next buffer to fill with audio
                    currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
                    if (currAudioPlayBuffer != NULL)
                    {
                        //Audio Generate
                        agStatus->processingBuffer = true;
                        audioGenEnd = AudioGenerateFx(&ag, currAudioPlayBuffer);
                        PlayBufferQueueAdd(&queue); //Add to tail
                        currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
                        agStatus->processingBuffer = false;
                        if (audioGenEnd)
                        {
                            agStatus->onOff = false;
                            agStatus->currentToneType = AUDIO_GEN_SILENCE;

                            guiData->onOff = false;
                            guiData->changeToneMode = true;
                            guiData->displayUpdate  = true;
                        }
                    }
                    
                } while (currAudioPlayBuffer != NULL);

                //Next State
                appData.state = APP_STATE_CODEC_SET_BUFFER_HANDLER;

            } //End CODEC Client Handle 
        }
        break;

        /* Set a handler for the audio buffer completion event */
        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
        {
            //Set Buffer Handler
            Audio_Codec_SetBufferEventHandler(&(appData.codecData));

            if (agStatus->onOff)
            {
                appData.state = APP_STATE_CODEC_ADD_BUFFER;   // o.w. wait for button press
            }
        }
        break;

        // Add an audio buffer to the codec driver queue to be transmittted */
        // NOTE: Initial buffer add of head buffer to CODEC output queue.
        case APP_STATE_CODEC_ADD_BUFFER:
        {
            //uint8_t index;
            bool isInvalidHandle;
            //int8_t queueIdx = 0;

            isInvalidHandle = false;

            //Get the next output play buffer
            //for(index=0; index < (APP_CODEC_WRITE_QUEUE_SIZE - 1); index++)
            currAudioPlayBuffer = PlayBufferQueueGetHead(&queue);

            if (currAudioPlayBuffer == NULL) 
            {
                SYS_DEBUG(0, "Play buffer empty! \r\n");
            }

            //Add the play buffer to the output queue 
            //
            // This function schedules a non-blocking write operation. 
            // -->The function returns with a valid buffer handle in the 
            // bufferHandle argument if the write request was scheduled 
            // successfully. 
            // -->The function adds the request to 
            // the hardware instance transmit queue and returns immediately.
            //
            // NOTE: While the request is in the msg queue, 
            //       the application buffer is owned by the driver and 
            //       should not be modified.
            appData.codecData.codecClient.txbufferObject = 
                    (uint8_t *) currAudioPlayBuffer->data;
            appData.codecData.codecClient.bufferSize =
                    currAudioPlayBuffer->dataLen*sizeof(APP_DATA_TYPE);
            //if ( !Audio_Codec_Addbuffer(&(appData.codecData),  
            //                           (int8_t *) currAudioPlayBuffer->data,
            //                           currAudioPlayBuffer->dataLen*sizeof(APP_DATA_TYPE)))
            if ( Audio_Codec_Addbuffer(&(appData.codecData),  
                        appData.codecData.codecClient.txbufferObject,
                        (size_t) appData.codecData.codecClient.bufferSize) )
            {
                isInvalidHandle = true;
                SYS_DEBUG(0, "Find out whats wrong \r\n");
            }

            if (isInvalidHandle == false)
            {
                //Next State
                appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
            }

        }
        break;

        //Wait for buffer to complete transmission
        // 1) Check if generator mode/params have changed
        // 2) Fill audio buffer queue
        // 3) Update display
        case APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE:
        {

            /* Audio data Transmission under process */
            //asm("nop");
            // Event head buffer played
            // -->and was removed from the play buffer queue.

            //Fill the queue at the tail.
            currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
            if (currAudioPlayBuffer != NULL)
            {
                BOOL audioGenEnd;
                if (agStatus->changeToneType)
                {
                    //Mode button AG reinitialize
                   
                    if (agStatus->onOff) 
                    {
                        //Initialize using APP AG Status nextTone<Param,type>
                        //agParamInit.sampleRate  = nextToneParam.sampleRate,  
                        //agParamInit.ampExpFS    = 0; //Tone ampExpFS
                        agParamInit.fHz1        = agStatus->nextToneParam.fHz1;    //tone or chirp1
                        agParamInit.fHz2        = agStatus->nextToneParam.fHz2;    //chirp2
                        agParamInit.timeDeltaMs = agStatus->nextToneParam.timeDeltaMs; 
                        //agParamInit.sigma       = nextToneParam.sigma;

	                     //New Waveform
                        AudioGenerateInit(&ag, appData.agStatus.nextToneType, 
                                          agParamInit);
                    }
                    else
                    {
                        //bool flag = SYS_INT_Disable();
                        //PlayBufferQueueClear(&queue);
                        //if (flag) SYS_INT_Enable();
                        
                        //Silence generated until onOff button initiate
                        //NOTE: Parameters are already set for the tone type
                        //      when it turns back on, but can be updated
                        //      using the parameter adjustments.
                        //agParamInit.sampleRate  = nextToneParam.sampleRate,  
                        //agParamInit.ampExpFS    = 0; //Tone ampExpFS
                        agParamInit.fHz1        = agStatus->nextToneParam.fHz1;    //tone or chirp1
                        agParamInit.fHz2        = agStatus->nextToneParam.fHz2;    //chirp2
                        agParamInit.timeDeltaMs = -1;  //inf
               			//agParamInit.sigma       = nextToneParam.sigma;
                        AudioGenerateInit(&ag, AUDIO_GEN_SILENCE, agParamInit);
                    }

                    //New APP AG Status/Display Update
                    agStatus->changeToneType  = false;
                    agStatus->currentToneType = appData.agStatus.nextToneType;
                    agStatus->currentToneParam = agParamInit;

                } //End changeToneType

                //Audio Buffer Generation
                appData.agStatus.processingBuffer = true;
                audioGenEnd = AudioGenerateFx(&ag, currAudioPlayBuffer);
                appData.agStatus.processingBuffer = false;

                if (audioGenEnd)
                {
                    //End of waveform - One-Shot
                    agStatus->changeToneType = true;
                    agStatus->onOff = false;

                    guiData->onOff  = false;
                    guiData->changeToneMode = true;
                    guiData->displayUpdate  = true;
                }

                //Add buffer to play queue.
                //bool flag = SYS_INT_Disable();
                if (currAudioPlayBuffer->dataLen > 0) PlayBufferQueueAdd(&queue);
                //if (flag) SYS_INT_Enable(); 
            }

            //Next State
            //appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
            //appData.state = APP_STATE_DISPLAY
        }
        break; //APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE


        // Audio data Transmission complete */
        // NOTE:  Only entered from SPI/I2S DMA ISR Buffer Complete.
        case APP_STATE_CODEC_BUFFER_COMPLETE:
        {
            //NOTE: SPI/I2S DMA Buffer Complete interrrupt 
            //      causes this state.
            
            //Head buffer played.
            // 1) Remove head buffer
            // 2) Play next buffer (new head buffer)
	         //bool flag = SYS_INT_Disable();
            PlayBufferQueueRemove(&queue);
            currAudioPlayBuffer = PlayBufferQueueGetHead(&queue);
            //if (flag) SYS_INT_Enable(); 

            if (currAudioPlayBuffer == NULL) 
            {
                //Under-run of queue by AudioGenerateFx 
                SYS_DEBUG(0, "Play buffer empty! \r\n");
                appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
            }
            else
            {
                appData.codecData.codecClient.txbufferObject = 
                        (uint8_t *) currAudioPlayBuffer->data;
                appData.codecData.codecClient.bufferSize =
                    (size_t) currAudioPlayBuffer->dataLen*sizeof(APP_DATA_TYPE);
                        
                if ( Audio_Codec_Addbuffer(&(appData.codecData),  
                            appData.codecData.codecClient.txbufferObject,
                            appData.codecData.codecClient.bufferSize) )
                {
                    //Next State
                    appData.state = APP_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
                }
                else
                {
                    SYS_DEBUG(0, "Find out whats wrong! \r\n");
                }
            }
        }
        break; //APP_STATE_CODEC_BUFFER_COMPLETE   

        default:
        {
            asm("nop");
        }
        break;
    }

} //End APP_Tasks()


//******************************************************************************
// 
// APP_CodecTxBufferComplete() - Set APP_Tasks Next state to buffer complete.
//
// NOTE: Called from APP_CODECBufferEventHandler().
//
// TODO: Put the appData instance pointer in the AUDIO_CODEC_DATA instance on
//       initialization of codecData and let the audio_codec instance change
//       the codec data and state. CAL
//
//******************************************************************************
void APP_CodecTxBufferComplete()
{
    //Next State -- after the buffer complete interrupt.
    appData.state = APP_STATE_CODEC_BUFFER_COMPLETE;
}


//******************************************************************************
// APP_CodecCommandClear() - Clears the CODEC sampling rate command.
//
// NOTE: Called from APP_CODECCommandEventHandler()
//
//******************************************************************************
void APP_CodecCommandClear()
{
    // SampleRate command executed successfully
    if(appData.codecData.codecClient.currentCommand == CODEC_COMMAND_SAMPLING_RATE_SET)
    {
        appData.codecData.codecClient.currentCommand = CODEC_COMMAND_NONE;
    }
}

//******************************************************************************
// End of File
