/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_display.c

  Summary:
    This file contains the functional implementation of the display task.   

  Description:
    This file contains the functional implementation of the display task. In 
    It implements the user interface and passes the requests to the appropriate
    consumer of the requests.
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

#include "app_display_task.h"
#include "app_freq_spectrum_task.h"
#include "gfx/libaria/libaria_init.h"
#include "app_microphone_task.h"
#include "app_speaker_task.h"
#include "app_tone_generator.h"
#include "app_graph.h"
#include <math.h>

typedef struct
{
    uint32_t x_pos;
    uint32_t y_pos;
    uint32_t x_size;
    uint32_t y_size;
}cordinates;

static APP_DISPLAY_DATA     appDisplayData = {{0}, NULL};
static laProgressBarWidget* ProgressBarWidget[MAX_FREQ_SPECTRUM_BANDS];

static void APP_DISPLAY_InitFreqSpectrumBars(void)
{
    ProgressBarWidget[0] = ProgressBarWidget1;
    ProgressBarWidget[1] = ProgressBarWidget2;
    ProgressBarWidget[2] = ProgressBarWidget3;
    ProgressBarWidget[3] = ProgressBarWidget4;
    ProgressBarWidget[4] = ProgressBarWidget5;
    ProgressBarWidget[5] = ProgressBarWidget6;
    ProgressBarWidget[6] = ProgressBarWidget7;
    ProgressBarWidget[7] = ProgressBarWidget8;
    ProgressBarWidget[8] = ProgressBarWidget9;
    ProgressBarWidget[9] = ProgressBarWidget10;
    ProgressBarWidget[10] = ProgressBarWidget11;
    ProgressBarWidget[11] = ProgressBarWidget12;
    ProgressBarWidget[12] = ProgressBarWidget13;
    ProgressBarWidget[13] = ProgressBarWidget14;
    ProgressBarWidget[14] = ProgressBarWidget15;
    ProgressBarWidget[15] = ProgressBarWidget16;
    ProgressBarWidget[16] = ProgressBarWidget17;
    ProgressBarWidget[17] = ProgressBarWidget18;
    ProgressBarWidget[18] = ProgressBarWidget19;
    ProgressBarWidget[19] = ProgressBarWidget20;
    ProgressBarWidget[20] = ProgressBarWidget21;
    ProgressBarWidget[21] = ProgressBarWidget22;
    ProgressBarWidget[22] = ProgressBarWidget23;
    ProgressBarWidget[23] = ProgressBarWidget24;
}

static void APP_DISPLAY_UpdateFreqSpectrum(void)
{   
    uint32_t i;    
    uint8_t value;
    
    if (PLAY_BUTTON_STATE_STOPPED == appDisplayData.playButtonState &&
            APP_DISPLAY_UI_ELEMENTS_TONE_BTN == appDisplayData.mode)
    {
        return;
    }
    
    for(i = 0; i < MAX_FREQ_SPECTRUM_BANDS; i++)
    {
        value = APP_FREQ_SPECTRUM_GetSpectrumValueAtFreq(i);
        if (0 == value)
        {
            value = 1;
        }
        laProgressBarWidget_SetValue(ProgressBarWidget[i], value);        
    }       
}

static void APP_DISPLAY_ClearFreqSpectrum(void)
{
    uint32_t i;
    
    for(i = 0; i < MAX_FREQ_SPECTRUM_BANDS; i++)
    {        
        laProgressBarWidget_SetValue(ProgressBarWidget[i], 1);        
    }    
}

bool APP_DISPLAY_RegisterCallback(APP_DISPLAY_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    
    if (evHandler)
    {
        appDisplayData.eventsCallback = evHandler;
        isSuccess = true;
    }
    
    return isSuccess;
}

bool APP_DISPLAY_AddCommand(APP_DISPLAY_CMD cmd)
{
    bool isSuccess = false;
    
    if (cmd < APP_DISPLAY_CMD_MAX)
    {
        if (true == APP_QUEUE_Push(appDisplayData.queueHandle, &cmd))
        {
            isSuccess = true;
        }
    }
    
    return isSuccess;
}

void APP_DISPLAY_FFTResultsEvHandler (
    FREQ_SPECTRUM_EVENT_TYPE eventType, 
    const void* const pEventData
)
{
    switch(eventType)
    {
        case FREQ_SPECTRUM_EVENT_TYPE_FFT_RESULTS_READY:
            APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UPDATE_FREQ_SPECTRUM);
            break;
        default:
            break;
    }    
}

void APP_DISPLAY_SpeakerEventHandler(
    APP_SPEAKER_EVENT event, 
    const void* const pEventData
)
{
    switch(event)
    {
        case APP_SPEAKER_EVENT_STOPPED:
            appDisplayData.playButtonState = PLAY_BUTTON_STATE_STOPPED;            
            if (APP_DISPLAY_ACTIVE_SCREEN_MAIN == appDisplayData.activeScreen)
            {
                laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));
            }
            break;
            
        default:
            break;
    }
}

static void APP_DISPLAY_EnableToneModeWidgets(laBool isEnable)
{             
    laWidget_SetVisible((laWidget*)SelectedSignalsPanel, isEnable );      
    laWidget_SetVisible((laWidget*)VolumeSliderPanel, isEnable );  
    laWidget_SetVisible((laWidget*)PlayButtonPanel, isEnable );
    laWidget_SetVisible((laWidget*)SignalSelectionPanel, isEnable );
    laWidget_SetVisible((laWidget*)WindowFuncSelectionPanel, isEnable );
                       
    laWidget_SetVisible((laWidget*)TimeDomainScreenButton, isEnable );                                   
}

static void APP_DISPLAY_VolumeSliderChanged(void)
{
    uint8_t volumeLevel;
    
    volumeLevel = laSliderWidget_GetSliderValue((laSliderWidget*)VolumeControl);
    
    if (appDisplayData.eventsCallback)
    {
        appDisplayData.eventsCallback(
            APP_DISPLAY_EVENT_TYPE_VOL_CHANGE_REQ,
            (void*)&volumeLevel
        );
    }
}

static void APP_DISPLAY_UpdateSignalParams(void)
{    
    APP_TONE_ReStartSignal();
    
    APP_TONE_SetSignalFreq(0, appDisplayData.toneParams[0].freqValue);
    APP_TONE_SetSignalFreq(1, appDisplayData.toneParams[1].freqValue);
    APP_TONE_SetSignalFreq(2, appDisplayData.toneParams[2].freqValue);
    
    APP_TONE_SetSignalAmplitude(0, appDisplayData.toneParams[0].dBFSValue);
    APP_TONE_SetSignalAmplitude(1, appDisplayData.toneParams[1].dBFSValue);
    APP_TONE_SetSignalAmplitude(2, appDisplayData.toneParams[2].dBFSValue);
}


static void APP_DISPLAY_PlayButtonPressed(void)
{    
    if (PLAY_BUTTON_STATE_PLAYING == appDisplayData.playButtonState)
    {
        appDisplayData.playButtonState = PLAY_BUTTON_STATE_STOPPED;
        laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));
        APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_STOP);
        APP_DISPLAY_ClearFreqSpectrum();
    }
    else
    {
        appDisplayData.playButtonState = PLAY_BUTTON_STATE_PLAYING;
        laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Stop));
        APP_DISPLAY_UpdateSignalParams();
        APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_START);
    }            
}

static void APP_DISPLAY_ModeButtonPressed(APP_DISPLAY_UI_ELEMENTS modeButton)
{    
    if (appDisplayData.mode != modeButton)
    {
        // Currently in MIC mode then...
        if (APP_DISPLAY_UI_ELEMENTS_MIC_BTN == appDisplayData.mode)
        {            
            laWidget_SetScheme((laWidget*)MicButton, &ButtonSchemeInActive);
            laWidget_SetScheme((laWidget*)ToneButton, &ButtonSchemeActive);
            APP_DISPLAY_EnableToneModeWidgets(LA_TRUE);                       
            
            if (WINDOW_TYPE_HANN == appDisplayData.activeWindowSelected)
            {
                laRadioButtonWidget_SetSelected(HannWindow_RadioButton);
            }
            else
            {
                laRadioButtonWidget_SetSelected(BlackmanWindow_RadioButton);
            }
            APP_FREQ_SPECTRUM_SetWindowType(appDisplayData.activeWindowSelected);
            
            //Stop the Microphone
            APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ_STOP);        
            appDisplayData.playButtonState = PLAY_BUTTON_STATE_STOPPED;
            laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));            
        }
        else
        {
            //Currently in Tone mode then ....
            laWidget_SetScheme((laWidget*)ToneButton, &ButtonSchemeInActive);
            laWidget_SetScheme((laWidget*)MicButton, &ButtonSchemeActive);
            APP_DISPLAY_EnableToneModeWidgets(LA_FALSE);            
            
            //Stop the speaker and start the microphone
            APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_STOP);
            appDisplayData.playButtonState = PLAY_BUTTON_STATE_STOPPED;
            //The windowing in MIC mode is fixed to Hanning window
            APP_FREQ_SPECTRUM_SetWindowType(WINDOW_TYPE_HANN);
            APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ_START);
            laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));
        }        
        APP_DISPLAY_ClearFreqSpectrum();
        appDisplayData.mode = modeButton; 
    }        
}

static void APP_DISPLAY_UpdataFreqLabels(
    APP_DISPLAY_UI_ELEMENTS freqLabel, 
    bool enable
)
{
    if (APP_DISPLAY_UI_ELEMENTS_F1_LABEL == freqLabel)
    {
        if (enable)
        {
            laWidget_SetScheme((laWidget*)F1_Label, &ButtonSchemeActive);
        }
        else
        {
            laWidget_SetScheme((laWidget*)F1_Label, &ButtonSchemeInActive);
        }        
    }    
    else if (APP_DISPLAY_UI_ELEMENTS_F2_LABEL == freqLabel)
    {
        if (enable)
        {
            laWidget_SetScheme((laWidget*)F2_Label, &ButtonSchemeActive);
        }
        else
        {
            laWidget_SetScheme((laWidget*)F2_Label, &ButtonSchemeInActive);
        }        
    }    
    else
    {
        if (enable)
        {
            laWidget_SetScheme((laWidget*)F3_Label, &ButtonSchemeActive);
        }
        else
        {
            laWidget_SetScheme((laWidget*)F3_Label, &ButtonSchemeInActive);
        }        
    }
}

static void APP_DISPLAY_UpdateTextBox(uint8_t activeSignal)
{
    static char strBuffer[64] = {0};
    static laString gfxString = {0}; 
    float freqValue_float;
    
    APP_DISPLAY_SIG_PARAMS* pActiveSigParams = &appDisplayData.toneParams[activeSignal];
    
    if (APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN == appDisplayData.activeUnitSelected)
    {
        sprintf((char*)strBuffer, "%d", pActiveSigParams->freqValue);
    }
    else if (APP_DISPLAY_UI_ELEMENTS_UNIT_KHZ_BTN == appDisplayData.activeUnitSelected)
    {
        freqValue_float = (float)pActiveSigParams->freqValue/1000.0;        
        sprintf((char*)strBuffer, "%.3f", (double)freqValue_float);
    }
    else
    {
        sprintf((char*)strBuffer, "%d", pActiveSigParams->dBFSValue);
    }
    
    gfxString = laString_CreateFromCharBuffer((const char*)strBuffer, &Arial_20_Bold);
    laTextFieldWidget_SetText(TextBox, gfxString);
}

static void APP_DISPLAY_SetUnitButton(uint32_t unitValue)
{
    if (APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN == unitValue)
    {
        laWidget_SetScheme((laWidget*)UnitsButtonHz, &ButtonSchemeActive);
        laWidget_SetScheme((laWidget*)UnitsButtonKHz, &ButtonSchemeInActive);
        laWidget_SetScheme((laWidget*)UnitsButtondBFS, &ButtonSchemeInActive);
    }
    else if (APP_DISPLAY_UI_ELEMENTS_UNIT_KHZ_BTN == unitValue)
    {
        laWidget_SetScheme((laWidget*)UnitsButtonKHz, &ButtonSchemeActive);
        laWidget_SetScheme((laWidget*)UnitsButtonHz, &ButtonSchemeInActive);
        laWidget_SetScheme((laWidget*)UnitsButtondBFS, &ButtonSchemeInActive);
    }        
    else
    {        
        laWidget_SetScheme((laWidget*)UnitsButtondBFS, &ButtonSchemeActive);        
        laWidget_SetScheme((laWidget*)UnitsButtonHz, &ButtonSchemeInActive);
        laWidget_SetScheme((laWidget*)UnitsButtonKHz, &ButtonSchemeInActive);
    }
}


static void APP_DISPLAY_UnitsButtonPressed(APP_DISPLAY_UI_ELEMENTS unitButton)
{        
    if (appDisplayData.activeUnitSelected != unitButton)
    {
        APP_DISPLAY_SetUnitButton(unitButton);
        appDisplayData.activeUnitSelected = unitButton;   
        APP_DISPLAY_UpdateTextBox(appDisplayData.activeSigSelected);
    }    
}

static APP_DISPLAY_UI_ELEMENTS APP_DISPLAY_GetFreqLabel(APP_DISPLAY_UI_ELEMENTS freq)
{
    APP_DISPLAY_UI_ELEMENTS freqLabel;
    
    if (APP_DISPLAY_UI_ELEMENTS_F1_RADIO_BTN == freq)
    {
        freqLabel = APP_DISPLAY_UI_ELEMENTS_F1_LABEL;
    }
    else if (APP_DISPLAY_UI_ELEMENTS_F2_RADIO_BTN == freq)
    {
        freqLabel = APP_DISPLAY_UI_ELEMENTS_F2_LABEL;
    }
    else
    {
        freqLabel = APP_DISPLAY_UI_ELEMENTS_F3_LABEL;
    }
    
    return freqLabel;
}

static void APP_DISPLAY_CheckSignalGainValues(void)
{
    static char strBuffer[200] = {0};
    static laString gfxString = {0}; 
    float totalDBFSValue = 0.0;
    uint8_t i = 0;
    
    for (i = 0; i < 3; i++)
    {
        if (true == appDisplayData.toneParams[i].isActive)
        {
            totalDBFSValue += powf((float)10, (float)((float)appDisplayData.toneParams[i].dBFSValue/20));
        }
    }
    
    if (totalDBFSValue > 1)
    {
        sprintf((char*)strBuffer, "Signal saturated. Lower the signal amplitude.");        
    }
    else
    {
        sprintf((char*)strBuffer, " ");
    }
    
    gfxString = laString_CreateFromCharBuffer((const char*)strBuffer, &Arial);
    laLabelWidget_SetText(MessageLabel, gfxString);    
}    

static void APP_DISPLAY_GridButtonPressed(void)
{
    if (GRID_BUTTON_STATE_ON == appDisplayData.gridButtonState)
    {
        appDisplayData.gridButtonState = GRID_BUTTON_STATE_OFF;
        laButtonWidget_SetText(GridButton, laString_CreateFromID(string_ON));
        APP_GRAPH_GridShow(false);
    }
    else
    {
        appDisplayData.gridButtonState = GRID_BUTTON_STATE_ON;
        laButtonWidget_SetText(GridButton, laString_CreateFromID(string_OFF));
        APP_GRAPH_GridShow(true);
    }
}

static void APP_DISPLAY_IncButtonPressed(void)
{
    APP_DISPLAY_SIG_PARAMS* pSigParams = &appDisplayData.toneParams[appDisplayData.activeSigSelected];        
    uint32_t incValue = 0;
            
    if (APP_DISPLAY_UI_ELEMENTS_UNIT_DBFS_BTN == appDisplayData.activeUnitSelected)
    {
        if (appDisplayData.sigLimits.highAmpLimitIndBFS > pSigParams->dBFSValue)
        {
            pSigParams->dBFSValue += 1;        
        }
    }
    else
    {
        if (APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN == appDisplayData.activeUnitSelected)
        {
            incValue = 100;
        }
        else
        {
            incValue = 1000;
        }
        pSigParams->freqValue += incValue;
        
        if (pSigParams->freqValue > appDisplayData.sigLimits.highFreqLimitInHz)
        {
            pSigParams->freqValue = appDisplayData.sigLimits.highFreqLimitInHz;
        }
        
        appDisplayData.toneParams[appDisplayData.activeSigSelected].isActive = true;
        
        APP_DISPLAY_UpdataFreqLabels(
            APP_DISPLAY_GetFreqLabel(appDisplayData.activeSigSelected), 
            true
        );
    }
                        
    APP_DISPLAY_CheckSignalGainValues();
    APP_DISPLAY_UpdateTextBox(appDisplayData.activeSigSelected);        
    APP_DISPLAY_UpdateSignalParams();
}

static void APP_DISPLAY_DecButtonPressed(void)
{
    APP_DISPLAY_SIG_PARAMS* pSigParams = &appDisplayData.toneParams[appDisplayData.activeSigSelected];    
    uint32_t decValue = 0;    
    
    if (APP_DISPLAY_UI_ELEMENTS_UNIT_DBFS_BTN == appDisplayData.activeUnitSelected)
    {
        if (appDisplayData.sigLimits.lowAmpLimitIndBFS < pSigParams->dBFSValue)
        {
            pSigParams->dBFSValue -= 1;
        }
    }
    else
    {
        if (APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN == appDisplayData.activeUnitSelected)
        {
            decValue = 100;
        }
        else
        {
            decValue = 1000;
        }
    
        if (pSigParams->freqValue >= decValue)
        {
            pSigParams->freqValue -= decValue;
        }
        else
        {
            pSigParams->freqValue = 0;
        }
        
        if (0 == pSigParams->freqValue)
        {
            appDisplayData.toneParams[appDisplayData.activeSigSelected].isActive = false;
            
            APP_DISPLAY_UpdataFreqLabels(
                APP_DISPLAY_GetFreqLabel(appDisplayData.activeSigSelected), 
                false
            );
        }
    }
    
    APP_DISPLAY_CheckSignalGainValues();
    APP_DISPLAY_UpdateTextBox(appDisplayData.activeSigSelected);            
    APP_DISPLAY_UpdateSignalParams();
}

static void APP_DISPLAY_ClrButtonPressed(void)
{    
    if (APP_DISPLAY_UI_ELEMENTS_UNIT_DBFS_BTN == appDisplayData.activeUnitSelected)
    {
        appDisplayData.toneParams[appDisplayData.activeSigSelected].dBFSValue = 0;
    }
    else
    {
        appDisplayData.toneParams[appDisplayData.activeSigSelected].freqValue = 0;
        
        appDisplayData.toneParams[appDisplayData.activeSigSelected].isActive = false;
        
        APP_DISPLAY_UpdataFreqLabels(
                APP_DISPLAY_GetFreqLabel(appDisplayData.activeSigSelected), 
                false
            );        
    }    
    
    APP_DISPLAY_CheckSignalGainValues();
    APP_DISPLAY_UpdateTextBox(appDisplayData.activeSigSelected);                
    APP_DISPLAY_UpdateSignalParams();
}

static void APP_DISPLAY_RadioButtonPressed(APP_DISPLAY_UI_ELEMENTS radioButton)
{     
    if (APP_DISPLAY_UI_ELEMENTS_F1_RADIO_BTN == radioButton)
    {
        appDisplayData.activeSigSelected = 0;    
    }
    else if (APP_DISPLAY_UI_ELEMENTS_F2_RADIO_BTN == radioButton)
    {
        appDisplayData.activeSigSelected = 1;    
    }
    else
    {
        appDisplayData.activeSigSelected = 2;    
    }        
    
    appDisplayData.activeUnitSelected = APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN;
            
    APP_DISPLAY_UpdateTextBox(appDisplayData.activeSigSelected);
    APP_DISPLAY_SetUnitButton(appDisplayData.activeUnitSelected);
}

static void APP_DISPLAY_WindowButtonPressed(APP_DISPLAY_UI_ELEMENTS windowButton)
{        
    if (APP_DISPLAY_UI_ELEMENTS_HANN_WIN_BTN == windowButton)
    {
        appDisplayData.activeWindowSelected = WINDOW_TYPE_HANN;        
    }
    else
    {
        appDisplayData.activeWindowSelected = WINDOW_TYPE_BLACKMAN;
    }
    
    APP_FREQ_SPECTRUM_SetWindowType(appDisplayData.activeWindowSelected);
}

static void APP_DISPLAY_InitUI(void)
{
    uint8_t i = 0;
    
    appDisplayData.mode = APP_DISPLAY_UI_ELEMENTS_MIC_BTN;
    appDisplayData.playButtonState = PLAY_BUTTON_STATE_STOPPED;
    appDisplayData.activeSigSelected = 0;    
    appDisplayData.activeUnitSelected = APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN;    
    appDisplayData.gridButtonState = GRID_BUTTON_STATE_ON;
    appDisplayData.activeWindowSelected = WINDOW_TYPE_HANN;
    APP_GRAPH_Init();
        
    for (i = 0; i < APP_DISPLAY_MAX_TONES; i++)
    {
        appDisplayData.toneParams[i].freqValue = 0;
        appDisplayData.toneParams[i].dBFSValue = -12;
        appDisplayData.toneParams[i].isActive = false;
    }            
    
    APP_TONE_GetFreqRangeInHz(
        &appDisplayData.sigLimits.lowFreqLimitInHz, 
        &appDisplayData.sigLimits.highFreqLimitInHz
    );
    
    APP_TONE_GetAmplitudeRagneInDBFS(
        &appDisplayData.sigLimits.lowAmpLimitIndBFS, 
        &appDisplayData.sigLimits.highAmpLimitIndBFS
    );    
    
    laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));    
    APP_DISPLAY_SetUnitButton(APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN);     
    
    APP_DISPLAY_EnableToneModeWidgets(LA_FALSE);
}

static void APP_DISPLAY_ScreenChanged(APP_DISPLAY_UI_ELEMENTS btn)
{
    if (btn == APP_DISPLAY_UI_ELEMENTS_TIME_DOMAIN_SCREEN_BTN)
    {                
        APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_STOP);
        laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));
        APP_GRAPH_Show();             
        APP_GRAPH_Update();
        APP_FREQ_SPECTRUM_UnRegisterCallback(APP_DISPLAY_FFTResultsEvHandler);        
        appDisplayData.activeScreen = APP_DISPLAY_ACTIVE_SCREEN_TIME_DOMAIN;        
    }
    else
    {
        APP_DISPLAY_InitFreqSpectrumBars();
        APP_FREQ_SPECTRUM_RegisterCallback(APP_DISPLAY_FFTResultsEvHandler);        
        appDisplayData.activeScreen = APP_DISPLAY_ACTIVE_SCREEN_MAIN;
    }        
}

void APP_DISPLAY_TaskInitialize(void)
{
    APP_DISPLAY_InitFreqSpectrumBars();
    
    APP_DISPLAY_InitUI();        
    
    APP_SPEAKER_RegisterCallback(APP_DISPLAY_SpeakerEventHandler);    
    
    APP_FREQ_SPECTRUM_RegisterCallback(APP_DISPLAY_FFTResultsEvHandler);                               
    
    appDisplayData.queueHandle = APP_QUEUE_Open(
        appDisplayData.cmdQueue, 
        sizeof(appDisplayData.cmdQueue)/APP_DISPLAY_QUEUE_SIZE,
        APP_DISPLAY_QUEUE_SIZE
    );
    
    if (NULL == appDisplayData.queueHandle)
    {
        //printf("Unable to get display Queue handle");
    }
    
    appDisplayData.activeScreen = APP_DISPLAY_ACTIVE_SCREEN_MAIN;
    
    appDisplayData.eventsCallback = NULL;                
    
    APP_DISPLAY_VolumeSliderChanged();                           
}

void APP_DISPLAY_Task(void)
{
    uint8_t queueElement;
    
    if (APP_QUEUE_Pull(appDisplayData.queueHandle, &queueElement))
    {
        switch(queueElement)
        {
            case APP_DISPLAY_CMD_UPDATE_FREQ_SPECTRUM:                                
                APP_DISPLAY_UpdateFreqSpectrum();                                    
                break;            
            case APP_DISPLAY_CMD_UI_EVENT_MIC_BTN_PRESSED:
                APP_DISPLAY_ModeButtonPressed(APP_DISPLAY_UI_ELEMENTS_MIC_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_TONE_BTN_PRESSED:
                APP_DISPLAY_ModeButtonPressed(APP_DISPLAY_UI_ELEMENTS_TONE_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_F1_RADIO_BTN_SEL:
                APP_DISPLAY_RadioButtonPressed(APP_DISPLAY_UI_ELEMENTS_F1_RADIO_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_F2_RADIO_BTN_SEL:
                APP_DISPLAY_RadioButtonPressed(APP_DISPLAY_UI_ELEMENTS_F2_RADIO_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_F3_RADIO_BTN_SEL:
                APP_DISPLAY_RadioButtonPressed(APP_DISPLAY_UI_ELEMENTS_F3_RADIO_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_INC_BTN_PRESSED:
                APP_DISPLAY_IncButtonPressed();
                break;
            case APP_DISPLAY_CMD_UI_EVENT_DEC_BTN_PRESSED:
                APP_DISPLAY_DecButtonPressed();
                break;
            case APP_DISPLAY_CMD_UI_EVENT_CLR_BTN_PRESSED:
                APP_DISPLAY_ClrButtonPressed();
                break;
            case APP_DISPLAY_CMD_UI_EVENT_HZ_BTN_PRESSED:
                APP_DISPLAY_UnitsButtonPressed(APP_DISPLAY_UI_ELEMENTS_UNIT_HZ_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_KHZ_BTN_PRESSED:
                APP_DISPLAY_UnitsButtonPressed(APP_DISPLAY_UI_ELEMENTS_UNIT_KHZ_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_DBFS_BTN_PRESSED:
                APP_DISPLAY_UnitsButtonPressed(APP_DISPLAY_UI_ELEMENTS_UNIT_DBFS_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_PLAY_BTN_PRESSED:
                APP_DISPLAY_PlayButtonPressed();
                break;
            case APP_DISPLAY_CMD_UI_EVENT_HANN_WIN_RADIO_BTN_PRESSED:
                APP_DISPLAY_WindowButtonPressed(APP_DISPLAY_UI_ELEMENTS_HANN_WIN_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_BLACKMAN_WIN_RADIO_BTN_PRESSED:
                APP_DISPLAY_WindowButtonPressed(APP_DISPLAY_UI_ELEMENTS_BLACKMAN_WIN_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_VOLUME_SLIDER_CHANGED:
                APP_DISPLAY_VolumeSliderChanged();                
                break;
            case APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_INC_BTN_PRESSED:
                APP_GRAPH_TimePerDivChanged(APP_GRAPH_CHANGE_TYPE_INC);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_DEC_BTN_PRESSED:
                APP_GRAPH_TimePerDivChanged(APP_GRAPH_CHANGE_TYPE_DEC);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_INC_BTN_PRESSED:
                APP_GRAPH_AmpScaleChanged(APP_GRAPH_CHANGE_TYPE_INC);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_DEC_BTN_PRESSED:
                APP_GRAPH_AmpScaleChanged(APP_GRAPH_CHANGE_TYPE_DEC);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_GRID_BTN_PRESSED:
                APP_DISPLAY_GridButtonPressed();
                break;
            case APP_DISPLAY_CMD_UI_EVENT_TIME_DOMAIN_SCREEN_BTN_PRESSED:
                APP_DISPLAY_ScreenChanged(APP_DISPLAY_UI_ELEMENTS_TIME_DOMAIN_SCREEN_BTN);
                break;
            case APP_DISPLAY_CMD_UI_EVENT_MAIN_SCREEN_BTN_PRESSED:
                APP_DISPLAY_ScreenChanged(APP_DISPLAY_UI_ELEMENTS_MAIN_SCREEN_BTN);
                break;
            default:
                break;
        }
    }
}

