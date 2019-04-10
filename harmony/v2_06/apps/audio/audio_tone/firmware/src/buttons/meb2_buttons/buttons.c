//******************************************************************************
// File:    pic32mx_btad_buttons/btad_buttons.c
// Author:  CAL (c16825)
// Created: 7/9/2015 
//
// Description:  
//
//     Button tasks state machine for the audio_tone GUI:
//      ICN Interrupts on the 5 bt_audio_dk board switches with timer
//      counting how long the key has been pressed:
//      
//     Actions:
//        SW1: Short release: cycles the AudioGenerateFx (AG) parameter select.
//                            (f1Hz, f2Hz, timeDurationMs)
//             Long release: Set selected parameter to minimum allowable value.
//        SW2: Short release: cycles the AudioGenerateFx (AG) modes
//                            (Sine Tone, Chirp)
//             Long release: Set selected parameter to maximum allowable value.
//        SW5/SW3: Short press down: Decrement/Increment the selected 
//                                   parameter. 
//                 Long press: Accelerates the Decrement/Increment
//
//        Sw4:  Short Press Release: Toggle Output On/Off 
//
//******************************************************************************

//#include "btad_buttons.h"
#include "app.h"
#include "system_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//Audio Generate
extern AUDIO_GENERATE ag;  //Instance class to generate a signal

//uint16_t ampDeltaFS       = 100;
uint32_t freqSlowDeltaHz  =  10; //AudioTone freq change delta
uint32_t freqFastDeltaHz  = 100; //AudioTone freq change delta
//int32_t  oldAmpFs         = 500;

//Buttons
#define LONG_PRESS_DELAY   30 //3s
#define DEBOUNCE_DELAY      2 //100ms
bool state=0;                 //Button UP/DWN- state

static unsigned int mButtonState;
//static unsigned int mNewButtonState;
//static uint8_t  mRepeatButton;
//static uint16_t mRepeatCount;
//static volatile uint32_t intStatus;
uint8_t      mButtonEvent = 0;   //Button Change Interrupt occurred


//static void setRepeatTimer(unsigned delay);
//static void clearRepeatTimer(void);

//******************************************************************************
//
// Push-Button Processing
//
// Description:
//
//   ICN (Interrupt Change Notification) - notifies up/down button presses
//   via interrupt handler (buttons_handleInterrupt()).  Time between up and down
//   presses determined by counting TMR_DRV2 interrupts occurring every 100 ms.  
//   ( restarted after counting in app.c APP_handlePeriodicTimerSignal() )
//
//   APP_ButtonTask() processes the button change interrupt to determing the 
//   button and its state of up or down.
//
//
//******************************************************************************

//******************************************************************************
//  APP_ButtonInit()
//******************************************************************************
void APP_ButtonInit(void)
{
    mButtonState = APP_READ_BUTTON_PORTS();
    mRepeatButton = INVALID_BUTTON;
    mRepeatCount = 0;
    mButtonEvent = 0;
}

//******************************************************************************
// APP_ButtonTask()
//
// Actions when bt_audio_dk Switch ICN occurred (mButtonEvent = 1) 
//    
// mNewButtonState - Button state from the ICN Interrupt handler
// mButtonEvent    - Button changed (from the ICN Interrupt handerl) 
// mRepeatButton   - button being held
// mRepeatCount    - Number of periods held down
// mButtonState
//
//******************************************************************************
bool buttons[TOUCH_NUM_BUTTONS];
bool oldButtons[TOUCH_NUM_BUTTONS];

void APP_ButtonTask(void)
{
    //Button Pressed
    if (mButtonEvent == 1) 
    { 
        mButtonEvent = 0;
       
        //SW 1 - Push Button Changed
        if (buttons[TOUCH_F1_PLUS] != oldButtons[TOUCH_F1_PLUS])
        {
            // Up/Down - State
           if (buttons[TOUCH_F1_PLUS])
            {
                //APP_OnButtonEvent(&ag, APP_BUTTON1, true, 0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_F1_PLUS;
                    mRepeatCount = 0;
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_F1_PLUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_F1_PLUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;              
                }
                else
                {
                    //APP_OnButtonEvent(&ag, APP_BUTTON2,false,0);
                }               
            }
        } //End SW 1

        //SW 2 - Push Button
        if (buttons[TOUCH_F1_MINUS] != oldButtons[TOUCH_F1_MINUS])
        {
            if (buttons[TOUCH_F1_MINUS])
            {
                //APP_OnButtonEvent(&ag, APP_BUTTON2, true, 0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_F1_MINUS;
                    mRepeatCount = 0;
                }
            }
             else
            {
                //UP
                if (mRepeatButton == TOUCH_F1_MINUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_F1_MINUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&ag, APP_BUTTON2,false,0);
                }               
            }
        }

        //SW 3 - Push Button -- allows repeat
        if (buttons[TOUCH_F2_PLUS] != oldButtons[TOUCH_F2_PLUS])
        {
            if (buttons[TOUCH_F2_PLUS])           
            {
                //APP_OnButtonEvent(&ag, APP_BUTTON3,true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_F2_PLUS;
                    mRepeatCount = 0;
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_F2_PLUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_F2_PLUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&ag, APP_BUTTON3,false,0);
                }
            }
        }

        //SW 4 - Push Button
        if (buttons[TOUCH_F2_MINUS] != oldButtons[TOUCH_F2_MINUS])
        {
            if (buttons[TOUCH_F2_MINUS])
            {
                //DOWN
                APP_OnButtonEvent(&appData, TOUCH_F2_MINUS, true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_F2_MINUS;
                    mRepeatCount = 0;
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_F2_MINUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_F2_MINUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON4, false, 0);
                }
            }
        } //End SW4

        //SW 5 - Push Button -- allows repeat
        if (buttons[TOUCH_T_PLUS] != oldButtons[TOUCH_T_PLUS])
        {
            if (buttons[TOUCH_T_PLUS])
            {
                APP_OnButtonEvent(&appData, TOUCH_T_PLUS,true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_T_PLUS;
                    mRepeatCount = 0;
                    //setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_T_PLUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_T_PLUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON5,false,0);
                }
            }
        }
        
        if (buttons[TOUCH_T_MINUS] != oldButtons[TOUCH_T_MINUS])
        {
            if (buttons[TOUCH_T_MINUS])
            {
                //DOWN
                APP_OnButtonEvent(&appData, TOUCH_T_MINUS, true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_T_MINUS;
                    mRepeatCount = 0;
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_T_MINUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_T_MINUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON4, false, 0);
                }
            }
        } //End SW4 
        
       //SW 5 - Push Button -- allows repeat
        if (buttons[TOUCH_VOLUME_PLUS] != oldButtons[TOUCH_VOLUME_PLUS])
        {
            if (buttons[TOUCH_VOLUME_PLUS])
            {
                APP_OnButtonEvent(&appData, TOUCH_VOLUME_PLUS,true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_VOLUME_PLUS;
                    mRepeatCount = 0;
                    //setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_VOLUME_PLUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_VOLUME_PLUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON5,false,0);
                }
            }
        }
        
        if (buttons[TOUCH_VOLUME_MINUS] != oldButtons[TOUCH_VOLUME_MINUS])
        {
            if (buttons[TOUCH_VOLUME_MINUS])
            {
                //DOWN
                APP_OnButtonEvent(&appData, TOUCH_VOLUME_MINUS, true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_VOLUME_MINUS;
                    mRepeatCount = 0;
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_VOLUME_MINUS)
                {
                    APP_OnButtonEvent(&appData, TOUCH_VOLUME_MINUS, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON4, false, 0);
                }
            }
        } //End SW4  

       //SW 5 - Push Button -- allows repeat
        if (buttons[TOUCH_PLAYPAUSE] != oldButtons[TOUCH_PLAYPAUSE])
        {
            if (buttons[TOUCH_PLAYPAUSE])
            {
                APP_OnButtonEvent(&appData, TOUCH_PLAYPAUSE,true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_PLAYPAUSE;
                    mRepeatCount = 0;
                    //setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_PLAYPAUSE)
                {
                    APP_OnButtonEvent(&appData, TOUCH_PLAYPAUSE, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON5,false,0);
                }
            }
        }
        
       //SW 5 - Push Button -- allows repeat
        if (buttons[TOUCH_SINECHIRP]!= oldButtons[TOUCH_SINECHIRP])
        {
            if (buttons[TOUCH_SINECHIRP])
            {
                APP_OnButtonEvent(&appData, TOUCH_SINECHIRP,true,0);
                if (mRepeatButton == INVALID_BUTTON)
                {
                    mRepeatButton = TOUCH_SINECHIRP;
                    mRepeatCount = 0;
                    //setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == TOUCH_SINECHIRP)
                {
                    APP_OnButtonEvent(&appData, TOUCH_SINECHIRP, false, mRepeatCount);
                    mRepeatButton = INVALID_BUTTON;
                    
                }
                else
                {
                    //APP_OnButtonEvent(&appData, APP_BUTTON5,false,0);
                }
            }
        }        
        
        oldButtons[TOUCH_F1_PLUS] = buttons[TOUCH_F1_PLUS];
        oldButtons[TOUCH_F1_MINUS] = buttons[TOUCH_F1_MINUS];
        oldButtons[TOUCH_F2_PLUS] = buttons[TOUCH_F2_PLUS];
        oldButtons[TOUCH_F2_MINUS] = buttons[TOUCH_F2_MINUS];
        oldButtons[TOUCH_T_PLUS] = buttons[TOUCH_T_PLUS];
        oldButtons[TOUCH_T_MINUS] = buttons[TOUCH_T_MINUS];
        oldButtons[TOUCH_VOLUME_PLUS] = buttons[TOUCH_VOLUME_PLUS];
        oldButtons[TOUCH_VOLUME_MINUS] = buttons[TOUCH_VOLUME_MINUS];
        oldButtons[TOUCH_PLAYPAUSE] = buttons[TOUCH_PLAYPAUSE];
        oldButtons[TOUCH_SINECHIRP] = buttons[TOUCH_SINECHIRP];        
    } //End Button Change Event

}

//******************************************************************************
// APP_OnButtonEvent() 
//
// Description:
//   Actions due to the 100ms timer interrupt while button is pressed.
//
// Arguments:
//   AUDIO_GENERATE * ag - The struct used for audio generation
//   uint8_t button      - button number
//   bool bButtonClosed  - Down/Up- indication
//   int32_t repeatCount - how many times the button has cycled down/up
//
// NOTE:  1) Called from APP_PeriodicTimerHandler every 100ms interval 
//        2) Called from the APP_ButtonTask() also when only the button
//           transitions are important.
//
//
//******************************************************************************
void APP_OnButtonEvent(APP_DATA * appData,
                       uint8_t button, 
                       bool bButtonClosed, 
                       int32_t repeatCount)
{
    //APP AG Status
    AUDIO_GENERATE_STATUS * agStat = &(appData->agStatus);
    //AUDIO_GEN_TYPE  currentToneType  = agStat->currentToneType;
    AUDIO_GEN_TYPE  nextToneType;
    //AUDIO_GEN_PARAM currentToneParam = agStat->currentToneParam;
    AUDIO_GEN_PARAM nextToneParam; 

    //Display Controls
    GUI_DATA * guiData  = &(appData->guiData);
//    int        selNum   = guiData->select;
    int        modeNum  = guiData->mode;
    guiData->lastButton = INVALID_BUTTON;       // default

    switch (button)
    {
        //======================================================================
        // SW3: Cycle through Parameter 
        //      Selection mode types
        //======================================================================
#if 0        
        case APP_BUTTON3:
        {
            if ( !bButtonClosed && 
                 (mRepeatCount >= DEBOUNCE_DELAY) &&
                 (mRepeatCount <= LONG_PRESS_DELAY ))
            { 
                //SW3 Short press release: Change Parameter Select Modification.
                //           -->Highlight the field on display 

                //Cycle the selection
                selNum++;
                if (selNum == GUI_NUMSEL) selNum = 0;
                guiData->select = (GUI_SELECT) selNum;
                guiData->onOff = false;
                guiData->changeToneMode= true;
                
                APP_LED1_OFF();
                APP_LED2_OFF();
                APP_LED3_OFF();
               
                switch (selNum)
                {
                    case 0:
                        APP_LED1_ON();      // indicate in mode 1 (f1))
                        break;
                    case 1:
                        APP_LED2_ON();      // indicate in mode 2 (f2))
                        break;
                    case 2:
                        APP_LED3_ON();      // indicate in mode 3 (t))
                        break;                        
                }

                guiData->changeMode = true;
            }
            else if (!bButtonClosed &&
                     (mRepeatCount > LONG_PRESS_DELAY ))
            {
                //SW3 Long Press - Go to maximum value of selection 
                //if (guiData->select == SELECT_F1) guiData->f1Hz = agStat->fMaxHz;  
                //else if (guiData->select == SELECT_F2) guiData->f2Hz = agStat->fMaxHz;  
                //else if (guiData->select == SELECT_T) guiData->timeDeltaMs= 3000;  
                
                guiData->onOff = false;
            }          
            guiData->displayUpdate = true;
        }
        break;
#endif            
        //======================================================================
        // SW5: Cycle through the Audio Generation (AG) Modes
        // switch between sine and chirp
        //======================================================================
        case TOUCH_SINECHIRP:
        {
            if ( !bButtonClosed &&
                 (mRepeatCount >= DEBOUNCE_DELAY) &&
                 (mRepeatCount <= LONG_PRESS_DELAY) )
            {
                //SW5 Short Press Release: Cycle sin/chirp

                //Cycle the AG modes
                guiData->mode++;
                if (guiData->mode == GUI_NUMMODES) guiData->mode = MODE_TONE;

                modeNum++;
                if (modeNum == GUI_NUMMODES) modeNum = 0;
                guiData->mode = (GUI_MODE) modeNum;
                
                switch (modeNum)
                {
                    case 0:
                        APP_LED5_OFF();      // sin
                        break;
                    case 1:
                        APP_LED5_ON();      // chirp
                        break;                       
                }                

                // Change the AG Mode using current display controls
                //
                // NOTE: 1) Only MODE_TONE and MODE_CHIRP for now.
                //       2) GUI only controls mode, f1Hz, f2Hz, timeDeltaMs
                //       3) a) Tone start on and allows real-time changing of frequency.
                //             --> can be used to set the f1Hz and f2Hz values for chirp
                //          b) Chirp starts off to allow for parameter setting
                //              for a one shot.
                //
                nextToneType  = (guiData->mode==MODE_TONE)?
                                AUDIO_GEN_TONE:AUDIO_GEN_CHIRP;
 
                guiData->onOff = false;
 
                //Update the audio generator using current parameters
                nextToneParam             = agStat->currentToneParam;
                nextToneParam.fHz1        = guiData->f1Hz;
                nextToneParam.fHz2        = guiData->f2Hz;
                nextToneParam.timeDeltaMs = guiData->timeDeltaMs;
                //Turn the output off.
                //guiData->onOff = (guiData->mode==MODE_TONE)?true:false;
                APP_audioGenSetNextToneType(&(appData->agStatus), 
                                            nextToneType, nextToneParam,
                                            guiData->onOff);
                guiData->changeToneMode = true;              
            }
            else if (!bButtonClosed &&
                     (mRepeatCount > LONG_PRESS_DELAY ))
            {
                //SW2 Long Press - Go to min value of parameter selection 
                //if (guiData->select == SELECT_F1) guiData->f1Hz = agStat->fMinHz;  
                //else if (guiData->select == SELECT_F2) guiData->f2Hz = agStat->fMinHz;  
                //else if (guiData->select == SELECT_T)  guiData->timeDeltaMs = -1; 

                guiData->onOff = false;
            }           
            guiData->displayUpdate = true;
        }
        break;
            
        //======================================================================
        // SW1: Increase Value of selection and current frequency
        //======================================================================
        case TOUCH_F1_PLUS:
        case TOUCH_F2_PLUS:
        case TOUCH_T_PLUS:
        case TOUCH_VOLUME_PLUS:
        {
            if ( bButtonClosed &&
                 mRepeatCount <= LONG_PRESS_DELAY )
            {
                //SW3 Push Down: Increase parameter value
                
                //Change the parameter setting
                switch (button)
                {
                    case TOUCH_F1_PLUS:
                        guiData->f1Hz += freqSlowDeltaHz;
                        if (guiData->f1Hz > agStat->fMaxHz)
                        {
                            guiData->f1Hz = agStat->fMaxHz;
                        }                          
                        //fHz = guiData->f1Hz;
                        break;
                    case TOUCH_F2_PLUS:
                        guiData->f2Hz += freqSlowDeltaHz;
                        if (guiData->f2Hz > agStat->fMaxHz)
                        {
                            guiData->f2Hz = agStat->fMaxHz;
                        }                          
                        //fHz = guiData->f2Hz;
                        break;
                    case TOUCH_T_PLUS:
                        if (guiData->timeDeltaMs == -1) guiData->timeDeltaMs = 0;
                        guiData->timeDeltaMs += 10; 
                        break;                        
                    case TOUCH_VOLUME_PLUS:
                        if (guiData->volume < 100)
                        {
                            guiData->volume++;                            
                            uint8_t mVolume = 255*(uint16_t)guiData->volume/100;
                            DRV_CODEC_VolumeSet(appData->codecData.codecClient.handle,
                                    DRV_CODEC_CHANNEL_LEFT_RIGHT, mVolume);                            
                        }
                        break;
                    default: break;
                }               
            }
            else if ( bButtonClosed &&
                      (mRepeatCount > LONG_PRESS_DELAY) )
            {
               //SW3 Long Push Down: 
                switch (button)
                {
                    case TOUCH_F1_PLUS:
                        guiData->f1Hz += freqFastDeltaHz;
                        if (guiData->f1Hz > agStat->fMaxHz)
                        {
                            guiData->f1Hz = agStat->fMaxHz;
                        }
                        //fHz = guiData->f1Hz;
                        break;
                    case TOUCH_F2_PLUS:
                        guiData->f2Hz += freqFastDeltaHz;
                        if (guiData->f2Hz > agStat->fMaxHz)
                        {
                            guiData->f2Hz = agStat->fMaxHz;
                        }                        
                        //fHz = guiData->f2Hz;
                        break;
                    case TOUCH_T_PLUS:
                        guiData->timeDeltaMs += 250;
                        break;
                    case TOUCH_VOLUME_PLUS:                        
                        guiData->volume += 10;
                        if (guiData->volume > 100)
                            guiData->volume = 100;
                        uint8_t mVolume = 255*(uint16_t)guiData->volume/100;
                        DRV_CODEC_VolumeSet(appData->codecData.codecClient.handle,
                                DRV_CODEC_CHANNEL_LEFT_RIGHT, mVolume);                           
                        break;                        
                    default: break;
                }
            }
            
            if (button != TOUCH_VOLUME_PLUS)
            {
                if (guiData->timeDeltaMs > 0)
                {
                    APP_LED4_ON();      // indicate timer > 0    
                }
                else
                {
                    APP_LED4_OFF();                   
                }            
                //TODO: Check limits

                guiData->onOff = false;

                nextToneType  = (guiData->mode==MODE_TONE)?
                                AUDIO_GEN_TONE:AUDIO_GEN_CHIRP;
                nextToneParam      = agStat->currentToneParam;
                nextToneParam.fHz1        = guiData->f1Hz;
                nextToneParam.fHz2        = guiData->f2Hz;
                nextToneParam.timeDeltaMs = guiData->timeDeltaMs;
                APP_audioGenSetNextToneType(agStat, 
                                            agStat->currentToneType,
                                            nextToneParam,
                                            guiData->onOff);

                guiData->changeToneMode = true;
            }
            guiData->lastButton = (int8_t)button;
            guiData->displayUpdate = true;            
        }
        break;
            
        //======================================================================
        // SW4: ON/OFF
        //
        //     1) Changes the type/mode to the parameter settings
        //     2) Sets up for 1 shot or continuous tone.
        //======================================================================
        case TOUCH_PLAYPAUSE:
        {
            if (!bButtonClosed &&
                (mRepeatCount >= DEBOUNCE_DELAY) &&
                (mRepeatCount <= LONG_PRESS_DELAY) )
            {
                // SW4 Push Release: Always reinit with same type as before,
                //                   but toggle the on/off status depending on 
                //                   mode, duration and playback on/off.
                //
                //   a) Get the current GUI param values for reinit.
                //   b) Duration:
                //       i) == inf:  Toggle Output On/Off.
                //      ii) < inf off:  1 shot using the reinit parameters then
                //                      output off.
                //     iii) < inf on: reinit but playback is off 
               
                

                //Always Toggle ON/OFF
                if (guiData->onOff)guiData->onOff = false;
                else guiData->onOff = true;

                //Reinit AG parameters from GUI params
                //nextToneType       = agStat->currentToneType;
                nextToneType  = (guiData->mode==MODE_TONE)?
                                 AUDIO_GEN_TONE:AUDIO_GEN_CHIRP;
                nextToneParam      = agStat->currentToneParam;
                nextToneParam.fHz1        = guiData->f1Hz;
                nextToneParam.fHz2        = guiData->f2Hz;
                nextToneParam.timeDeltaMs = guiData->timeDeltaMs;

                //Set the APP AG status to reinit at the next buffer fill.
                APP_audioGenSetNextToneType(agStat, 
                                            agStat->currentToneType,
                                            nextToneParam,
                                            guiData->onOff);
                guiData->changeToneMode = true;
            }
            else if (!bButtonClosed &&
                    (mRepeatCount > LONG_PRESS_DELAY) )
            {
                //SW4 Long Press:  Play stored "Ringtone" sequence
                Nop();
            }
            if (guiData->onOff)
            {
                // if restarting, kickstart adding buffer to queue
                appData->state = APP_STATE_CODEC_ADD_BUFFER;                            
            }

            guiData->displayUpdate = true;            
        }
        break;

        //======================================================================
        // SW2: Decrease the Value of selection and current frequency
        //======================================================================
        case TOUCH_F1_MINUS:
        case TOUCH_F2_MINUS:
        case TOUCH_T_MINUS:
        case TOUCH_VOLUME_MINUS:
        {
            //int32_t fHz;

            if ( bButtonClosed &&
                 mRepeatCount <= LONG_PRESS_DELAY )
            {
                //SW2 Push Down: Increase parameter value
                
                //Change the parameter setting
                switch (button)
                {
                    case TOUCH_F1_MINUS:
                        guiData->f1Hz -= freqSlowDeltaHz;
                        if (guiData->f1Hz < agStat->fMinHz)
                        {
                            guiData->f1Hz = agStat->fMinHz;
                        }                        
                        //fHz = guiData->f1Hz;
                        break;
                    case TOUCH_F2_MINUS:
                        guiData->f2Hz -= freqSlowDeltaHz;
                        if (guiData->f2Hz < agStat->fMinHz)
                        {
                            guiData->f2Hz = agStat->fMinHz;
                        }                         
                        //fHz = guiData->f2Hz;
                        break;
                    case TOUCH_T_MINUS:
                        guiData->timeDeltaMs -= 10; 
                        if (guiData->timeDeltaMs < 0)
                        {
                            guiData->timeDeltaMs = -1;
                        }                        
                        break;
                    case TOUCH_VOLUME_MINUS: 
                        if (guiData->volume > 0)
                        {
                            guiData->volume--;
                            uint8_t mVolume = 255*(uint16_t)guiData->volume/100;
                            DRV_CODEC_VolumeSet(appData->codecData.codecClient.handle,
                                    DRV_CODEC_CHANNEL_LEFT_RIGHT, mVolume);   
                        }
                        break;
                    default: break;
                }
            }
            else if ( bButtonClosed &&
                      (mRepeatCount > LONG_PRESS_DELAY) )
            {
               //SW5 Long Push Down: 
                switch (button)
                {
                    case TOUCH_F1_MINUS:
                        guiData->f1Hz -= freqFastDeltaHz;
                        if (guiData->f1Hz < agStat->fMinHz)
                        {
                            guiData->f1Hz = agStat->fMinHz;
                        }                         
                        //fHz = guiData->f1Hz;
                        break;
                    case TOUCH_F2_MINUS:
                        guiData->f2Hz -= freqFastDeltaHz;
                        if (guiData->f2Hz < agStat->fMinHz)
                        {
                            guiData->f2Hz = agStat->fMinHz;
                        }                         
                        //fHz = guiData->f2Hz;
                        break;
                    case TOUCH_T_MINUS:
                        guiData->timeDeltaMs -= 250;
                        if (guiData->timeDeltaMs < 0)
                        {
                            guiData->timeDeltaMs = -1;
                        }                          
                        break;
                    case TOUCH_VOLUME_MINUS:
                        if (guiData->volume > 10)
                            guiData->volume -= 10;
                        else guiData->volume = 0;                                                      
                        uint8_t mVolume = 255*(uint16_t)guiData->volume/100;
                        DRV_CODEC_VolumeSet(appData->codecData.codecClient.handle,
                                DRV_CODEC_CHANNEL_LEFT_RIGHT, mVolume);                           
                    default: break;
                }
            }

            if (button != TOUCH_VOLUME_MINUS)
            {            
                if (guiData->timeDeltaMs > 0)
                {
                    APP_LED4_ON();      // indicate timer > 0    
                }
                else
                {
                    APP_LED4_OFF();                   
                }

                guiData->onOff = false;

                nextToneType  = (guiData->mode==MODE_TONE)?
                                AUDIO_GEN_TONE:AUDIO_GEN_CHIRP;
                nextToneParam      = agStat->currentToneParam;
                nextToneParam.fHz1        = guiData->f1Hz;
                nextToneParam.fHz2        = guiData->f2Hz;
                nextToneParam.timeDeltaMs = guiData->timeDeltaMs;
                APP_audioGenSetNextToneType(agStat, 
                                            agStat->currentToneType,
                                            nextToneParam,
                                            guiData->onOff);

                guiData->changeToneMode = true;
            }
            guiData->lastButton = (int8_t)button;
            guiData->displayUpdate = true;
        } //End APP_BUTTON5
        break;
            
        default: break;
    } //End button Switch 

} //End APP_OnButtonEvent()

//******************************************************************************
// buttons_handleInterrupt() - TMR Index 1 (TMR_ID_2) 100 ms interrupt handler
//
// Called within button change detection ISR
//
//******************************************************************************
void buttons_handleInterrupt()
{
    //Remember new button press state change
    mButtonEvent = 1;
}
