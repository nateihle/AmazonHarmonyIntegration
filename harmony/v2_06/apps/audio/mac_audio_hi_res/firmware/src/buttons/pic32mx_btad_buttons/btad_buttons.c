//******************************************************************************
// File:    pic32mx_btad_buttons/btad_buttons.c
//
// Description:  
//
//     Button tasks state machine for the mac hi res GUI:
//      ICN Interrupts on the 5 bt_audio_dk board switches
//      
//     Actions:
//        SW1: volume up.
//        SW2: volume down
//        SW4:  mute on/off
//
//******************************************************************************

#include "app.h"
#include "system_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

uint8_t      mButtonEvent = 0;   //Button Change Interrupt occurred

//******************************************************************************
//
// Push-Button Processing
//
// Description:
//
//   ICN (Interrupt Change Notification) - notifies up/down button presses
//   via interrupt handler (buttons_handleInterrupt()).  
//
//   APP_ButtonTask() processes the button change interrupt to determining the 
//   button and its state of up or down.
//
//
//******************************************************************************

//******************************************************************************
// APP_ButtonTask()
//
// Actions when bt_audio_dk Switch ICN occurred (mButtonEvent = 1) 
//    
//
//******************************************************************************
bool buttons[NUM_APP_BUTTONS];
bool oldButtons[NUM_APP_BUTTONS];

void APP_ButtonTask(void)
{       
    //Button Pressed
    if (mButtonEvent == 1) 
    {        
        // invert so 1 if pushbutton pressed, 0 if up
        read_buttons();

        mButtonEvent = 0;         
       
        //SW 1 - increase volume
        if (buttons[APP_SWITCH_1] != oldButtons[APP_SWITCH_1])
        {
           if (buttons[APP_SWITCH_1])
            {
                //DOWN               
                appData.state = APP_DAC_VOLUME_INCREASE;
            }
        }

        //SW 2 - decrease volume
        if (buttons[APP_SWITCH_2] != oldButtons[APP_SWITCH_2])
        {
            if (buttons[APP_SWITCH_2])
            {
                //DOWN                
                appData.state = APP_DAC_VOLUME_DECREASE;
            }
        }

        //SW 4 - mute on/off
        if (buttons[APP_SWITCH_4] != oldButtons[APP_SWITCH_4])
        {
            if (buttons[APP_SWITCH_4])
            {
                //DOWN
                appData.state = APP_DAC_MUTE;
            }
        } 

        oldButtons[APP_SWITCH_1] = buttons[APP_SWITCH_1];
        oldButtons[APP_SWITCH_2] = buttons[APP_SWITCH_2];
        oldButtons[APP_SWITCH_3] = buttons[APP_SWITCH_3];
        oldButtons[APP_SWITCH_4] = buttons[APP_SWITCH_4];
        oldButtons[APP_SWITCH_5] = buttons[APP_SWITCH_5];
    } //End Button Change Event
}

//******************************************************************************
// read_buttons()()
//
// Called within button change detection ISR and locally to read all buttons
// Assumes buttons are active low
//
//******************************************************************************
void read_buttons()
{
    // inverted so 1 if pushbutton pressed, 0 if up
    buttons[APP_SWITCH_1] = !BSP_SwitchStateGet( BSP_SWITCH_1 ) ;
    buttons[APP_SWITCH_2] = !BSP_SwitchStateGet( BSP_SWITCH_2 ) ;
    buttons[APP_SWITCH_3] = !BSP_SwitchStateGet( BSP_SWITCH_3 ) ;
    buttons[APP_SWITCH_4] = !BSP_SwitchStateGet( BSP_SWITCH_4 ) ;
    buttons[APP_SWITCH_5] = !BSP_SwitchStateGet( BSP_SWITCH_5 ) ;
}

//******************************************************************************
// buttons_handleInterrupt()
//
// Called within button change detection ISR
//
//******************************************************************************
void buttons_handleInterrupt()
{
    //Remember new button press state change
    mButtonEvent = 1;
}