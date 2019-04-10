/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   Display_tasks.c

  Summary:
    Contains the functional implementation of display task for

  Description:
    This file contains the functional implementation of data parsing functions
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "gfx/libaria/libaria_init.h"

static char * modeSineTxt   = "Sine";
static char * modeChirpTxt  = "Chirp";
static char * modeSilentTxt = "     ";

void DigitalMeterValueSet32(laLabelWidget* lbl, uint32_t value, BOOL addPercent);

void APP_DisplayInit(GUI_DATA * guiData, AUDIO_GENERATE_STATUS * agStatus)
{
    laString tempStr;  
    
    //Parameters
    DigitalMeterValueSet32(GFX_F1_VALUE, (uint32_t) guiData->f1Hz, false);
    laWidget_SetScheme((laWidget*)GFX_F1_BOX, &redBoxes);
    laWidget_SetScheme((laWidget*)GFX_F2_BOX, &redBoxes);
    laWidget_SetScheme((laWidget*)GFX_TMS_BOX,&redBoxes);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_BOX,&redBoxes);    
    DigitalMeterValueSet32(GFX_F2_VALUE, (uint32_t) guiData->f2Hz, false);
    if (guiData->timeDeltaMs > 0)
    {
        DigitalMeterValueSet32(GFX_TMS_VALUE, (uint32_t) guiData->timeDeltaMs, false);
    }
    else
    {
        DigitalMeterValueSet32(GFX_TMS_VALUE, (uint32_t) 999999, false); //inf
    }
#ifndef HAS_VOLUME_CTRL    
    DigitalMeterValueSet32(GFX_VOLUME_VALUE, (uint32_t) guiData->volume, true);
#endif    
    //Mode
    if (guiData->mode == MODE_TONE)
    {
        tempStr = laString_CreateFromCharBuffer((char *) modeSineTxt, &LiberationSans14);
        laLabelWidget_SetText(GFX_TONEMODE, tempStr);
        laString_Destroy(&tempStr);
    }
    else if (guiData->mode == MODE_CHIRP)
    {
        tempStr = laString_CreateFromCharBuffer((char *) modeChirpTxt, &LiberationSans14);
        laLabelWidget_SetText(GFX_TONEMODE, tempStr);
        laString_Destroy(&tempStr);            
    }
    else
    {
        //Silent
        tempStr = laString_CreateFromCharBuffer((char *) modeSilentTxt, &LiberationSans14);
        laLabelWidget_SetText(GFX_TONEMODE, tempStr);
        laString_Destroy(&tempStr);        
    }
    
    if (guiData->onOff)
    {
        laButtonWidget_SetPressedImage(GFX_PLAYPAUSE, &AudioMute16_3p);
        laButtonWidget_SetReleasedImage(GFX_PLAYPAUSE, &AudioMute16_3); 
    }
    else
    {
        laButtonWidget_SetPressedImage(GFX_PLAYPAUSE, &AudioPlay16_3p);
        laButtonWidget_SetReleasedImage(GFX_PLAYPAUSE, &AudioPlay16_3);        
    }
}

//******************************************************************************
// APP_DisplayTask()
//
// GUI_DATA:
//       bool       displayUpdate;
//       GUI_MODE   mode;     MODE_TONE/MODE_CHIRP
//       GUI_SELECT select;   SELECT_F1/SELECT_F2/SELECT_T/SELECT_DUTY
//       int32_t    f1Hz;
//       int32_t    f2Hz;
//       int32_t    timeDeltaMs; //NOTE <0 implies inf. 
//       //int16_t    dutyCycle;
//       bool       onOff;   //Generated output
//
//******************************************************************************
void APP_DisplayTask(GUI_DATA * guiData)
{
    //int Red, Blue, Green, temp;
    laString tempStr;    

    if (guiData->displayUpdate == true)
    {
        if (guiData->lastButton != INVALID_BUTTON)
        {
            //Update the selected parameter value display
            switch (guiData->lastButton)
            {
                case TOUCH_F1_PLUS:
                case TOUCH_F1_MINUS:                
                    DigitalMeterValueSet32(GFX_F1_VALUE, (uint32_t) guiData->f1Hz, false);
                    break;

                case TOUCH_F2_PLUS:
                case TOUCH_F2_MINUS:
                    DigitalMeterValueSet32(GFX_F2_VALUE, (uint32_t) guiData->f2Hz, false);
                    break;

                case TOUCH_T_PLUS:
                case TOUCH_T_MINUS:
                    if (guiData->timeDeltaMs > 0)
                    {
                        DigitalMeterValueSet32(GFX_TMS_VALUE,  (uint32_t) guiData->timeDeltaMs, false);
                    }
                    else
                    {
                        //Show infinity
                        DigitalMeterValueSet32(GFX_TMS_VALUE,  (uint32_t) 999999, false);              
                    }
                    break;
                    
                case TOUCH_VOLUME_PLUS:
                case TOUCH_VOLUME_MINUS:
                    DigitalMeterValueSet32(GFX_VOLUME_VALUE, (uint32_t) guiData->volume, true);                  
                    break;                    

                default: break;
            }
        }
        
        //Mode Display Change
        if (guiData->changeToneMode)
        {
            //Mode
            if (guiData->mode == MODE_TONE)
            {
                tempStr = laString_CreateFromCharBuffer((char *) modeSineTxt, &LiberationSans14);
                laLabelWidget_SetText(GFX_TONEMODE, tempStr);
                laString_Destroy(&tempStr);
            }
            else if (guiData->mode == MODE_CHIRP)
            {
                tempStr = laString_CreateFromCharBuffer((char *) modeChirpTxt, &LiberationSans14);
                laLabelWidget_SetText(GFX_TONEMODE, tempStr);
                laString_Destroy(&tempStr);            
            }
            else
            {
                //Silent
                tempStr = laString_CreateFromCharBuffer((char *) modeSilentTxt, &LiberationSans14);
                laLabelWidget_SetText(GFX_TONEMODE, tempStr);
                laString_Destroy(&tempStr);        
            }

            if (guiData->onOff)
            {
                laButtonWidget_SetPressedImage(GFX_PLAYPAUSE, &AudioMute16_3p);
                laButtonWidget_SetReleasedImage(GFX_PLAYPAUSE, &AudioMute16_3);
            }
            else
            {
                laButtonWidget_SetPressedImage(GFX_PLAYPAUSE, &AudioPlay16_3p);
                laButtonWidget_SetReleasedImage(GFX_PLAYPAUSE, &AudioPlay16_3);      
            }
        }

        guiData->displayUpdate = false;
        guiData->changeMode = false;
        guiData->changeToneMode = false;

    } //End displayUpdate

} //End APP_DisplayTask()

void DigitalMeterValueSet32(laLabelWidget* lbl, uint32_t value, BOOL addPercent)
{
    char valueStrBuf[6];     // enough for 99999
    laString tempStr;
    
    if (value == 999999)
    {
        //tempStr = laString_CreateFromCharBuffer((char *) "disable", &LiberationSans12);
        tempStr = laString_CreateFromCharBuffer((char *) "disable", &LiberationSans12);
        laLabelWidget_SetText(lbl, tempStr);
        laString_Destroy(&tempStr);     
    }
    else
    {
        if (value > 99999)
        {
            value = 99999;
        }
        if (addPercent)
        {
            sprintf(valueStrBuf,"%2d%%",value);        
        }
        else
        {
            sprintf(valueStrBuf,"%5d",value);
        }
        
        tempStr = laString_CreateFromCharBuffer((char *) valueStrBuf, &LiberationSans12);
        laLabelWidget_SetText(lbl, tempStr);
        laString_Destroy(&tempStr);    
    }
}
