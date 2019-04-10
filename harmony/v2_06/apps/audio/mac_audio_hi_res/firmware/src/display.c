/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   display.c

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

void DisplayTasks(void)
{
    laString tempStr;
    
    if(appData.display.status == APP_DISP_STATUS_USB_CONNECTED)
    {
        laWidget_SetVisible((laWidget*)GFX_USB_DISCONNECTED, LA_FALSE);
        laWidget_SetVisible((laWidget*)GFX_USB_CONNECTED, LA_TRUE);
         
        //Volume progress bar control interfering with USB, disabled for now
        //laProgressBarWidget_SetValue(GFX_VOLUME_PBAR, appData.display.volumeP);
        tempStr = laString_CreateFromCharBuffer(appData.display.volumePercent,&LiberationSans12);
        laLabelWidget_SetText(GFX_VOLUME_VALUE, tempStr);
        laString_Destroy(&tempStr);
        
        laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
        laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_TRUE);         
    }    
    else if(appData.display.status == APP_DISP_STATUS_USB_DISCONNECTED)
    {   
        laWidget_SetVisible((laWidget*)GFX_USB_DISCONNECTED, LA_TRUE);
        laWidget_SetVisible((laWidget*)GFX_USB_CONNECTED, LA_FALSE);        
    }             
    else if(appData.display.status == APP_DISP_STATUS_APP_MUTE_ON)
    {       
        laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_TRUE);
        laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_FALSE);        
    }
    else if(appData.display.status == APP_DISP_STATUS_MUTE_OFF)
    {      
        laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
        laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_TRUE);         
    }
    else if(appData.display.status == APP_DISP_STATUS_VOLUME_INCREASE ||
            appData.display.status == APP_DISP_STATUS_VOLUME_DECREASE)
    {
        //Volume progress bar control interfering with USB, disabled for now        
        //laProgressBarWidget_SetValue(GFX_VOLUME_PBAR, appData.display.volumeP);
        tempStr = laString_CreateFromCharBuffer(appData.display.volumePercent,&LiberationSans12);
        laLabelWidget_SetText(GFX_VOLUME_VALUE, tempStr);                
        laString_Destroy(&tempStr);
    }
    else if(appData.display.status == APP_DISP_STATUS_SAMPLING_RATE)
    {
        tempStr = laString_CreateFromCharBuffer(appData.display.samplingRate,&LiberationSans12);
        laLabelWidget_SetText(GFX_FREQ_VALUE, tempStr);        
        laString_Destroy(&tempStr);
    }
    
    appData.display.update = false;      
}