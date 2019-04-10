/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_events.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version 2.06
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

#include "gfx/libaria/libaria_events.h"

// PlayButton - ReleasedEvent
void PlayButton_ReleasedEvent(laButtonWidget* btn)
{
    // PlayButton_ReleasedEvent
    if(PlayButton->releasedImage == (&Pause)){
            // if current state is pause, change to play
            laButtonWidget_SetReleasedImage((laButtonWidget*)PlayButton, &Play);
            APP_PlayerCommand ( PLAYER_CMD_PAUSE );
        }else{
            laButtonWidget_SetReleasedImage((laButtonWidget*)PlayButton, &Pause);
            APP_PlayerCommand ( PLAYER_CMD_PLAY );
        }
}

// NextTrackButton - ReleasedEvent
void NextTrackButton_ReleasedEvent(laButtonWidget* btn)
{
    // NextTrackButton_PressedEvent
    APP_PlayerCommand ( PLAYER_CMD_NEXT_FILE );
}

// PrevTrackButton - ReleasedEvent
void PrevTrackButton_ReleasedEvent(laButtonWidget* btn)
{
    // PrevTrackButton_ReleaseEvent
    APP_PlayerCommand ( PLAYER_CMD_PREV_FILE );
}

// ToFileExplorerButton - ReleasedEvent
void ToFileExplorerButton_ReleasedEvent(laButtonWidget* btn)
{
    // ToFileExplorerButton_PressedEvent
    APP_PlayerEventHandler (PLAYER_EVENT_GOTO_EXPLORER_WINDOW,0);
}

// ToPlayerButton - ReleasedEvent
void ToPlayerButton_ReleasedEvent(laButtonWidget* btn)
{
    // ToPlayerButton_PressedEvent
    APP_PlayerEventHandler (PLAYER_EVENT_GOTO_PLAYER_WINDOW,0);
}





