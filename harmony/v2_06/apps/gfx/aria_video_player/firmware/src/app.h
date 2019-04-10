/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "gfx/hal/inc/gfx_context.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
    APP_STATE_SPLASH,
	APP_STATE_MENU,
    APP_STATE_SETTINGS,
    APP_STATE_PLAYING,
    APP_STATE_HELP,

	/* TODO: Define states used by the application state machine. */

} APP_STATES;

typedef enum
{
    APP_EVENT_INT_STATE_SHOW_SCREEN,
    APP_EVENT_INT_STATE_INIT,
    APP_EVENT_INT_PLAYBACK_SHOW_CONTROLS,
    APP_EVENT_INT_PLAYBACK_HIDE_CONTROLS,
    APP_EVENT_INT_PLAYBACK_BLACKOUT_SCREEN,
    APP_EVENT_INT_PLAYBACK_CLEAR_BLACKOUT_SCREEN,
    APP_EVENT_INT_PLAYBACK_PLAY_AUTOHIDE_CONTROLS,
    APP_EVENT_INT_PROGRESS_UPDATE,
    APP_EVENT_INT_NO_MEDIA,
    APP_EVENT_INT_MAX = 31,
} APP_INTERNAL_EVENTS;

typedef enum
{
    APP_EVENT_USER_BACK_TO_MAIN_MENU,
    APP_EVENT_USER_PLAYBACK_PLAY,
    APP_EVENT_USER_PLAYBACK_PAUSE,
    APP_EVENT_USER_PLAYBACK_FF,
    APP_EVENT_USER_PLAYBACK_RW,
    APP_EVENT_USER_PLAYBACK_STOP,
    APP_EVENT_USER_PLAYBACK_END,
    APP_EVENT_USER_PLAYBACK_RESTART,
    APP_EVENT_USER_PLAYBACK_SEEK,
    APP_EVENT_USER_START_SD_PLAY,
    APP_EVENT_USER_START_USB_PLAY,
    APP_EVENT_USER_OPEN_SETTINGS,
    APP_EVENT_USER_OPEN_HELP,
    APP_EVENT_USER_SD_INSERTED,
    APP_EVENT_USER_SD_REMOVED,
    APP_EVENT_USER_USB_INSERTED,
    APP_EVENT_USER_USB_REMOVED,            
    APP_EVENT_USER_SETTINGS_SET_FPS,
    APP_EVENT_USER_SETTINGS_SET_RESOLUTION,
    APP_EVENT_USER_SETTINGS_SET_HORZ_ALIGN,
    APP_EVENT_USER_SETTINGS_SET_VERT_ALIGN,
    APP_EVENT_USER_SETTINGS_SHOW_FRAME_RATE,
    APP_EVENT_USER_SETTINGS_HIDE_FRAME_RATE,
    APP_EVENT_USER_MAX = 31,
} APP_USER_EVENTS;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    APP_STATES nextState;
    uint32_t appUserEvent;
    uint32_t appInternalEvent;
    uint32_t appTimerEvent;
} APP_DATA;

extern APP_DATA appData;

#define APP_CHECK_EVENT(events, flag) (events & (1 << flag))

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );
int APP_InitializeSettingsScreen(void);
void APP_InitializePlaybackScreen(void);
int APP_InitializeMainMenuScreen(void);
void APP_SetUserEvent(APP_USER_EVENTS event);
void APP_SetInternalEvent(APP_INTERNAL_EVENTS event);
void APP_ClearUserEvent(APP_USER_EVENTS event);
void APP_ClearInternalEvent(APP_INTERNAL_EVENTS event);

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

