/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_events.h

  Summary:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated definitions header based on output by the MPLAB Harmony
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

#ifndef _LIBARIA_EVENTS_H
#define _LIBARIA_EVENTS_H

#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_init.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

//CUSTOM CODE - Please do not remove while regenerating
    void Load_image_from_usb(void);
    void Load_image_from_int(void);
    void Load_image_from_sqi(void);
    void Slides(void);
    void HitSlidesButton(void);
    void PlayAllSource(void);
    void InfoButton(void);
    void BackFromHelp(void);
// END OF CUSTOM CODE



// Generated Event Handler - Origin: ButtonUSB, Event: ReleasedEvent
void ButtonUSB_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonSQI, Event: ReleasedEvent
void ButtonSQI_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonIntMem, Event: ReleasedEvent
void ButtonIntMem_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonInfo, Event: ReleasedEvent
void ButtonInfo_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonSlides, Event: ReleasedEvent
void ButtonSlides_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonAllPlay, Event: ReleasedEvent
void ButtonAllPlay_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonWidget_back, Event: ReleasedEvent
void ButtonWidget_back_ReleasedEvent(laButtonWidget* btn);



//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_EVENTS_H
/*******************************************************************************
 End of File
*/
