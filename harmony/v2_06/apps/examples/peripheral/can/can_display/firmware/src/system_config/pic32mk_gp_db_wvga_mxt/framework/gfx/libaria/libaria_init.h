/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_macros.h

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

#ifndef _LIBARIA_INIT_H
#define _LIBARIA_INIT_H

#ifndef NATIVE
#include "system_config.h"
#include "system_definitions.h"
#endif

#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_events.h"

#include "gfx/gfx_assets.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   1

// reference IDs for generated libaria screens
// screen "default"
#define default_ID    0



extern laScheme defaultScheme;
extern laWidget* PanelSettings;
extern laGroupBoxWidget* GroupBoxWidget5;
extern laRadioButtonWidget* RadioButtonWidget6;
extern laRadioButtonWidget* RadioButtonWidget7;
extern laButtonWidget* ButtonWidget8;
extern laWidget* PanelMain;
extern laGroupBoxWidget* CanRxMessageGB;
extern laLabelWidget* CAN_MESSAGE_1;
extern laLabelWidget* CAN_MESSAGE_2;
extern laLabelWidget* CAN_MESSAGE_3;
extern laLabelWidget* CAN_MESSAGE_5;
extern laLabelWidget* CAN_MESSAGE_4;
extern laLabelWidget* CAN_MESSAGE_6;
extern laLabelWidget* CAN_MESSAGE_7;
extern laLabelWidget* CAN_MESSAGE_8;
extern laLabelWidget* CAN_Address;
extern laLabelWidget* CAN_DLC;
extern laLabelWidget* LabelWidget14;
extern laLabelWidget* LabelWidget12;
extern laLabelWidget* LabelWidget11;
extern laLabelWidget* LabelWidget10;
extern laLabelWidget* LabelWidget13;
extern laLabelWidget* LabelWidget15;
extern laLabelWidget* LabelWidget16;
extern laLabelWidget* LabelWidget19;
extern laLabelWidget* LabelWidget18;
extern laLabelWidget* LabelWidget17;
extern laButtonWidget* TxMessage1;
extern laGroupBoxWidget* TxGroupBox;
extern laLabelWidget* TX_DATA_1;
extern laLabelWidget* TX_DATA_2;
extern laLabelWidget* TX_DATA_3;
extern laLabelWidget* TX_DATA_5;
extern laLabelWidget* TX_DATA_4;
extern laLabelWidget* TX_DATA_6;
extern laLabelWidget* TX_DATA_7;
extern laLabelWidget* TX_DATA_8;
extern laLabelWidget* CAN_TX_ID;
extern laLabelWidget* CAN_TX_DLC;
extern laLabelWidget* LabelWidget21;
extern laLabelWidget* LabelWidget22;
extern laLabelWidget* lblTxDlc;
extern laLabelWidget* lblTxId;
extern laLabelWidget* LabelWidget25;
extern laLabelWidget* LabelWidget26;
extern laLabelWidget* LabelWidget27;
extern laLabelWidget* LabelWidget28;
extern laLabelWidget* LabelWidget29;
extern laLabelWidget* LabelWidget30;
extern laButtonWidget* TxMessage2;
extern laGradientWidget* GradientWidget2;
extern laImageWidget* ImageWidget1;
extern laLabelWidget* LabelWidget2;
extern laLabelWidget* VersionNumberlbl;
extern laButtonWidget* SettingsButton;
extern laButtonWidget* MainMenu;


int32_t libaria_initialize(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_INIT_H
/*******************************************************************************
 End of File
*/
