/*******************************************************************************
  AUDIO TONE CONTROL DEMO 

  Company:
    Microchip Technology Inc.

  File Name:
    app_config.h (24Bit_Audio Version)

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

#include "system_definitions.h"

//NOTE: Below only depends on system_definition.h CAL

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END


//Voice Prompt Parameters
#define LOW_FREQ  300
#define MID_FREQ  400
#define MIDHIGH_FREQ 600
#define HIGH_FREQ 800
#define HIGHER_FREQ 1000

//#define PROMPT_DURATION_SAMPLES ((int16_t)(floor(SAMPLERATE * SEC)))
//#define PROMPT_DURATION_SAMPLES   1024  //Pop prompt
#define PROMPT_POP_SEC        0.0400  //Pop Prompt Duration
#define PROMPT_LONG_SEC       2.000  //Long Prompt Duration 
#define PROMPT_SHORT_SEC      0.240  //Short prompt duration

//Math Constants (Program Memory)
#define PI     3.1415926
#define TWOPI (2*3.1414926)

#if 0
//BT Channel Parameters
#define PROMPT_POP_SAMPLES    1024   //Sec*SAMPLERATE  
//#define PROMPT_LONG_SAMPLES   132300 //3 Sec
#define PROMPT_LONG_SAMPLES   44100    //<3 Sec
#define PROMPT_LONGER_SAMPLES   (1*44100)   //3 Sec
#define PROMPT_SHORT_SAMPLES  20584    //.240*44100 

#define INV_AUDIO_PROMPT_SAMPLERATE (1.0/AUDIO_PROMPT_SAMPLERATE)
#define INV_AP_SAMPLERATEQ      Fl2FxPnt16(0.301029996,11)
#endif

#define MAX_INT 2e15
//APP_ButtonTask: Long button press
#define INVALID_BUTTON -1
#define APP_BUTTON1 1
#define APP_BUTTON2 2
#define APP_BUTTON3 3
#define APP_BUTTON4 4
#define APP_BUTTON5 5

// User Configuration
#define BT_AUDIO_DK
#ifdef BT_AUDIO_DK

// User Config - I2S Buffer Queue
#define APP_CODEC_WRITE_QUEUE_SIZE                      QUEUE_SIZE_TX_IDX0

//16 Bit stereo I2S Codec Data
#define APP_TONE_TYPE                                   DRV_I2S_DATA24
#define APP_DATA_TYPE                                   uint32_t

// User Config - BT Audio DK Application LED's 
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

//User Config - BT Audio DK Buttons 
#define APP_BUTTON1_PIN   (1<<BSP_SWITCH_1)
#define APP_BUTTON2_PIN   (1<<BSP_SWITCH_2)
#define APP_BUTTON3_PIN   (1<<BSP_SWITCH_3)
#define APP_BUTTON4_PIN   (1<<BSP_SWITCH_4)
#define APP_BUTTON5_PIN   (1<<BSP_SWITCH_5)

#define APP_BUTTON_CN_PRIO      INT_PRIORITY_LEVEL2
#define APP_BUTTON_CN_SUBPRIO   INT_SUBPRIORITY_LEVEL0
#define APP_BUTTON_CN_ISR       _CHANGE_NOTICE_VECTOR
#define APP_BUTTON_CN_SOURCE_1  INT_SOURCE_CHANGE_NOTICE_A
#define APP_BUTTON_CN_SOURCE_2  INT_SOURCE_CHANGE_NOTICE_B
#define APP_BUTTON_CN_VECTOR    INT_VECTOR_CHANGE_NOTICE_B

//User Config - BT Audio DK (MX processor) - Button Interrupts
#define APP_READ_BUTTON_PORTS() SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A) |\
                                SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B)

#define APP_ENABLE_BUTTON_CHANGE_NOTICE()  \
      PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
      PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
      PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
      PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
      PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5)
#endif //BT_AUDIO_DK

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END
