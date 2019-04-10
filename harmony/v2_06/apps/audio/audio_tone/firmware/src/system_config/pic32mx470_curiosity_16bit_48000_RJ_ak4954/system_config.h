/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 2.06
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/
#include "bsp.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "2.06"
#define SYS_VERSION               20600

// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        120000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            60000000ul
#define SYS_CLK_BUS_REFERENCE_1             12288000ul
#define SYS_CLK_UPLL_BEFORE_DIV2_FREQ       96000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         20000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
#define SYS_CLK_CONFIG_FREQ_ERROR_LIMIT     10
#define SYS_CLK_WAIT_FOR_SWITCH             true
#define SYS_CLK_ON_WAIT                     OSC_ON_WAIT_IDLE 
   
/*** Ports System Service Configuration ***/
#define SYS_PORT_B_ANSEL        0x7CDC
#define SYS_PORT_B_TRIS         0x7DDC
#define SYS_PORT_B_LAT          0x0000
#define SYS_PORT_B_ODC          0x0000
#define SYS_PORT_B_CNPU         0x0000
#define SYS_PORT_B_CNPD         0x0000
#define SYS_PORT_B_CNEN         0x0000

#define SYS_PORT_C_ANSEL        0xFFFF
#define SYS_PORT_C_TRIS         0xFFFF
#define SYS_PORT_C_LAT          0x0000
#define SYS_PORT_C_ODC          0x0000
#define SYS_PORT_C_CNPU         0x0000
#define SYS_PORT_C_CNPD         0x0000
#define SYS_PORT_C_CNEN         0x0000

#define SYS_PORT_D_ANSEL        0xD002
#define SYS_PORT_D_TRIS         0xFFFE      // KEEP -- RD6 is switch 1, not generating right
#define SYS_PORT_D_LAT          0x0000
#define SYS_PORT_D_ODC          0x0000
#define SYS_PORT_D_CNPU         0x20B0
#define SYS_PORT_D_CNPD         0x0000
#define SYS_PORT_D_CNEN         0x0040      // KEEP -- RD6 is switch 1, not generating right

#define SYS_PORT_E_ANSEL        0xFF00
#define SYS_PORT_E_TRIS         0xFF08
#define SYS_PORT_E_LAT          0x0000
#define SYS_PORT_E_ODC          0x0000
#define SYS_PORT_E_CNPU         0x0000
#define SYS_PORT_E_CNPD         0x0000
#define SYS_PORT_E_CNEN         0x0000

#define SYS_PORT_F_ANSEL        0xFFC4
#define SYS_PORT_F_TRIS         0xFFFF
#define SYS_PORT_F_LAT          0x0000
#define SYS_PORT_F_ODC          0x0000
#define SYS_PORT_F_CNPU         0x0000
#define SYS_PORT_F_CNPD         0x0000
#define SYS_PORT_F_CNEN         0x0000

#define SYS_PORT_G_ANSEL        0xFCBF
#define SYS_PORT_G_TRIS         0xFFFF
#define SYS_PORT_G_LAT          0x0000
#define SYS_PORT_G_ODC          0x0000
#define SYS_PORT_G_CNPU         0x0000
#define SYS_PORT_G_CNPD         0x0000
#define SYS_PORT_G_CNEN         0x0000


/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

/*** Codec Driver Configuration ***/

#define DRV_AK4954_CLIENTS_NUMBER                           1 
#define DRV_AK4954_INSTANCES_NUMBER                         1
#define DRV_AK4954_INPUT_REFCLOCK    	                	6
#define DRV_AK4954_AUDIO_SAMPLING_RATE                      48000
#define DRV_AK4954_MCLK_SAMPLE_FREQ_MULTPLIER	            (SYS_CLK_BUS_REFERENCE_1/DRV_AK4954_AUDIO_SAMPLING_RATE)
#define DRV_AK4954_BCLK_BIT_CLK_DIVISOR	                	4
#define DRV_AK4954_DELAY_INITIALIZATION                      false
       
#define DRV_AK4954_I2S_DRIVER_MODULE_INDEX_IDX0             DRV_I2S_INDEX_0
#define DRV_AK4954_I2C_DRIVER_MODULE_INDEX_IDX0             DRV_I2C_INDEX_0
#define DRV_AK4954_VOLUME                                   220	
#define DRV_AK4954_VOLUME_MIN                               0x0                                           
#define DRV_AK4954_VOLUME_MAX                               0xFF
#define DRV_AK4954_AUDIO_DATA_FORMAT_MACRO                  DRV_AK4954_AUDIO_DATA_FORMAT_24BIT_MSB_SDTO_16BIT_LSB_SDTI

/* CODEC Driver Abstraction definition */

#define DRV_CODEC_INDEX_0                                   DRV_AK4954_INDEX_0
#define sysObjdrvCodec0                                     sysObj.drvak4954Codec0
#define DRV_CODEC_CHANNEL                                   DRV_AK4954_CHANNEL
#define DRV_CODEC_CHANNEL_LEFT                              DRV_AK4954_CHANNEL_LEFT
#define DRV_CODEC_CHANNEL_RIGHT                             DRV_AK4954_CHANNEL_RIGHT
#define DRV_CODEC_CHANNEL_LEFT_RIGHT                        DRV_AK4954_CHANNEL_LEFT_RIGHT
#define DRV_CODEC_BUFFER_HANDLE                             DRV_AK4954_BUFFER_HANDLE
#define DRV_CODEC_BUFFER_HANDLE_INVALID                     DRV_AK4954_BUFFER_HANDLE_INVALID
#define DRV_CODEC_BUFFER_EVENT_HANDLER                      DRV_AK4954_BUFFER_EVENT_HANDLER
#define DRV_CODEC_BUFFER_EVENT                              DRV_AK4954_BUFFER_EVENT
#define DRV_CODEC_BUFFER_EVENT_COMPLETE                     DRV_AK4954_BUFFER_EVENT_COMPLETE
#define DRV_CODEC_BUFFER_EVENT_ERROR                        DRV_AK4954_BUFFER_EVENT_ERROR
#define DRV_CODEC_BUFFER_EVENT_ABORT                        DRV_AK4954_BUFFER_EVENT_ABORT
#define DRV_CODEC_COMMAND_EVENT_HANDLER                     DRV_AK4954_COMMAND_EVENT_HANDLER
#define DRV_CODEC_VOLUME_MIN                                DRV_AK4954_VOLUME_MIN                                                                             
#define DRV_CODEC_VOLUME_MAX                                DRV_AK4954_VOLUME_MAX 
#define DRV_CODEC_MICROPHONE_TYPE                           DRV_AK4954_INT_EXT_MIC
#define DRV_CODEC_MICROPHONE_TYPE_INTERNAL                  INT_MIC
#define DRV_CODEC_MICROPHONE_TYPE_EXTERNAL                  EXT_MIC
#define DRV_CODEC_MICROPHONE_SOUND                          DRV_AK4954_MONO_STEREO_MIC
#define DRV_CODEC_MICROPHONE_SOUND_NONE                     ALL_ZEROS
#define DRV_CODEC_MICROPHONE_SOUND_MONO_RIGHT               MONO_RIGHT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_MONO_LEFT                MONO_LEFT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_STEREO                   STEREO
#define DRV_CODEC_NUMBER_MIC                                DRV_AK4954_NUMBER_MIC

#define DRV_CODEC_Initialize                                DRV_AK4954_Initialize
#define DRV_CODEC_Deinitialize                              DRV_AK4954_Deinitialize
#define DRV_CODEC_Status                                    DRV_AK4954_Status
#define DRV_CODEC_Tasks                                     DRV_AK4954_Tasks
#define DRV_CODEC_Open                                      DRV_AK4954_Open
#define DRV_CODEC_Close                                     DRV_AK4954_Close
#define DRV_CODEC_BufferEventHandlerSet                     DRV_AK4954_BufferEventHandlerSet
#define DRV_CODEC_BufferAddWrite                            DRV_AK4954_BufferAddWrite
#define DRV_CODEC_BufferAddRead                             DRV_AK4954_BufferAddRead
#define DRV_CODEC_BufferAddWriteRead                        DRV_AK4954_BufferAddWriteRead
#define DRV_CODEC_SamplingRateSet                           DRV_AK4954_SamplingRateSet
#define DRV_CODEC_SamplingRateGet                           DRV_AK4954_SamplingRateGet
#define DRV_CODEC_VolumeSet                                 DRV_AK4954_VolumeSet
#define DRV_CODEC_VolumeGet                                 DRV_AK4954_VolumeGet
#define DRV_CODEC_MuteOn                                    DRV_AK4954_MuteOn
#define DRV_CODEC_MuteOff                                   DRV_AK4954_MuteOff
#define DRV_CODEC_MicrophoneTypeSet                         DRV_AK4954_IntExtMicSet
#define DRV_CODEC_MicrophoneSoundSet                        DRV_AK4954_MonoStereoMicSet
#define DRV_CODEC_SetAudioCommunicationMode                 DRV_AK4954_SetAudioCommunicationMode
#define DRV_CODEC_CommandEventHandlerSet                    DRV_AK4954_CommandEventHandlerSet
#define DRV_CODEC_EnableInitialization                      DRV_AK4954_EnableInitialization
#define DRV_CODEC_IsInitializationDelayed                   DRV_AK4954_IsInitializationDelayed
#define DRV_CODEC_MicSet                                    DRV_AK4954_MicSet
#define DRV_CODEC_BufferQueueFlush                          DRV_AK4954_BufferQueueFlush


// *****************************************************************************
/* I2C Driver Configuration Options
*/
#define DRV_I2C_INTERRUPT_MODE                    		true
#define DRV_I2C_CLIENTS_NUMBER                    		1
#define DRV_I2C_INSTANCES_NUMBER                  		1

#define DRV_I2C_PERIPHERAL_ID_IDX0                		I2C_ID_2
#define DRV_I2C_OPERATION_MODE_IDX0               		DRV_I2C_MODE_MASTER
#define DRV_SCL_PORT_IDX0                               PORT_CHANNEL_F
#define DRV_SCL_PIN_POSITION_IDX0                       PORTS_BIT_POS_5
#define DRV_SDA_PORT_IDX0                               PORT_CHANNEL_F
#define DRV_SDA_PIN_POSITION_IDX0                       PORTS_BIT_POS_4
#define DRV_I2C_BIT_BANG_IDX0                           false
#define DRV_I2C_STOP_IN_IDLE_IDX0                       false
#define DRV_I2C_SMBus_SPECIFICATION_IDX0			    false
#define DRV_I2C_BAUD_RATE_IDX0                    		50000
#define DRV_I2C_BRG_CLOCK_IDX0	                  		60000000
#define DRV_I2C_SLEW_RATE_CONTROL_IDX0      			false
#define DRV_I2C_MASTER_INT_SRC_IDX0               		INT_SOURCE_I2C_2_MASTER
#define DRV_I2C_SLAVE_INT_SRC_IDX0                		
#define DRV_I2C_ERR_MX_INT_SRC_IDX0               		INT_SOURCE_I2C_2_ERROR
#define DRV_I2C_INT_VECTOR_IDX0                         INT_VECTOR_I2C2
#define DRV_I2C_ISR_VECTOR_IDX0                         _I2C_2_VECTOR
#define DRV_I2C_INT_PRIORITY_IDX0                 		INT_PRIORITY_LEVEL1
#define DRV_I2C_INT_SUB_PRIORITY_IDX0             		INT_SUBPRIORITY_LEVEL0
#define DRV_I2C_POWER_STATE_IDX0                  		SYS_MODULE_POWER_RUN_FULL
#define DRV_I2C_INTERRUPT_MODE                    		true


/*** I2S Driver Configuration ***/


#define DRV_I2S_INTERRUPT_MODE					true
#define DRV_I2S_CLIENTS_NUMBER					1
#define DRV_I2S_INSTANCES_NUMBER				1
#define DRV_I2S_STOP_IN_IDLE					false
#define DRV_I2S_PERIPHERAL_ID_IDX0				SPI_ID_2
#define DRV_I2S_USAGE_MODE_IDX0					DRV_I2S_MODE_MASTER
#define DRV_I2S_STOP_IN_IDLE_IDX0				false
#define SPI_BAUD_RATE_CLK_IDX0					SPI_BAUD_RATE_MCLK_CLOCK
#define DRV_I2S_BAUD_RATE                       48000
#define DRV_I2S_CLK_MODE_IDX0					DRV_I2S_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
#define SPI_AUDIO_COMM_WIDTH_IDX0				SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL
#define SPI_AUDIO_TRANSMIT_MODE_IDX0			SPI_AUDIO_TRANSMIT_STEREO
#define SPI_INPUT_SAMPLING_PHASE_IDX0			SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE
#define DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0		DRV_I2S_AUDIO_RIGHT_JUSTIFIED
#define DRV_I2S_TX_INT_SRC_IDX0					INT_SOURCE_SPI_2_TRANSMIT
#define DRV_I2S_RX_INT_SRC_IDX0					INT_SOURCE_SPI_2_RECEIVE
#define QUEUE_SIZE_TX_IDX0                      3
#define QUEUE_SIZE_RX_IDX0                      2
#define DRV_I2S_TX_DMA_CHANNEL_IDX0				DMA_CHANNEL_2
#define DRV_I2S_TX_DMA_SOURCE_IDX0				INT_SOURCE_DMA_2
#define DRV_I2S_POWER_STATE_IDX0				SYS_MODULE_POWER_RUN_FULL
#define DRV_I2S_QUEUE_DEPTH_COMBINED     		5

/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           3
#define DRV_TMR_CLIENTS_NUMBER             1

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_4
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_4
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T4
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_4_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

#define DRV_TMR_PERIPHERAL_ID_IDX1          TMR_ID_1
#define DRV_TMR_INTERRUPT_SOURCE_IDX1       INT_SOURCE_TIMER_1
#define DRV_TMR_INTERRUPT_VECTOR_IDX1       INT_VECTOR_T1
#define DRV_TMR_ISR_VECTOR_IDX1             _TIMER_1_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX1     INT_PRIORITY_LEVEL1
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX1 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX1           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX1               TMR_PRESCALE_VALUE_1
#define DRV_TMR_OPERATION_MODE_IDX1         DRV_TMR_OPERATION_MODE_16_BIT

#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX1     false
#define DRV_TMR_POWER_STATE_IDX1            SYS_MODULE_POWER_RUN_FULL
/*** Timer Driver 2 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX2          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX2       INT_SOURCE_TIMER_3
#define DRV_TMR_INTERRUPT_VECTOR_IDX2       INT_VECTOR_T3
#define DRV_TMR_ISR_VECTOR_IDX2             _TIMER_3_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX2     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX2 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX2           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX2               TMR_PRESCALE_VALUE_1
#define DRV_TMR_OPERATION_MODE_IDX2         DRV_TMR_OPERATION_MODE_32_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX2     false
#define DRV_TMR_POWER_STATE_IDX2            SYS_MODULE_POWER_RUN_FULL
 
 
// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* BSP Configuration Options
*/
#define BSP_OSC_FREQUENCY 8000000


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
/*** Application Defined Pins ***/

/*** Functions for BSP_LED_2 pin ***/
#define BSP_LED_2Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_2On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_2Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_2StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)

/*** Functions for BSP_LED_3 pin ***/
#define BSP_LED_3Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7)
#define BSP_LED_3On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7)
#define BSP_LED_3Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7)
#define BSP_LED_3StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7)

/*** Functions for BSP_RGB_LED_RED pin ***/
#define BSP_RGB_LED_REDToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0)
#define BSP_RGB_LED_REDOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0)
#define BSP_RGB_LED_REDOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0)
#define BSP_RGB_LED_REDStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0)))

/*** Functions for BSP_RGB_LED_GREEN pin ***/
#define BSP_RGB_LED_GREENToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1)))

/*** Functions for BSP_RGB_LED_BLUE pin ***/
#define BSP_RGB_LED_BLUEToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2)
#define BSP_RGB_LED_BLUEOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2)
#define BSP_RGB_LED_BLUEOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2)
#define BSP_RGB_LED_BLUEStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2)))

/*** Functions for BSP_LED_1 pin ***/
#define BSP_LED_1Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_1On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_1Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_1StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)

/*** Functions for BSP_SWITCH_1 pin ***/
#define BSP_SWITCH_1StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6)

/*** Functions for BSP_AK4384_CONTROL_CLK pin ***/
#define BSP_AK4384_CONTROL_CLKToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5)
#define BSP_AK4384_CONTROL_CLKOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5)
#define BSP_AK4384_CONTROL_CLKOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5)
#define BSP_AK4384_CONTROL_CLKStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5)
#define BSP_AK4384_CONTROL_CLKStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5, Value)

/*** Functions for BSP_BM64_WAKEUP pin ***/
#define BSP_BM64_WAKEUPToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_BM64_WAKEUPOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_BM64_WAKEUPOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_BM64_WAKEUPStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_BM64_WAKEUPStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1, Value)

/*** Functions for BSP_BM64_RST pin ***/
#define BSP_BM64_RSTToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_BM64_RSTOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_BM64_RSTOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_BM64_RSTStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_BM64_RSTStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0, Value)

/*** Functions for BSP_AK4384_CONTROL_CS pin ***/
#define BSP_AK4384_CONTROL_CSToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9)
#define BSP_AK4384_CONTROL_CSOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9)
#define BSP_AK4384_CONTROL_CSOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9)
#define BSP_AK4384_CONTROL_CSStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9)
#define BSP_AK4384_CONTROL_CSStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9, Value)

/*** Functions for BSP_AK4384_CONTROL_DO pin ***/
#define BSP_AK4384_CONTROL_DOToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_AK4384_CONTROL_DOOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_AK4384_CONTROL_DOOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_AK4384_CONTROL_DOStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_AK4384_CONTROL_DOStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15, Value)

/*** Functions for BSP_AK4201_AMPLIFIER_PDN pin ***/
#define BSP_AK4201_AMPLIFIER_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0)
#define BSP_AK4201_AMPLIFIER_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0)
#define BSP_AK4201_AMPLIFIER_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0)
#define BSP_AK4201_AMPLIFIER_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0)
#define BSP_AK4201_AMPLIFIER_PDNStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0, Value)

/*** Functions for BSP_AK4954_PDN pin ***/
#define BSP_AK4954_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7)
#define BSP_AK4954_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7)
#define BSP_AK4954_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7)
#define BSP_AK4954_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7)
#define BSP_AK4954_PDNStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7, Value)


/*** Application Instance 0 Configuration ***/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/
