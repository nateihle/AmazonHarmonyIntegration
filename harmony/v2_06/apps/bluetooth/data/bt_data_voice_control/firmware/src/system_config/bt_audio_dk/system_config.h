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
#define SYS_CLK_FREQ                        96000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            48000000ul
#define SYS_CLK_BUS_REFERENCE_1             4096000ul
#define SYS_CLK_UPLL_BEFORE_DIV2_FREQ       96000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         12000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       0ul
#define SYS_CLK_CONFIG_FREQ_ERROR_LIMIT     10
#define SYS_CLK_WAIT_FOR_SWITCH             true
#define SYS_CLK_ON_WAIT                     OSC_ON_WAIT_IDLE 
   
/*** Ports System Service Configuration ***/
#define SYS_PORT_A_ANSEL        0x3900
#define SYS_PORT_A_TRIS         0xFD0F
#define SYS_PORT_A_LAT          0x0000
#define SYS_PORT_A_ODC          0x0000
#define SYS_PORT_A_CNPU         0x0403
#define SYS_PORT_A_CNPD         0x0000
#define SYS_PORT_A_CNEN         0x0403

#define SYS_PORT_B_ANSEL        0x00FB
#define SYS_PORT_B_TRIS         0x7FFF
#define SYS_PORT_B_LAT          0x0000
#define SYS_PORT_B_ODC          0x0000
#define SYS_PORT_B_CNPU         0x7000
#define SYS_PORT_B_CNPD         0x0000
#define SYS_PORT_B_CNEN         0x7000

#define SYS_PORT_C_ANSEL        0x0FE1
#define SYS_PORT_C_TRIS         0xFFFD
#define SYS_PORT_C_LAT          0x0000
#define SYS_PORT_C_ODC          0x0000
#define SYS_PORT_C_CNPU         0x0000
#define SYS_PORT_C_CNPD         0x0000
#define SYS_PORT_C_CNEN         0x0000

#define SYS_PORT_D_ANSEL        0x0000
#define SYS_PORT_D_TRIS         0xE7F1
#define SYS_PORT_D_LAT          0x0000
#define SYS_PORT_D_ODC          0x0000
#define SYS_PORT_D_CNPU         0x0000
#define SYS_PORT_D_CNPD         0x0000
#define SYS_PORT_D_CNEN         0x0000

#define SYS_PORT_E_ANSEL        0xFC00
#define SYS_PORT_E_TRIS         0xFFFF
#define SYS_PORT_E_LAT          0x0000
#define SYS_PORT_E_ODC          0x0000
#define SYS_PORT_E_CNPU         0x0000
#define SYS_PORT_E_CNPD         0x0000
#define SYS_PORT_E_CNEN         0x0000

#define SYS_PORT_F_ANSEL        0xCEC0
#define SYS_PORT_F_TRIS         0xFFFF
#define SYS_PORT_F_LAT          0x0000
#define SYS_PORT_F_ODC          0x0000
#define SYS_PORT_F_CNPU         0x0000
#define SYS_PORT_F_CNPD         0x0000
#define SYS_PORT_F_CNEN         0x0000

#define SYS_PORT_G_ANSEL        0x0C3C
#define SYS_PORT_G_TRIS         0x7FFF
#define SYS_PORT_G_LAT          0x0000
#define SYS_PORT_G_ODC          0x0000
#define SYS_PORT_G_CNPU         0x0003
#define SYS_PORT_G_CNPD         0x0000
#define SYS_PORT_G_CNEN         0x0000


/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true
/*** Timer System Service Configuration ***/
#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      3
#define SYS_TMR_FREQUENCY               100
#define SYS_TMR_FREQUENCY_TOLERANCE     5
#define SYS_TMR_UNIT_RESOLUTION         1000
#define SYS_TMR_CLIENT_TOLERANCE        1
#define SYS_TMR_INTERRUPT_NOTIFICATION  true

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

/*** Codec Driver Configuration ***/

#define DRV_AK4642_CLIENTS_NUMBER                           1
#define DRV_AK4642_INSTANCES_NUMBER                         1
#define DRV_AK4642_INPUT_REFCLOCK    	                	0
#define DRV_AK4642_AUDIO_SAMPLING_RATE                      16000
#define DRV_AK4642_MCLK_SAMPLE_FREQ_MULTPLIER               (SYS_CLK_BUS_REFERENCE_1/DRV_AK4642_AUDIO_SAMPLING_RATE)
#define DRV_AK4642_BCLK_BIT_CLK_DIVISOR                     4

#define DRV_AK4642_I2S_DRIVER_MODULE_INDEX_IDX0             DRV_I2S_INDEX_0
#define DRV_AK4642_I2C_DRIVER_MODULE_INDEX_IDX0             DRV_I2C_INDEX_0
#define DRV_AK4642_VOLUME                                   150	
#define DRV_AK4642_VOLUME_MIN                               0x0                                           
#define DRV_AK4642_VOLUME_MAX                               0xFF
#define DRV_AK4642_AUDIO_DATA_FORMAT_MACRO                  DRV_AK4642_AUDIO_DATA_FORMAT_I2S
/* CODEC Driver Abstraction definition */

#define DRV_CODEC_INDEX_0                                   DRV_AK4642_INDEX_0
#define sysObjdrvCodec0                                     sysObj.drvak4642Codec0
#define DRV_CODEC_CHANNEL                                   DRV_AK4642_CHANNEL
#define DRV_CODEC_CHANNEL_LEFT                              DRV_AK4642_CHANNEL_LEFT
#define DRV_CODEC_CHANNEL_RIGHT                             DRV_AK4642_CHANNEL_RIGHT
#define DRV_CODEC_CHANNEL_LEFT_RIGHT                        DRV_AK4642_CHANNEL_LEFT_RIGHT
#define DRV_CODEC_BUFFER_HANDLE                             DRV_AK4642_BUFFER_HANDLE
#define DRV_CODEC_BUFFER_HANDLE_INVALID                     DRV_AK4642_BUFFER_HANDLE_INVALID
#define DRV_CODEC_BUFFER_EVENT_HANDLER                      DRV_AK4642_BUFFER_EVENT_HANDLER
#define DRV_CODEC_BUFFER_EVENT                              DRV_AK4642_BUFFER_EVENT
#define DRV_CODEC_BUFFER_EVENT_COMPLETE                     DRV_AK4642_BUFFER_EVENT_COMPLETE
#define DRV_CODEC_BUFFER_EVENT_ERROR                        DRV_AK4642_BUFFER_EVENT_ERROR
#define DRV_CODEC_BUFFER_EVENT_ABORT                        DRV_AK4642_BUFFER_EVENT_ABORT
#define DRV_CODEC_COMMAND_EVENT_HANDLER                     DRV_AK4642_COMMAND_EVENT_HANDLER
#define DRV_CODEC_VOLUME_MIN                                DRV_AK4642_VOLUME_MIN
#define DRV_CODEC_VOLUME_MAX                                DRV_AK4642_VOLUME_MAX
//#define DRV_CODEC_MICROPHONE_TYPE                         DRV_AK4642_INT_EXT_MIC
#define DRV_CODEC_MICROPHONE_SELECT                         DRV_AK4642_MIC
#define DRV_CODEC_MICROPHONE_SELECT_1                       MIC1
#define DRV_CODEC_MICROPHONE_SELECT_2                       MIC2
#define DRV_CODEC_MICROPHONE_TYPE_INTERNAL                  INT_MIC
#define DRV_CODEC_MICROPHONE_TYPE_EXTERNAL                  EXT_MIC
#define DRV_CODEC_MICROPHONE_SOUND                          DRV_AK4642_MONO_STEREO_MIC
#define DRV_CODEC_MICROPHONE_SOUND_NONE                     ALL_ZEROS
#define DRV_CODEC_MICROPHONE_SOUND_MONO_RIGHT               MONO_RIGHT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_MONO_LEFT                MONO_LEFT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_STEREO                   STEREO

#define DRV_CODEC_Initialize                                DRV_AK4642_Initialize
#define DRV_CODEC_Deinitialize                              DRV_AK4642_Deinitialize
#define DRV_CODEC_Status                                    DRV_AK4642_Status
#define DRV_CODEC_Tasks                                     DRV_AK4642_Tasks
#define DRV_CODEC_Open                                      DRV_AK4642_Open
#define DRV_CODEC_Close                                     DRV_AK4642_Close
#define DRV_CODEC_BufferEventHandlerSet                     DRV_AK4642_BufferEventHandlerSet
#define DRV_CODEC_BufferAddWrite                            DRV_AK4642_BufferAddWrite
#define DRV_CODEC_BufferAddRead                             DRV_AK4642_BufferAddRead
#define DRV_CODEC_BufferAddWriteRead                        DRV_AK4642_BufferAddWriteRead
#define DRV_CODEC_SamplingRateSet                           DRV_AK4642_SamplingRateSet
#define DRV_CODEC_SamplingRateGet                           DRV_AK4642_SamplingRateGet
#define DRV_CODEC_VolumeSet                                 DRV_AK4642_VolumeSet
#define DRV_CODEC_VolumeGet                                 DRV_AK4642_VolumeGet
#define DRV_CODEC_MuteOn                                    DRV_AK4642_MuteOn
#define DRV_CODEC_MuteOff                                   DRV_AK4642_MuteOff

#define DRV_CODEC_MicrophoneTypeSet                         DRV_AK4642_IntExtMicSet
#define DRV_CODEC_MicSet                                    DRV_AK4642_MicSet
#define DRV_CODEC_BufferQueueFlush                          DRV_AK4642_BufferQueueFlush

#define DRV_CODEC_MicrophoneSoundSet                        DRV_AK4642_MonoStereoMicSet
#define DRV_CODEC_SetAudioCommunicationMode                 DRV_AK4642_SetAudioCommunicationMode
#define DRV_CODEC_CommandEventHandlerSet                    DRV_AK4642_CommandEventHandlerSet


// *****************************************************************************
/* I2C Driver Configuration Options
*/
#define DRV_I2C_INTERRUPT_MODE                    		true
#define DRV_I2C_CLIENTS_NUMBER                    		1
#define DRV_I2C_INSTANCES_NUMBER                  		1

#define DRV_I2C_PERIPHERAL_ID_IDX0                		I2C_ID_1
#define DRV_I2C_OPERATION_MODE_IDX0               		DRV_I2C_MODE_MASTER
#define DRV_SCL_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SCL_PIN_POSITION_IDX0                       PORTS_BIT_POS_14
#define DRV_SDA_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SDA_PIN_POSITION_IDX0                       PORTS_BIT_POS_15
#define DRV_I2C_BIT_BANG_IDX0                           false
#define DRV_I2C_STOP_IN_IDLE_IDX0                       false
#define DRV_I2C_SMBus_SPECIFICATION_IDX0			    false
#define DRV_I2C_BAUD_RATE_IDX0                    		50000
#define DRV_I2C_BRG_CLOCK_IDX0	                  		48000000
#define DRV_I2C_SLEW_RATE_CONTROL_IDX0      			false
#define DRV_I2C_MASTER_INT_SRC_IDX0               		INT_SOURCE_I2C_1_MASTER
#define DRV_I2C_SLAVE_INT_SRC_IDX0                		
#define DRV_I2C_ERR_MX_INT_SRC_IDX0               		INT_SOURCE_I2C_1_ERROR
#define DRV_I2C_INT_VECTOR_IDX0                         INT_VECTOR_I2C1
#define DRV_I2C_ISR_VECTOR_IDX0                         _I2C_1_VECTOR
#define DRV_I2C_INT_PRIORITY_IDX0                 		INT_PRIORITY_LEVEL1
#define DRV_I2C_INT_SUB_PRIORITY_IDX0             		INT_SUBPRIORITY_LEVEL0
#define DRV_I2C_POWER_STATE_IDX0                  		SYS_MODULE_POWER_RUN_FULL
#define DRV_I2C_INTERRUPT_MODE                    		true


/*** I2S Driver Configuration ***/


#define DRV_I2S_INTERRUPT_MODE					true
#define DRV_I2S_CLIENTS_NUMBER					1
#define DRV_I2S_INSTANCES_NUMBER				1
#define DRV_I2S_STOP_IN_IDLE					false
#define DRV_I2S_PERIPHERAL_ID_IDX0				SPI_ID_1
#define DRV_I2S_USAGE_MODE_IDX0					DRV_I2S_MODE_MASTER
#define DRV_I2S_STOP_IN_IDLE_IDX0				false
#define SPI_BAUD_RATE_CLK_IDX0					SPI_BAUD_RATE_MCLK_CLOCK
#define DRV_I2S_BAUD_RATE                       16000
#define DRV_I2S_CLK_MODE_IDX0					DRV_I2S_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
#define SPI_AUDIO_COMM_WIDTH_IDX0				SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL
#define SPI_AUDIO_TRANSMIT_MODE_IDX0			SPI_AUDIO_TRANSMIT_STEREO
#define SPI_INPUT_SAMPLING_PHASE_IDX0			SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE
#define DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0		DRV_I2S_AUDIO_I2S
#define DRV_I2S_TX_INT_SRC_IDX0					INT_SOURCE_SPI_1_TRANSMIT
#define DRV_I2S_RX_INT_SRC_IDX0					INT_SOURCE_SPI_1_RECEIVE
#define QUEUE_SIZE_TX_IDX0                      3
#define QUEUE_SIZE_RX_IDX0                      2
#define DRV_I2S_RX_DMA_CHANNEL_IDX0				DMA_CHANNEL_2
#define DRV_I2S_RX_DMA_SOURCE_IDX0				INT_SOURCE_DMA_2
#define DRV_I2S_POWER_STATE_IDX0				SYS_MODULE_POWER_RUN_FULL
#define DRV_I2S_QUEUE_DEPTH_COMBINED     		5


#define USE_8BIT_PMP

/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           2
#define DRV_TMR_CLIENTS_NUMBER             2

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_1
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_1
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T1
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_1_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL1
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_8
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

#define DRV_TMR_PERIPHERAL_ID_IDX1          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX1       INT_SOURCE_TIMER_3
#define DRV_TMR_INTERRUPT_VECTOR_IDX1       INT_VECTOR_T3
#define DRV_TMR_ISR_VECTOR_IDX1             _TIMER_3_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX1     INT_PRIORITY_LEVEL3
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX1 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX1           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX1               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX1         DRV_TMR_OPERATION_MODE_32_BIT

#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX1     false
#define DRV_TMR_POWER_STATE_IDX1            SYS_MODULE_POWER_RUN_FULL

 // *****************************************************************************
/* USART Driver Configuration Options
*/
#define DRV_USART_INTERRUPT_MODE                    true

#define DRV_USART_BYTE_MODEL_SUPPORT                false

#define DRV_USART_READ_WRITE_MODEL_SUPPORT          false

#define DRV_USART_BUFFER_QUEUE_SUPPORT              true

#define DRV_USART_CLIENTS_NUMBER                    1
#define DRV_USART_INSTANCES_NUMBER                  1

#define DRV_USART_PERIPHERAL_ID_IDX0                USART_ID_2
#define DRV_USART_OPER_MODE_IDX0                    DRV_USART_OPERATION_MODE_NORMAL
#define DRV_USART_OPER_MODE_DATA_IDX0               
#define DRV_USART_INIT_FLAG_WAKE_ON_START_IDX0      false
#define DRV_USART_INIT_FLAG_AUTO_BAUD_IDX0          false
#define DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX0       false
#define DRV_USART_INIT_FLAGS_IDX0                   0
#define DRV_USART_BRG_CLOCK_IDX0                    48000000
#define DRV_USART_BAUD_RATE_IDX0                    115200
#define DRV_USART_LINE_CNTRL_IDX0                   DRV_USART_LINE_CONTROL_8NONE1
#define DRV_USART_HANDSHAKE_MODE_IDX0               DRV_USART_HANDSHAKE_FLOWCONTROL
#define DRV_USART_LINES_ENABLE_IDX0                 USART_ENABLE_TX_RX_CTS_RTS_USED
#define DRV_USART_XMIT_INT_SRC_IDX0                 INT_SOURCE_USART_2_TRANSMIT
#define DRV_USART_RCV_INT_SRC_IDX0                  INT_SOURCE_USART_2_RECEIVE
#define DRV_USART_ERR_INT_SRC_IDX0                  INT_SOURCE_USART_2_ERROR

#define DRV_USART_XMIT_QUEUE_SIZE_IDX0              1
#define DRV_USART_RCV_QUEUE_SIZE_IDX0               1


#define DRV_USART_XMIT_DMA_CH_IDX0                  DMA_CHANNEL_0
#define DRV_USART_XMIT_DMA_INT_SRC_IDX0                INT_SOURCE_DMA_0
#define DRV_USART_RCV_DMA_CH_IDX0                   DMA_CHANNEL_3
#define DRV_USART_RCV_DMA_INT_SRC_IDX0                INT_SOURCE_DMA_3
#define DRV_USART_POWER_STATE_IDX0                  SYS_MODULE_POWER_RUN_FULL

#define DRV_USART_QUEUE_DEPTH_COMBINED              2

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* BSP Configuration Options
*/
#define BSP_OSC_FREQUENCY 12000000


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
/*** Application Defined Pins ***/

/*** Functions for BSP_LED_9 pin ***/
#define BSP_LED_9Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_9)
#define BSP_LED_9On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_9)
#define BSP_LED_9Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_9)
#define BSP_LED_9StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_9)))

/*** Functions for BSP_LED_5 pin ***/
#define BSP_LED_5Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define BSP_LED_5On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define BSP_LED_5Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define BSP_LED_5StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)))

/*** Functions for BSP_LED_6 pin ***/
#define BSP_LED_6Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_LED_6On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_LED_6Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_LED_6StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)))

/*** Functions for BSP_LED_7 pin ***/
#define BSP_LED_7Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6)
#define BSP_LED_7On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6)
#define BSP_LED_7Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6)
#define BSP_LED_7StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6)))

/*** Functions for BSP_LED_8 pin ***/
#define BSP_LED_8Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7)
#define BSP_LED_8On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7)
#define BSP_LED_8Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7)
#define BSP_LED_8StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7)))

/*** Functions for BSP_SWITCH_1 pin ***/
#define BSP_SWITCH_1StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)

/*** Functions for BSP_SWITCH_3 pin ***/
#define BSP_SWITCH_3StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_10)

/*** Functions for BSP_SWITCH_2 pin ***/
#define BSP_SWITCH_2StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_1)

/*** Functions for BSP_SWITCH_4 pin ***/
#define BSP_SWITCH_4StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12)

/*** Functions for BSP_SWITCH_5 pin ***/
#define BSP_SWITCH_5StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13)

/*** Functions for BSP_SWITCH_6 pin ***/
#define BSP_SWITCH_6StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14)

/*** Functions for USB_SW1 pin ***/
#define USB_SW1StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1)

/*** Functions for USB_SW0 pin ***/
#define USB_SW0StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_0)

/*** Functions for BSP_AK4642_PDN pin ***/
#define BSP_AK4642_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15, Value)

/*** Functions for BSP_DisplayReset pin ***/
#define BSP_DisplayResetToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1)
#define BSP_DisplayResetOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1)
#define BSP_DisplayResetOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1)
#define BSP_DisplayResetStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1)
#define BSP_DisplayResetStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1, Value)

/*** Functions for BSP_DisplaySet pin ***/
#define BSP_DisplaySetToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_DisplaySetOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_DisplaySetOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_DisplaySetStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15)
#define BSP_DisplaySetStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15, Value)

/*** Functions for BSP_Display pin ***/
#define BSP_DisplayToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11)
#define BSP_DisplayOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11)
#define BSP_DisplayOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11)
#define BSP_DisplayStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11)
#define BSP_DisplayStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, Value)


// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Configuration
// *****************************************************************************
// *****************************************************************************
/* Bluetooth Device ID settings
 *
 * These macros define the bluetooth device ID.
 *
 * BT_DEVICE_DESIGN_ID         - sets the most significant 8 bytes
 * BT_DEVICE_ID_2LSB           - sets the least significant 4 bytes
 *
 * BT_DEVICE_ID_2LSB_RANDOMIZE - controls randomization of the 2LSB portion.
 *                               Note: 2LSB is randomized at power-on only.
 *
 *                               0 = disabled
 *                               1 = enabled
 *
 * device ID scheme:
 *        Design   2LSB
 *     -- -- -- -- ++ ++
 * ID :XX:XX:XX:XX:XX:XX
*/
//                                      Device
//                                      DesignID ID
//                                      --------++++
	
#define BT_CONNECTION_NAME 					    	"spp voice control"
#define BT_DEVICE_DESIGN_ID           				0x12345678
#define BT_MAX_PORTS_SPP 							7
#define BT_DEVICE_ID_2LSB_RANDOMIZE   				1 //enabled
#define BT_DEVICE_ID_2LSB             				0xFFFF
#define BT_CONTROLLER 								BT_CONTROLLER_FLC_BTM805

// *****************************************************************************
// *****************************************************************************
 
 

 
#define APP_BUTTON1_PIN 				(1<<PORTS_BIT_POS_0)
#define APP_BUTTON2_PIN 				(1<<PORTS_BIT_POS_1)
#define APP_BUTTON3_PIN 				(1<<PORTS_BIT_POS_10)
#define APP_BUTTON4_PIN 				(1<<PORTS_BIT_POS_12)
#define APP_BUTTON5_PIN 				(1<<PORTS_BIT_POS_13)
#define APP_BUTTON6_PIN 				(1<<PORTS_BIT_POS_14)
    
/* Application LED's */
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



/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN                           BSP_AK4384_PDNOn

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_6);
#define APP_VIRTUAL_LED_Y_OFFSET    0


 
 
/*** Audio Encoders Configuration ***/
#define ADPCM_ENCODER

/* Customize buffer size depends on device SRAM */
#define DECODER_MAX_INPUT_BUFFER_SIZE (1024*7)
#define DECODER_MAX_OUTPUT_BUFFER_SIZE (1024*7)

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
