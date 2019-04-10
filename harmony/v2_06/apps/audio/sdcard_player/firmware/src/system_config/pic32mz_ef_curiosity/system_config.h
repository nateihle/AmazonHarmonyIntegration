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
#define SYS_CLK_FREQ                        198000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            99000000ul
#define SYS_CLK_BUS_PERIPHERAL_2            99000000ul
#define SYS_CLK_BUS_PERIPHERAL_3            99000000ul
#define SYS_CLK_BUS_PERIPHERAL_4            99000000ul
#define SYS_CLK_BUS_PERIPHERAL_5            99000000ul
#define SYS_CLK_BUS_PERIPHERAL_7            198000000ul
#define SYS_CLK_BUS_PERIPHERAL_8            99000000ul
#define SYS_CLK_BUS_REFERENCE_1             12288000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         24000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
#define SYS_CLK_CONFIG_FREQ_ERROR_LIMIT     10
#define SYS_CLK_WAIT_FOR_SWITCH             true
#define SYS_CLK_ON_WAIT                     OSC_ON_WAIT_IDLE
   
/*** Ports System Service Configuration ***/
#define SYS_PORT_A_ANSEL        0x3F02
#define SYS_PORT_A_TRIS         0xFFDE
#define SYS_PORT_A_LAT          0x0000
#define SYS_PORT_A_ODC          0x0000
#define SYS_PORT_A_CNPU         0x001C
#define SYS_PORT_A_CNPD         0x0000
#define SYS_PORT_A_CNEN         0x0000

#define SYS_PORT_B_ANSEL        0x3FDC
#define SYS_PORT_B_TRIS         0x3FDC
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

#define SYS_PORT_D_ANSEL        0x81C0
#define SYS_PORT_D_TRIS         0xFFFF
#define SYS_PORT_D_LAT          0x0000
#define SYS_PORT_D_ODC          0x0000
#define SYS_PORT_D_CNPU         0x0000
#define SYS_PORT_D_CNPD         0x0000
#define SYS_PORT_D_CNEN         0x0000

#define SYS_PORT_E_ANSEL        0xFF80
#define SYS_PORT_E_TRIS         0xFFA7
#define SYS_PORT_E_LAT          0x0000
#define SYS_PORT_E_ODC          0x0000
#define SYS_PORT_E_CNPU         0x0000
#define SYS_PORT_E_CNPD         0x0000
#define SYS_PORT_E_CNEN         0x0000

#define SYS_PORT_F_ANSEL        0xFEC0
#define SYS_PORT_F_TRIS         0xFFFF
#define SYS_PORT_F_LAT          0x0000
#define SYS_PORT_F_ODC          0x0000
#define SYS_PORT_F_CNPU         0x0000
#define SYS_PORT_F_CNPD         0x0000
#define SYS_PORT_F_CNEN         0x0000

#define SYS_PORT_G_ANSEL        0x8F3C
#define SYS_PORT_G_TRIS         0xDFFF
#define SYS_PORT_G_LAT          0x0000
#define SYS_PORT_G_ODC          0x0000
#define SYS_PORT_G_CNPU         0x1000
#define SYS_PORT_G_CNPD         0x0000
#define SYS_PORT_G_CNEN         0x0000


/*** File System Service Configuration ***/

#define SYS_FS_MEDIA_NUMBER         	1

#define SYS_FS_VOLUME_NUMBER		(1)

#define SYS_FS_AUTOMOUNT_ENABLE		true
#define SYS_FS_CLIENT_NUMBER		1
#define SYS_FS_MAX_FILES	    	2
#define SYS_FS_MAX_FILE_SYSTEM_TYPE 	1
#define SYS_FS_MEDIA_MAX_BLOCK_SIZE  	512
#define SYS_FS_MEDIA_MANAGER_BUFFER_SIZE 2048
#define SYS_FS_FILE_NAME_LEN 255
#define SYS_FS_CWD_STRING_LEN 1024


#define SYS_FS_MEDIA_TYPE_IDX0 				SYS_FS_MEDIA_TYPE_SD_CARD
#define SYS_FS_TYPE_IDX0 					FAT








#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0 			"/mnt/myDrive"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0 			"/dev/mmcblka1"


/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true
/*** Timer System Service Configuration ***/
#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      5
#define SYS_TMR_FREQUENCY               1000
#define SYS_TMR_FREQUENCY_TOLERANCE     10
#define SYS_TMR_UNIT_RESOLUTION         10000
#define SYS_TMR_CLIENT_TOLERANCE        10
#define SYS_TMR_INTERRUPT_NOTIFICATION  false

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

/*** Codec Driver Configuration ***/

#define DRV_AK4642_CLIENTS_NUMBER                           1
#define DRV_AK4642_INSTANCES_NUMBER                         1
#define DRV_AK4642_INPUT_REFCLOCK    	                	7
#define DRV_AK4642_AUDIO_SAMPLING_RATE                      48000
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

#define DRV_I2C_PERIPHERAL_ID_IDX0                		I2C_ID_2
#define DRV_I2C_OPERATION_MODE_IDX0               		DRV_I2C_MODE_MASTER
#define DRV_SCL_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SCL_PIN_POSITION_IDX0                       PORTS_BIT_POS_2
#define DRV_SDA_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SDA_PIN_POSITION_IDX0                       PORTS_BIT_POS_3
#define DRV_I2C_BIT_BANG_IDX0                           false
#define DRV_I2C_STOP_IN_IDLE_IDX0                       false
#define DRV_I2C_SMBus_SPECIFICATION_IDX0			    false
#define DRV_I2C_BAUD_RATE_IDX0                    		50000
#define DRV_I2C_BRG_CLOCK_IDX0	                  		99000000
#define DRV_I2C_SLEW_RATE_CONTROL_IDX0      			false
#define DRV_I2C_MASTER_INT_SRC_IDX0               		INT_SOURCE_I2C_2_MASTER
#define DRV_I2C_SLAVE_INT_SRC_IDX0                		
#define DRV_I2C_ERR_MZ_INT_SRC_IDX0               		INT_SOURCE_I2C_2_BUS
#define DRV_I2C_MASTER_INT_VECTOR_IDX0            		INT_VECTOR_I2C2_MASTER
#define DRV_I2C_MASTER_ISR_VECTOR_IDX0                  _I2C2_MASTER_VECTOR
#define DRV_I2C_MASTER_INT_PRIORITY_IDX0          		INT_PRIORITY_LEVEL2
#define DRV_I2C_MASTER_INT_SUB_PRIORITY_IDX0      		INT_SUBPRIORITY_LEVEL0
#define DRV_I2C_SLAVE_INT_VECTOR_IDX0             		INT_VECTOR_I2C2_SLAVE
#define DRV_I2C_SLAVE_ISR_VECTOR_IDX0			  	    _I2C2_SLAVE_VECTOR
#define DRV_I2C_SLAVE_INT_PRIORITY_IDX0           		
#define DRV_I2C_SLAVE_INT_SUB_PRIORITY_IDX0       		
#define DRV_I2C_ERR_INT_VECTOR_IDX0               		INT_VECTOR_I2C2_BUS
#define DRV_I2C_ERR_ISR_VECTOR_IDX0                     _I2C2_BUS_VECTOR
#define DRV_I2C_ERR_INT_PRIORITY_IDX0             		INT_PRIORITY_LEVEL2
#define DRV_I2C_ERR_INT_SUB_PRIORITY_IDX0         		INT_SUBPRIORITY_LEVEL0
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
#define DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0		DRV_I2S_AUDIO_I2S
#define DRV_I2S_TX_INT_SRC_IDX0					INT_SOURCE_SPI_2_TRANSMIT
#define DRV_I2S_RX_INT_SRC_IDX0					INT_SOURCE_SPI_2_RECEIVE
#define QUEUE_SIZE_TX_IDX0                      2
#define QUEUE_SIZE_RX_IDX0                      1
#define DRV_I2S_TX_DMA_CHANNEL_IDX0				DMA_CHANNEL_2
#define DRV_I2S_TX_DMA_SOURCE_IDX0				INT_SOURCE_DMA_2
#define DRV_I2S_POWER_STATE_IDX0				SYS_MODULE_POWER_RUN_FULL
#define DRV_I2S_QUEUE_DEPTH_COMBINED     		3


/*** SDCARD Driver Configuration ***/
#define DRV_SDCARD_INSTANCES_NUMBER     1
#define DRV_SDCARD_CLIENTS_NUMBER       1
#define DRV_SDCARD_INDEX_MAX            1
#define DRV_SDCARD_INDEX                DRV_SDCARD_INDEX_0
#define DRV_SDCARD_QUEUE_POOL_SIZE      10
#define DRV_SDCARD_SPI_DRV_INSTANCE     0

#define DRV_SDCARD_SYS_FS_REGISTER




/*** SPI Driver Configuration ***/
#define DRV_SPI_NUMBER_OF_MODULES		6
/*** Driver Compilation and static configuration options. ***/
/*** Select SPI compilation units.***/
#define DRV_SPI_POLLED 				0
#define DRV_SPI_ISR 				1
#define DRV_SPI_MASTER 				1
#define DRV_SPI_SLAVE 				0
#define DRV_SPI_RM 					0
#define DRV_SPI_EBM 				1
#define DRV_SPI_8BIT 				1
#define DRV_SPI_16BIT 				0
#define DRV_SPI_32BIT 				0
#define DRV_SPI_DMA 				0

/*** SPI Driver Static Allocation Options ***/
#define DRV_SPI_INSTANCES_NUMBER 		1
#define DRV_SPI_CLIENTS_NUMBER 			1
#define DRV_SPI_ELEMENTS_PER_QUEUE 		10
/* SPI Driver Instance 0 Configuration */
#define DRV_SPI_SPI_ID_IDX0 				SPI_ID_1
#define DRV_SPI_TASK_MODE_IDX0 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX0				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX0			false
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX0 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX0 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_CLOCK_SOURCE_IDX0 		    SPI_BAUD_RATE_PBCLK_CLOCK
#define DRV_SPI_SPI_CLOCK_IDX0 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX0 				5000000
#define DRV_SPI_BUFFER_TYPE_IDX0 			DRV_SPI_BUFFER_TYPE_ENHANCED
#define DRV_SPI_CLOCK_MODE_IDX0 			DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX0 			SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE
#define DRV_SPI_TRANSMIT_DUMMY_BYTE_VALUE_IDX0      0xFF

#define DRV_SPI_TX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_1_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_1_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX0 		INT_SOURCE_SPI_1_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX0			INT_VECTOR_SPI1_TX
#define DRV_SPI_RX_INT_VECTOR_IDX0			INT_VECTOR_SPI1_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX0	INT_VECTOR_SPI1_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL3
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL3
#define DRV_SPI_RX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL3
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL3
#define DRV_SPI_ERROR_INT_PRIORITY_IDX0 	INT_PRIORITY_LEVEL3
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL3
#define DRV_SPI_QUEUE_SIZE_IDX0 			10
#define DRV_SPI_RESERVED_JOB_IDX0 			1
/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           1
#define DRV_TMR_CLIENTS_NUMBER             1

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_1
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_1
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T1
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_1_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL1
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

 
 
// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* BSP Configuration Options
*/
#define BSP_OSC_FREQUENCY 24000000


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
#define APP_SDCARD_AUDIO_CODEC_WRITE_QUEUE_SIZE         QUEUE_SIZE_TX_IDX0
#define APP_SDCARD_AUDIO_CARD_MAX_DIRS   20
#define APP_SDCARD_AUDIO_CARD_MAX_FILES  500
    
#define APP_CODEC_AUDIO_SAMPLING_RATE                   DRV_AK4642_AUDIO_SAMPLING_RATE    
#define APP_AUDIO_SRC_SWITCH                            BSP_SWITCH_1    

#define APP_OFF_RGB_LED()                           BSP_RGB_LED_REDOff(); \
    BSP_RGB_LED_GREENOff(); \
    BSP_RGB_LED_BLUEOff();
    
#define APP_SDCARD_AUDIO_SRC_ON()                       BSP_LED_1On()
#define APP_SDCARD_AUDIO_SRC_OFF()                      BSP_LED_1Off()
#define APP_TRACK_CHANGE_INDICATE()                     BSP_LED_3Toggle()
    
    
#define APP_SDCARD_AUDIO_CARD_MOUNT_NAME                SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0
#define APP_SDCARD_AUDIO_CARD_DEVICE_NAME               SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0
    
/*** Application Defined Pins ***/

/*** Functions for BSP_LED_3 pin ***/
#define BSP_LED_3Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_3On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_3Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)
#define BSP_LED_3StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6)

/*** Functions for BSP_RGB_LED_RED pin ***/
#define BSP_RGB_LED_REDToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5)
#define BSP_RGB_LED_REDOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5)
#define BSP_RGB_LED_REDOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5)
#define BSP_RGB_LED_REDStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5)))

/*** Functions for BSP_RGB_LED_GREEN pin ***/
#define BSP_RGB_LED_GREENToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)
#define BSP_RGB_LED_GREENStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_1)))

/*** Functions for BSP_RGB_LED_BLUE pin ***/
#define BSP_RGB_LED_BLUEToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_RGB_LED_BLUEOn() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_RGB_LED_BLUEOff() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)
#define BSP_RGB_LED_BLUEStateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0)))

/*** Functions for BSP_LED_1 pin ***/
#define BSP_LED_1Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3)
#define BSP_LED_1On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3)
#define BSP_LED_1Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3)
#define BSP_LED_1StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3)

/*** Functions for BSP_LED_2 pin ***/
#define BSP_LED_2Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_2On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_2Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)
#define BSP_LED_2StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4)

/*** Functions for BSP_SWITCH_1 pin ***/
#define BSP_SWITCH_1StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_12)

/*** Functions for BSP_AK4642_PDN pin ***/
#define BSP_AK4642_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_AK4642_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_AK4642_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_AK4642_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_AK4642_PDNStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5, Value)

/*** Functions for BSP_WIFI_SLEEP pin ***/
#define BSP_WIFI_SLEEPToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0, Value)

/*** Functions for BSP_WIFI_INT pin ***/
#define BSP_WIFI_INTStateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_4)


/*** Audio Decoders Configuration ***/
#define WAV_STREAMING_ENABLED

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
