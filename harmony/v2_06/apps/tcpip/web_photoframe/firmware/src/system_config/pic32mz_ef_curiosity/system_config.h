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
#define SYS_CLK_FREQ                        200000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_2            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_3            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_4            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_5            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_7            200000000ul
#define SYS_CLK_BUS_PERIPHERAL_8            100000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         24000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
   
/*** Ports System Service Configuration ***/
#define SYS_PORT_A_ANSEL        0x3F00
#define SYS_PORT_A_TRIS         0xFFDE
#define SYS_PORT_A_LAT          0x0000
#define SYS_PORT_A_ODC          0x0000
#define SYS_PORT_A_CNPU         0x0000
#define SYS_PORT_A_CNPD         0x0000
#define SYS_PORT_A_CNEN         0x0000

#define SYS_PORT_B_ANSEL        0x00D8
#define SYS_PORT_B_TRIS         0xFED8
#define SYS_PORT_B_LAT          0x0000
#define SYS_PORT_B_ODC          0x0000
#define SYS_PORT_B_CNPU         0x0000
#define SYS_PORT_B_CNPD         0x0000
#define SYS_PORT_B_CNEN         0x0000

#define SYS_PORT_C_ANSEL        0xEFF3
#define SYS_PORT_C_TRIS         0xFFF3
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

#define SYS_PORT_E_ANSEL        0xFFA0
#define SYS_PORT_E_TRIS         0xFFA0
#define SYS_PORT_E_LAT          0x0000
#define SYS_PORT_E_ODC          0x0000
#define SYS_PORT_E_CNPU         0x0000
#define SYS_PORT_E_CNPD         0x0000
#define SYS_PORT_E_CNEN         0x0000

#define SYS_PORT_F_ANSEL        0xDEC0
#define SYS_PORT_F_TRIS         0xFFFF
#define SYS_PORT_F_LAT          0x0000
#define SYS_PORT_F_ODC          0x0000
#define SYS_PORT_F_CNPU         0x0000
#define SYS_PORT_F_CNPD         0x0000
#define SYS_PORT_F_CNEN         0x0000

#define SYS_PORT_G_ANSEL        0x8CFC
#define SYS_PORT_G_TRIS         0xDFFF
#define SYS_PORT_G_LAT          0x0000
#define SYS_PORT_G_ODC          0x0000
#define SYS_PORT_G_CNPU         0x1000
#define SYS_PORT_G_CNPD         0x0000
#define SYS_PORT_G_CNEN         0x0000


/*** Command Processor System Service Configuration ***/
#define SYS_CMD_ENABLE
#define SYS_CMD_DEVICE_MAX_INSTANCES    SYS_CONSOLE_DEVICE_MAX_INSTANCES
#define SYS_CMD_PRINT_BUFFER_SIZE       1024
#define SYS_CMD_BUFFER_DMA_READY        __attribute__((coherent)) __attribute__((aligned(16)))
#define SYS_CMD_REMAP_SYS_CONSOLE_MESSAGE
#define SYS_CMD_REMAP_SYS_DEBUG_MESSAGE

/*** Console System Service Configuration ***/

#define SYS_CONSOLE_OVERRIDE_STDIO
#define SYS_CONSOLE_DEVICE_MAX_INSTANCES        2
#define SYS_CONSOLE_INSTANCES_NUMBER            1
#define SYS_CONSOLE_USB_CDC_INSTANCE       USB_DEVICE_CDC_INDEX_0
#define SYS_CONSOLE_USB_CDC_COMM_BAUD_RATE 921600
#define SYS_CONSOLE_USB_CDC_RD_QUEUE_DEPTH 1
#define SYS_CONSOLE_USB_CDC_WR_QUEUE_DEPTH 128
#define SYS_CONSOLE_USB_CDC_READ_BUFFER_SIZE   512
#define SYS_CONSOLE_BUFFER_DMA_READY        __attribute__((coherent)) __attribute__((aligned(16)))



/*** Debug System Service Configuration ***/
#define SYS_DEBUG_ENABLE
#define DEBUG_PRINT_BUFFER_SIZE       512
#define SYS_DEBUG_BUFFER_DMA_READY        __attribute__((coherent)) __attribute__((aligned(16)))
#define SYS_DEBUG_USE_CONSOLE

/*** File System Service Configuration ***/

#define SYS_FS_MEDIA_NUMBER         	1

#define SYS_FS_VOLUME_NUMBER		(1)

#define SYS_FS_AUTOMOUNT_ENABLE		true
#define SYS_FS_CLIENT_NUMBER		1
#define SYS_FS_MAX_FILES	    	25
#define SYS_FS_MAX_FILE_SYSTEM_TYPE 	1
#define SYS_FS_MEDIA_MAX_BLOCK_SIZE  	2048
#define SYS_FS_MEDIA_MANAGER_BUFFER_SIZE 2048
#define SYS_FS_FILE_NAME_LEN 255
#define SYS_FS_CWD_STRING_LEN 1024


#define SYS_FS_MEDIA_TYPE_IDX0 				SYS_FS_MEDIA_TYPE_SD_CARD
#define SYS_FS_TYPE_IDX0 					FAT








#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0 			"/mnt/myDrive1"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0 			"/dev/mmcblka1"


/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true
// *****************************************************************************
/* Random System Service Configuration Options
*/

#define SYS_RANDOM_CRYPTO_SEED_SIZE  55

/*** Timer System Service Configuration ***/
#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      5
#define SYS_TMR_FREQUENCY               1000
#define SYS_TMR_FREQUENCY_TOLERANCE     10
#define SYS_TMR_UNIT_RESOLUTION         10000
#define SYS_TMR_CLIENT_TOLERANCE        10
#define SYS_TMR_INTERRUPT_NOTIFICATION  true

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

/*** MIIM Driver Configuration ***/
#define DRV_MIIM_ETH_MODULE_ID              ETH_ID_0
#define DRV_MIIM_INSTANCES_NUMBER           1
#define DRV_MIIM_INSTANCE_OPERATIONS        4
#define DRV_MIIM_INSTANCE_CLIENTS           2
#define DRV_MIIM_CLIENT_OP_PROTECTION   false
#define DRV_MIIM_COMMANDS   false
#define DRV_MIIM_DRIVER_OBJECT              DRV_MIIM_OBJECT_BASE_Default
#define DRV_MIIM_DRIVER_INDEX               DRV_MIIM_INDEX_0              

/*** SDCARD Driver Configuration ***/
#define DRV_SDCARD_INSTANCES_NUMBER     1
#define DRV_SDCARD_CLIENTS_NUMBER       2
#define DRV_SDCARD_INDEX_MAX            2
#define DRV_SDCARD_INDEX                DRV_SDCARD_INDEX_0
#define DRV_SDCARD_QUEUE_POOL_SIZE      10
#define DRV_SDCARD_SPI_DRV_INSTANCE     1

#define DRV_SDCARD_SYS_FS_REGISTER




/*** SPI Driver Configuration ***/
#define DRV_SPI_NUMBER_OF_MODULES		6
/*** Driver Compilation and static configuration options. ***/
/*** Select SPI compilation units.***/
#define DRV_SPI_POLLED 				0
#define DRV_SPI_ISR 				1
#define DRV_SPI_MASTER 				1
#define DRV_SPI_SLAVE 				0
#define DRV_SPI_RM 					1
#define DRV_SPI_EBM 				1
#define DRV_SPI_8BIT 				1
#define DRV_SPI_16BIT 				0
#define DRV_SPI_32BIT 				0
#define DRV_SPI_DMA 				1

/*** SPI Driver Static Allocation Options ***/
#define DRV_SPI_INSTANCES_NUMBER 		2
#define DRV_SPI_CLIENTS_NUMBER 			2
#define DRV_SPI_ELEMENTS_PER_QUEUE 		10
/*** SPI Driver DMA Options ***/
#define DRV_SPI_DMA_TXFER_SIZE 			512
#define DRV_SPI_DMA_DUMMY_BUFFER_SIZE 	512
/* SPI Driver Instance 0 Configuration */
#define DRV_SPI_SPI_ID_IDX0 				SPI_ID_3
#define DRV_SPI_TASK_MODE_IDX0 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX0				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX0			false
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX0 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX0 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_CLOCK_SOURCE_IDX0 		    SPI_BAUD_RATE_PBCLK_CLOCK
#define DRV_SPI_SPI_CLOCK_IDX0 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX0 				8000000
#define DRV_SPI_BUFFER_TYPE_IDX0 			DRV_SPI_BUFFER_TYPE_ENHANCED
#define DRV_SPI_CLOCK_MODE_IDX0 			DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX0 			SPI_INPUT_SAMPLING_PHASE_AT_END
#define DRV_SPI_TRANSMIT_DUMMY_BYTE_VALUE_IDX0      0xFF

#define DRV_SPI_TX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_3_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_3_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX0 		INT_SOURCE_SPI_3_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX0			INT_VECTOR_SPI3_TX
#define DRV_SPI_RX_INT_VECTOR_IDX0			INT_VECTOR_SPI3_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX0	INT_VECTOR_SPI3_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL1
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_RX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL1
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_ERROR_INT_PRIORITY_IDX0 	INT_PRIORITY_LEVEL1
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_QUEUE_SIZE_IDX0 			10
#define DRV_SPI_RESERVED_JOB_IDX0 			1
#define DRV_SPI_TX_DMA_CHANNEL_IDX0 		DMA_CHANNEL_1
#define DRV_SPI_TX_DMA_THRESHOLD_IDX0 		16
#define DRV_SPI_RX_DMA_CHANNEL_IDX0 		DMA_CHANNEL_0
#define DRV_SPI_RX_DMA_THRESHOLD_IDX0 		16
/* SPI Driver Instance 1 Configuration */
#define DRV_SPI_SPI_ID_IDX1 				SPI_ID_1
#define DRV_SPI_TASK_MODE_IDX1 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX1				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX1			true
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX1 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX1 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_CLOCK_SOURCE_IDX1 		    SPI_BAUD_RATE_PBCLK_CLOCK
#define DRV_SPI_SPI_CLOCK_IDX1 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX1 				20000000
#define DRV_SPI_BUFFER_TYPE_IDX1 			DRV_SPI_BUFFER_TYPE_STANDARD
#define DRV_SPI_CLOCK_MODE_IDX1 			DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX1 			SPI_INPUT_SAMPLING_PHASE_AT_END
#define DRV_SPI_TRANSMIT_DUMMY_BYTE_VALUE_IDX1      0xFF

#define DRV_SPI_TX_INT_SOURCE_IDX1 			INT_SOURCE_SPI_1_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX1 			INT_SOURCE_SPI_1_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX1 		INT_SOURCE_SPI_1_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX1			INT_VECTOR_SPI1_TX
#define DRV_SPI_RX_INT_VECTOR_IDX1			INT_VECTOR_SPI1_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX1	INT_VECTOR_SPI1_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX1 		INT_PRIORITY_LEVEL1
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX1 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_RX_INT_PRIORITY_IDX1 		INT_PRIORITY_LEVEL1
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX1 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_ERROR_INT_PRIORITY_IDX1 	INT_PRIORITY_LEVEL1
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX1 INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_QUEUE_SIZE_IDX1 			10
#define DRV_SPI_RESERVED_JOB_IDX1 			1
/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           1
#define DRV_TMR_CLIENTS_NUMBER             1

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_2
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T2
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_2_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

 /*** Wi-Fi Driver Configuration ***/
#define MRF_INT_SOURCE INT_SOURCE_EXTERNAL_3
#define MRF_INT_VECTOR INT_VECTOR_INT3

#define WDRV_SPI_INDEX 0
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx0

#define WDRV_BOARD_TYPE WDRV_BD_TYPE_CUSTOM

#define WDRV_EXT_RTOS_INIT_TASK_SIZE 512u
#define WDRV_EXT_RTOS_INIT_TASK_PRIORITY 1u
#define WDRV_EXT_RTOS_MAIN_TASK_SIZE 2048u
#define WDRV_EXT_RTOS_MAIN_TASK_PRIORITY 2u

// I/O mappings for general control pins, including CS, INTERRUPT, POWER and RESET.
#define WF_CS_PORT_CHANNEL          PORT_CHANNEL_B
#define WF_CS_BIT_POS               8

#define WF_INT_PORT_CHANNEL         PORT_CHANNEL_F
#define WF_INT_BIT_POS              4

#define WF_POWER_PORT_CHANNEL       PORT_CHANNEL_A
#define WF_POWER_BIT_POS            0

#define WF_RESET_PORT_CHANNEL       PORT_CHANNEL_A
#define WF_RESET_BIT_POS            10

#define WDRV_DEFAULT_NETWORK_TYPE WDRV_NETWORK_TYPE_SOFT_AP
#define WDRV_DEFAULT_CHANNEL 6
#define WDRV_DEFAULT_SSID "MCHPSoftAP"

#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_OPEN
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN

#define WDRV_DEFAULT_POWER_SAVE WDRV_FUNC_DISABLED

#define MODULE_EVENT_PRINT 0

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************
/*** Crypto Library Configuration ***/

#define WC_NO_HARDEN
#define MICROCHIP_MPLAB_HARMONY
#define HAVE_MCAPI
#define MICROCHIP_PIC32
#define NO_CERTS
#define NO_PWDBASED
#define NO_OLD_TLS
#define NO_AES
#define NO_ASN
#define NO_RSA

/*** OSAL Configuration ***/
#define OSAL_USE_RTOS          9


// *****************************************************************************
// *****************************************************************************
// Section: TCPIP Stack Configuration
// *****************************************************************************
// *****************************************************************************
#define TCPIP_STACK_USE_IPV4
#define TCPIP_STACK_USE_TCP
#define TCPIP_STACK_USE_UDP

#define TCPIP_STACK_TICK_RATE		        		5
#define TCPIP_STACK_SECURE_PORT_ENTRIES             10

#define TCPIP_STACK_ALIAS_INTERFACE_SUPPORT   false

#define TCPIP_PACKET_LOG_ENABLE     0

/* TCP/IP stack event notification */
#define TCPIP_STACK_USE_EVENT_NOTIFICATION
#define TCPIP_STACK_USER_NOTIFICATION   false
#define TCPIP_STACK_DOWN_OPERATION   true
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   true
#define TCPIP_STACK_MAC_DOWN_OPERATION  true
#define TCPIP_STACK_INTERFACE_CHANGE_SIGNALING   false
#define TCPIP_STACK_CONFIGURATION_SAVE_RESTORE   true
/*** TCPIP Heap Configuration ***/

#define TCPIP_STACK_USE_INTERNAL_HEAP
#define TCPIP_STACK_DRAM_SIZE                       39250
#define TCPIP_STACK_DRAM_RUN_LIMIT                  2048

#define TCPIP_STACK_MALLOC_FUNC                     malloc

#define TCPIP_STACK_CALLOC_FUNC                     calloc

#define TCPIP_STACK_FREE_FUNC                       free


#define TCPIP_STACK_DRAM_DEBUG_ENABLE

#define TCPIP_STACK_HEAP_USE_FLAGS                   TCPIP_STACK_HEAP_FLAG_ALLOC_UNCACHED

#define TCPIP_STACK_HEAP_USAGE_CONFIG                TCPIP_STACK_HEAP_USE_DEFAULT

#define TCPIP_STACK_SUPPORTED_HEAPS                  1

/*** ARP Configuration ***/
#define TCPIP_ARP_CACHE_ENTRIES                 		5
#define TCPIP_ARP_CACHE_DELETE_OLD		        	true
#define TCPIP_ARP_CACHE_SOLVED_ENTRY_TMO			1200
#define TCPIP_ARP_CACHE_PENDING_ENTRY_TMO			60
#define TCPIP_ARP_CACHE_PENDING_RETRY_TMO			2
#define TCPIP_ARP_CACHE_PERMANENT_QUOTA		    		50
#define TCPIP_ARP_CACHE_PURGE_THRESHOLD		    		75
#define TCPIP_ARP_CACHE_PURGE_QUANTA		    		1
#define TCPIP_ARP_CACHE_ENTRY_RETRIES		    		3
#define TCPIP_ARP_GRATUITOUS_PROBE_COUNT			1
#define TCPIP_ARP_TASK_PROCESS_RATE		        	2
#define TCPIP_ARP_PRIMARY_CACHE_ONLY		        	true

/*** DHCP Configuration ***/
#define TCPIP_STACK_USE_DHCP_CLIENT
#define TCPIP_DHCP_TIMEOUT                          2
#define TCPIP_DHCP_TASK_TICK_RATE                   200
#define TCPIP_DHCP_HOST_NAME_SIZE                   20
#define TCPIP_DHCP_CLIENT_CONNECT_PORT              68
#define TCPIP_DHCP_SERVER_LISTEN_PORT               67
#define TCPIP_DHCP_CLIENT_ENABLED                   true

/*** DHCP Server Configuration ***/
#define TCPIP_STACK_USE_DHCP_SERVER
#define TCPIP_DHCPS_TASK_PROCESS_RATE                               200
#define TCPIP_DHCPS_LEASE_ENTRIES_DEFAULT                           15
#define TCPIP_DHCPS_LEASE_SOLVED_ENTRY_TMO                          1200
#define TCPIP_DHCPS_LEASE_REMOVED_BEFORE_ACK                        5
#define TCPIP_DHCP_SERVER_DELETE_OLD_ENTRIES                        true
#define TCPIP_DHCPS_LEASE_DURATION	TCPIP_DHCPS_LEASE_SOLVED_ENTRY_TMO

/*** DHCP Server Instance 0 Configuration ***/
#define TCPIP_DHCPS_DEFAULT_IP_ADDRESS_RANGE_START_IDX0             "192.168.1.100"

#define TCPIP_DHCPS_DEFAULT_SERVER_IP_ADDRESS_IDX0                  "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_NETMASK_ADDRESS_IDX0             "255.255.255.0"

#define TCPIP_DHCPS_DEFAULT_SERVER_GATEWAY_ADDRESS_IDX0             "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_PRIMARY_DNS_ADDRESS_IDX0         "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_SECONDARY_DNS_ADDRESS_IDX0       "192.168.1.1"

#define TCPIP_DHCP_SERVER_INTERFACE_INDEX_IDX0                      0

#define TCPIP_DHCP_SERVER_POOL_ENABLED_IDX0                         true

/*** DHCP Server Instance 1 Configuration ***/
#define TCPIP_DHCPS_DEFAULT_IP_ADDRESS_RANGE_START_IDX1             "192.168.1.100"

#define TCPIP_DHCPS_DEFAULT_SERVER_IP_ADDRESS_IDX1                  "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_NETMASK_ADDRESS_IDX1             "255.255.255.0"

#define TCPIP_DHCPS_DEFAULT_SERVER_GATEWAY_ADDRESS_IDX1             "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_PRIMARY_DNS_ADDRESS_IDX1         "192.168.1.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_SECONDARY_DNS_ADDRESS_IDX1       "192.168.1.1"

#define TCPIP_DHCP_SERVER_INTERFACE_INDEX_IDX1                      1

#define TCPIP_DHCP_SERVER_POOL_ENABLED_IDX1                         true


/*** DNS Client Configuration ***/
#define TCPIP_STACK_USE_DNS
#define TCPIP_DNS_CLIENT_SERVER_TMO					60
#define TCPIP_DNS_CLIENT_TASK_PROCESS_RATE			200
#define TCPIP_DNS_CLIENT_CACHE_ENTRIES				5
#define TCPIP_DNS_CLIENT_CACHE_ENTRY_TMO			0
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV4_ADDRESS		5
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV6_ADDRESS		1
#define TCPIP_DNS_CLIENT_ADDRESS_TYPE			    IP_ADDRESS_TYPE_IPV4
#define TCPIP_DNS_CLIENT_CACHE_DEFAULT_TTL_VAL		1200
#define TCPIP_DNS_CLIENT_CACHE_UNSOLVED_ENTRY_TMO	10
#define TCPIP_DNS_CLIENT_LOOKUP_RETRY_TMO			5
#define TCPIP_DNS_CLIENT_MAX_HOSTNAME_LEN			32
#define TCPIP_DNS_CLIENT_MAX_SELECT_INTERFACES		4
#define TCPIP_DNS_CLIENT_DELETE_OLD_ENTRIES			true
#define TCPIP_DNS_CLIENT_USER_NOTIFICATION   false

/*** DNS Server Configuration ***/
#define TCPIP_STACK_USE_DNS_SERVER
#define TCPIP_DNSS_HOST_NAME_LEN		    		64
#define TCPIP_DNSS_REPLY_BOARD_ADDR				true
#define TCPIP_DNSS_CACHE_PER_IPV4_ADDRESS			2
#define TCPIP_DNSS_CACHE_PER_IPV6_ADDRESS			
#define TCPIP_DNSS_TTL_TIME							600
#define TCPIP_DNSS_TASK_PROCESS_RATE			    33
#define TCPIP_DNSS_DELETE_OLD_LEASE				true

/***Maximum DNS server Cache entries. It is the sum of TCPIP_DNSS_CACHE_PER_IPV4_ADDRESS and TCPIP_DNSS_CACHE_PER_IPV6_ADDRESS.***/
#define TCPIP_DNSS_CACHE_MAX_SERVER_ENTRIES     (TCPIP_DNSS_CACHE_PER_IPV4_ADDRESS+TCPIP_DNSS_CACHE_PER_IPV6_ADDRESS)


/*** HTTP Configuration ***/
#define TCPIP_STACK_USE_HTTP_SERVER
#define TCPIP_HTTP_MAX_HEADER_LEN		    		15
#define TCPIP_HTTP_CACHE_LEN		        		"600"
#define TCPIP_HTTP_TIMEOUT		            		45
#define TCPIP_HTTP_MAX_CONNECTIONS		    		4
#define TCPIP_HTTP_DEFAULT_FILE		        		"index.htm"
#define TCPIP_HTTPS_DEFAULT_FILE	        		"index.htm"
#define TCPIP_HTTP_DEFAULT_LEN		        		10
#define TCPIP_HTTP_MAX_DATA_LEN		        		100
#define TCPIP_HTTP_MIN_CALLBACK_FREE				16
#define TCPIP_HTTP_SKT_TX_BUFF_SIZE		    		0
#define TCPIP_HTTP_SKT_RX_BUFF_SIZE		    		0
#define TCPIP_HTTP_CONFIG_FLAGS		        		1
#define TCPIP_HTTP_USE_POST
#define TCPIP_HTTP_USE_COOKIES
#define TCPIP_HTTP_USE_BASE64_DECODE
#define TCPIP_HTTP_USE_AUTHENTICATION
#define TCPIP_HTTP_TASK_RATE					33
#define TCPIP_HTTP_MALLOC_FUNC                     0
#define TCPIP_HTTP_FREE_FUNC                        0

/*** ICMPv4 Server Configuration ***/
#define TCPIP_STACK_USE_ICMP_SERVER
#define TCPIP_ICMP_ECHO_ALLOW_BROADCASTS    false

/*** ICMPv4 Client Configuration ***/
#define TCPIP_STACK_USE_ICMP_CLIENT
#define TCPIP_ICMP_CLIENT_USER_NOTIFICATION   true
#define TCPIP_ICMP_ECHO_REQUEST_TIMEOUT       500
#define TCPIP_ICMP_TASK_TICK_RATE             33









/*** TCP Configuration ***/
#define TCPIP_TCP_MAX_SEG_SIZE_TX		        	1460
#define TCPIP_TCP_SOCKET_DEFAULT_TX_SIZE			512
#define TCPIP_TCP_SOCKET_DEFAULT_RX_SIZE			512
#define TCPIP_TCP_DYNAMIC_OPTIONS             			true
#define TCPIP_TCP_START_TIMEOUT_VAL		        	1000
#define TCPIP_TCP_DELAYED_ACK_TIMEOUT		    		100
#define TCPIP_TCP_FIN_WAIT_2_TIMEOUT		    		5000
#define TCPIP_TCP_KEEP_ALIVE_TIMEOUT		    		10000
#define TCPIP_TCP_CLOSE_WAIT_TIMEOUT		    		0
#define TCPIP_TCP_MAX_RETRIES		            		5
#define TCPIP_TCP_MAX_UNACKED_KEEP_ALIVES			6
#define TCPIP_TCP_MAX_SYN_RETRIES		        	2
#define TCPIP_TCP_AUTO_TRANSMIT_TIMEOUT_VAL			40
#define TCPIP_TCP_WINDOW_UPDATE_TIMEOUT_VAL			200
#define TCPIP_TCP_MAX_SOCKETS		                10
#define TCPIP_TCP_TASK_TICK_RATE		        	5
#define TCPIP_TCP_MSL_TIMEOUT		        	    0
#define TCPIP_TCP_QUIET_TIME		        	    0
#define TCPIP_TCP_COMMANDS   false

/*** announce Configuration ***/
#define TCPIP_STACK_USE_ANNOUNCE
#define TCPIP_ANNOUNCE_MAX_PAYLOAD 	512
#define TCPIP_ANNOUNCE_TASK_RATE    333
#define TCPIP_ANNOUNCE_NETWORK_DIRECTED_BCAST             			false

/*** TCPIP MAC Configuration ***/
#define TCPIP_EMAC_TX_DESCRIPTORS				    8
#define TCPIP_EMAC_RX_DESCRIPTORS				    10
#define TCPIP_EMAC_RX_DEDICATED_BUFFERS				4
#define TCPIP_EMAC_RX_INIT_BUFFERS				    0
#define TCPIP_EMAC_RX_LOW_THRESHOLD				    1
#define TCPIP_EMAC_RX_LOW_FILL				        2
#define TCPIP_EMAC_MAX_FRAME		    			1536
#define TCPIP_EMAC_LINK_MTU		    			    1500
#define TCPIP_EMAC_RX_BUFF_SIZE		    			1536
#define TCPIP_EMAC_RX_FRAGMENTS		    			1

#define TCPIP_EMAC_RX_FILTERS                       \
                                                    TCPIP_MAC_RX_FILTER_TYPE_BCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_MCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_UCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_RUNT_REJECT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_CRC_ERROR_REJECT |\
                                                    0
#define TCPIP_EMAC_ETH_OPEN_FLAGS       			\
                                                    TCPIP_ETH_OPEN_AUTO |\
                                                    TCPIP_ETH_OPEN_FDUPLEX |\
                                                    TCPIP_ETH_OPEN_HDUPLEX |\
                                                    TCPIP_ETH_OPEN_100 |\
                                                    TCPIP_ETH_OPEN_10 |\
                                                    TCPIP_ETH_OPEN_MDIX_AUTO |\
                                                    TCPIP_ETH_OPEN_RMII |\
                                                    0

#define TCPIP_EMAC_MODULE_ID		    			ETH_ID_0
#define TCPIP_EMAC_INTERRUPT_MODE        			true
#define DRV_ETHMAC_INSTANCES_NUMBER				1
#define DRV_ETHMAC_CLIENTS_NUMBER				1
#define DRV_ETHMAC_INDEX	    	    			1
#define DRV_ETHMAC_PERIPHERAL_ID				1
#define DRV_ETHMAC_INTERRUPT_VECTOR				INT_VECTOR_ETHERNET
#define DRV_ETHMAC_INTERRUPT_SOURCE				INT_SOURCE_ETH_1
#define DRV_ETHMAC_POWER_STATE		    			SYS_MODULE_POWER_RUN_FULL

#define DRV_ETHMAC_INTERRUPT_MODE        			true


#define TCPIP_EMAC_PHY_CONFIG_FLAGS     			\
                                                    DRV_ETHPHY_CFG_AUTO | \
                                                    0                                                    

#define TCPIP_EMAC_PHY_LINK_INIT_DELAY  			500
#define TCPIP_EMAC_PHY_ADDRESS		    			0
#define DRV_ETHPHY_INSTANCES_NUMBER				1
#define DRV_ETHPHY_CLIENTS_NUMBER				1
#define DRV_ETHPHY_INDEX		        		1
#define DRV_ETHPHY_PERIPHERAL_ID				1
#define DRV_ETHPHY_NEG_INIT_TMO		    			10
#define DRV_ETHPHY_NEG_DONE_TMO		    			7000
#define DRV_ETHPHY_RESET_CLR_TMO				1000
#define DRV_ETHPHY_USE_DRV_MIIM                     true
#define TCPIP_EMAC_AUTO_FLOW_CONTROL_ENABLE        	true
#define TCPIP_EMAC_FLOW_CONTROL_PAUSE_BYTES         3072
#define TCPIP_EMAC_FLOW_CONTROL_FULL_WMARK          2
#define TCPIP_EMAC_FLOW_CONTROL_EMPTY_WMARK         0



/*** UDP Configuration ***/
#define TCPIP_UDP_MAX_SOCKETS		                	10
#define TCPIP_UDP_SOCKET_DEFAULT_TX_SIZE		    	512
#define TCPIP_UDP_SOCKET_DEFAULT_TX_QUEUE_LIMIT    	 	3
#define TCPIP_UDP_SOCKET_DEFAULT_RX_QUEUE_LIMIT			5
#define TCPIP_UDP_USE_POOL_BUFFERS   false
#define TCPIP_UDP_USE_TX_CHECKSUM             			true
#define TCPIP_UDP_USE_RX_CHECKSUM             			true
#define TCPIP_UDP_COMMANDS   false

#define TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
#define TCPIP_ZC_LL_PROBE_WAIT 1
#define TCPIP_ZC_LL_PROBE_MIN 1
#define TCPIP_ZC_LL_PROBE_MAX 2
#define TCPIP_ZC_LL_PROBE_NUM 3
#define TCPIP_ZC_LL_ANNOUNCE_WAIT 2
#define TCPIP_ZC_LL_ANNOUNCE_NUM 2
#define TCPIP_ZC_LL_ANNOUNCE_INTERVAL 2
#define TCPIP_ZC_LL_MAX_CONFLICTS 10
#define TCPIP_ZC_LL_RATE_LIMIT_INTERVAL 60
#define TCPIP_ZC_LL_DEFEND_INTERVAL 10
#define TCPIP_ZC_LL_IPV4_LLBASE 0xa9fe0100
#define TCPIP_ZC_LL_IPV4_LLBASE_MASK 0x0000FFFF
#define TCPIP_ZC_LL_TASK_TICK_RATE 333
#define TCPIP_STACK_USE_ZEROCONF_MDNS_SD
#define TCPIP_ZC_MDNS_TASK_TICK_RATE 63
#define TCPIP_ZC_MDNS_PORT 5353
#define TCPIP_ZC_MDNS_MAX_HOST_NAME_SIZE 32
#define TCPIP_ZC_MDNS_MAX_LABEL_SIZE 64
#define TCPIP_ZC_MDNS_MAX_RR_NAME_SIZE 256
#define TCPIP_ZC_MDNS_MAX_SRV_TYPE_SIZE 32
#define TCPIP_ZC_MDNS_MAX_SRV_NAME_SIZE 64
#define TCPIP_ZC_MDNS_MAX_TXT_DATA_SIZE 128
#define TCPIP_ZC_MDNS_RESOURCE_RECORD_TTL_VAL 3600
#define TCPIP_ZC_MDNS_MAX_RR_NUM 4
#define TCPIP_ZC_MDNS_PROBE_WAIT 750
#define TCPIP_ZC_MDNS_PROBE_INTERVAL 250
#define TCPIP_ZC_MDNS_PROBE_NUM 3
#define TCPIP_ZC_MDNS_MAX_PROBE_CONFLICT_NUM 30
#define TCPIP_ZC_MDNS_ANNOUNCE_NUM 3
#define TCPIP_ZC_MDNS_ANNOUNCE_INTERVAL 250
#define TCPIP_ZC_MDNS_ANNOUNCE_WAIT 250

/*** tcpip_cmd Configuration ***/
#define TCPIP_STACK_COMMAND_ENABLE
#define TCPIP_STACK_COMMANDS_STORAGE_ENABLE
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUESTS         4
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUEST_DELAY    1000
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_TIMEOUT          5000
#define TCPIP_STACK_COMMANDS_WIFI_ENABLE             	true
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUEST_BUFF_SIZE    2000
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUEST_DATA_SIZE    100


/*** IPv4 Configuration ***/

/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX0		"MRF24WN"
#define TCPIP_IF_MRF24WN
#define TCPIP_NETWORK_DEFAULT_HOST_NAME_IDX0				"MCHPBOARD_W"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX0				0
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS_IDX0			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_IP_MASK_IDX0				"255.255.255.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY_IDX0				"192.168.1.1"
#define TCPIP_NETWORK_DEFAULT_DNS_IDX0					"192.168.1.1"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS_IDX0			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE_IDX0			"full"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS_IDX0			\
													TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON |\
													TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON |\
													TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER_IDX0			WDRV_MRF24WN_MACObject
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX0			0
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH_IDX0	0
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX0			0
/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1		"PIC32INT"
#define TCPIP_IF_PIC32INT
#define TCPIP_NETWORK_DEFAULT_HOST_NAME_IDX1				"MCHPBOARD_E"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1				0
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS_IDX1			"192.168.100.116"
#define TCPIP_NETWORK_DEFAULT_IP_MASK_IDX1				"255.255.255.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY_IDX1				"192.168.100.1"
#define TCPIP_NETWORK_DEFAULT_DNS_IDX1					"192.168.100.1"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS_IDX1			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE_IDX1			"full"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS_IDX1			\
													TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON |\
													TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON |\
													TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER_IDX1			DRV_ETHMAC_PIC32MACObject
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1			0
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH_IDX1	0
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1			0
/*** TCPIP SYS FS Wrapper ***/
#define SYS_FS_MAX_PATH						80
#define LOCAL_WEBSITE_PATH_FS				"/mnt/myDrive1"
#define LOCAL_WEBSITE_PATH					"/mnt/myDrive1/"
#define SYS_FS_DRIVE						"SDCARD"
#define SYS_FS_SD_VOL						"/dev/mmcblka1"
#define SYS_FS_FATFS_STRING					"FATFS"
#define SYS_FS_MPFS_STRING					"MPFS2"

/*** USB Driver Configuration ***/


/* Enables Device Support */
#define DRV_USBHS_DEVICE_SUPPORT      true

/* Disable Host Support */
#define DRV_USBHS_HOST_SUPPORT      false

/* Maximum USB driver instances */
#define DRV_USBHS_INSTANCES_NUMBER    1

/* Interrupt mode enabled */
#define DRV_USBHS_INTERRUPT_MODE      true


/* Number of Endpoints used */
#define DRV_USBHS_ENDPOINTS_NUMBER    3




/*** USB Device Stack Configuration ***/










/* The USB Device Layer will not initialize the USB Driver */
#define USB_DEVICE_DRIVER_INITIALIZE_EXPLICIT

/* Maximum device layer instances */
#define USB_DEVICE_INSTANCES_NUMBER     1

/* EP0 size in bytes */
#define USB_DEVICE_EP0_BUFFER_SIZE      64










/* Maximum instances of CDC function driver */
#define USB_DEVICE_CDC_INSTANCES_NUMBER     1










/* CDC Transfer Queue Size for both read and
   write. Applicable to all instances of the
   function driver */
#define USB_DEVICE_CDC_QUEUE_DEPTH_COMBINED 3



// *****************************************************************************
/* BSP Configuration Options
*/
#define BSP_OSC_FREQUENCY 24000000


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
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

/*** Functions for BSP_STBY_RST pin ***/
#define BSP_STBY_RSTToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_STBY_RSTOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_STBY_RSTOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_STBY_RSTStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5)
#define BSP_STBY_RSTStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5, Value)

/*** Functions for BSP_WIFI_SLEEP pin ***/
#define BSP_WIFI_SLEEPToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_WIFI_SLEEPStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0, Value)


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
