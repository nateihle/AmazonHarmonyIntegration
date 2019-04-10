/*******************************************************************************
  File Name:
    app.h

  Summary:


  Description:

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
// DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

#define WIFI_EASY_CONFIG_DEMO
#define WIFI_EASY_CONFIG_DEMO_VERSION_NUMBER "1.0"

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"

#if defined(TCPIP_IF_MRF24WN) /* Wi-Fi Interface */
#include "app_wifi_mrf24wn.h"
#elif defined(TCPIP_IF_WINC1500)
#include "app_wifi_winc1500.h"
#endif

/* BSP LED and Switch Re-directs */
/* This section is highly customizable based on application's specific needs. */
#if defined(BSP_SWITCH_4StateGet) // very roughly assume that pic32mx795_pim__e16 is used

#define APP_LED_1 BSP_LED_3
#define APP_LED_2 BSP_LED_4
#define APP_LED_3 BSP_LED_5

#define APP_SWITCH_1StateGet() BSP_SWITCH_4StateGet()
#define APP_SWITCH_2StateGet() BSP_SWITCH_5StateGet()
#define APP_SWITCH_3StateGet() BSP_SWITCH_6StateGet()

#elif !defined(BSP_SWITCH_2StateGet) // very roughly assume that pic32mz_ef_curiosity is used

#define APP_LED_1 BSP_LED_1
#define APP_LED_2 BSP_LED_2
#define APP_LED_3 BSP_LED_3

#define APP_SWITCH_1StateGet() BSP_SWITCH_1StateGet()
#define APP_SWITCH_2StateGet() BSP_SWITCH_1StateGet()
#define APP_SWITCH_3StateGet() BSP_SWITCH_1StateGet()

#else

#define APP_LED_1 BSP_LED_3
#define APP_LED_2 BSP_LED_2
#define APP_LED_3 BSP_LED_1

#define APP_SWITCH_1StateGet() true // UART Console occupies this pin on PIC32 MZ EC/EF SK
#define APP_SWITCH_2StateGet() BSP_SWITCH_2StateGet()
#define APP_SWITCH_3StateGet() BSP_SWITCH_1StateGet()

#endif

#define WF_SCAN_RESULTS_BUFFER_SIZE 32

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
 */
typedef enum
{
    /* The application mounts the disk. */
    APP_MOUNT_DISK = 0,

    /* In this state, the application waits for the initialization of the TCP/IP stack
       to complete. */
    APP_TCPIP_WAIT_INIT,

    /* The application configures the Wi-Fi settings. */
    APP_WIFI_CONFIG,

    /* In this state, the application runs the Wi-Fi prescan. */
    APP_WIFI_PRESCAN,

    /* In this state, the application enables TCP/IP modules such as DHCP, NBNS and mDNS
       in all available interfaces. */
    APP_TCPIP_MODULES_ENABLE,

    /* In this state, the application can do TCP/IP transactions. */
    APP_TCPIP_TRANSACT,

    /* In this state, the application performs module FW update over the air. */
    APP_FW_OTA_UPDATE,

    /* In this state, the application waits till FW update gets completed. */
    APP_WAIT_FOR_FW_UPDATE,

    /* The application waits in this state for the driver to be ready
       before sending the "hello world" message. */
    //APP_STATE_WAIT_FOR_READY,

    /* The application waits in this state for the driver to finish
       sending the message. */
    //APP_STATE_WAIT_FOR_DONE,

    /* The application does nothing in the idle state. */
    //APP_STATE_IDLE

    APP_USERIO_LED_DEASSERTED,

    APP_USERIO_LED_ASSERTED,

    APP_TCPIP_ERROR,

} APP_STATE;

typedef enum
{
    /* Initialize the state machine, and also checks if prescan is allowed. */
    APP_WIFI_PRESCAN_INIT,

    /* In this state the application waits for the prescan to finish. */
    APP_WIFI_PRESCAN_WAIT,

    /* After prescan, Wi-Fi module is reset in this state. */
    APP_WIFI_PRESCAN_RESET,

    /* In this state, the application waits for Wi-Fi reset to finish. */
    APP_WIFI_PRESCAN_WAIT_RESET,

    /* Prescan is complete. */
    APP_WIFI_PRESCAN_DONE,

} APP_WIFI_PRESCAN_STATE;

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
    /* SYS_FS file handle */
    SYS_FS_HANDLE fileHandle;

    /* application's current state */
    APP_STATE state;

    /* prescan's current state */
    APP_WIFI_PRESCAN_STATE prescanState;

    /* application data buffer */
    //uint8_t data[64];

    //uint32_t nBytesWritten;

    //uint32_t nBytesRead;
} APP_DATA;

/* It is intentionally declared this way to sync with WDRV_DEVICE_TYPE. */
typedef enum {
    MRF24WN_MODULE = 3,
    WINC1500_MODULE = 4,
    WILC1000_MODULE = 5
} WF_MODULE_TYPE;

typedef struct {
    uint8_t numberOfResults;
    WF_SCAN_RESULT results[WF_SCAN_RESULTS_BUFFER_SIZE];
} WF_SCAN_CONTEXT;

typedef struct {
    uint8_t ssid[WF_MAX_SSID_LENGTH + 1]; // 32-byte SSID plus null terminator
    uint8_t networkType;
    uint8_t prevSSID[WF_MAX_SSID_LENGTH + 1]; // previous SSID
    uint8_t prevNetworkType; // previous network type
    uint8_t wepKeyIndex;
    uint8_t securityMode;
    uint8_t securityKey[WDRV_MAX_SECURITY_KEY_LENGTH]; // null terminator is included
    uint8_t securityKeyLen; // number of bytes in security key (can be 0)
} WF_REDIRECTION_CONFIG;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize(void)

  Summary:
     This routine initializes the application object.

  Description:
    This routine initializes the application object. The application state is
    set to wait for media attach.

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
void APP_Initialize(void);

/*******************************************************************************
  Function:
    void APP_Tasks(void)

  Summary:
    Application Tasks Function

  Description:
    This routine implements the application in a non blocking manner.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this function.

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
void APP_Tasks(void);

/*******************************************************************************
  Function:
    uint8_t APP_WIFI_Prescan(void)

  Summary:
    Wi-Fi Prescan Function

  Description:
    This function implements the Wi-Fi prescan in a non blocking manner.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this function.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
uint8_t APP_WIFI_Prescan(void);

#endif /* _APP_H */

/*******************************************************************************
 End of File
*/
