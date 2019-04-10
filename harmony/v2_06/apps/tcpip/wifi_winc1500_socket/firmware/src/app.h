/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application
    entry point, app.c and means to configure specific demo examples.

  Description:
    When it comes to configurations, there are 2 categories.
    1. Demo Example
    2. FW Update Helper
    For details, refer to the below configuration setup.
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

#include "system_config.h"
#include "system_definitions.h"

/*
 * There are 2 configuration categories below.
 *
 * 1. Demo Example
 * 2. FW Update Helper
 *
 * Note that at any given time, only 1 definition macro should turn to 1. If you turn on
 * multiple macros at once, then it will cause a compilation error due to duplicated
 * symbols.
 */

/*
 * Demo Example Configurations
 */

#define AP_SCAN_EXAMPLE				1 /* Scans APs around you and displays the results, and then connects to the target AP */

#define CHIP_INFO_GET_EXAMPLE		0 /* Gets the chip information of WINC1500 */

#define EMAIL_SEND_EXAMPLE			0 /* Demonstrates email sending */

#define HTTP_DOWNLOAD_EXAMPLE		0 /* Demonstrates file downloading by HTTP client */

#define IP_ADDR_LOCATE_EXAMPLE		0 /* Gets current location where my IP is used */

#define MAC_ADDRESS_GET_EXAMPLE		0 /* Gets the MAC address of WINC1500 */

#define MDNS_EXAMPLE				0 /* Demonstrates mDNS server with service discovery support */

#define MODE_AP_EXAMPLE				0 /* Demonstrates starting SoftAP with WPA, WEP or open security */

#define MODE_CLIENT_STA_EXAMPLE		0 /* Demonstrates client mode connection with WPA */

#define MULTI_SOCKET_EXAMPLE		0 /* Demonstrates running multiple TCP clients at the same time */

#define P2P_EXAMPLE					0 /* Demonstrates Wi-Fi Direct GC function */

#define POWER_SAVE_EXAMPLE			0 /* Demonstrates power save features */

#define PROVISION_AP_EXAMPLE		0 /* Demonstrates provisioning using SoftAP function through Android application */

#define PROVISION_HTTP_EXAMPLE		0 /* Demonstrates provisioning using SoftAP function through webpage */

#define PUBNUB_CLOUD_EXAMPLE		0 /* Demonstrates publishing and subscribing using PubNub */

#define SECURITY_WEP_WPA_EXAMPLE	0 /* Demonstrates client mode connection with WPA */

#define SIMPLE_GROWL_EXAMPLE		0 /* Demonstrates notification transmitting among WINC1500, public remote server and smartphone application */

#define SSL_CLIENT_EXAMPLE			0 /* Demonstrates SSL client */

#define SSL_SERVER_EXAMPLE			0 /* Demonstrates SSL server */

#define TCP_CLIENT_EXAMPLE			0 /* Demonstrates TCP client */

#define TCP_SERVER_EXAMPLE			0 /* Demonstrates TCP server */

#define TIME_CLIENT_EXAMPLE			0 /* Demonstrates SNTP client */

#define UDP_EXAMPLE					0 /* Demonstrates UDP server and client */

#define UDP_CLIENT_EXAMPLE			0 /* Demonstrates UDP client */

#define UDP_SERVER_EXAMPLE			0 /* Demonstrates UDP server */

#define WEATHER_CLIENT_EXAMPLE		0 /* Weather client retrieves weather information of the target location using HTTP query */

#define WPS_CONNECT_EXAMPLE			0 /* Demonstrates WPS security in client mode */

#if IP_ADDR_LOCATE_EXAMPLE || HTTP_DOWNLOAD_EXAMPLE
#define IOT_SUPPORT 1 /* Enables additional IoT supporting features, for instance, HTTP client */
#endif

/*
 * FW Update Helper Configurations
 */

#define FW_UPDATE_OTA				0 /* Supports FW update over the air */

#define FW_UPDATE_OVER_SERIAL		0 /* Supports FW update over serial port */

/*
 * Configuration Validity Check
 */

#define EXAMPLES_ON_DECK AP_SCAN_EXAMPLE \
	+ CHIP_INFO_GET_EXAMPLE \
	+ EMAIL_SEND_EXAMPLE \
	+ HTTP_DOWNLOAD_EXAMPLE \
	+ IP_ADDR_LOCATE_EXAMPLE \
	+ MAC_ADDRESS_GET_EXAMPLE \
	+ MDNS_EXAMPLE \
	+ MODE_AP_EXAMPLE \
	+ MODE_CLIENT_STA_EXAMPLE \
	+ MULTI_SOCKET_EXAMPLE \
	+ P2P_EXAMPLE \
	+ POWER_SAVE_EXAMPLE \
	+ PROVISION_AP_EXAMPLE \
	+ PROVISION_HTTP_EXAMPLE \
	+ PUBNUB_CLOUD_EXAMPLE \
	+ SECURITY_WEP_WPA_EXAMPLE \
	+ SIMPLE_GROWL_EXAMPLE \
	+ SSL_CLIENT_EXAMPLE \
	+ SSL_SERVER_EXAMPLE \
	+ TCP_CLIENT_EXAMPLE \
	+ TCP_SERVER_EXAMPLE \
	+ TIME_CLIENT_EXAMPLE \
	+ UDP_EXAMPLE \
	+ UDP_CLIENT_EXAMPLE \
	+ UDP_SERVER_EXAMPLE \
	+ WEATHER_CLIENT_EXAMPLE \
	+ WPS_CONNECT_EXAMPLE \
	+ FW_UPDATE_OTA \
	+ FW_UPDATE_OVER_SERIAL

#if EXAMPLES_ON_DECK == 1
/* Okay. Only 1 example can build at a time. */
#elif EXAMPLES_ON_DECK > 1
#error "Oops! You seem turning on multiple examples, which cannot build at once due to duplicated symbols. Check it and enable only 1 example at a time. "
#else
#error "Oops! All examples seem turned off. Check it and enable 1 example you wish to test. "
#endif

/*******************************************************************************
  Structure:
    fw_update_param

  Summary:
    Firmware update configuration/initialization data.

  Description:
    This structure defines firmware update configuration/initialization data.
*/
struct fw_update_param
{
	bool over_serial;
	bool over_the_air;
	char *ota_server;
};

/*******************************************************************************
  Structure:
    radio_init_param

  Summary:
    Radio initialization data.

  Description:
    This structure defines radio initialization data regarding firmware update option.
*/
struct radio_init_param
{
	bool fw_update_go;
	struct fw_update_param fw_update;
};

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
void APP_Initialize(void);

/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony application tasks function.

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
void APP_Tasks(void);

/*******************************************************************************
  Function:
    void radio_init(void *arg)

  Summary:
    Initialize WINC1500 module.

  Description:
    This routine initializes WINC1500 module.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    arg - used for fw update.

  Returns:
    None.

  Example:
    <code>
    radio_init(NULL);
    </code>

  Remarks:
    None
 */
void radio_init(void *arg);

/*******************************************************************************
  Function:
    void radio_deinit(void)

  Summary:
    Deinitialize WINC1500 module.

  Description:
    This routine deinitializes WINC1500 module.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    radio_deinit();
    </code>

  Remarks:
    None
 */
void radio_deinit(void);

#endif /* _APP_HEADER_H */

/*******************************************************************************
 End of File
 */
