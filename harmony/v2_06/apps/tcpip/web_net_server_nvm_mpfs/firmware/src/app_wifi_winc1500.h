/*******************************************************************************
  File Name:
    app_wifi_winc1500.h

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

#ifndef _APP_WIFI_WINC1500_H
#define _APP_WIFI_WINC1500_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "wdrv_winc1500_iwpriv.h"

#define WF_DISABLED WDRV_FUNC_DISABLED
#define WF_ENABLED WDRV_FUNC_ENABLED

#define WF_MAX_SSID_LENGTH WDRV_MAX_SSID_LENGTH
#define WF_MAX_SECURITY_KEY_LENGTH WDRV_MAX_SECURITY_KEY_LENGTH

#define WF_NETWORK_TYPE_INFRASTRUCTURE WDRV_NETWORK_TYPE_INFRASTRUCTURE
#define WF_NETWORK_TYPE_ADHOC -1 /* Unsupported */
#define WF_NETWORK_TYPE_P2P -1 /* Unsupported */
#define WF_NETWORK_TYPE_SOFT_AP WDRV_NETWORK_TYPE_SOFT_AP

#define WF_SECURITY_OPEN WDRV_SECURITY_OPEN
#define WF_SECURITY_WEP_40 WDRV_SECURITY_WEP_40
#define WF_SECURITY_WEP_104 WDRV_SECURITY_WEP_104
#define WF_SECURITY_WPA_WITH_KEY 0xff /* Unsupported */
#define WF_SECURITY_WPA_WITH_PASS_PHRASE 0xff /* Unsupported */
#define WF_SECURITY_WPA2_WITH_KEY 0xff /* Unsupported */
#define WF_SECURITY_WPA2_WITH_PASS_PHRASE 0xff /* Unsupported */
#define WF_SECURITY_WPA_AUTO_WITH_KEY 0xff /* Unsupported */
#define WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE
#define WF_SECURITY_WPS_PUSH_BUTTON WDRV_SECURITY_WPS_PUSH_BUTTON
#define WF_SECURITY_WPS_PIN WDRV_SECURITY_WPS_PIN

#define WF_DEFAULT_POWER_SAVE WDRV_DEFAULT_POWER_SAVE

#define WF_WEP_KEY_INVALID 0xff

#define WF_ASSERT(condition, msg) WDRV_ASSERT(condition, msg)

typedef WDRV_CONFIG WF_CONFIG;
typedef WDRV_DEVICE_INFO WF_DEVICE_INFO;
typedef WDRV_SCAN_RESULT WF_SCAN_RESULT;

#endif /* _APP_WIFI_WINC1500_H */

/*******************************************************************************
 End of File
 */
