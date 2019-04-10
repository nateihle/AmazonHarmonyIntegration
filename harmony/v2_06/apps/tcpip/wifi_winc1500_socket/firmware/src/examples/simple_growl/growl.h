/*******************************************************************************
  File Name:
    growl.h

  Summary:
    Growl Client Interface
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#include "socket.h"

#ifndef _GROWL_H
#define _GROWL_H

#define PROWL_CLIENT                    1
#define NMA_CLIENT                      2

#define GROWL_EVENT_MAX_LENGTH          16
#define GROWL_DESCRIPTION_MAX_LENGTH    72
#define GROWL_APPNAME_MAX_LENGTH        16

/* Possible codes could be returned by the NMA server. */
#define GROWL_SUCCESS                   20      /* NMA notification sent successfully. Actual returned code = "200". */

#define GROWL_ERR_BAD_REQUEST           40      /* The sent notification has a format error. Actual returned code = "400". */
#define GROWL_ERR_NOT_AUTHORIZED        41      /* The API Key supplied with the request is invalid. Actual returned code = "401". */
#define GROWL_ERR_NOT_ACCEPTED          42      /* Maximum number of API calls per hour exceeded. Actual returned code = "402". */
#define GROWL_ERR_API_EXCEED            46      /* Actual returned code = "406". */
#define GROWL_ERR_NOT_APPROVED          49      /* Actual returned code = "409". */
#define GROWL_ERR_SERVER_ERROR          50      /* Internal server error. Actual returned code = "500". */

#define GROWL_ERR_LOCAL_ERROR           30      /* An error occured on the Wi-Fi device due to internal problem. */

#define GROWL_ERR_CONN_FAILED           10      /* Maximum retry counts exceeded. */
#define GROWL_ERR_RESOLVE_DNS           11

#define GROWL_RETRY                     12

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

void NMI_GrowlInit(uint8_t *pNmaKey, uint8_t *pPrwKey);
void NMI_GrowldeInit(void);

/*
 * Send a specific notification to a registered NMA device.
 *
 * clientName: name of the Growl Client: NMA or Prowl.
 * pApp: Application which generates the notification.
 * pEvent: event triggered by the notification.
 * pu8Description: message describes the event happened.
 * bUseSSL: flag indicating if SSL is being used.
 */
int8_t NMI_GrowlSendNotification
(
    uint8_t clientName,
    uint8_t *pApp,
    uint8_t *pEvent,
    uint8_t *pu8Description,
    uint8_t bUseSSL
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _GROWL_H */
