/*******************************************************************************
  WiFi MAC interface functions

  File Name:
    wifi_web_config.h

  Summary:
   WiFi-specific MAC function prototypes called by TCP/IP stack.

  Description:
    The functions in this header file are accessed by the TCP/IP stack via
    function pointers.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc. All rights reserved.

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

#ifndef _WIFI_WEB_CONFIG_H
#define _WIFI_WEB_CONFIG_H

typedef struct
{
    unsigned char configBits;      /* used to dictate MAC behavior following the calculation */
    unsigned char phraseLen;       /* number of valid bytes in passphrase */
    unsigned char ssidLen;         /* number of valid bytes in SSID */
    unsigned char reserved;        /* alignment byte */
    char ssid[32+1];       /* the string of characters representing the SSID */
    char passPhrase[64+1]; /* the string of characters representing the pass phrase */
} t_wfPskCalcReq;

typedef struct {
    unsigned char             ssid[32+1];        // 32 for SSID and one for term character
    unsigned char             type;              // net type: infrastructure/adhoc
    unsigned char             prevSSID[32+1];    // previous SSID we were connected to
    unsigned char             prevWLAN;          // previous WLAN type

    unsigned char             security;          // security type   
    unsigned char             cipher;            //cipher
    unsigned char             defaultWepKey;     // WEP index value
    unsigned char             key[64+1];         // 64-byte key plus term character
    unsigned char             SecurityKeyLength; // number of bytes in security key (can be 0)
 
} t_wfEasyConfigCtx;

typedef struct
{
    unsigned char     scanState;
    unsigned short   numScanResults;
    unsigned short   displayIdx;
} WIFI_CONFIG_SCAN_CONTEXT;

#define WIFI_WEB_BSSID_LENGTH                   (6)
#define WIFI_WEB_MAX_SSID_LENGTH                (33)

typedef struct
{
    /* Network BSSID value */
    unsigned char      bssid[WIFI_WEB_BSSID_LENGTH];

    /*  Network SSID value */
    unsigned char      ssid[WIFI_WEB_MAX_SSID_LENGTH];

    /* Access Point config */
    unsigned char      apdot11info;

    /* Signal strength of received frame beacon or probe response.  Will range
       from a low of 43 to a high of 128. */
    unsigned char      rssi;

    /* DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE or DRV_WIFI_NETWORK_TYPE_ADHOC */
    unsigned char      bssType;

    /* Channel number */
    unsigned char      channel;

    /* Number of valid characters in ssid */
    unsigned char      ssidLen;

} WIFI_WEB_SCAN_RESULT;

//DRV_WIFI_SCAN_RESULT preScanResult[50]; //WF_PRESCAN  May change struct later for memory optimization

/* Easy Config Public Functions */
void WFEasyConfigInit(void);

void get_proper_ap_full_info();

void WFInitScan(void);

unsigned char WiFiStartScan(void);

unsigned char  WFRetrieveScanResult(unsigned char  Idx, WIFI_WEB_SCAN_RESULT *p_ScanResult);


#if 0
void WFScanEventHandler(unsigned short  scanResults);

void WFDisplayScanMgr(void);

void WF_InitForSoftApReDirection_enable(void);
int WF_InitForSoftApReDirection(void);
#endif

/* Cipher Type*/
typedef enum{
            WIFI_SECURITY_OPEN = 0,
            WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE = 1,
            WIFI_SECURITY_WPA_AUTO_WITH_KEY =2,
            WIFI_SECURITY_WEP_40 = 3,
            WIFI_SECURITY_WEP_104 = 4,
            WIFI_INVALID_SECURITY_MODE = 5
} WIFI_SECURITY_CIPHER_T;

/* Macros */
#define WF_WEP_KEY_INVALID    0xff

/* Definitions for stall feature of state machine */
#define WF_EASY_CONFIG_DELAY_TIME    (1ul)  /* In seconds */

/* Scan status/control bits */
#define SCAN_STATE_IN_PROGRESS  0x0001 /* If scan is in progress */
#define SCAN_STATE_VALID_RESULTS  0x0002 /* If we have the valid scan results */
#define SCAN_STATE_DISPLAY_RESULTS  0x0004 /* This flag is only used to control WFDisplayScanMgr() */

#define IS_SCAN_IN_PROGRESS(x)        ((x) & SCAN_STATE_IN_PROGRESS)
#define IS_SCAN_STATE_VALID(x)        ((x) & SCAN_STATE_VALID_RESULTS)
#define IS_SCAN_STATE_DISPLAY(x)      ((x) & SCAN_STATE_DISPLAY_RESULTS)

#define SCAN_SET_IN_PROGRESS(x)       ((x) |= SCAN_STATE_IN_PROGRESS)
#define SCAN_SET_VALID(x)             ((x) |= SCAN_STATE_VALID_RESULTS)
#define SCAN_SET_DISPLAY(x)           ((x) |= SCAN_STATE_DISPLAY_RESULTS)

#define SCAN_CLEAR_IN_PROGRESS(x)     ((x) &= ~SCAN_STATE_IN_PROGRESS)
#define SCAN_CLEAR_VALID(x)           ((x) &= ~SCAN_STATE_VALID_RESULTS)
#define SCAN_CLEAR_DISPLAY(x)         ((x) &= ~SCAN_STATE_DISPLAY_RESULTS)

#endif /* _WIFI_WEB_CONFIG_H */

// DOM-IGNORE-END
