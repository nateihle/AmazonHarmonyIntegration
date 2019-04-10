/*************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wdrv_mrf24wn_common.h

  Summary:
    MRF24WN Wireless Driver Common Header File

  Description:

 *************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

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

#ifndef _WDRV_MRF24WN_COMMON_H
#define _WDRV_MRF24WN_COMMON_H

#include "osal/osal.h"

#include "system/common/sys_module.h"

#include "system/command/sys_command.h"
#include "system/console/sys_console.h"
#include "system/int/sys_int.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

// Do not make this an enumerated type!
#define WDRV_FUNC_ENABLED 1
#define WDRV_FUNC_DISABLED 0

#define WDRV_NETWORK_TYPE_INFRASTRUCTURE 1
#define WDRV_NETWORK_TYPE_ADHOC          2 /* TODO: Invalid for now. Validation is TBD. */
#define WDRV_NETWORK_TYPE_P2P            3 /* TODO: Invalid for now. Validation is TBD. */
#define WDRV_NETWORK_TYPE_SOFT_AP        4

#define WDRV_BSSID_LENGTH 6
#define WDRV_MAX_SSID_LENGTH 32

/* WPA Passphrase Length Definitions */
#define WDRV_MIN_WPA_PASS_PHRASE_LENGTH 8
#define WDRV_MAX_WPA_PASS_PHRASE_LENGTH 63 // string terminator is not included

#define WDRV_MAX_SECURITY_KEY_LENGTH WDRV_MAX_WPA_PASS_PHRASE_LENGTH + 1

#define WDRV_WPS_PIN_LENGTH 8 // 7 digits + 1 checksum digit

#define WDRV_SECURITY_OPEN                     0
#define WDRV_SECURITY_WEP_40                   1
#define WDRV_SECURITY_WEP_104                  2
#define WDRV_SECURITY_WPA_WITH_PASS_PHRASE     3
#define WDRV_SECURITY_WPA2_WITH_PASS_PHRASE    4
#define WDRV_SECURITY_WPS_PUSH_BUTTON          6
#define WDRV_SECURITY_WPS_PIN                  7

#define WDRV_APCONFIG_BIT_PRIVACY              (0x10)
#define WDRV_APCONFIG_BIT_PREAMBLE_LONG        (0x20)
#define WDRV_APCONFIG_BIT_WPA                  (0x40)
#define WDRV_APCONFIG_BIT_WPA2                 (0x80)

/* Board Type Definitions */
#define WDRV_BD_TYPE_EXP16 0
#define WDRV_BD_TYPE_MX_ESK 1
#define WDRV_BD_TYPE_MZ_ESK 2
#define WDRV_BD_TYPE_CUSTOM 3

#define WDRV_ENABLE_INTR()              SYS_INT_Enable()
#define WDRV_DISABLE_INTR()             SYS_INT_Disable()

#define WDRV_MUTEX_CREATE(handle)       WDRV_MutexInit(handle)
#define WDRV_MUTEX_DELETE(handle)       WDRV_MutexDestroy(handle)
#define WDRV_MUTEX_LOCK(handle, waitMS) WDRV_MutexLock(handle, waitMS)
#define WDRV_MUTEX_UNLOCK(handle)       WDRV_MutexUnlock(handle)

#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
#define WDRV_TASK_CREATE(func, alias, stackDepth, param, prio, handle) \
                                        WDRV_TaskCreate(func, alias, stackDepth, param, prio, handle, 0)
#define WDRV_TASK_DELETE(handle)        WDRV_TaskDestroy(handle)
#define WDRV_TIME_DELAY(msec)           WDRV_UsecDelay(msec * 1000)
#endif /* (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9) */

#define WDRV_ASSERT(condition, msg) WDRV_Assert(condition, msg, __FILE__, __LINE__)

#define WDRV_DBG_NONE    0
#define WDRV_DBG_ERROR   1
#define WDRV_DBG_INFORM  2
#define WDRV_DBG_TRACE   3
#define WDRV_DBG_VERBOSE 4
#define WDRV_DBG_LEVEL   WDRV_DBG_INFORM

#if WDRV_DBG_LEVEL != WDRV_DBG_NONE
#define DEBUG_LOCK WDRV_MUTEX_LOCK(g_debugConsoleLock, 0)
#define DEBUG_UNLOCK WDRV_MUTEX_UNLOCK(g_debugConsoleLock)
#define WDRV_DBG_ERROR_PRINT(...) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_ERROR) { DEBUG_LOCK; SYS_CONSOLE_PRINT(__VA_ARGS__); DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_INFORM_PRINT(...) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_INFORM) { DEBUG_LOCK; SYS_CONSOLE_PRINT(__VA_ARGS__); DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_TRACE_PRINT(...) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_TRACE) { DEBUG_LOCK; SYS_CONSOLE_PRINT(__VA_ARGS__); DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_VERBOSE_PRINT(...) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_VERBOSE) { DEBUG_LOCK; SYS_CONSOLE_PRINT(__VA_ARGS__); DEBUG_UNLOCK; } } while (0)
#else
#define WDRV_DBG_ERROR_PRINT(...) do { } while (0)
#define WDRV_DBG_INFORM_PRINT(...) do { } while (0)
#define WDRV_DBG_TRACE_PRINT(...) do { } while (0)
#define WDRV_DBG_VERBOSE_PRINT(...) do { } while (0)
#endif

typedef struct __attribute__((__packed__))
{
    uint32_t verifyFlag; // 0x00000000: empty;    0xffffffff: empty;    0x5a5a5a5a: verified.
    uint8_t networkType;
    uint8_t ssid[WDRV_MAX_SSID_LENGTH + 1];
    uint8_t ssidLen;
    uint8_t securityMode; // WDRV_SECURITY_OPEN or one of the other security modes
    uint8_t securityKey[WDRV_MAX_SECURITY_KEY_LENGTH]; // Wi-Fi security key, or passphrase
    uint8_t securityKeyLen; // number of bytes in security key (can be 0)
} WDRV_CONFIG;

/* Intentionally define MRF24WN's device type to 3 to provide backward
 * compatibility with MRF24WG.
 */
typedef enum
{
    WDRV_MRF24WN0M_DEVICE = 3
} WDRV_DEVICE_TYPE;

typedef struct
{
    uint8_t deviceType;
    uint8_t romVersion;
    uint8_t patchVersion;
} WDRV_DEVICE_INFO;

/*******************************************************************************
  Summary:
    Contains data pertaining to MRF24WN connection context

  Description:
    Wi-Fi Connection Context

    This structure contains MRF24WN connection context data.
 */
typedef struct
{
    /* channel number of current connection */
    uint16_t channel;

    /* BSSID of connected AP */
    uint8_t bssid[6];

} WDRV_CONNECTION_CONTEXT;

typedef struct
{
    bool scanInProgress;
    uint16_t numberOfResults;
} WDRV_SCAN_STATUS;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi scan results

  Description:
    Wi-Fi Scan Results

    This structure contains the result of Wi-Fi scan operation. See
    WDRV_CLI_ScanGetResult().

    apConfig Bit Mask
    <table>
    Bit 7       Bit 6       Bit 5       Bit 4       Bit 3       Bit 2       Bit 1       Bit 0
    -----       -----       -----       -----       -----       -----       -----       -----
    WPA2        WPA         Preamble    Privacy     Reserved    Reserved    Reserved    IE
    </table>

    <table>
    IE         1 if AP broadcasting one or more Information Elements, else 0
    Privacy    0 : AP is open (no security) 1: AP using security, if neither WPA
                and WPA2 set then security is WEP.
    Preamble   0: AP transmitting with short preamble 1: AP transmitting with long preamble
    WPA        Only valid if Privacy is 1. 0: AP does not support WPA 1: AP
                supports WPA
    WPA2       Only valid if Privacy is 1. 0: AP does not support WPA2 1: AP supports WPA2
    </table>
 */
typedef struct
{
    /* Network BSSID value */
    uint8_t      bssid[WDRV_BSSID_LENGTH];

    /* Network SSID value */
    uint8_t      ssid[WDRV_MAX_SSID_LENGTH + 1];

    /* Access point configuration (see description) */
    uint8_t      apConfig;

    /* Not used */
    uint8_t      reserved;

    /* Network beacon interval */
    uint16_t     beaconPeriod;

    /* Only valid if bssType = WDRV_NETWORK_TYPE_INFRASTRUCTURE */
    uint16_t     atimWindow;

    /*
      List of Network basic rates.  Each rate has the following format:

          Bit 7
      * 0: rate is not part of the basic rates set
      * 1: rate is part of the basic rates set

          Bits 6:0
      Multiple of 500kbps giving the supported rate.  For example, a value of 2
      (2 * 500kbps) indicates that 1mbps is a supported rate.  A value of 4 in
      this field indicates a 2mbps rate (4 * 500kbps).
     */
    uint8_t      basicRateSet[8];

    uint8_t      extRateSet[4];

    /* Signal strength of received frame beacon or probe response.  Will range
       from a low of -95 to a high of 0. */
    int16_t      rssi;

    /* Number of valid rates in basicRates */
    uint8_t      numRates;

    /* Part of TIM element */
    uint8_t      dtimPeriod;

    /* WDRV_NETWORK_TYPE_INFRASTRUCTURE or WDRV_NETWORK_TYPE_ADHOC */
    uint8_t      bssType;

    /* Channel number */
    uint8_t      channel;

    /* Number of valid characters in SSID */
    uint8_t      ssidLen;

} WDRV_SCAN_RESULT;

typedef struct
{
   void (*CopyFrameToStackPacketBuffer_CB)(uint32_t len, const uint8_t *const frame);
   void (*ProceedConnectEvent_CB)(uint32_t connected, uint8_t devID, uint8_t *bssid, bool bssConn, uint8_t reason);
   void (*RFReady_CB)(const uint8_t *const addr);
   void (*ScanDone_CB)(uint32_t status);
   void (*InitDone_CB)(void);
   void (*WPSDone_CB)(void);
} WDRV_CALLBACKS;

typedef uint32_t *WDRV_MUTEX_HANDLE_TYPE;

typedef enum
{
    WDRV_ERROR                = -1,
    WDRV_SUCCESS              = 0,
    WDRV_INVALID_TASK_ID      = 1,
    WDRV_INVALID_PARAMETER    = 2,
    WDRV_INVALID_POINTER      = 3,
    WDRV_ALREADY_EXISTS       = 4,
    WDRV_INVALID_EVENT        = 5,
    WDRV_EVENT_TIMEOUT        = 6,
    WDRV_INVALID_MUTEX        = 7,
    WDRV_TASK_ALREADY_LOCKED  = 8,
    WDRV_MUTEX_ALREADY_LOCKED = 9,
    WDRV_OUT_OF_MEMORY        = 10,
} WDRV_OSAL_STATUS;

uint32_t WDRV_MutexInit(OSAL_MUTEX_HANDLE_TYPE **mutex_ptr);
uint32_t WDRV_MutexDestroy(OSAL_MUTEX_HANDLE_TYPE **mutex_ptr);
uint32_t WDRV_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex_ptr, uint32_t tick_count);
uint32_t WDRV_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex_ptr);
bool WDRV_SemInit(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemTake(OSAL_SEM_HANDLE_TYPE *SemID, uint16_t timeout);
void WDRV_SemGive(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemGiveFromISR(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemDeInit(OSAL_SEM_HANDLE_TYPE *SemID);

#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
WDRV_OSAL_STATUS WDRV_TaskCreate(void Task(void *), const char *task_name, int stack_size, void *param,
    unsigned long task_priority, TaskHandle_t *task_handle, bool auto_start);
WDRV_OSAL_STATUS WDRV_TaskDestroy(TaskHandle_t task_handle);
void WDRV_UsecDelay(uint32_t uSec);
#endif /* (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9) */

void WDRV_Assert(int condition, const char *msg, const char *file, int line);

extern OSAL_MUTEX_HANDLE_TYPE *g_debugConsoleLock;

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _WDRV_MRF24WN_COMMON_H */
