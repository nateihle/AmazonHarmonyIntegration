/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/attribs.h>
#include "app.h"


int g_wifi_link_up = 0;

#if defined(TCPIP_IF_MRF24WN) /* Wi-Fi Interface */
#include "app_wifi_mrf24wn.h"
#elif defined(TCPIP_IF_WINC1500)
#include "app_wifi_winc1500.h"
#elif defined(TCPIP_IF_WILC1000)
#include "app_wifi_wilc1000.h"
#endif


#define MICROCHIP_PIC32
#define PIC32_STARTER_KIT 

#include <stdio.h>
#include <stdlib.h>

#include <p32xxxx.h>
/*
    #define _SUPPRESS_PLIB_WARNING
    #define _DISABLE_OPENADC10_CONFIGPORT_WARNING
    #if __XC32_VERSION < 1400
    #include <plib.h>
    #endif
 */
#include <sys/appio.h>

#include "wolfmqtt/mqtt_client.h"
#include "examples/mqttexample.h"
#include "examples/mqttclient/mqttclient.h"
/*
 * OSAL_USE_RTOS == 1 means FreeRTOS version 8.x.x is used.
 * OSAL_USE_RTOS == 9 means the latest FreeRTOS version that comes with Harmony
 *  is used.
 * Following functions are implemented specifically for these two cases.
 */


#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
#define APP_OSAL_MUTEX_LOCK() APP_OSAL_MutexLock(&s_appLock, OSAL_WAIT_FOREVER)
#define APP_OSAL_MUTEX_UNLOCK() APP_OSAL_MutexUnlock(&s_appLock)
#else
#define APP_OSAL_MUTEX_LOCK() do {} while (0)
#define APP_OSAL_MUTEX_UNLOCK() do {} while (0)
#endif /* (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9) */

#if defined(TCPIP_IF_MRF24WN)
#define WIFI_INTERFACE_NAME "MRF24WN"
#elif defined(TCPIP_IF_WINC1500)
#define WIFI_INTERFACE_NAME "WINC1500"
#elif defined(TCPIP_IF_WILC1000)
#define WIFI_INTERFACE_NAME "WILC1000"
#endif

#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
#define IS_MDNS_RUN() true
#else
#define IS_MDNS_RUN() false
#define TCPIP_MDNS_ServiceRegister(a, b, c, d, e, f, g, h) do {} while (0)
#define TCPIP_MDNS_ServiceDeregister(a) do {} while (0)
#endif /* defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD) */


#if defined(TCPIP_STACK_USE_NBNS)
#define IS_NBNS_RUN() true
#else
#define IS_NBNS_RUN() false
#endif /* defined(TCPIP_STACK_USE_NBNS) */

#define IS_WF_INTF(x) ((strcmp(x, "MRF24W") == 0) || (strcmp(x, "MRF24WN") == 0) || (strcmp(x, "WINC1500") == 0) || (strcmp(x, "WILC1000") == 0))

#define APP_WIFI_RECONNECTION_RETRY_LIMIT 16

#define APP_WIFI_DHCP_WAIT_THRESHOLD 60 /* seconds */

#ifdef TCPIP_DHCP_CLIENT_ENABLED
#define timestamp_dhcp_kickin(x) do { x = 1; } while (0)
#else
#define timestamp_dhcp_kickin(x) do { x = 1; } while (0)
#endif

extern uint32_t TCPIP_SNTP_UTCSecondsGet(void);


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

static APP_DATA s_appData;
static MQTTCtx mqttCtx;
static BSP_LED_STATE s_LEDstate = BSP_LED_STATE_OFF;
static IWPRIV_GET_PARAM s_app_get_param;
static IWPRIV_SET_PARAM s_app_set_param;

#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
static OSAL_MUTEX_HANDLE_TYPE s_appLock;
#endif

WF_CONFIG g_wifi_cfg;
WF_DEVICE_INFO g_wifi_deviceInfo;
static IWPRIV_SET_PARAM s_app_set_param;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
/* The application task runs forever, so no de-init function is provided. */
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex);
static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS);
static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex);
#endif

static void APP_CONSOLE_HeaderDisplay(void);
static void APP_WIFI_IPv6MulticastFilter_Set(TCPIP_NET_HANDLE netH);
static void APP_WIFI_PowerSave_Config(bool enable);
static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Disable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Enable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Down(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Up(TCPIP_NET_HANDLE netH);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_WIFI_Initialize(void)

  Remarks:
    None.
 */

void APP_WIFI_Initialize(void)
{    
#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
	if (!APP_OSAL_MutexInit(&s_appLock)) {
		SYS_CONSOLE_MESSAGE("APP: Mutex initialization failed!\r\n");
		return;
	}
#endif
	s_appData.state = APP_MOUNT_DISK;

    s_app_set_param.conn.initConnAllowed = true;
    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
    DBINIT();

    SYS_CONSOLE_MESSAGE("===  Initializing wolfMQTT  ===\n");
    mqtt_init_ctx(&mqttCtx);
    mqttCtx.app_name = "mqttclient";
    mqttCtx.qos = MQTT_QOS_2;
    mqttCtx.test_mode = 1;
#ifdef ENABLE_MQTT_TLS 
    mqttCtx.use_tls = 1;
#endif    
    
}

static bool disk_mount(void)
{
	if (SYS_FS_Mount(SYS_FS_NVM_VOL, LOCAL_WEBSITE_PATH_FS, MPFS2, 0, NULL) == 0) {
		APP_CONSOLE_HeaderDisplay();
		s_appData.state = APP_TCPIP_WAIT_INIT;
		return true;
	}
#if 0    
	return false;
#else
    s_appData.state = APP_TCPIP_WAIT_INIT;
    return true;
#endif    
}

static bool tcpip_init_wait(void)
{
	SYS_STATUS tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);

	if (tcpipStat < 0) {
		SYS_CONSOLE_MESSAGE("APP: TCP/IP stack initialization failed!\r\n");
		s_appData.state = APP_TCPIP_ERROR;
		return true;
	} else if (tcpipStat == SYS_STATUS_READY) {
		s_appData.state = APP_WIFI_CONFIG;
		return true;
	}
	return false;
}

static bool wifi_config(IPV4_ADDR *defaultIpWiFi, TCPIP_NET_HANDLE *netHandleWiFi)
{
	/*
	 * Following "if condition" is useless when demo firstly
	 * boots up, since stack's status has already been checked in
	 * APP_TCPIP_WAIT_INIT. But it is necessary in Wi-Fi interface
	 * reset due to connection errors.
	 */
	iwpriv_get(DRVSTATUS_GET, &s_app_get_param);
	if (s_app_get_param.driverStatus.isOpen) {
		s_app_get_param.devInfo.info = &g_wifi_deviceInfo;
		iwpriv_get(DEVICEINFO_GET, &s_app_get_param);
		*netHandleWiFi = TCPIP_STACK_NetHandleGet(WIFI_INTERFACE_NAME);
		defaultIpWiFi->Val = TCPIP_STACK_NetAddress(*netHandleWiFi);
		s_appData.state = APP_TCPIP_MODULES_ENABLE;
		return true;
	}
	return false;
}

static void tcpip_module_enable(void)
{
	int i, nNets;

	// check available interfaces
	nNets = TCPIP_STACK_NumberOfNetworksGet();
	for (i = 0; i < nNets; ++i)
		APP_TCPIP_IFModules_Enable(TCPIP_STACK_IndexToNet(i));
	s_appData.state = APP_TCPIP_TRANSACT;
}

static void network_run(int16_t *ipWait, TCPIP_NET_HANDLE *netHandleWiFi, IPV4_ADDR *defaultIpWiFi)
{
	int i, nNets;
	static bool isWiFiPowerSaveConfigured = false;
	static bool wasNetUp[2] = {true, true}; // this app supports 2 interfaces so far
	static uint32_t reconn_retries = 0;
	static uint32_t startTick = 0;
	static IPV4_ADDR dwLastIP[2] = { {-1}, {-1} }; // this app supports 2 interfaces so far

	if (g_wifi_deviceInfo.deviceType == WINC1500_MODULE) {
		IWPRIV_GET_PARAM param;
		iwpriv_get(FWUPGRADEREQUEST_GET, &param);
		if (param.fwUpgrade.requested) {
			s_appData.state = APP_FW_OTA_UPDATE;
			return;
		}
	}

	iwpriv_get(CONNSTATUS_GET, &s_app_get_param);
	if (s_app_get_param.conn.status == IWPRIV_CONNECTION_SUCCESSFUL) {
		// resetting reconnection retries
		reconn_retries = 0;
	} else if (s_app_get_param.conn.status == IWPRIV_CONNECTION_FAILED) {
		if (reconn_retries++ < APP_WIFI_RECONNECTION_RETRY_LIMIT) {
			SYS_CONSOLE_PRINT("\r\nCouldn't connect to target AP, resetting Wi-Fi module and trying to reconnect, retries left: %u\r\n\n",
				APP_WIFI_RECONNECTION_RETRY_LIMIT - reconn_retries);
			APP_TCPIP_IFModules_Disable(*netHandleWiFi);
			APP_TCPIP_IF_Down(*netHandleWiFi);
			APP_TCPIP_IF_Up(*netHandleWiFi);
			isWiFiPowerSaveConfigured = false;
			s_appData.state = APP_WIFI_CONFIG;
			return;
		}
	} else if (s_app_get_param.conn.status == IWPRIV_CONNECTION_REESTABLISHED) {
		// restart dhcp client and config power save
		TCPIP_DHCP_Disable(*netHandleWiFi);
		TCPIP_DHCP_Enable(*netHandleWiFi);
		isWiFiPowerSaveConfigured = false;
		timestamp_dhcp_kickin(*ipWait);
	}

	/*
	 * Following for loop is to deal with manually controlling
	 * interface down/up (for example, through console commands
	 * or web page).
	 */
	nNets = TCPIP_STACK_NumberOfNetworksGet();
	for (i = 0; i < nNets; ++i) {
		TCPIP_NET_HANDLE netH = TCPIP_STACK_IndexToNet(i);
		if (!TCPIP_STACK_NetIsUp(netH) && wasNetUp[i]) {
			const char *netName = TCPIP_STACK_NetNameGet(netH);
			wasNetUp[i] = false;
			APP_TCPIP_IFModules_Disable(netH);
			if (IS_WF_INTF(netName))
				isWiFiPowerSaveConfigured = false;
		}

		if (TCPIP_STACK_NetIsUp(netH) && !wasNetUp[i]) {
			wasNetUp[i] = true;
			APP_TCPIP_IFModules_Enable(netH);
		}
	}

	/*
	 * If we get a new IP address that is different than the default one,
	 * we will run PowerSave configuration.
	 */
	if (!isWiFiPowerSaveConfigured &&
		TCPIP_STACK_NetIsUp(*netHandleWiFi) &&
		(TCPIP_STACK_NetAddress(*netHandleWiFi) != defaultIpWiFi->Val)) {
		APP_WIFI_PowerSave_Config(true);
		isWiFiPowerSaveConfigured = true;
	}

	APP_WIFI_DHCPS_Sync(*netHandleWiFi);

	APP_OSAL_MUTEX_LOCK();
	/*
	 * If the IP address of an interface has changed,
	 * display the new value on console.
	 */
	for (i = 0; i < nNets; ++i) {
		IPV4_ADDR ipAddr;
		TCPIP_NET_HANDLE netH = TCPIP_STACK_IndexToNet(i);
		ipAddr.Val = TCPIP_STACK_NetAddress(netH);
		if (dwLastIP[i].Val != ipAddr.Val) {
			dwLastIP[i].Val = ipAddr.Val;
			if (ipAddr.Val != 0) {
				SYS_CONSOLE_PRINT("%s IPv4 Address: %d.%d.%d.%d \r\n", TCPIP_STACK_NetNameGet(netH),
					ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
				*ipWait = 0;
				g_wifi_link_up = 1;
			}
		}
	}

	if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() / 2ul) {
		if (*ipWait && ++*ipWait > APP_WIFI_DHCP_WAIT_THRESHOLD) {
			*ipWait = 0;
			if (s_app_get_param.conn.status == IWPRIV_CONNECTION_SUCCESSFUL)
				SYS_CONSOLE_MESSAGE("\r\nFailed to obtain an IP address from DHCP server\r\n"\
					"If WEP security is used, double-check if the key is valid\r\n");
		}
		startTick = SYS_TMR_TickCountGet();
	}
	APP_OSAL_MUTEX_UNLOCK();
}

static void led_toggle(void)
{
	static uint32_t startTick = 0;

	if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() / 2ul) {
		startTick = SYS_TMR_TickCountGet();
		s_LEDstate ^= BSP_LED_STATE_ON;
		BSP_LEDStateSet(APP_LED_1, s_LEDstate);
	}
}
#if 0
static void firmware_update(TCPIP_NET_HANDLE *netHandleWiFi)
{
	APP_TCPIP_IFModules_Disable(*netHandleWiFi);
	APP_TCPIP_IF_Down(*netHandleWiFi);
	APP_TCPIP_IF_Up(*netHandleWiFi);
	s_appData.state = APP_WAIT_FOR_FW_UPDATE;
}
#endif



/******************************************************************************
  Function:
    void APP_WIFI_Tasks(void)

  Remarks:
    None.
 */
void APP_WIFI_Tasks(void)
{
	static int16_t ipWait = 0;
	static IPV4_ADDR defaultIpWiFi = {-1};
	static TCPIP_NET_HANDLE netHandleWiFi = NULL;
    
	switch (s_appData.state) {
	case APP_MOUNT_DISK:
		if (!disk_mount())
			break;
	case APP_TCPIP_WAIT_INIT:
		if (!tcpip_init_wait())
			break;
	case APP_WIFI_CONFIG:
		if (!wifi_config(&defaultIpWiFi, &netHandleWiFi))
			break;
	case APP_TCPIP_MODULES_ENABLE:
		tcpip_module_enable();
		timestamp_dhcp_kickin(ipWait);
	case APP_TCPIP_TRANSACT:
		SYS_CMD_READY_TO_READ();
		network_run(&ipWait, &netHandleWiFi, &defaultIpWiFi);        
		led_toggle();
		if (g_wifi_link_up)
		{
            if (TCPIP_SNTP_UTCSecondsGet())
            {
	            int rc = mqttclient_test(&mqttCtx);
                if (mqttCtx.stat == WMQ_DONE)
                    g_wifi_link_up = 0;
                    
                
	            if (rc != MQTT_CODE_CONTINUE) {
	                /* reset mqttCtx.stat and reconnect/try again */
	                mqttCtx.stat = WMQ_BEGIN;
	            }

            }                
		}
		break;
#if 0
	case APP_TCPIP_WAIT_ON_DNS:
		{
			dns_resolve_resp();
		}
		break;

	case APP_TCPIP_START_CONNECTION:
		{
			start_socket();
		}
		break;

	case APP_TCPIP_WAIT_FOR_CONNECTION:
		{
			if (!NET_PRES_SocketIsConnected(s_appData.socket))
			{
				return;
			}
			s_appData.connectionOpened = SYS_TMR_SystemCountGet();
			
			if (!NET_PRES_SocketEncryptSocket(s_appData.socket))
			{
				SYS_CONSOLE_MESSAGE("SSL Create Connection Failed - Aborting\r\n");
				s_appData.state = APP_TCPIP_CLOSE_CONNECTION;
			}
			else
			{
				s_appData.state = APP_TCPIP_WAIT_FOR_SSL_CONNECT;
			
			}
		}
	break;
	
	case APP_TCPIP_WAIT_FOR_SSL_CONNECT:
		{
            if (NET_PRES_SocketIsNegotiatingEncryption(s_appData.socket))
            {
                break;
            }
            if (!NET_PRES_SocketIsSecure(s_appData.socket))
            {
                SYS_CONSOLE_MESSAGE("SSL Connection Negotiation Failed - Aborting\r\n");
                s_appData.state = APP_TCPIP_CLOSE_CONNECTION;
                break;
            }
            SYS_CONSOLE_MESSAGE("SSL Connection Opened: Starting Clear Text Communication\r\n");
            s_appData.cyaSSLLogEnabled = 0;
            s_appData.sslNegComplete = SYS_TMR_SystemCountGet();;
            s_appData.state = APP_TCPIP_SEND_REQUEST_SSL;			
		}
		break;

		case APP_TCPIP_SEND_REQUEST_SSL:
		{
			char const *tmp_buff = "search?q=get+request&oq=get+request&gs_l=psy-ab.3..0i67k1l3j0.9282.16102.0.18165.38.20.0.0.0.0.411.2316.0j6j3j1j1.11.0....0...1.1.64.psy-ab..28.10.2134.6..35i39k1j0i22i30k1j0i131k1.Zssy2xN39BQ&gws_rd=cr&ei=olmRWdiFCcbtvgS4rIb4Bg";
			sprintf(networkBuffer, "GET /%s HTTP/1.1\r\n"
					"Host: %s\r\n"
					"Connection: close\r\n\r\n", tmp_buff, s_appData.host);
			SYS_CONSOLE_PRINT("In APP_TCPIP_SEND_REQUEST_SSL %s\r\n", networkBuffer);
			int ret;
			ret = NET_PRES_SocketWrite(s_appData.socket, (uint8_t*)networkBuffer, strlen(networkBuffer));
			s_appData.clearBytesSent += ret;
			s_appData.state = APP_TCPIP_WAIT_FOR_RESPONSE_SSL;
			break;
		}
		case APP_TCPIP_WAIT_FOR_RESPONSE_SSL:
		{
			if (NET_PRES_SocketReadIsReady(s_appData.socket) == 0)
			{
				if (NET_PRES_SocketWasReset(s_appData.socket))
				{
					s_appData.state = APP_TCPIP_CLOSE_CONNECTION;
				}
				break;
			}
			if (s_appData.firstRxDataPacket == 0)
			{
				s_appData.firstRxDataPacket = SYS_TMR_SystemCountGet();
			}
			s_appData.lastRxDataPacket = SYS_TMR_SystemCountGet();
			uint16_t res = NET_PRES_SocketRead(s_appData.socket, (uint8_t*)networkBuffer, sizeof(networkBuffer));
			s_appData.clearBytesReceived += res;
			s_appData.rawBytesReceived += res;
			SYS_CONSOLE_PRINT("%s\r\n", networkBuffer);
			break;
		}
#endif

	default:
		break;
	}
}

#if (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9)
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    if (OSAL_MUTEX_Create(mutex) == OSAL_RESULT_TRUE)
        return true;
    else
        return false;
}

static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS)
{
    OSAL_MUTEX_Lock(mutex, waitMS);
}

static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    OSAL_MUTEX_Unlock(mutex);
}
#endif /* (OSAL_USE_RTOS == 1 || OSAL_USE_RTOS == 9) */

static void APP_CONSOLE_HeaderDisplay(void)
{
    #if defined(WIFI_TCPIP_WEB_SERVER_DEMO)
        SYS_CONSOLE_MESSAGE("\r\n====================================");
        SYS_CONSOLE_MESSAGE("\r\n*** Wi-Fi TCP/IP Web Server Demo ***");
        SYS_CONSOLE_MESSAGE("\r\n====================================\r\n");
    #else
        SYS_CONSOLE_MESSAGE("\r\n===================================");
        SYS_CONSOLE_MESSAGE("\r\n*** Microchip Wi-Fi TCP/IP Demo ***");
        SYS_CONSOLE_MESSAGE("\r\n===================================\r\n");
    #endif
}

static void APP_WIFI_IPv6MulticastFilter_Set(TCPIP_NET_HANDLE netH)
{
#if defined(TCPIP_STACK_USE_IPV6)
    const uint8_t *pMacAddr = TCPIP_STACK_NetAddressMac(netH);
    uint8_t solicitedNodeMulticastMACAddr[] = {0x33, 0x33, 0xff, 0x00, 0x00, 0x00};
    uint8_t allNodesMulticastMACAddr[] = {0x33, 0x33, 0x00, 0x00, 0x00, 0x01};
	int i;

    for (i = 3; i < 6; i++)
        solicitedNodeMulticastMACAddr[i] = pMacAddr[i];

    s_app_set_param.multicast.addr = solicitedNodeMulticastMACAddr;
    iwpriv_set(MULTICASTFILTER_SET, &s_app_set_param);
    s_app_set_param.multicast.addr = allNodesMulticastMACAddr;
    iwpriv_set(MULTICASTFILTER_SET, &s_app_set_param);
#endif /* defined(TCPIP_STACK_USE_IPV6) */
}

static void APP_WIFI_PowerSave_Config(bool enable)
{
#if WF_DEFAULT_POWER_SAVE == WF_ENABLED
    s_app_set_param.powerSave.enabled = enable;
    iwpriv_set(POWERSAVE_SET, &s_app_set_param);
#endif
}

static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH)
{
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    bool updated;
    TCPIP_MAC_ADDR addr;

    s_app_get_param.clientInfo.addr = addr.v;
    iwpriv_get(CLIENTINFO_GET, &s_app_get_param);
    updated = s_app_get_param.clientInfo.updated;

    if (updated)
        TCPIP_DHCPS_LeaseEntryRemove(netH, (TCPIP_MAC_ADDR *)&addr);
#endif /* defined(TCPIP_STACK_USE_DHCP_SERVER) */
}

static void APP_TCPIP_IFModules_Disable(TCPIP_NET_HANDLE netH)
{
	const char *netName = TCPIP_STACK_NetNameGet(netH);

	if (IS_WF_INTF(netName) && TCPIP_STACK_NetIsUp(netH))
		APP_WIFI_PowerSave_Config(false);
	TCPIP_DHCP_Disable(netH);
	TCPIP_DNS_Disable(netH, true);
	TCPIP_MDNS_ServiceDeregister(netH);
}

static void APP_TCPIP_IFModules_Enable(TCPIP_NET_HANDLE netH)
{
	int netIndex = TCPIP_STACK_NetIndexGet(netH);
	const char *netName = TCPIP_STACK_NetNameGet(netH);

	TCPIP_DHCP_Enable(netH);
	TCPIP_DNS_Enable(netH, TCPIP_DNS_ENABLE_DEFAULT);
	if (IS_WF_INTF(netName))
		APP_WIFI_IPv6MulticastFilter_Set(netH);
	if (IS_NBNS_RUN()) {
		const char *netBiosName = TCPIP_STACK_NetBIOSName(netH);
		SYS_CONSOLE_PRINT("  Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
	}
	if (IS_MDNS_RUN()) {
		char mDNSServiceName[] = "MyWebServiceNameX "; // base name of the service Must not exceed 16 bytes long
		// the last digit will be incremented by interface
		mDNSServiceName[sizeof(mDNSServiceName) - 2] = '1' + netIndex;
		TCPIP_MDNS_ServiceRegister(netH, mDNSServiceName, "_http._tcp.local", 80, ((const uint8_t *)"path=/index.htm"),
			1, NULL, NULL);
	}
}

static void APP_TCPIP_IF_Down(TCPIP_NET_HANDLE netH)
{
    TCPIP_STACK_NetDown(netH);
}

static void APP_TCPIP_IF_Up(TCPIP_NET_HANDLE netH)
{
    SYS_MODULE_OBJ tcpipStackObj;
    TCPIP_STACK_INIT tcpip_init_data;
    const TCPIP_NETWORK_CONFIG *pIfConf;
    uint16_t net_ix = TCPIP_STACK_NetIndexGet(netH);

    tcpipStackObj = TCPIP_STACK_Initialize(0, 0);
    TCPIP_STACK_InitializeDataGet(tcpipStackObj, &tcpip_init_data);
    pIfConf = tcpip_init_data.pNetConf + net_ix;
    TCPIP_STACK_NetUp(netH, pIfConf);
}

/*******************************************************************************
 End of File
 */
