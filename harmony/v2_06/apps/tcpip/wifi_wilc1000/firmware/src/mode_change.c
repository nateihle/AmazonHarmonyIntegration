/*******************************************************************************
  File Name:
    mode_change.c

  Summary:


  Description:

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
#include <sys/attribs.h>
#include "app.h"
#include "app_wifi_wilc1000.h"
#include "m2m_types.h"
#include "m2m_wifi.h"

#define WIFI_INTERFACE_NAME "WILC1000"

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

#define IS_WF_INTF(x) ((strcmp(x, "MRF24WN") == 0) || (strcmp(x, "WINC1500") == 0) || (strcmp(x, "WILC1000") == 0))

#define APP_WIFI_RECONNECTION_RETRY_LIMIT 16

#define APP_WIFI_DHCP_WAIT_THRESHOLD 60 /* seconds */

#ifdef TCPIP_DHCP_CLIENT_ENABLED
#define timestamp_dhcp_kickin(x) do { x = 1; } while (0)
#else
#define timestamp_dhcp_kickin(x) do { x = 1; } while (0)
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
static APP_DATA s_appData;
static BSP_LED_STATE s_LEDstate = BSP_LED_STATE_OFF;
static IWPRIV_GET_PARAM s_app_get_param;
static IWPRIV_SET_PARAM s_app_set_param;

WF_CONFIG g_wifi_cfg;
WF_DEVICE_INFO g_wifi_deviceInfo;

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

static void APP_CONSOLE_HeaderDisplay(void);
static void APP_WIFI_PowerSave_Config(bool enable);
static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Disable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Enable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Down(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Up(TCPIP_NET_HANDLE netH);

void mode_change(void);

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
	s_appData.state = APP_MOUNT_DISK;

    s_app_set_param.conn.initConnAllowed = true;
    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
}

static bool disk_mount(void)
{
	if (SYS_FS_Mount(SYS_FS_NVM_VOL, LOCAL_WEBSITE_PATH_FS, MPFS2, 0, NULL) == 0) {
		SYS_CONSOLE_PRINT("SYS_Initialize: The %s File System is mounted\r\n", SYS_FS_MPFS_STRING);
		APP_CONSOLE_HeaderDisplay();
		s_appData.state = APP_TCPIP_WAIT_INIT;
		return true;
	}
	return false;
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

	iwpriv_get(CONNSTATUS_GET, &s_app_get_param);
	if (s_app_get_param.conn.status == IWPRIV_CONNECTION_SUCCESSFUL) {
		// reset reconnection retries
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
#define HOLD_TIME_IN_MODE               (60000)
#define DELAY_FOR_MODE_CHANGE           (500)


/**
 * \brief AP mode
 *
 * Initializes the AP mode for a while and terminate.
 */
static int8_t enable_disable_ap_mode(void)
{
	int8_t ret;
	tstrM2MAPConfig strM2MAPConfig;
    
#define MAIN_WLAN_SSID  "WILC1000_AP"
#define MAIN_WLAN_AP_CHANNEL    M2M_WIFI_CH_6
#define MAIN_WLAN_AUTH          M2M_WIFI_SEC_OPEN 
    
	SYS_CONSOLE_PRINT("AP mode, start\r\n");

	/* Configure AP. */
	memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
	strcpy((char *)&strM2MAPConfig.au8SSID, MAIN_WLAN_SSID);
	strM2MAPConfig.u8ListenChannel = MAIN_WLAN_AP_CHANNEL;
	strM2MAPConfig.u8SecType = MAIN_WLAN_AUTH;

	/* Start AP mode. */
	ret = m2m_wifi_enable_ap(&strM2MAPConfig);
	if (M2M_SUCCESS != ret) {
		return ret;
	}

	/* Keep in AP mode for a while. */
	nm_sleep(HOLD_TIME_IN_MODE);

	/* Stop AP mode. */
	ret = m2m_wifi_disable_ap();
	if (M2M_SUCCESS != ret) {
		return ret;
	}

	SYS_CONSOLE_PRINT("AP mode, end\r\n");

	return ret;
}


void enable_p2p_mode(void)
{
	int8_t ret;

#define WLAN_DEVICE_NAME    "WILC1000_P2P" /* WILC1000's Device Name in P2P Mode */
#define WLAN_CHANNEL        M2M_WIFI_CH_6 /* WILC1000's Listen Channel in P2P Mode */
    
	/* Set device name to be shown in peer device. */
	ret = m2m_wifi_set_device_name((uint8_t *)WLAN_DEVICE_NAME, strlen(WLAN_DEVICE_NAME));
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_set_device_name call error!(%d)\r\n", ret);
		return;
	}

	/* Bring up P2P mode with channel number. */
	ret = m2m_wifi_p2p(WLAN_CHANNEL);
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_p2p call error!(%d)\r\n", ret);
		return;
	}

	SYS_CONSOLE_PRINT("Device name: %s\r\nChannel: %d\r\nWaiting for P2P connection\r\n",
		WLAN_DEVICE_NAME, WLAN_CHANNEL);

}
/*******************************************************************************
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
    static int16_t mode_change_done = 0;
    
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
        
        if (mode_change_done == 0)
        {
            mode_change();
            mode_change_done = 1;
        }
		break;
	default:
		break;
	}
}

static void APP_CONSOLE_HeaderDisplay(void)
{
    SYS_CONSOLE_MESSAGE("\r\n=======================================");
    SYS_CONSOLE_MESSAGE("\r\n*** WILC1000 FreeRTOS Mode Change Demo ***");
    SYS_CONSOLE_MESSAGE("\r\n=======================================\r\n");
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
	
/**
 * \brief mode changes entry function.
 */
void mode_change(void)
{
    int ret = 0;
	/**
	 * AP mode.
	 * Turn On and off AP mode.
	 */
	ret = enable_disable_ap_mode();
	if (M2M_SUCCESS != ret) {
		SYS_CONSOLE_PRINT("enable_disable_ap_mode call error!\r\n");
		while (1) {
		}
	}

	nm_sleep(DELAY_FOR_MODE_CHANGE);

	enable_p2p_mode();
}

/*******************************************************************************
 End of File
*/
