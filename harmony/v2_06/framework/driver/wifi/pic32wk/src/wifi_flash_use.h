/*******************************************************************************
  File Name:
    WiFi_commands.h

  Summary:
  commands for the communicating with WLAN MAC library.
  
  Description:
  
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _WIFI_FLASH_USE_H
#define _WIFI_FLASH_USE_H

#include "system_config.h"
#include "system_definitions.h"
#include "wifi_init.h"
#include "WiFiMW/wifi_bsp_led.h"

#define IPF_WIFI_CALIBRATION_START 			0x001FE000
#define IPF_WIFI_CALIBRATION_LENTH 			256
#define IPF_SECTOR_SIZE 					4096
#define IPF_WIFI_REGDOMAINCONFIG_OFFSET		0x1000
#define IPF_WIFI_REGDOMAINCONFIG_START 		(IPF_WIFI_CALIBRATION_START+IPF_WIFI_REGDOMAINCONFIG_OFFSET)
#define IPF_WIFI_OTA_ADDRESS_OFFSET			0x400

#define IPF_WIFI_MACADDRESS_OFFSET 			0x200
#define IPF_WIFI_MACADDRESS_START 			(IPF_WIFI_CALIBRATION_START+IPF_WIFI_MACADDRESS_OFFSET)
#define DEFAULT_MACADDRESS 					{0x00, 0x49, 0x54, 0x54, 0x0B, 0x4E}
#define MACADDRESS_LENGTH   				6

#define IPF_WIFI_CONFIG_START               0x001F7000
#define DEFAULT_SSID                        "microchip"
#define DEFAULT_COUNTRYCODE                 "USA"

#define IPF_SECTOR_ADDRESS_1         		0x001FE000
#define IPF_SECTOR_ADDRESS_2         		0x001FF000
#define MAX_LED_ON_BOARD    				3
#define INVALID 							(-1)
//#define DUMP_SECUREREGION_DATA
// *****************************************************************************
/*  IPF Event handler states 

  Summary:
    Selections for IPF Event handler states.

  Description
    This states is used for IPF event handling.
*/
typedef enum
{
	/* Application's state machine's initial state. */
    IPF_STATE_INIT=0,
    IPF_STATE_READ,
    IPF_STATE_READ_REGDOMAINCONFIG,
    IPF_STATE_READ_CAL,
    IPF_STATE_READ_MACADDR,
    IPF_STATE_READ_SECUREREGION,
    IPF_STATE_WRITE_SECUREREGION,
    IPF_STATE_WRITE,
    IPF_STATE_WRITE_REGDOMAINCONFIG,
    IPF_STATE_ERASE,
    IPF_STATE_ERASE_REGDOMAINCONFIG,
    IPF_STATE_ERASE_SECUREREGION,
    IPF_STATE_PROTECTCAL,
    IPF_STATE_UNPROTECTCAL,
    IPF_STATE_ERROR,
    IPF_STATE_IDLE,
} IPF_STATES;

// *****************************************************************************
/*  IPF secure region types 

  Summary:
    IPF has 8KB block secure region.
	Secure region block has two sectors: sector 1(4KB) and sector 2(4KB)
	RF Calibration and MAC address are stored into : sector 1
	RF regulatory domain info and OTA address : sector 2

  Description
    This enumeration define IPF Sectors.
*/
typedef enum {
    IPF_SECTOR_INVALID=0,
    IPF_SECTOR_1,
    IPF_SECTOR_2,        
} IPF_SECTOR;

// *****************************************************************************
/*
  Summary:
    Contains IPF information.

  Description:
    This structure contains IPF info. This structure Contains state,preState,
	IPF driver handler and request complete flag.
*/

typedef struct
{
    IPF_STATES state;     /* The IPF's current state from WiFi*/
    IPF_STATES prevState;
    DRV_HANDLE IPF_Handle;
    uint8_t reqComplete;
    /* TODO: Define any additional data used by the application. */
} IPF_INFO;

IPF_INFO ipfInfo;
#ifdef DRV_WIFI_OTA_ENABLE
extern DRV_HANDLE otaHandler;
#endif
// *****************************************************************************
/*  RF regulatory domain country code.

  Summary:
	Different country code for regulatory domain.

  Description
    This enumeration define RF regulatory domain TX power country code.
*/


typedef enum
{
	CC_ALL  =  1,
	CC_TWN  =  158,
	CC_SGP  =  702,
	CC_USA  =  840, 	/* NON-FCC*/
	CC_JPN  =  392,
	CC_RUS  =  643,
	CC_KOR  =  410,
	CC_ISR  =  376, 	/* EMEA*/
	CC_CHN  =  156,
	MAX_REGIONS = 10,
}TxPowerCountryCode;
// *****************************************************************************
/*  WLAN operation types.

  Summary:
	WLAN operation id.

  Description
    This enumeration define wlan operation identification.
*/

typedef enum
{
	WLAN_OPER_GET_TX_PWR=4,
	WLAN_OPER_RX_PWR_ADJ,
    WLAN_OPER_COUNTRYCODE,
    WLAN_OPER_2GENB2040,
    WLAN_OPER_CHNBITMAP2G,
    WLAN_OPER_CHNBITMAP5G,
    WLAN_OPER_MAC_CONN_STATUS,
    WLAN_OPER_MAC_DATA_PROGRESS,
}WLAN_OPER_ID;
// *****************************************************************************
/*  Device Boot mode.

  Summary:
	device can boot as STA(station)or AP(access point) mode.	

  Description
    This enumeration define wlan boot mode.
*/
typedef enum
{
	/* MAC Boot Mode */
    BOOT_STA_MODE =0,    
    BOOT_AP_MODE,
    BOOT_INVALID_MODE,
} MAC_BOOT_MODES;

#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED
// *****************************************************************************
/*  LED link.

  Summary:
	define LED link speed.	

  Description
    This enumeration define LED link speed.
*/
typedef enum
{
    FAST_BLINK = 9999,
    MEDIUM_BLINK = 29997,
    SLOW_BLINK = 59994
} BLINK_LED;
// *****************************************************************************
/*  BSP LED number.

  Summary:
	define BSP LED number.	

  Description
    This enumeration define BSP LED number.
*/
typedef enum
{
    BSP_LED2 = 0,
    BSP_LED1 = 1,
    BSP_LED3 = 2
} BSP_LED_NUMBER;
// *****************************************************************************
/*  BSP LED behaviour.

  Summary:
	define BSP LED behaviour.	

  Description
    This enumeration define BSP LED behaviour.
*/
typedef enum
{
    LED_ON = 1,
    LED_OFF = 2,        
    LED_BLINK = 3
} BSP_LED_STATUS;

//****************************************************************************
/*  
  Function:
    void LED_Task(void);

  Summary:
	LED on/off indication in hardware.
	
  Description:
	This function handles LED status according to the global value set for
	LED's of the Starter Kit

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void LED_Task(void);
//****************************************************************************
/*  
  Function:
    void wlan_set_LED_values(uint8_t led_number, uint8_t led_status, int led_blink_speed );

  Summary:
	This function set the global variables for LED's of starter kit
	
  Description:
	This function set the global variables for LED's of starter kit

  Precondition:
    led_number - LED number
	led_status - LED status
	led_blink_speed - LED speed

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void wlan_set_LED_values(uint8_t led_number, uint8_t led_status, int led_blink_speed );
//****************************************************************************
/*  
  Function:
    void wlan_set_LED();

  Summary:
	This function triggers setting of global variables for LED's based on present
	status of the Device(DUT)
	
  Description:
	This function triggers setting of global variables for LED's based on present
	status of the Device(DUT)

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void wlan_set_LED();

#endif

//****************************************************************************
/*  
  Function:
    bool WIFI_IPF_open();

  Summary:
	This function Open IPF driver
	
  Description:
	This function Open IPF driver and enables read/write on IPF
	After successful opening of the driver, set the state of ipfInfo to
	the state IPF_STATE_INIT
	
  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_open();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_write();

  Summary:
	This function enable IPF write functionality.
	
  Description:
	This function is to write the required content into IPF

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_write();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_read();

  Summary:
	This function enable IPF read functionality.
	
  Description:
	This function is to read the required content from IPF.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_read();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_erase();

  Summary:
	This function enable IPF erase functionality.
	
  Description:
	This function is a to erase the flash, before doing write operation

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_erase(void);
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_usrconfig_memalloc();

  Summary:
	This function enable IPF memory allocation for  user configuration area.
	
  Description:
	This function enable memory allocation for  user configuration area.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_usrconfig_memalloc();
//****************************************************************************
/*  
  Function:
    bool WIFI_IPF_usrconfig_memfree();

  Summary:
	This function enable IPF memory free for  user configuration area.
	
  Description:
	This function enable IPF memory free for  user configuration area.

  Precondition:
    memory should be allocated by WIFI_IPF_usrconfig_memalloc() function only. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_usrconfig_memfree();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_readAndBackup();

  Summary:
	This function is to read and backup the g_wifiConfigData data before
                          updating new user data into the Flash.
	
  Description:
	This function is to read and backup the g_wifiConfigData data before
                          updating new user data into the Flash.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_readAndBackup();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_save_regdom();

  Summary:
	This function is to save regulatory domain information into IPF.
	
  Description:
	This function is to save regulatory domain information into IPF.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_save_regdom();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_save();

  Summary:
	This function is to save user configuration information into IPF.
	
  Description:
	This function is to save the content which was received from user
    into the IPF

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_save();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_read_cal();

  Summary:
	This function is to enable reading RF calibration data from flash.
	
  Description:
	This function is to enable reading RF calibration data from flash.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_read_cal();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_cal_memalloc();

  Summary:
	This function is to enable memory allocation of RF calibration data.
	
  Description:
	This function is to enable memory allocation of RF calibration data.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_cal_memalloc();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_cal_memfree();

  Summary:
	This function is to free memory allocation of RF calibration data.
	
  Description:
	This function is to free memory allocation of RF calibration data.

  Precondition:
     memory should be only be allocated by  WIFI_IPF_cal_memalloc() function. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_cal_memfree();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_regdomconfig_memalloc();

  Summary:
	This function is to allocate memory allocation of RF regulatory domain .
	
  Description:
	This function is to allocate memory allocation of RF regulatory domain.

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_regdomconfig_memalloc();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_regdomconfig_memfree();

  Summary:
	This function is to free memory allocation of RF regulatory domain .
	
  Description:
	This function is to free memory allocation of RF regulatory domain.

  Precondition:
    memory should be only be allocated by  WIFI_IPF_regdomconfig_memalloc() function.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
bool WIFI_IPF_regdomconfig_memfree();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_protect();

  Summary:
	This function enable protection of 8KB IPF Secure region.
	
  Description:
	This function enable protection of 8KB IPF Secure region.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_protect();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_unprotect();

  Summary:
	This function disable protection of 8KB IPF Secure region.
	
  Description:
	This function disable protection of 8KB IPF Secure region.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_unprotect();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_open(uint8_t sector_type);

  Summary:
	This function read the secure region sector from IPF and 
	map the memory with modification field(MAC address,
	RF regulatory domain data and OTA address).
	
	
  Description:
	This function read the secure region sector from IPF and 
	map the memory with modification field(MAC address,
	RF regulatory domain data and OTA address).

  Precondition:
    None.

  Parameters:
    sector_type - sector type .	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_open(IPF_SECTOR sector_type);
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_read();

  Summary:
	This function read the secure region sector from IPF.
	
  Description:
	This function read the secure region sector from IPF.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_read();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_write();

  Summary:
	This function write the secure region sector to IPF.
	
  Description:
	This function write the secure region sector to IPF.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_write();
//****************************************************************************
/*  
  Function:
    uint8_t WIFI_IPF_secureregion_read_status();

  Summary:
	This function provide status of read secure region completion.
	
  Description:
	This function provide status of read secure region completion.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    0 -  read from secure region sector is completed.
    1 -  read from secure region sector is not completed.
	
  Remarks:
    None.
*/
uint8_t WIFI_IPF_secureregion_read_status();
//****************************************************************************
/*  
  Function:
    void* WIFI_IPF_secureregion_macoffset();

  Summary:
	This function provide pointer to MAC address for modifying MAC address.
	
  Description:
	This function provide pointer to MAC address for modifying MAC address.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    pointer to modifiable MAC address offset. 
    
  Remarks:
    None.
*/
void* WIFI_IPF_secureregion_macoffset();
//****************************************************************************
/*  
  Function:
    void* WIFI_IPF_secureregion_otaaddroffset();

  Summary:
	This function provide pointer to OTA address for modifying OTA address.
	
  Description:
	This function provide pointer to OTA address for modifying OTA address.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    pointer to modifiable OTA address offset.
    
  Remarks:
    None.
*/
void* WIFI_IPF_secureregion_otaaddroffset();
//****************************************************************************
/*  
  Function:
    void* WIFI_IPF_secureregion_regdomoffset();

  Summary:
	This function provide pointer to Regulatory domain  data address for modifying
	Regulatory domain  data address.
	
  Description:
	This function provide pointer to Regulatory domain  data address for modifying
	Regulatory domain  data address.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    pointer to modifiable Regulatory domain  data address offset.
    
  Remarks:
    None.
*/
void* WIFI_IPF_secureregion_regdomoffset();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_sector_get();

  Summary:
	This function provide secure region sector set information.
	
  Description:
	This function provide secure region sector set information.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
	0 - secure region sector invalid
    1 - secure region sector 1
	2 - secure region sector 2
    
  Remarks:
    None.
*/
IPF_SECTOR WIFI_IPF_secureregion_sector_get();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_sector_set(IPF_SECTOR sector_type);

  Summary:
	This function provide selection option of secure region sector.
	
  Description:
	This function provide selecting option of secure region sector.

  Precondition:
    None.

  Parameters:
    sector_type - sector type.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_sector_set(IPF_SECTOR sector_type);
//****************************************************************************
/*  
  Function:
    int32_t WIFI_IPF_secureregion_address();

  Summary:
	This function provide active secure region sector address.
	
  Description:
	This function provide active secure region sector address.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
int32_t WIFI_IPF_secureregion_address();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_modify();

  Summary:
	This function provide functionality for modifying(erase,write and protect) IPF functionality.
	
  Description:
	This function provide functionality for modifying(erase,write and protect) IPF functionality.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_modify();
//****************************************************************************
/*  
  Function:
    void WIFI_IPF_secureregion_close();

  Summary:
	This function provide close secure region sector.
	
  Description:
	This function provide close secure region sector.

  Precondition:
    None.

  Parameters:
    None.	
		
  Returns:
    None.
    
  Remarks:
    None.
*/
void WIFI_IPF_secureregion_close();

#endif
/*******************************************************************************
 End of File
 */

