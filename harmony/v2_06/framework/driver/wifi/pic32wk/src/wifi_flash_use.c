/*******************************************************************************
  File Name: 
	wifi_flash_use.c
  
  Summary:
    To Access IPF for Setting and Reading WiFi Configuration

  Description:
    - Saves the WiFi configuration data
    - Reads the WiFi configuration data
    - Set the saved WiFi configuration data in WLAN driver
    
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#include "wifi_flash_use.h"
#include "WiFi_MW.h"
#include "drv_mchpwlan.h"
#include "WiFi_commands.h"
#include "tcpip/src/tcpip_manager_control.h" 
#include "driver/wifi/pic32wk/src/wifi_web_config.h"

/*******************************************************************************
    Global Variables
    Info: Flash uses only Global variables for writing/reading data
*******************************************************************************/
char IPF_ready_use_state = 0;
unsigned int g_bootMode; 
uint8_t macaddr_mem[12]; 
uint8_t macaddr_val[6];
uint8_t *l_macaddr=NULL;
uint32_t *ota_address=NULL;
int web_data = 0;
IPF_SECTOR secure_region = IPF_SECTOR_INVALID;

#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED
//LED information
typedef struct
{
    uint8_t led_status;   /* ON, OFF or BLINK */
    uint8_t led_number;
    int led_blink_speed;
} LED_INFO;

LED_INFO onBoardLED[MAX_LED_ON_BOARD];
#endif

unsigned char g_wifi_mac_status; //default: MAC_DISCONNECTED
char g_wifi_data_progress_path;

//Read Write testing buffers
uint8_t buf[100];
uint8_t ipf_read_buf[100] = {0, };
uint8_t ipf_sector[IPF_SECTOR_SIZE]={0};

//Calibration Memory
unsigned int *Calib_mem = NULL; 
wifiRegdomainConfig *g_wifiRegdomainConfig=NULL;
uint8_t Secure_read_completed = 1;

#ifdef DRV_WIFI_OTA_ENABLE
    DRV_HANDLE otaHandler;
#endif

//Wifi Config Data to Save in Flash
extern wifiConfigData    *g_wifiConfigData;

extern t_wfEasyConfigCtx    g_easyConfigCtx;
static wifiConfigData temp_wifiConfigData;
static wifiRegdomainConfig  temp_wifiRegdomainConfig;

extern unsigned char mac_debug_module_level[5];

void DRV_IPF_UnProtectMemoryVolatile (DRV_HANDLE clientHandle,DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
                                        uintptr_t memAddress,DRV_IPF_PROT_MODE protMode);
void DRV_IPF_ProtectMemoryVolatile (DRV_HANDLE clientHandle,DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
                                        uintptr_t memAddress,DRV_IPF_PROT_MODE protMode);
extern uint8_t             g_macaddress[6];
typedef int (*FUNC_PTR)(WLAN_OPER_ID operId, unsigned char *dataPtr);
extern int hook_wlan_ctrl_handle(FUNC_PTR wlan_ctl_handle);
/*******************************************************************************/
/*****************************************************************************                                                                        
  Function Name : checkreadssid                                  
                                                                           
  Description   : This function check the SSID received from IPF. 
                                        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return 1       - if SSID in IPF is not set(default erase value of IPF).
                 return 0   	- if SSID in IPF is set(User set SSID)                                                  
*****************************************************************************/
bool checkreadssid ()
{
    unsigned char idx =0;
    for (idx =0;((idx<32) && (g_wifiConfigData->ssid[idx] == 0xff));idx++);
    
    if(idx == 32){
        return 1;
     }
     return 0;
}

/****************************************************************************************************                                                                        
  Function Name : checkregdomainconfig                                  
                                                                           
  Description   : This function check the regulatory domain configuration value from IPF. 
                                        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return 1       - if Reg domain values in IPF is not set(default erase value of IPF).
                 return 0   	- if Reg domain values in IPF is set(either set by BIST mode or by user)                                                  
*******************************************************************************************************/
bool checkregdomainconfig()
{
    
    if((g_wifiRegdomainConfig->ChannelBitmap2Ghz == 0xff) && (g_wifiRegdomainConfig->ChannelBitmap5Ghz== 0xff))
    {
        return 1;
    }
    return 0;
}

/*****************************************************************************                                                                        
  Function Name : updatedefault_wifiuserconfig                                  
                                                                           
  Description   : This function set default setting of wlan user configuration .
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void updatedefault_wifiuserconfig()
{
   //updating Default ssid
    strcpy((char *)g_wifiConfigData->ssid,DEFAULT_SSID);
}

/*****************************************************************************                                                                        
  Function Name : updatedefault_regdomainconfig                                  
                                                                           
  Description   : This function set default setting of regulatory domain configuration .
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void updatedefault_regdomainconfig()
{
    unsigned char tx2G_20_US[14]      =   {133,141,141,141,141,141,141,141,141,141,131,1,1,1};
    unsigned char tx2G_BB_US[14]      =   {111,111,111,174,174,174,174,174,174,174,173,1,1,1};
    unsigned char tx2G_40_US[11]      =   {1,1,124,131,131,131,131,131,90,1,1};
    unsigned char tx5G_20_US[28]      =   {125,157,157,158,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,132,124,124,124,127,1,1,1,1};
    unsigned char tx5G_40_US[11]      =   {118,162,1,1,1,1,1,1,1,136,137};
    unsigned char tx2G_20_EU[14]      =   {142,141,141,141,141,141,141,141,141,141,141,141,149,1};
    unsigned char tx2G_BB_EU[14]      =   {140,154,154,154,154,154,154,154,154,154,154,154,154,1};
    unsigned char tx2G_40_EU[11]      =   {1,1,125,134,134,134,134,134,134,134,129};
    unsigned char tx5G_20_EU[28]      =   {123,123,123,123,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    unsigned char tx5G_40_EU[11]      =   {124,124,1,1,1,1,1,1,1,1,1};
    unsigned char tx2G_20_JP[14]      =   {142,141,141,141,141,141,141,141,141,141,141,141,149,1};
    unsigned char tx2G_BB_JP[14]      =   {140,154,154,154,154,154,154,154,154,154,154,154,154,1};
    unsigned char tx2G_40_JP[14]      =   {1,1,125,134,134,134,134,134,134,134,129};
    unsigned char tx5G_20_JP[28]      =   {123,123,123,123,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    unsigned char tx5G_40_JP[11]      =   {124,124,1,1,1,1,1,1,1,1,1};
    unsigned char tx2G_20_GN[14]      =   {170,170,170,170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx2G_BB_GN[14]      =   {200,200,200,200,200,200,200,200,200,200,200,200,200,200};
    unsigned char tx2G_40_GN[11]      =   {170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx5G_20_GN[28]      =   {170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx5G_40_GN[11]      =   {170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx2G_20_CUST[14]    =   {142,141,141,141,141,141,141,141,141,141,131,1,1,1};
    unsigned char tx2G_BB_CUST[14]    =   {185,188,188,188,188,188,188,188,188,188,179,1,1,1};
    unsigned char tx2G_40_CUST[11]    =   {1,1,124,131,131,131,131,131,90,1,1};
    unsigned char tx5G_20_CUST[28]    =   {123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,100,123,123,123,123,1,1,1,1};
    unsigned char tx5G_40_CUST[11]    =   {124,124,124,124,124,124,124,124,124,107,124};
    
    unsigned char   rx2G_20[14]       =   {98,98,98,98,98,98,98,98,98,98,98,98,98,98};
    unsigned char   rx2G_40[11]       =   {98,98,98,98,98,98,98,98,98,98,98};
    unsigned char   rx5G_20[28]       =   {100,100,100,100,100,100,100,100,100,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,94,94,94,94};
    unsigned char   rx5G_40[11]       =   {100,100,100,100,100,96,96,96,96,96,96};
 
    
    g_wifiRegdomainConfig->ChannelBitmap5Ghz = g_wifiRegdomainConfig->ChannelBitmap2Ghz = 0;
    
    strcpy((char *)g_wifiRegdomainConfig->CountryCode,DEFAULT_COUNTRYCODE);
    g_wifiRegdomainConfig->FreqBand               = 3;
    g_wifiRegdomainConfig->Enable2g2040           = 0;
    
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_US,(const char *)tx2G_20_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_US,(const char *)tx2G_BB_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_US,(const char *)tx2G_40_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_US,(const char *)tx5G_20_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_US,(const char *)tx5G_40_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_EU,(const char *)tx2G_20_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_EU,(const char *)tx2G_BB_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_EU,(const char *)tx2G_40_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_EU,(const char *)tx5G_20_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_EU,(const char *)tx5G_40_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_JP,(const char *)tx2G_20_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_JP,(const char *)tx2G_BB_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_JP,(const char *)tx2G_40_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_JP,(const char *)tx5G_20_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_JP,(const char *)tx5G_40_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_GN,(const char *)tx2G_20_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_GN,(const char *)tx2G_BB_GN);    
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_GN,(const char *)tx2G_40_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_GN,(const char *)tx5G_20_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_GN,(const char *)tx5G_40_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_CUST,(const char *)tx2G_20_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_CUST,(const char *)tx2G_BB_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_CUST,(const char *)tx2G_40_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_CUST,(const char *)tx5G_20_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_CUST,(const char *)tx5G_40_CUST);
    g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G      = 20;
    g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G      = 35 ;
 
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_2G_20,(const char *)rx2G_20);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_2G_40,(const char *)rx2G_40);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_5G_20,(const char *)rx5G_20);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_5G_40,(const char *)rx5G_40);
}

/*****************************************************************************                                                                        
  Function Name : convertCountryCode                                  
                                                                           
  Description   : This function provide Country power information.           
                                                                           
  Inputs        : code: Country code                
                                                                           
  return      	: TxPowerCountryCode based on received country code
*****************************************************************************/
TxPowerCountryCode convertCountryCode(const char *code)
{
	if (strncmp(code, "EMEA", 4)==0) 
	{
		return CC_ISR;
	} 
	else if (strncmp(code, "NFCC", 4)==0) 
	{
		return CC_USA;
	}
	else if (strncmp(code, "RUS", 3)==0) 
	{
		return CC_RUS;
	} 
	else if (strncmp(code, "USA", 3)==0) 
	{
		return CC_USA;
	}
	else if (strncmp(code, "TWN", 3)==0)
	{
		return CC_TWN;
	}
	else if (strncmp(code, "ISR", 3)==0)
	{
		return CC_ISR;
	}
	else if (strncmp(code, "CHN", 3)==0)
	{
		return CC_CHN;
	}
	else if (strncmp(code, "JPN", 3)==0)
	{
		return CC_JPN;
	}
	else if (strncmp(code, "SGP", 3)==0)
	{
		return CC_SGP;
	}
	else if (strncmp(code, "KOR", 3)==0)
	{
		return CC_KOR;
	}	
	else if(strncmp(code, "JP", 2) == 0)
	{
		return CC_JPN;
	}
	else if(strncmp(code, "EU", 2) == 0)
	{
		return CC_ISR;
	}
	else if(strncmp(code, "US", 2) == 0)
	{
		return CC_USA;
	}	
	else
	{
		return CC_USA;
	}
}

/*****************************************************************************                                                                        
  Function Name : getConfigTxPowerLevel                                  
                                                                           
  Description   : This function is to get Tx power level
                                                                           
  Inputs        : pwr_val: power value               
                                                                           
  return        : integer  0: Success, -1 : failure                                                     
*****************************************************************************/
int getConfigTxPowerLevel(unsigned char* pwr_val)
{
    TxPowerCountryCode CountryCode = CC_USA;
    unsigned char *str_20_2G;
    unsigned char *str_BB_2G;
    unsigned char *str_40_2G;
    unsigned char *str_20_5G;
    unsigned char *str_40_5G;
    unsigned char Inx =0;
    
    CountryCode=convertCountryCode((const char *)g_wifiRegdomainConfig->CountryCode);
    if(g_wifiRegdomainConfig->ChannelBitmap2Ghz != 0 || g_wifiRegdomainConfig->ChannelBitmap5Ghz != 0)
	{
        str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_CUST;
        str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_CUST;
        str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_CUST;
        str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_CUST;
        str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_CUST;
  	} else {
        
        if(CC_USA == CountryCode)
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_US;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_US;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_US;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_US;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_US;
           
		}
		else if(CC_ISR == CountryCode) //Channel mapping is same as EMEA
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_EU;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_EU;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_EU;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_EU;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_EU;
		}
		else if(CC_JPN == CountryCode)
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_JP;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_JP;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_JP;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_JP;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_JP;
		}
		else
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_GN;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_GN;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_GN;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_GN;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_GN;
		}
	}
    for(Inx=0;Inx<14;Inx++){
        pwr_val[Inx]=str_20_2G[Inx];
    }
   
    for(Inx=0;Inx<14;Inx++){
        pwr_val[14+Inx]=str_BB_2G[Inx];
    }
    for(Inx=0;Inx<11;Inx++){
        pwr_val[28+Inx]=str_40_2G[Inx];
    }    
    for(Inx=0;Inx<28;Inx++){
        pwr_val[39+Inx]=str_20_5G[Inx];
    }    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[67+Inx]=str_40_5G[Inx];
    }    

    pwr_val[78] = (CC_USA == CountryCode)?g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G:20;
    pwr_val[79] = (CC_USA == CountryCode)?g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G:35;
 
    return 0;
}

/*****************************************************************************                                                                        
  Function Name : getConfigRxPowerLevel                                  
                                                                           
  Description   : This function is to get Rx power level
                                                                           
  Inputs        : opwr_val: power value                               
                                                                           
  return      : integer  0: Success, -1 : failure                                                     
*****************************************************************************/
int getConfigRxPowerLevel(unsigned char* pwr_val)
{
    unsigned char Inx =0;
    
    for(Inx=0;Inx<14;Inx++){
        pwr_val[Inx]=g_wifiRegdomainConfig->RxpowAdj_2G_20[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[14+Inx]=g_wifiRegdomainConfig->RxpowAdj_2G_40[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[25+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_20[Inx];
    }
    
    for(Inx=0;Inx<28;Inx++){
        pwr_val[25+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_20[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[53+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_40[Inx];
    }
    return 0;
}

/*****************************************************************************                                                                        
  Function Name : WIFI_oper_Handle                                  
                                                                           
  Description   : This function provide access to wlan configuration.           
                                                                           
  Inputs        : operationId: operation id
                  argStruct : where request id information need to be updated                
                                                                           
  return      	: integer  0: Success, -1 : failure                                                     
*****************************************************************************/
int WIFI_oper_Handle(WLAN_OPER_ID operationId, unsigned char *argStruct)
{
    
    switch (operationId)
	{
           case WLAN_OPER_GET_TX_PWR:
	    {
			if(getConfigTxPowerLevel(argStruct) != 0)
			{
				return -1;
			}
			break;
	    }
	    case WLAN_OPER_RX_PWR_ADJ:
	    {
			if(getConfigRxPowerLevel(argStruct) != 0)
			{
				return -1;
			}
			break;
	    }
    	case WLAN_OPER_COUNTRYCODE:
		{
            unsigned short *country_code= (unsigned short *)argStruct;
			*country_code = convertCountryCode((const char *)g_wifiRegdomainConfig->CountryCode);            
			break;
		}
        case WLAN_OPER_2GENB2040:
		{
            *argStruct = g_wifiRegdomainConfig->Enable2g2040;            
			break;
		}
       case WLAN_OPER_CHNBITMAP2G:
		{
            unsigned int *chnbitmap2g= (unsigned int *)argStruct;
			*chnbitmap2g = g_wifiRegdomainConfig->ChannelBitmap2Ghz;            
			break;
		}
        case WLAN_OPER_CHNBITMAP5G:
		{
            unsigned int *chnbitmap5g= (unsigned int *)argStruct;
           *chnbitmap5g = g_wifiRegdomainConfig->ChannelBitmap5Ghz;            
			break;
		}
        case WLAN_OPER_MAC_CONN_STATUS:
            {
                g_wifi_mac_status = *argStruct;
                //printf("\n MAC STATUS:%d", g_wifi_mac_status);
                break;
            }

        case WLAN_OPER_MAC_DATA_PROGRESS:
            {
                g_wifi_data_progress_path = *argStruct;
                //printf("\n DATA PATH:%d", g_wifi_data_progress_path);
                break;
            }
        
        default:
			break;

    }
    return 0;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_secureregion_protect                                  
                                                                           
  Description   : This function enable protection of 8KB IPF secure region.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_secureregion_protect()
{
    /*Enabling Write protection for Wi-Fi calibration region*/
        DRV_IPF_ProtectMemoryVolatile(ipfInfo.IPF_Handle,NULL,IPF_WIFI_CALIBRATION_START,DRV_IPF_WRITE_PROTECT);
        ipfInfo.prevState = ipfInfo.state ;
        ipfInfo.state = IPF_STATE_PROTECTCAL;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_secureregion_unprotect                                  
                                                                           
  Description   : This function disable protection of 8KB IPF secure region.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_secureregion_unprotect()
{
        /*Disabling Write protection for Wi-Fi calibration region*/
        DRV_IPF_UnProtectMemoryVolatile(ipfInfo.IPF_Handle,NULL,IPF_WIFI_CALIBRATION_START,DRV_IPF_WRITE_PROTECT);
        ipfInfo.prevState = ipfInfo.state;
        ipfInfo.state = IPF_STATE_UNPROTECTCAL;
}

/*****************************************************************************                                                                        
  Function Name : WIFI_initialization                                  
                                                                           
  Description   : This function do WIFI initialization           
                                                                           
  Inputs        : None                
                                                                           
  return      :     
    true  - Initialization was successful
    false - Initialization was not successful                                                    
*****************************************************************************/
bool WIFI_initialization()
{
    bool flash_open_status = 0;
    WiFi_Commands_Init();
    hook_wlan_ctrl_handle(WIFI_oper_Handle);
    flash_open_status =  WIFI_IPF_open();
    if(flash_open_status)
    {
        WIFI_IPF_secureregion_protect();
    }
    return flash_open_status;
}

/*****************************************************************************                                                                        
  Function Name : WIFI_IPF_setConfig                                  
                                                                           
  Description   : This function sets the configuration after Reading from Flash
                            using DRV_WIFI_Write call with WID and value.            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_setConfig()
{
    bool status = 1;
#if 0
    //Set SSID
    status = DRV_WIFI_Write(DRV_WIFI_WID_SSID, g_wifiConfigData->ssid);

    //Set PSK   
    status = DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK, "12345678");
    status = DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE, 73);    
    status = DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE, 0);
    status = DRV_WIFI_Write(DRV_WIFI_WID_11G_OPERATING_MODE, 2);
    status = DRV_WIFI_Write(DRV_WIFI_WID_ACK_POLICY, 0);
    status = DRV_WIFI_Write(DRV_WIFI_WID_11N_ENABLE, 1);
#endif    
#if 1
        if(g_wifiConfigData->ssid != NULL)
        {
            printf("\nSet SSID:%s to MAC, keyLen:%d\n",g_wifiConfigData->ssid,g_wifiConfigData->SecurityKeyLength);
            status = DRV_WIFI_Write(DRV_WIFI_WID_SSID,(unsigned int)g_wifiConfigData->ssid,strlen((const char*)g_wifiConfigData->ssid));
        }
        if(g_wifiConfigData->SecurityKeyLength != 0 && g_wifiConfigData->SecurityKeyLength != 0xff)
        {
            printf("\nSet Mode:%d and Key:%s to MAC", g_wifiConfigData->cipher,g_wifiConfigData->key);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,(unsigned int)g_wifiConfigData->key,strlen((const char*)g_wifiConfigData->key));
            status = DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,(unsigned int)g_wifiConfigData->cipher,1);    
            status = DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE, 0,1);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11G_OPERATING_MODE, 2,1);
            status = DRV_WIFI_Write(DRV_WIFI_WID_ACK_POLICY, 0,1);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11N_ENABLE, 1,1);            
        }
#if defined(DRV_WIFI_DEBUG_ENABLE)
            printf("\ndebug level to MAC:%d",DRV_DEBUG_PRINT_LEVEL);
            mac_debug_module_level[0] = DRV_DEBUG_PRINT_LEVEL;
            mac_debug_module_level[1] = 0xFF;
            mac_debug_module_level[2] = 0xFF;
            mac_debug_module_level[3] = 0xFF;
            mac_debug_module_level[4] = 0xFF;
                    
            printf(", Module value to MAC: %d, %x:%x:%x:%x\n",mac_debug_module_level[0], mac_debug_module_level[1], mac_debug_module_level[2],
                                         mac_debug_module_level[3], mac_debug_module_level[4] );            
            status = DRV_WIFI_Write(DRV_WIFI_DEBUG_MODULE_LEVEL, (unsigned int)mac_debug_module_level, 5);           
#endif        
        if(status == 0)
        {
            SYS_CONSOLE_MESSAGE("SUCCESSFUL \r\n");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("ERROR \r\n");
        } 
        status = DRV_WIFI_Write(DRV_WIFI_WID_START_SCAN_REQ,1,1);
#endif        
}

/*****************************************************************************                                                                        
  Function Name : validate_update_macaddress                                  
                                                                           
  Description   : This function check whether valid MAC address is read from IPF.
                In case invalid MAC address update the default MAC address. 
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void validate_update_macaddress()
{ 
    //MAC address 00:00:00:00:00:00 and ff:ff:ff:ff:ff:ff considered as invalid.
    if(
        ((g_macaddress[0]== 0x00)&&(g_macaddress[1] == 0x00)&&(g_macaddress[2] == 0x00)&&(g_macaddress[3] == 0x00)&&(g_macaddress[4] == 0x00)&&(g_macaddress[5] == 0x00)) ||
        ((g_macaddress[0]== 0xff)&&(g_macaddress[1] == 0xff)&&(g_macaddress[2] == 0xff)&&(g_macaddress[3] == 0xff)&&(g_macaddress[4] == 0xff)&&(g_macaddress[5] == 0xff))
      )
    {
        uint8_t mac_address[6]=DEFAULT_MACADDRESS;
        memcpy(g_macaddress,mac_address,MACADDRESS_LENGTH);
    }
    printf("MAC Address: %x:%x:%x:%x:%x:%x\n",g_macaddress[0],g_macaddress[1],g_macaddress[2],g_macaddress[3],g_macaddress[4],g_macaddress[5]);
}

/*****************************************************************************                                                                        
  Function Name : WIFI_IPF_readMacAddr                                  
                                                                           
  Description   : This function reads the saved/stored MAC address from specified location
                            using DRV_IPF_BlockRead call.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_readMacAddr()
{
    ipfInfo.reqComplete = 0;	 
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL, g_macaddress , IPF_WIFI_MACADDRESS_START, MACADDRESS_LENGTH);    
    ipfInfo.state = IPF_STATE_READ_MACADDR;
    
    return;
}

/*******************************************************************************************                                                            
  Function Name : WIFI_IPF_write_regdomain                                  
                                                                           
  Description   : This function is to write the data of g_wifiRegdomainConfig into IPF        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
******************************************************************************************/
void WIFI_IPF_write_regdomain()
{
    int cfg_data_size;
    
    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiRegdomainConfig);
   
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL,(uint8_t *)g_wifiRegdomainConfig, IPF_WIFI_REGDOMAINCONFIG_START,cfg_data_size);
    ipfInfo.state = IPF_STATE_WRITE_REGDOMAINCONFIG;
    return;
}

/******************************************************************************************************                                                                        
  Function Name : WIFI_IPF_read_regdomain                                  
                                                                           
  Description   : This function reads the saved/stored Regulatory domain configuration from IPF
                            using DRV_IPF_BlockRead call.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*******************************************************************************************************/
void WIFI_IPF_read_regdomain()
{
    
    ipfInfo.reqComplete = 0;	 
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *)g_wifiRegdomainConfig , IPF_WIFI_REGDOMAINCONFIG_START, sizeof(wifiRegdomainConfig));    
    ipfInfo.state = IPF_STATE_READ_REGDOMAINCONFIG;
    
    return;
}

/*****************************************************************************                                                                        
  Function Name : update_bootMode                                  
                                                                           
  Description   : This function updates the BOOTMODE of Device to
					 global variable g_bootMode
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void update_bootMode()
{
    if(g_wifiConfigData->bootMode == 1)    
        g_bootMode = BOOT_AP_MODE;
    
    else if(g_wifiConfigData->bootMode == 2)
        g_bootMode = BOOT_STA_MODE;
    
    else
        g_bootMode = BOOT_INVALID_MODE;

    return;
}

/*****************************************************************************                                                                        
  Function Name : switch_mac_operation_mode                                  
                                                                           
  Description   : This function switches the mode of MAC to the g_bootMode value
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void switch_mac_operation_mode()
{
    bool status = 1;
    TCPIP_NET_HANDLE netH;

        //Set SwitchMode WID
        status = DRV_WIFI_Write(DRV_WIFI_WID_SWITCH_MODE, g_bootMode,1);
    
        if(status == 0)
        {
            printf("\nBootMode Switch successful\n");
        }
        else
        {
            printf("\nBootMode Switch Failed\n");
        }

        //Switch the DHCP/DHCPS as per WiFi mode

        if(g_bootMode == BOOT_AP_MODE)
        {
           netH = TCPIP_STACK_NetHandleGet(TCPIP_NETWORK_DEFAULT_INTERFACE_NAME);
            if(TCPIP_DHCP_IsEnabled(netH) == true)       
                {
                    printf("\nStop DHCP Clinet");
                    TCPIP_DHCP_Disable(netH);
                    TCPIP_STACK_AddressServiceSelect((TCPIP_NET_IF*)&netH, TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON);
                    printf("\nStart DHCP Server");
                    TCPIP_DHCPS_Enable(netH);        
                }
            else
                printf("\nAlready DHCP Server running...!");            
        }
        else if(g_bootMode == BOOT_STA_MODE)
        {
            netH = TCPIP_STACK_NetHandleGet(TCPIP_NETWORK_DEFAULT_INTERFACE_NAME);
            if(TCPIP_DHCPS_IsEnabled(netH) == true)
                {
                    printf("\nStop DHCP Server");
                    TCPIP_DHCPS_Disable(netH);
            
                    TCPIP_STACK_AddressServiceSelect((TCPIP_NET_IF*)&netH, TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON);
                    printf("\nStart DHCP Client");
                    TCPIP_DHCP_Enable(netH);                  
                }
            else
                printf("\nAlready DHCP Client running...!");
        }
}

/*****************************************************************************                                                                        
  Function Name : wifi_do_softReboot                                  
                                                                           
  Description   : This function triggers a soft reboot
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void wifi_do_softReboot()
{
  //  int tempCounter;
    SYSKEY = 0xAA996655;  //write key1 to SYSKEY
    SYSKEY = 0x556699AA;  //write key2 to SYSKEY
                
    // OSCCON is now unlocked
    /* set SWRST bit to arm reset */
    RSWRSTSET = 1;
    /* read RSWRST register to trigger reset */
    //tempCounter = RSWRST;
    /* prevent any unwanted code execution until reset occurs*/
    while(1);

    return;
}

#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED
/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void LED_Task(void)
{
    static int blink_intensity[MAX_LED_ON_BOARD];
    int led_val;

    for(led_val=0; led_val<MAX_LED_ON_BOARD; led_val++)
    {
        blink_intensity[led_val]++;
        if(blink_intensity[led_val] == (onBoardLED[led_val].led_blink_speed+1))
            blink_intensity[led_val] = 0;

        switch(onBoardLED[led_val].led_status)
        {
            case LED_ON:/*ON*/    
                BSP_LEDOn(onBoardLED[led_val].led_number);
                blink_intensity[led_val] = 0;                
                break;
            
            case LED_OFF:/*OFF*/
                BSP_LEDOff(onBoardLED[led_val].led_number);
                blink_intensity[led_val] = 0;
                break;
            
            case LED_BLINK: /*BLINK*/
                if(blink_intensity[led_val] == onBoardLED[led_val].led_blink_speed)
                    BSP_LEDToggle(onBoardLED[led_val].led_number);
                break;
            
            default:
                BSP_LEDOff(onBoardLED[led_val].led_number);
        }
    }
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void wlan_set_LED_values(uint8_t led_number, uint8_t led_status, int led_blink_speed )
{
    onBoardLED[led_number].led_number = led_number;
    onBoardLED[led_number].led_status = led_status;
    onBoardLED[led_number].led_blink_speed = led_blink_speed;
    
//    LED_Task(); //Set the values once immediately after setting the global variables
        
    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void wlan_set_LED()
{
    static int led1_intensity,led2_intensity; // These are used to DATA transfer progress
    
    //Scanning
    if(g_wifi_mac_status != 1)
        wlan_set_LED_values(BSP_LED3, LED_BLINK, FAST_BLINK); // LED-3 Green Fast Blink
    
    //Connected
    if(g_wifi_mac_status == 1)
        wlan_set_LED_values(BSP_LED3, LED_ON, 0); // LED-3 Green ON

    //Data Transfer
    switch(g_wifi_data_progress_path)
    {
        case 1://Tx
            wlan_set_LED_values(BSP_LED1, LED_ON, 0); // LED-1 Red, Toggle for Tx      
            led1_intensity++;
            break;
            
        case 2://Rx
            wlan_set_LED_values(BSP_LED2, LED_ON, 0); // LED-2 Orange, Toggle for Rx                  
            led2_intensity++;
            break;
            
         default:
            {
                wlan_set_LED_values(BSP_LED1, LED_OFF, 0); // LED-1 Red OFF; No data progess
                wlan_set_LED_values(BSP_LED2, LED_OFF, 0); // LED-2 Orange OFF; No data progess            
            }
    }
    if(led1_intensity == 9999 || led2_intensity == 9999)
    {
        g_wifi_data_progress_path = 0;    
        led1_intensity = 0;
        led2_intensity = 0;
    }
    
    return;
}
#endif

/*****************************************************************************                                                            
  Function Name : IPF_RW_UpdateStatus                                  
                                                                           
  Description   : This function is a callback function after completing every
                          Read/Write/Erase on Flash            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void IPF_RW_UpdateStatus
(
    DRV_IPF_BLOCK_EVENT event,
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
 
    if(event == DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE)
    {
//        printf("\n Read_Write is sucess, present IPF state:%d",ipfInfo.state);     
        ipfInfo.reqComplete = 1;
    if(ipfInfo.state == IPF_STATE_PROTECTCAL){
        
        if(ipfInfo.prevState == IPF_STATE_INIT){
            WIFI_IPF_read_cal();
        }
        ipfInfo.prevState = IPF_STATE_PROTECTCAL;
        ipfInfo.prevState = IPF_STATE_IDLE;
    }
    else if(ipfInfo.state == IPF_STATE_READ_SECUREREGION)
    {
        ipfInfo.prevState =IPF_STATE_READ_SECUREREGION;
        Secure_read_completed = 0;
        ipfInfo.state = IPF_STATE_IDLE;
    }
    else if(ipfInfo.state == IPF_STATE_UNPROTECTCAL)
    {   
        WIFI_IPF_secureregion_modify();        
        ipfInfo.prevState = IPF_STATE_UNPROTECTCAL;
        ipfInfo.state = IPF_STATE_IDLE;
       
    }else if(ipfInfo.state == IPF_STATE_READ_CAL)
    {
            WIFI_IPF_readMacAddr();
            ipfInfo.prevState = IPF_STATE_READ_CAL;
	}
    else if(ipfInfo.state == IPF_STATE_READ_MACADDR)
       {
                   
            ipfInfo.state = IPF_STATE_IDLE;
            validate_update_macaddress();
            WIFI_IPF_read_regdomain();
            ipfInfo.prevState = IPF_STATE_READ_MACADDR;
        }
       else if(ipfInfo.state == IPF_STATE_ERASE)
       {
            WIFI_IPF_write();
            ipfInfo.prevState = IPF_STATE_ERASE;
       }
        else if(ipfInfo.state == IPF_STATE_ERASE_REGDOMAINCONFIG)
       {
            WIFI_IPF_write_regdomain();
            ipfInfo.prevState = IPF_STATE_ERASE_REGDOMAINCONFIG;
       }
       else if(ipfInfo.state == IPF_STATE_READ_REGDOMAINCONFIG)
       {
             if(checkregdomainconfig())
              {               
               updatedefault_regdomainconfig();
             }
            if(ipfInfo.prevState == IPF_STATE_READ_MACADDR) {
                WIFI_IPF_read();
            }else{             
                ipfInfo.state = IPF_STATE_IDLE;
            }
             ipfInfo.prevState = IPF_STATE_READ_REGDOMAINCONFIG;
       }         
       else if(ipfInfo.state == IPF_STATE_READ)
       {
            //Call to set the configuration
            if(checkreadssid())
            {
                updatedefault_wifiuserconfig();
            }
               printf("\nSSID:%s \n",g_wifiConfigData->ssid);
             //  WIFI_IPF_setConfig();
            

            if(g_wifiConfigData->bootMode == 0xff)
            {
                printf("Boot mode Stored in Flash is:%d, Assigning default as AP-mode",g_wifiConfigData->bootMode);     
                printf(" Please update bootmode in flash using command \"wlan bootmode [val]\"");                
                g_wifiConfigData->bootMode = 1; //1=AP , 2=STA and default is AP if not set in IPF
            }
#if 0

             if(strcmp(DRV_WIFI_MODE,"STA_MODE") == 0)
                {
                    g_wifiConfigData->bootMode = 2;
                    printf("\nWifi Mode is %s",DRV_WIFI_MODE);
                }

             if(strcmp(DRV_WIFI_MODE,"AP_MODE") == 0)
                {
                    g_wifiConfigData->bootMode = 1;
                    printf("\nWifi Mode is %s",DRV_WIFI_MODE);
                }             
#endif

            if((strcmp(DRV_WIFI_MODE,"AP_MODE") == 0) && (g_wifiConfigData->bootMode ==1)) 
                {
                    SYS_CONSOLE_MESSAGE("Application Boot mode and Boot mode stored in Flash are same and set to AP-mode\r\n");
#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED
                    wlan_set_LED_values(BSP_LED1, LED_OFF, 0);            //LED-1 OFF        
#endif                    
                }
            if((strcmp(DRV_WIFI_MODE,"STA_MODE") == 0) && (g_wifiConfigData->bootMode ==2)) 
                {
                    SYS_CONSOLE_MESSAGE("Application Boot mode and Boot mode stored in Flash are same and set to STA-mode\r\n");
#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED
                    wlan_set_LED_values(BSP_LED1, LED_OFF, 0);            //LED-1 OFF        
#endif                    
                }
            if((strcmp(DRV_WIFI_MODE,"AP_MODE") == 0) && (g_wifiConfigData->bootMode ==2)) 
                {
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");
                    SYS_CONSOLE_MESSAGE("\t\t\tBOOTMODE MISMATCH\r\n");
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");                        
                    SYS_CONSOLE_MESSAGE("* Application Boot mode is AP-mode and Boot mode stored in Flash is STA-mode\r\n");
                    SYS_CONSOLE_MESSAGE("* Both are not matching\r\n");
                    SYS_CONSOLE_MESSAGE("* Update Application bootmode to STA mode from MHC\r\n");                    
                    SYS_CONSOLE_MESSAGE("* OR Update bootmode in flash using below commands\r\n");
                    SYS_CONSOLE_MESSAGE("\twlan open\r\n");                    
                    SYS_CONSOLE_MESSAGE("\twlan bootmode 1 (AP-mode)\r\n");
                    SYS_CONSOLE_MESSAGE("\twlan save config\r\n");                    
                    SYS_CONSOLE_MESSAGE("* After update NEED to restart the device\r\n");     
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");                    
                    g_wifiConfigData->bootMode = 0;

#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED                               
                    wlan_set_LED_values(BSP_LED1, LED_BLINK, FAST_BLINK); // LED-1 Fast Blink
#endif                    
                }
            if((strcmp(DRV_WIFI_MODE,"STA_MODE") == 0) && (g_wifiConfigData->bootMode ==1)) 
                {
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");
                    SYS_CONSOLE_MESSAGE("\t\t\tBOOTMODE MISMATCH\r\n");
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");      
                    SYS_CONSOLE_MESSAGE("* Application Boot mode is STA-mode and Boot mode stored in Flash is AP-mode\r\n");
                    SYS_CONSOLE_MESSAGE("* Both are not matching\r\n");
                    SYS_CONSOLE_MESSAGE("* Please update Application bootmode to AP mode from MHC\r\n");
                    SYS_CONSOLE_MESSAGE("* OR Please update bootmode in flash using below commands\r\n");
                    SYS_CONSOLE_MESSAGE("\twlan open\r\n");                    
                    SYS_CONSOLE_MESSAGE("\twlan bootmode 0 (STA-mode)\r\n");
                    SYS_CONSOLE_MESSAGE("\twlan save config\r\n");                       
                    SYS_CONSOLE_MESSAGE("* After update NEED to restart the device\r\n");     
                    SYS_CONSOLE_MESSAGE("*************************************************************************\r\n");  
                    g_wifiConfigData->bootMode = 0;

#ifdef BSP_PIC32WK_GPB_GPD_SK_MODULE_ENABLED                    
                    wlan_set_LED_values(BSP_LED1, LED_BLINK, FAST_BLINK); // LED-1 Fast Blink                    
#endif                    
                }
            
            update_bootMode();
            
            /*After Successful Read move to the IDLE state*/
            ipfInfo.state = IPF_STATE_IDLE;

            if(g_bootMode != BOOT_INVALID_MODE)
			 IPF_ready_use_state = 1;

        if(web_data == 1)
            {
                /*Save the received SSID in Flash*/
                strcpy((char *)g_wifiConfigData->ssid,(const char *)g_easyConfigCtx.ssid);

                g_wifiConfigData->SecurityKeyLength = g_easyConfigCtx.SecurityKeyLength;
                printf("\nSecurity Key Length:%d",g_wifiConfigData->SecurityKeyLength);
                /*Save Security Parameters*/
                if(g_wifiConfigData->SecurityKeyLength !=0)
                {
                    strcpy((char *)g_wifiConfigData->key,(const char *)g_easyConfigCtx.key);
                    g_wifiConfigData->cipher = g_easyConfigCtx.cipher;
                }
                
                /*Boot mode set to STA-mode, and connects to above configuration AP*/
                g_wifiConfigData->bootMode = 2;

                printf("\nWEB SSID:%s",g_wifiConfigData->ssid);           
                printf("\nWEB Boot mode:%d",g_wifiConfigData->bootMode);
                WIFI_IPF_save();
            }
            ipfInfo.prevState = IPF_STATE_READ;
        }
        else if(ipfInfo.state == IPF_STATE_WRITE_REGDOMAINCONFIG)
       {
            
            ipfInfo.state = IPF_STATE_IDLE;
            ipfInfo.prevState = IPF_STATE_WRITE_REGDOMAINCONFIG;
       } 
        else if(ipfInfo.state == IPF_STATE_WRITE)
        {
                printf("\nweb_data:%d,g_bootMode:%d",web_data,g_bootMode);     
            if(web_data == 1)
            {
                if(g_bootMode == BOOT_STA_MODE)
                {
                    printf("\nUpdate the SSID of AP received from Wep-page in STA mode, to connect to that AP\n");
                    WIFI_IPF_setConfig();
                }
                else if(g_bootMode == BOOT_AP_MODE)
                {
                    //DO Soft-reboot of the DEVICE
                    printf("\nDoing Soft Reboot......!");
                    wifi_do_softReboot();
                }
                web_data = 0;
            }
            /*After Successful Write, free the memory created before write and move to the IDLE state*/
            WIFI_IPF_usrconfig_memfree();
            ipfInfo.state = IPF_STATE_IDLE;
            ipfInfo.prevState = IPF_STATE_WRITE;
        }		
    }
    else         
        ipfInfo.state = IPF_STATE_ERROR;
    //DRV_IPF_Close(IPF_Handle);  
    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_open()
{
    bool status = 0;
    ipfInfo.state = IPF_STATE_ERROR;
    ipfInfo.IPF_Handle = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
 
    if(ipfInfo.IPF_Handle != (DRV_HANDLE) DRV_HANDLE_INVALID)
    {
    
        WIFI_IPF_regdomconfig_memalloc();
        WIFI_IPF_usrconfig_memalloc();        
    	WIFI_IPF_cal_memalloc();
    	DRV_IPF_BlockEventHandlerSet(ipfInfo.IPF_Handle, IPF_RW_UpdateStatus, (uintptr_t) NULL);
        ipfInfo.state= IPF_STATE_INIT;
         printf("\n IPF Driver opened");
        status = 1;
    }
    else
    {
        printf("\n error opening driver\r%s, line %u\r\n", __FUNCTION__, __LINE__);
    }

#ifdef DRV_WIFI_OTA_ENABLE
     
    otaHandler = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
    if(otaHandler != (DRV_HANDLE) DRV_HANDLE_INVALID)
    {
            printf("\n OTA Driver opened %x", otaHandler);
    }
#endif    
    
    return status;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_write()
{
    int cfg_data_size;
    
    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiConfigData);

    printf("\n\t\t\tWriting SSID:%s, Configdata size:%d",g_wifiConfigData->ssid, cfg_data_size);    
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL,(uint8_t *) g_wifiConfigData, IPF_WIFI_CONFIG_START,cfg_data_size);
#if 0    
    for(abc=0;abc<100;abc++){    
           buf[abc] = 0x48;//abc+1;
        }                   

    printf("\nWriting %s\n",buf);          
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL, &buf[0], 0,100);
#endif

    ipfInfo.state = IPF_STATE_WRITE;
    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_read()
{
    int cfg_data_size;

    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiConfigData);
    
//    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL, ipf_read_buf , 0x90001000, 100);
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *) g_wifiConfigData , IPF_WIFI_CONFIG_START, cfg_data_size);    
    ipfInfo.state = IPF_STATE_READ;

    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_read_cal()
{
	 ipfInfo.reqComplete = 0;	 
	 DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *)Calib_mem , IPF_WIFI_CALIBRATION_START, IPF_WIFI_CALIBRATION_LENTH);    
    ipfInfo.state = IPF_STATE_READ_CAL;

    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_cal_memfree()
{
    if (Calib_mem){
		free(Calib_mem);
		Calib_mem = NULL;
    }
    return 0;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_usrconfig_memfree()
{
    if (g_wifiConfigData){
        
		//free(g_wifiConfigData);
		//g_wifiConfigData = NULL;
        return 0;
    }
    return 1;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_regdomconfig_memfree()
{
    if (g_wifiRegdomainConfig){
		//free(g_wifiRegdomainConfig);
		//g_wifiRegdomainConfig = NULL;
        return 0;
    }
    return 1;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_cal_memalloc()
{
	
    Calib_mem = malloc(IPF_WIFI_CALIBRATION_LENTH);
	if (Calib_mem){
		return 0;
	}
	return 1;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_usrconfig_memalloc()
{
    g_wifiConfigData = &temp_wifiConfigData;
    //g_wifiConfigData = malloc(sizeof(wifiConfigData));
	if (g_wifiConfigData){
        
		return 0;
	}
	return 1;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool WIFI_IPF_regdomconfig_memalloc()
{
    
    g_wifiRegdomainConfig = &temp_wifiRegdomainConfig;
    //g_wifiRegdomainConfig = malloc(sizeof(wifiRegdomainConfig));
	if (g_wifiRegdomainConfig){
		return 0;
	}
	return 1;
}

/*****************************************************************************                                                            
  Function Name : get_PersParam                                  
                                                                           
  Description   : This function provide RF calibration data to WLAN library.            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return pointer to RF calibration data.                                                     
*****************************************************************************/

unsigned int  * get_PersParam()     
{
    if(IPF_ready_use_state == 1) {
	return Calib_mem;
    } 
    return NULL;
}

/*****************************************************************************                                                            
  Function Name : get_PersParamfree                                  
                                                                           
  Description   : This function is to free persistent details            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void get_PersParamfree()
{
    WIFI_IPF_setConfig();    
    WIFI_IPF_cal_memfree();
    WIFI_IPF_usrconfig_memfree();
    WIFI_IPF_regdomconfig_memfree();
}

/*****************************************************************************                                                            
  Function Name : do_switchMode                                  
                                                                           
  Description   : This function is to switch mode of WiFi            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void do_switchMode()
{
    if(g_bootMode < BOOT_INVALID_MODE)
	{
	    printf("\nBoot Switch started for mode:%d",g_bootMode);
           switch_mac_operation_mode();	
	}
    return;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_erase(void)
{
    ipfInfo.reqComplete = 0;

    DRV_IPF_BlockErase(ipfInfo.IPF_Handle, NULL, IPF_WIFI_CONFIG_START, 1);
    ipfInfo.state = IPF_STATE_ERASE;
   
}
/*****************************************************************************                                                            
  Function Name : WIFI_IPF_erase_regdomain                                  
                                                                           
  Description   : This function is a to erase the reg domine area
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_erase_regdomain(void)
{
    ipfInfo.reqComplete = 0;

    DRV_IPF_BlockErase(ipfInfo.IPF_Handle, NULL, IPF_WIFI_REGDOMAINCONFIG_START, 1);
    ipfInfo.state = IPF_STATE_ERASE_REGDOMAINCONFIG;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_save()
{
    printf("\nipfInfo.state:%d",ipfInfo.state);
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            WIFI_IPF_erase();
        }
    else
            printf("\nSave failed");
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_save_regdom()
{
    
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            WIFI_IPF_erase_regdomain();
        }
    else
            printf("\n RegDom Save failed");
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
void WIFI_IPF_readAndBackup()
{
    printf("\nipfInfo.state:%d",ipfInfo.state);
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            if(WIFI_IPF_usrconfig_memalloc() == 0)
                {
                        WIFI_IPF_read();
                }
            else
                printf("\n Memory allocation for g_wifiConfigData failed");
        }
    else
        printf("\nRead to Backup Failed");
}

/*****************************************************************************                                                            
  All below funtions are related to accessing Secure region.
  REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
/*****************************************************************************                                                            
  Function Name : secureregion_memmap                                  
                                                                           
  Description   : This function map the read IPF secure data to MAC address,
                  Regulatory domain and OTA address pointers.            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None.                                                     
*****************************************************************************/
void secureregion_memmap(uint8_t sector_type){
    
    
    if(sector_type == IPF_SECTOR_1)
    {
        
        /*RF Calibration address offset memory mapping*/
       // Calib_mem= (unsigned int *)ipf_sector; 
        
        /*MAC address offset memory mapping*/
        l_macaddr=(ipf_sector+IPF_WIFI_MACADDRESS_OFFSET); 
       
        
    } else if(sector_type == IPF_SECTOR_2)
    {
        /*Regulatory domain data address offset memory mapping*/
        g_wifiRegdomainConfig = (wifiRegdomainConfig *)ipf_sector; 
        
        /*OTA address offset memory mapping*/
        ota_address=(uint32_t *) (ipf_sector + IPF_WIFI_OTA_ADDRESS_OFFSET); 
        
    }
}
/*****************************************************************************                                                            
  Function Name : dump_secureregion_data                                  
                                                                           
  Description   : This function dump secure data.
                  User can dump secure region data before and after modification.
                  By default DUMP_SECUREREGION_DATA disabled.
                  User need to enable this preprocessing macro for dumping data.
                                                                           
  Inputs       : None                
                                                                           
  Outputs      : None.                                                     
*****************************************************************************/
#ifdef DUMP_SECUREREGION_DATA
void dump_secureregion_data()
{
    int Inx=0;
    uint8_t sector = WIFI_IPF_secureregion_sector_get();
    printf("\nSecure region sector %d data are : ",sector);
    if(sector && !WIFI_IPF_secureregion_read_status())
    for(Inx=0;Inx<IPF_SECTOR_SIZE;Inx++)
        printf("%x ",ipf_sector[Inx]);
    printf("\n");
}
#endif    

void WIFI_IPF_secureregion_read()
{
    int32_t address = WIFI_IPF_secureregion_address();
    
    if(address != INVALID )
    {
       DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *)ipf_sector , address, IPF_SECTOR_SIZE);
       ipfInfo.state = IPF_STATE_READ_SECUREREGION;
    } 
    else 
    {
        SYS_MESSAGE("\nIPF Secure region is not open for erase\n");
    }
}
void WIFI_IPF_secureregion_write()
{
    int32_t address = WIFI_IPF_secureregion_address();
    
    if(address != INVALID )
    {
       DRV_IPF_BlockWrite(ipfInfo.IPF_Handle, NULL,(uint8_t *)ipf_sector , address, IPF_SECTOR_SIZE);    
    } 
}
int32_t WIFI_IPF_secureregion_address()
{
    uint8_t sector_type = WIFI_IPF_secureregion_sector_get();
    int32_t ret = INVALID;
    
    if(sector_type != IPF_SECTOR_INVALID )
    {
        ret = (sector_type == IPF_SECTOR_1) ? IPF_SECTOR_ADDRESS_1:IPF_SECTOR_ADDRESS_2;
    } else 
    {
        SYS_MESSAGE("\nIPF Secure region is not open \n");        
    }
    return ret;
}
void WIFI_IPF_secureregion_erase()
{
    int32_t address = WIFI_IPF_secureregion_address();
    
    if(address != INVALID )
    {
        DRV_IPF_BlockErase(ipfInfo.IPF_Handle, NULL, address, 1);
    } 
    
}
void WIFI_IPF_secureregion_open(IPF_SECTOR sector_type)
{
    if(WIFI_IPF_secureregion_sector_get() == IPF_SECTOR_INVALID ){
        WIFI_IPF_secureregion_sector_set(sector_type);
    } else {
        SYS_MESSAGE("\nIPF Secure region already into use\n");
    }
    
    //need to crate dynamic memory here for sector 
    secureregion_memmap(sector_type);
    Secure_read_completed = 1;  
    WIFI_IPF_secureregion_read();
          
}
void WIFI_IPF_secureregion_sector_set(IPF_SECTOR sector_type)
{    
    secure_region = sector_type;
}
IPF_SECTOR WIFI_IPF_secureregion_sector_get()
{
    return secure_region;
}

uint8_t WIFI_IPF_secureregion_read_status()
{
    return Secure_read_completed;
}

void* WIFI_IPF_secureregion_macoffset()
{
#ifdef DUMP_SECUREREGION_DATA
    dump_secureregion_data();
#endif    
    return l_macaddr;
}

void* WIFI_IPF_secureregion_otaaddroffset()
{
#ifdef DUMP_SECUREREGION_DATA
    dump_secureregion_data();
#endif    
    return ota_address;
}

void* WIFI_IPF_secureregion_regdomoffset()
{
#ifdef DUMP_SECUREREGION_DATA
    dump_secureregion_data();
#endif    
    return g_wifiRegdomainConfig;
}

void WIFI_IPF_secureregion_modify()
{
#ifdef DUMP_SECUREREGION_DATA
    dump_secureregion_data();
#endif    
    
    WIFI_IPF_secureregion_erase();
    WIFI_IPF_secureregion_write();
    WIFI_IPF_secureregion_protect();
        
}

void WIFI_IPF_secureregion_close()
{
    memset(ipf_sector,0x00,IPF_SECTOR_SIZE);
    //need to free dynamic memory which is created into open 
    
    WIFI_IPF_secureregion_sector_set(IPF_SECTOR_INVALID);
    
    Secure_read_completed =1;
}
/*******************************************************************************
 End of File
 */
