/*******************************************************************************
  Sample Application

  File Name:
    WiFi_MW.c

  Summary:
	This file contains functions to process the commands and trigger the
	corresponding API's in WiFi library.

  Description:
    
 *******************************************************************************/
// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END
#include <ctype.h>
#include "WiFi_commands.h"
#include "WiFi_MW.h"
#include "drv_mchpwlan.h"
#include "wifi_flash_use.h"


typedef struct wificmd {
char str[10];
short wid;
}iwificmd;

/*******************************************************************************
    Global Variables    
*******************************************************************************/
#define WLAN_REGDOMAIN_DEBUG
wifiConfigData    *g_wifiConfigData=NULL;

int *temp_mem_pool;
int temp_mem_pool_size;
int configSavedinFlash = 1;
unsigned char mac_debug_module_level[5];

static iwificmd Wificmdtable []=
{
	{"ssid",   			DRV_WIFI_WID_SSID,                  },	//"Network SSID"
	{"txrate",   		DRV_WIFI_WID_CURRENT_TX_RATE		},  // set current tx rate
	{"imode",  			DRV_WIFI_WID_11I_MODE               },  //"WEP/802.11i Config (0=>Disable 0x3=>WEP40 0x7=>WEP104 0x29=>WPA-AES 0x49=>WPA-TKIP 0x31=>WPA2-AES 0x51=>WPA2-TKIP 0x79=>WPA/WPA2-AES+TKIP)"
	{"wepkey",  		DRV_WIFI_WID_WEP_KEY_VALUE          },  //"WEP Key-0 (5 bytes for WEP40 and 13 bytes for WEP104)"
	{"psk",    			DRV_WIFI_WID_11I_PSK				},  //"AES/TKIP WPA/RSNA Pre-Shared Key (Any string of length between 8-64 Bytes)"
	{"auth",   			DRV_WIFI_WID_AUTH_TYPE              },  //"WEP Authentication Type (0=>Open 1=>Shared-Key 3=>Any-Type 4=>802.1X Auth)"
	{"gmode",  			DRV_WIFI_WID_11G_OPERATING_MODE     },  //11G Operating Mode (0=> 11bOnly 1=>Gonly 2=>Mixed1 3=>Mixed2 4=>Custom)"
	{"ackp",   			DRV_WIFI_WID_ACK_POLICY             },  //"ACK Policy (0=>Normal 1=>NoACK)"
	{"ht",     			DRV_WIFI_WID_11N_ENABLE             },  //"HT Capability (0=>Disable 1=>Enable)"
	{"mcs",    			DRV_WIFI_WID_11N_CURRENT_TX_MCS     },  //"Transmit MCS (0 - 127)"
	{"usrpch",  		DRV_WIFI_WID_USER_PREF_CHANNEL      },   //user preferred channel
	{"swtmode", 		DRV_WIFI_WID_SWITCH_MODE            },    //switch mode
	{"scan", 			DRV_WIFI_WID_START_SCAN_REQ         }    // perform scan operation


};

/*****************************************************************************                                                                        
  Function Name : setwidvalue                                  
                                                                           
  Description   : This function is to set the WID value. 
                                        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return the size                                                 
*****************************************************************************/
static int setwidvalue(unsigned short wid, unsigned char*pbuf, unsigned char* pvalue, int size )
{

   DRV_WIFI_WID_TYPE_T wid_type = (DRV_WIFI_WID_TYPE_T)( (wid & 0xF000) >> 12);

   switch(wid_type)
   {
   case DRV_WIFI_WID_CHAR:
      size = 1;
      break;
   case DRV_WIFI_WID_SHORT:
      size = 2;
      break;
   case DRV_WIFI_WID_INT:
      size = 4;
      break;
   default:
      break;
   }
   pbuf[0] = wid & 0xFF;
   pbuf[1] = (wid >> 8) & 0xFF;
   pbuf[2] = size & 0xFF;
   pbuf[3] = (size >> 8) & 0xFF;
   memcpy( pbuf+4, pvalue, size );
   return size + 4;
}

/*****************************************************************************                                                                        
  Function Name : performSiteScan                                  
                                                                           
  Description   : This function is to trigger SCAN in WLAN MAC using
					multiple WID's
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return number of WID's
*****************************************************************************/
static char performSiteScan(char ival[],char *ssid,bool bActive)
{
	char nWids =0;
	short index=0;
	unsigned char bcastfalse = 0;
	unsigned char bcasttrue = 1;
	unsigned char ScanReq=1;
	unsigned char ScanType= bActive ? 1 : 0;

	index += setwidvalue( DRV_WIFI_WID_SCAN_TYPE,(unsigned char*) ival, (unsigned char*)&ScanType,sizeof(ScanType));
	   nWids++;
	if(ssid) {
		index += setwidvalue( DRV_WIFI_WID_SCAN_SSID,(unsigned char*) ival+index, (unsigned char*)ssid, strlen(ssid) );
	     nWids++;
		 index += setwidvalue( DRV_WIFI_WID_BCAST_SSID,(unsigned char*) ival+index, (unsigned char*)&bcastfalse,sizeof(bcastfalse));
		 nWids++;
	} else {
   		index += setwidvalue( DRV_WIFI_WID_BCAST_SSID,(unsigned char*) ival+index, (unsigned char*)&bcasttrue,sizeof(bcasttrue));
     	nWids++;
	}
	 index += setwidvalue( DRV_WIFI_WID_START_SCAN_REQ,(unsigned char*) ival+index, (unsigned char*)&ScanReq,sizeof(ScanReq) );
	  nWids++;
	  
	return nWids;
}

/*****************************************************************************
REFER HEADER FILE FOR COMPLETE INFORMATION OF FUNCTION  
*****************************************************************************/
bool wifi_command_process(int argc,char *argv[])
{
	short idx =0;
	bool ret =1 ;
    static bool user_config_enable = 0;

	/*Example command: wlan set ssid microchip*/
#if 1 //This is added when working on "malloc" issue. Need to delete later
    if((argv[1]) && (argv[2]) && (!strcmp((char*)argv[1],"memalloc"))) {
        temp_mem_pool_size = strtoul( argv[2], 0, 10);
        printf("\nCreating Memory of %d ",temp_mem_pool_size);
        temp_mem_pool = malloc(temp_mem_pool_size);
        printf("with address:%p",temp_mem_pool);
    }
    else if((argv[1]) && (!strcmp((char*)argv[1],"memfree"))) {
        if(temp_mem_pool != NULL)
        {
            printf("\nMemeory free of %p",temp_mem_pool);
            free(temp_mem_pool);
        }
        else
            printf("\nnot able to free");
    
    }
    else
#endif
    if((argc >= 2)&&(!strcmp((char*)argv[1],"open")))
    {
        if(argc == 3) {
            if(!strcmp((char*)argv[2],"secure1"))
            {
                WIFI_IPF_secureregion_open(IPF_SECTOR_1);

            } 
            else if(!strcmp((char*)argv[2],"secure2")) 
            {
                WIFI_IPF_secureregion_open(IPF_SECTOR_2);             
            }
            else
            {
                SYS_CONSOLE_PRINT("\nEnter valid secure region command\n");   
            }
        }
        else if(argc == 2)
        {
            WIFI_IPF_usrconfig_memalloc();
            WIFI_IPF_read();
            user_config_enable = 1;
        }
    }
    else if((argc >= 2)&& (!strcmp((char*)argv[1],"close")))
    {
        if((argc == 3)&&((!strcmp((char*)argv[2],"secure1"))||(!strcmp((char*)argv[2],"secure2"))) &&
                (WIFI_IPF_secureregion_sector_get() != IPF_SECTOR_INVALID))
        {
            WIFI_IPF_secureregion_close();
               
        } else if((argc == 2)&&(user_config_enable)) {
            
            WIFI_IPF_usrconfig_memfree();
            user_config_enable = 0;             
        }
        else
        {
                SYS_CONSOLE_PRINT("\nEnter valid command\n");   
        }
 		
    }  
    else if((argc >= 4)&&(user_config_enable) && (!strcmp((char*)argv[1],"set")))
    {
        //SYS_CONSOLE_PRINT( "SET input from user is  argv[1]=%s argv[2]=%s argv[3]=%s noelem=%d\r\n",argv[1],argv[2],argv[3],sizeof(Wificmdtable)/sizeof(Wificmdtable[0]));
        /*Set the required config to ConfigData
          * NOTE: The content set here will be save on "save" command to IPF
          * Applied to MAC on "apply" command*/

        if(!strcmp("ssid",argv[2]))
        {
            strcpy((char *)g_wifiConfigData->ssid,argv[3]);
            printf("\nSaving SSID:%s in ConfigData",g_wifiConfigData->ssid);
            g_wifiConfigData->SecurityKeyLength = 0; //This is to disable the Security
            g_wifiConfigData->cipher = 0;
            strcpy((char *)g_wifiConfigData->key, "\0");
        }
        else if(!strcmp("psk",argv[2]))
        {
            strcpy((char *)g_wifiConfigData->key,argv[3]);
            g_wifiConfigData->SecurityKeyLength = strlen((char *)g_wifiConfigData->key);
            printf("\nSaving PSK:%s of len:%d in ConfigData",g_wifiConfigData->key,g_wifiConfigData->SecurityKeyLength);
        }
        else if(!strcmp("imode",argv[2]))
        {
            g_wifiConfigData->cipher = strtoul( argv[3], 0, 16);
            printf("\nSaving Mode:%d in ConfigData",g_wifiConfigData->cipher);
        }
		else if(!strcmp("wid",argv[2]))
		{
			int wid = strtoul( argv[3], 0, 16);
            if(( (wid & 0xF000) >> 12) <= DRV_WIFI_WID_INT ) 
            ret = DRV_WIFI_Write(wid,strtoul( argv[4], 0, 10),4);
            else 
            ret = DRV_WIFI_Write(wid,(unsigned int)argv[4],strlen(argv[4]));    
            return 0;
		}
        else
            printf("\nWrong input values");
        configSavedinFlash = 0;
    }
    else if((argc >= 3)&& (user_config_enable) && (!strcmp((char*)argv[1],"get")))
    {
        char buf[1024];
        int iret =0;	
		
		 if(!strcmp((char*)argv[2],"wid")){
            int wid = strtoul( argv[3], 0, 16);
            iret = DRV_WIFI_Read(wid,buf);
            if(( (wid & 0xF000) >> 12) <= DRV_WIFI_WID_INT ){
                SYS_MESSAGE("output=");
				for(idx =0 ;idx < iret; idx++) {
					SYS_CONSOLE_PRINT("%d \r\n",buf[idx]);
				}
            }else 
                SYS_CONSOLE_PRINT("output buf=%s\r\n",buf);
            return 0;
        }
		
        for(idx=0;(idx < sizeof(Wificmdtable)/sizeof(Wificmdtable[0])) ;idx ++)
        {
            if(!strcmp((char*)Wificmdtable[idx].str,argv[2]))
            {		
                iret = DRV_WIFI_Read(Wificmdtable[idx].wid,buf);
                if(( (Wificmdtable[idx].wid & 0xF000) >> 12) <= DRV_WIFI_WID_INT)
                { // Wid type of char=0,short=1 or int=2
                    int Inx =0;
                    for(Inx =0 ;Inx < iret; Inx++)
                    {
                        //	SYS_CONSOLE_PRINT("buf[%d]=%d buf[%d]=%x\r\n",Inx,buf[Inx],Inx,buf[Inx]);
                    }
                }
                else
                {
                    //SYS_CONSOLE_PRINT("output buf=%s\r\n",buf);
                }
                ret = iret ? 0: 1 ;
                break;
            }
        }
    }
    else if((argc >= 3) && (!strcmp((char*)argv[1],"save")))
    {
                /*Save the configuration to FLASH*/
        printf("\nSave Command issued with write %s",argv[2]);
        if((argc==3)&&(user_config_enable ) && (!strcmp("config",argv[2])))
        { //Save the details to Flash
            //g_wifiConfigData.validity = WifiConfig_Valid;
            configSavedinFlash = 1;
            WIFI_IPF_save();
        }
        else if((argc==3)&&((!strcmp("secure1",argv[2]))||(!strcmp("secure2",argv[2])))
                && (WIFI_IPF_secureregion_sector_get() != IPF_SECTOR_INVALID))
        {
            WIFI_IPF_secureregion_unprotect();
            
        } else {
            SYS_CONSOLE_PRINT("\nEnter valid command\n");   
        }
        if(!strcmp("readconfig",argv[2]))
        { //Read the saved details from Flash
             //TODO
        }	
    }
    else if((argc >= 3)&& (user_config_enable) && (!strcmp((char*)argv[1],"apply")))
    {
#if 0    
        for(idx=0;(idx < sizeof(Wificmdtable)/sizeof(Wificmdtable[0])) ;idx ++)
        {
            if(!strcmp((char*)Wificmdtable[idx].str,argv[2]))
            {
                //SYS_CONSOLE_PRINT("widtype =%d \n",( (Wificmdtable[idx].wid & 0xF000) >> 12));
		  if(( (Wificmdtable[idx].wid & 0xF000) >> 12) <= DRV_WIFI_WID_INT )
                { // Wid type of char=0,short=1 or int=2
		      ret = DRV_WIFI_Write(Wificmdtable[idx].wid,strtoul( argv[3], 0, 16));
                }
                else
                {  // DRV_WIFI_WID type of String=3 or Binary=4                  
                    ret = DRV_WIFI_Write(Wificmdtable[idx].wid,(unsigned int)argv[3]);
                }

            }
        }
#endif        
        if(g_wifiConfigData->ssid != NULL)
        {
            printf("\nSet SSID:%s to MAC",g_wifiConfigData->ssid);
            ret = DRV_WIFI_Write(DRV_WIFI_WID_SSID,(unsigned int)g_wifiConfigData->ssid,strlen((const char *)g_wifiConfigData->ssid));
        }
        if(g_wifiConfigData->SecurityKeyLength != 0)
        {
            printf("\nSet Mode:%d and Key:%s to MAC, ret:%d", g_wifiConfigData->cipher,g_wifiConfigData->key,ret);
            ret = DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,(unsigned int)g_wifiConfigData->key,strlen((const char *)g_wifiConfigData->key));
            ret = DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,(unsigned int) g_wifiConfigData->cipher,1);    
            ret = DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE, 0,1);
            ret = DRV_WIFI_Write(DRV_WIFI_WID_11G_OPERATING_MODE, 2,1);
            ret = DRV_WIFI_Write(DRV_WIFI_WID_ACK_POLICY, 0,1);
            ret = DRV_WIFI_Write(DRV_WIFI_WID_11N_ENABLE, 1,1);            
        }
        if(ret == 0)
        {
            SYS_CONSOLE_MESSAGE("SUCCESSFUL \r\n");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("ERROR \r\n");
        }
        ret = DRV_WIFI_Write(DRV_WIFI_WID_START_SCAN_REQ,1,1);
    }
    else if((argc >= 3) && (user_config_enable) && (!strcmp((char*)argv[1],"bootmode")))
    {
        /*Save the Boot-Mode to FLASH*/   
        if((!strcmp("1",argv[2])) || (!strcmp("0",argv[2])))
        {
            SYS_CONSOLE_PRINT("Boot-mode:%s issued\r\n",(strtoul(argv[2],0,10)==1)?"AP":"STA");
            if(strtoul( argv[2], 0, 10 ) == 1) 
            { //Save the details to Flash
                g_wifiConfigData->bootMode = 1; //AP-mode
            }
            else
                g_wifiConfigData->bootMode = 2; //STA-mode
            //WIFI_IPF_save();
        }
        else
            SYS_CONSOLE_MESSAGE("Invalid Boot Mode.\r\nAvailable Boot modes are:\r\n 0 for STA\r\n 1 for AP\r\n");
    }       
        else if((argc >= 3) && (user_config_enable) && (!strcmp((char*)argv[1],"debug_level")))
    {
#if defined(DRV_WIFI_DEBUG_ENABLE)    
        /*argv[2]=debug_level, argv[3]=module id*/   
        int debug_level = strtoul(argv[2],0,10);
        mac_debug_module_level[0] = (char)(debug_level & 0x000f);

        if(argc > 3)
        {
            int module_val = strtoul(argv[3],0,16);
            char mdl_lvl;

//        printf("\nReceived values: %d, %d(%x)",debug_level,module_val,module_val);       
            mac_debug_module_level[4] = module_val & 0x00ff;

            module_val = strtoul(argv[3],0,10);
            mdl_lvl = (module_val & 0x0000ff00) >> 8;
            mac_debug_module_level[3] = mdl_lvl;

            module_val = strtoul(argv[3],0,10);
            mdl_lvl = (module_val & 0x00ff0000) >> 16;                
            mac_debug_module_level[2] = mdl_lvl;

            module_val = strtoul(argv[3],0,10);
            mdl_lvl = (module_val & 0xff000000) >> 24;
            mac_debug_module_level[1] = mdl_lvl;
         }
        else
        {
            mac_debug_module_level[1] = 0xFF;
            mac_debug_module_level[2] = 0xFF;
            mac_debug_module_level[3] = 0xFF;
            mac_debug_module_level[4] = 0xFF;
        }

        printf("Boot level info to MAC:%d, %x:%x:%x:%x",mac_debug_module_level[0],mac_debug_module_level[1],mac_debug_module_level[2],mac_debug_module_level[3],mac_debug_module_level[4]);
        
        DRV_WIFI_Write(DRV_WIFI_DEBUG_MODULE_LEVEL, (unsigned int)mac_debug_module_level, 5);
#else
        SYS_CONSOLE_PRINT("----WIFI DEBUGGING NOT ENABLED----\r\n");
#endif
    } 
    else if((argc == 4) && (user_config_enable) && (!strcmp((char*)argv[1],"dumpmem")))
    {
        //To show the content from the given address location upto the given size    
        uint8_t *data=(uint8_t *)strtoul( argv[2], 0, 16);
        uint32_t length=strtoul( argv[3], 0, 10),Idx=0;
        
        printf("\n DUMP at 0x%x of lenght:%d\n ",(unsigned int)data,length);
        for(Idx=0;Idx<length;Idx++){
            printf("%x ",data[Idx]);
         }
        
    }
    else if((argc >= 2) && (user_config_enable) && (!strcmp((char*)argv[1],"macinfo")))
    {
        unsigned char conf_ssid[32+1], conf_key[64+1], conf_mode;
        
        SYS_CONSOLE_PRINT("----WIFI MAC configuration----\r\n");
        
        SYS_CONSOLE_PRINT("BootMode: %s\r\n",(g_wifiConfigData->bootMode == 1)?"AP":"STA");
#ifdef TCPIP_STACK_USE_HTTP_SERVER
        SYS_CONSOLE_PRINT("HTTP: Enable\r\n");
#else
        SYS_CONSOLE_PRINT("HTTP: Disable\r\n");
#endif
#ifdef DRV_WIFI_OTA_ENABLE        
        SYS_CONSOLE_PRINT("OTA: Enable\r\n");
#else
        SYS_CONSOLE_PRINT("OTA: Disable\r\n");
#endif        
        SYS_CONSOLE_PRINT("SSID: %s\r\n",g_wifiConfigData->ssid);
        if(g_wifiConfigData->SecurityKeyLength != 0)
        {
            SYS_CONSOLE_PRINT("PSK: %s\r\n",g_wifiConfigData->key);
            SYS_CONSOLE_PRINT("Mode: 0x%2x\r\n",g_wifiConfigData->cipher);
        }
        else
            SYS_CONSOLE_PRINT("Security: NONE\r\n");
        
        if(configSavedinFlash == 1)
            SYS_CONSOLE_PRINT("The above Configuration is Saved in Flash\r\n");
        else
            SYS_CONSOLE_PRINT("The above configuration is not saved in Flash\r\n");
        
        SYS_CONSOLE_PRINT("****************************************************\r\n");
        if(g_wifiConfigData->bootMode == 1)
            SYS_CONSOLE_PRINT("----MAC Started as AP with following Details----\r\n");
        else
            SYS_CONSOLE_PRINT("----MAC Connected to AP with following Details----\r\n");
        DRV_WIFI_Read(DRV_WIFI_WID_SSID,conf_ssid);
            SYS_CONSOLE_PRINT("SSID from MAC:%s\r\n",conf_ssid);
            DRV_WIFI_Read(DRV_WIFI_WID_11I_MODE,&conf_mode);
        if((DRV_WIFI_Read(DRV_WIFI_WID_11I_PSK,conf_key) != 0) && (conf_mode != 0))
        {
            SYS_CONSOLE_PRINT("PSK: %s\r\n",conf_key);
            SYS_CONSOLE_PRINT("Mode: 0x%x",conf_mode);
        }
        else
            SYS_CONSOLE_PRINT("Security: NONE");
    }
    else if((argc >= 3) && (user_config_enable) && (!strcmp((char*)argv[1],"multiset")))
    {  //wlan multiset scan 1-Active/0-passive ssid
        unsigned char ival[50];
        int nWids = 0;
        for(idx=0;(idx < sizeof(Wificmdtable)/sizeof(Wificmdtable[0])) ;idx ++)
        {
            if(!strcmp((char*)Wificmdtable[idx].str,argv[2]))
            {
                nWids=performSiteScan((char *)ival,argv[4],strtoul( argv[3], 0, 16));
                ret = DRV_WIFI_MultiWrite(ival,nWids);
            }
        }
    } 
    else if((argc >=3) && (!strcmp((char*)argv[1],"ccath")))
    {
         if((argv[2]) && (argv[3]) && (argv[4]) && (!strcmp((char*)argv[2],"set")))
        {
            short busy_th = 0, clear_th = 0;
            busy_th= strtoul( argv[3], 0, 10);
            clear_th= strtoul( argv[4], 0, 10);            
            DRV_WIFI_Write(DRV_WIFI_WID_CCA_THRESHOLD,(((busy_th&0x01FF)<<15)|(clear_th & 0x01FF)),4);
        }  else if((argv[2]) && (!strcmp((char*)argv[2],"get")))
        {
            char buff[10];
            DRV_WIFI_Read(DRV_WIFI_WID_CCA_THRESHOLD,buff);
        }
    }
    else if((argc ==3) && (!strcmp((char*)argv[1],"--dgchigh")))
    {
        int val = strtoul( argv[2], 0, 10);
        DRV_WIFI_Write(DRV_WIFI_WID_DGC_RSSI_TH_HIGH,val,4);
     }
    else if((argc ==3) && (!strcmp((char*)argv[1],"--dgclow")))
    {
        int val = strtoul( argv[2], 0, 10);
        DRV_WIFI_Write(DRV_WIFI_WID_DGC_RSSI_TH_LOW,val,4);
    }
    else if((argc == 3) && (!strcmp((char*)argv[1],"--bdth")))
    {
        unsigned int val = strtoul( argv[2], 0, 16);
        DRV_WIFI_Write(DRV_WIFI_WID_BD_TH_SET,val,4);
    }
    else if((argc ==2) && (!strcmp((char*)argv[1],"--getrssi")))
    {   
        char buff[10];
        DRV_WIFI_Read(DRV_WIFI_WID_RSSI,buff);
    }
    else if((argc == 3) && (!strcmp((char*)argv[1],"--otaaddr")))
    {
        uint32_t ota_addr = strtoul( argv[2], 0, 16);
        if((WIFI_IPF_secureregion_sector_get() != IPF_SECTOR_2)||(WIFI_IPF_secureregion_read_status())){
            SYS_CONSOLE_PRINT("\n Enter valid command \n");   
            return 0;
        } 
        uint32_t* ota_addr_p= WIFI_IPF_secureregion_otaaddroffset();
        *ota_addr_p = ota_addr;
    }
    else if((argc >=3) && (!strcmp((char*)argv[1],"--saddr")))
    {
        unsigned char mac_t[6],Idx=0,*parg=(unsigned char *)argv[2];
        unsigned char option = (argc >=4) ? strtoul( argv[3], 0, 10):0;
        for(Idx=0;Idx<6;Idx++){
            mac_t[Idx] = strtoul((const char *)parg,(char **)&parg, 16);
            parg++;
        }
        
        if(option== 1)
        {
            if((WIFI_IPF_secureregion_sector_get() != IPF_SECTOR_1) || (WIFI_IPF_secureregion_read_status()))
            {    
                SYS_CONSOLE_PRINT("\n Enter valid command ");   
                return 0;
            }
         
         char *mac_addr = WIFI_IPF_secureregion_macoffset();
         memcpy(mac_addr,mac_t,MACADDRESS_LENGTH);
        } 
        else if(option ==0 )
        {
            DRV_WIFI_Write(DRV_WIFI_WID_MAC_ADDR,(unsigned int)mac_t,MACADDRESS_LENGTH);
        }
    }
#ifdef WLAN_REGDOMAIN_DEBUG    
    else if ((argc >= 3) && ((WIFI_IPF_secureregion_sector_get() == IPF_SECTOR_2) && (!WIFI_IPF_secureregion_read_status())) && (!strcmp((char*)argv[1],"pow"))) {
        char *parg=NULL;
        unsigned char *configptr=NULL,flage=1,size=0,Inx = 0;
        wifiRegdomainConfig *g_wifiRegdomainConfig=WIFI_IPF_secureregion_regdomoffset();
        if((argv[2]) && (!strcmp((char*)argv[2],"print"))) {
            printf("\n ChannelBitmap5Ghz =%d\nChannelBitmap2Ghz=%d\nCountryCode=%s\n",g_wifiRegdomainConfig->ChannelBitmap5Ghz,g_wifiRegdomainConfig->ChannelBitmap2Ghz,g_wifiRegdomainConfig->CountryCode);
            printf("\n Enable2g2040 =%d ",g_wifiRegdomainConfig->Enable2g2040);

            printf("\n Txpow_2G_20_US =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_20_US);Inx++)
            {
                printf("%d ",(unsigned char)g_wifiRegdomainConfig->Txpow_2G_20_US[Inx]);
            }

            printf("\n Txpow_2G_BB_US =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_US);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_BB_US[Inx]);
            }

            printf("\n Txpow_2G_40_US =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_40_US);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_40_US[Inx]);
            }

            printf("\n Txpow_5G_20_US =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_20_US);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_20_US[Inx]);
            }

            printf("\n Txpow_5G_40_US =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_40_US);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_40_US[Inx]);
            }

            printf("\n Txpow_2G_20_EU =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_20_EU);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_20_EU[Inx]);
            }

            printf("\n Txpow_2G_BB_EU =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_EU);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_BB_EU[Inx]);
            }

            printf("\n Txpow_2G_40_EU =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_40_EU);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_40_EU[Inx]);
            }

            printf("\n Txpow_2G_40_EU =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_20_EU);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_20_EU[Inx]);
            }

            printf("\n Txpow_5G_40_EU =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_40_EU);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_40_EU[Inx]);
            }

            printf("\n Txpow_2G_20_JP =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_20_JP);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_20_JP[Inx]);
            }

            printf("\n Txpow_2G_BB_JP =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_JP);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_BB_JP[Inx]);
            }

            printf("\n Txpow_2G_BB_JP =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_40_JP);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_40_JP[Inx]);
            }

            printf("\n Txpow_2G_BB_JP =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_20_JP);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_20_JP[Inx]);
            }

            printf("\n Txpow_5G_40_JP =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_40_JP);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_40_JP[Inx]);
            }

            printf("\n Txpow_2G_20_GN =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_20_GN);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_20_GN[Inx]);
            }

            printf("\n Txpow_2G_BB_GN =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_GN);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_BB_GN[Inx]);
            }

            printf("\n Txpow_2G_40_GN =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_40_GN);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_40_GN[Inx]);
            }

            printf("\n Txpow_5G_20_GN =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_20_GN);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_20_GN[Inx]);
            }

            printf("\n Txpow_5G_40_GN =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_40_GN);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_40_GN[Inx]);
            }

            printf("\n Txpow_2G_20_CUST =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_20_CUST);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_20_CUST[Inx]);
            }

            printf("\n Txpow_2G_BB_CUST =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_CUST);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_BB_CUST[Inx]);
            }

            printf("\n Txpow_2G_40_CUST =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_2G_40_CUST);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_2G_40_CUST[Inx]);
            }

            printf("\n Txpow_5G_20_CUST =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_20_CUST);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_20_CUST[Inx]);
            }

            printf("\n Txpow_5G_40_CUST =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->Txpow_5G_40_CUST);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->Txpow_5G_40_CUST[Inx]);
            }

            printf("\n Txpow_ANT_GAIN_2G =%d ",g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G);
            printf("\n Txpow_ANT_GAIN_5G =%d ",g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G);

            printf("\n RxpowAdj_2G_20 =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->RxpowAdj_2G_20);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->RxpowAdj_2G_20[Inx]);
            }

            printf("\n RxpowAdj_2G_40 =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->RxpowAdj_2G_40);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->RxpowAdj_2G_40[Inx]);
            }

            printf("\n RxpowAdj_5G_20 =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->RxpowAdj_5G_20);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->RxpowAdj_5G_20[Inx]);
            }

            printf("\n RxpowAdj_5G_40 =");
            for(Inx = 0;Inx < sizeof(g_wifiRegdomainConfig->RxpowAdj_5G_40);Inx++)
            {
                printf("%d ",g_wifiRegdomainConfig->RxpowAdj_5G_40[Inx]);
            }
			 return 0;
        } else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_20_US"))) {
            configptr = g_wifiRegdomainConfig->Txpow_2G_20_US; 
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_20_US);
        } else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_BB_US"))) {
            configptr = g_wifiRegdomainConfig->Txpow_2G_BB_US;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_US);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_40_US"))) {
            configptr = g_wifiRegdomainConfig->Txpow_2G_40_US;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_40_US);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_20_US"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_20_US;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_20_US);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_40_US"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_40_US;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_40_US);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_20_EU"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_20_EU;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_20_EU);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_BB_EU"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_BB_EU;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_EU);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_40_EU"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_40_EU;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_40_EU);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_20_EU"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_20_EU;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_20_EU);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_40_EU"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_40_EU;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_40_EU);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_20_JP"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_20_JP;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_20_JP);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_BB_JP"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_BB_JP;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_JP);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_40_JP"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_40_JP;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_40_JP);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_20_JP"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_20_JP;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_20_JP);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_40_JP"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_40_JP;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_40_JP);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_20_GN"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_20_GN;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_20_GN);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_BB_GN"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_BB_GN;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_GN);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_40_GN"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_40_GN;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_40_GN);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_20_GN"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_20_GN;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_20_GN);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_40_GN"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_40_GN;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_40_GN);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_20_CUST"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_20_CUST;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_20_CUST);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_BB_CUST"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_BB_CUST;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_BB_CUST);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx2G_40_CUST"))){
            configptr = g_wifiRegdomainConfig->Txpow_2G_40_CUST;
			size = sizeof(g_wifiRegdomainConfig->Txpow_2G_40_CUST);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_20_CUST"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_20_CUST;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_20_CUST);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"tx5G_40_CUST"))){
            configptr = g_wifiRegdomainConfig->Txpow_5G_40_CUST;
			size = sizeof(g_wifiRegdomainConfig->Txpow_5G_40_CUST);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"txANT_GAIN_2G"))){
            g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G = strtoul( argv[3], 0, 10);
            return 0;
        }else if((argv[2]) && (!strcmp((char*)argv[2],"txANT_GAIN_5G"))){
            g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G = strtoul( argv[3], 0, 10);
            return 0;
        }else if((argv[2]) && (!strcmp((char*)argv[2],"rx2G_20"))){
            configptr = g_wifiRegdomainConfig->RxpowAdj_2G_20;
			size = sizeof(g_wifiRegdomainConfig->RxpowAdj_2G_20);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"rx2G_40"))){
            configptr = g_wifiRegdomainConfig->RxpowAdj_2G_40;
			size = sizeof(g_wifiRegdomainConfig->RxpowAdj_2G_40);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"rx5G_20"))){
            configptr = g_wifiRegdomainConfig->RxpowAdj_5G_20;
			size = sizeof(g_wifiRegdomainConfig->RxpowAdj_5G_20);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"rx5G_40"))){    
            configptr = g_wifiRegdomainConfig->RxpowAdj_5G_40;
			size = sizeof(g_wifiRegdomainConfig->RxpowAdj_5G_40);
        }else if((argv[2]) && (!strcmp((char*)argv[2],"save"))){
            WIFI_IPF_save_regdom();             
	        return 0;
        }else if((argv[2]) && (argv[3]) &&(!strcmp((char*)argv[2],"chnbitmap5"))){ 
            g_wifiRegdomainConfig->ChannelBitmap5Ghz= strtoul( argv[3], 0, 10);
            return 0;
        }else if((argv[2]) && (argv[3]) &&(!strcmp((char*)argv[2],"chnbitmap2"))){
            g_wifiRegdomainConfig->ChannelBitmap2Ghz= strtoul( argv[3], 0, 10);
            return 0;
        }else if(((argv[2]) && (argv[3])) &&((!strcmp((char*)argv[2],"contrycode")) || (!strcmp((char*)argv[2],"countrycode")))){
            strcpy((char *)g_wifiRegdomainConfig->CountryCode,argv[3]);
            return 0;
        }else if((argv[2]) && (argv[3]) &&(!strcmp((char*)argv[2],"freqband"))){
            g_wifiRegdomainConfig->FreqBand= strtoul( argv[3], 0, 10);
            return 0;
        }else if((argv[2]) && (argv[3]) &&(!strcmp((char*)argv[2],"enable2g2040"))){
            g_wifiRegdomainConfig->Enable2g2040= strtoul( argv[3], 0, 10);
            return 0;
        } 
        
        if((size==0) && (configptr == NULL)){
            printf("\n\r *** Command Processor: unknown command. ***\r\n");
            return 0;
        }
        if(flage && (argc>=4))
        {
            parg=argv[3];
            Inx =0;
            do {
                    configptr[Inx++] = strtol(parg, &parg, 10);
                    parg++;
					size --;
                    
            } while((*parg!=NULL) && (size!=0));
        }
    }    
#endif    
#if 000 //test code disable    
       else if(!strcmp((char*)argv[1],"test1")) {
		SYS_CONSOLE_MESSAGE("test1");
		DRV_WIFI_Write(DRV_WIFI_WID_SSID,"NETGEAR");
		DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,0x31);
		DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,"microchip");
		DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE,0);
	} else if(!strcmp((char*)argv[1],"test2")) {
		SYS_CONSOLE_MESSAGE("test2");
		DRV_WIFI_Write(DRV_WIFI_WID_SSID,"NETGEAR");
		DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,"microchip");
		DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,0x79);
		DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE,0);
	} else if(!strcmp((char*)argv[1],"test3")) {
			SYS_CONSOLE_MESSAGE("test3");
			DRV_WIFI_Write(DRV_WIFI_WID_SSID,argv[2]);
			DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,argv[3]);
			DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,strtoul( argv[3], 0, 16));
			DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE,0);
	} else if(!strcmp((char*)argv[1],"widhex")) {
		SYS_CONSOLE_MESSAGE("widhex");
		DRV_WIFI_Write(strtoul( argv[2], 0, 16),strtoul( argv[3], 0, 16));
	} else if(!strcmp((char*)argv[1],"widdec")) {
		SYS_CONSOLE_MESSAGE("widdec");
		DRV_WIFI_Write(strtoul( argv[2], 0, 16),strtoul( argv[3], 0, 10));
	}
#endif    
    else {
		SYS_CONSOLE_MESSAGE("*** Command Processor: unknown command. ***");
		ret = 1;
	}
	
	return ret;
}

