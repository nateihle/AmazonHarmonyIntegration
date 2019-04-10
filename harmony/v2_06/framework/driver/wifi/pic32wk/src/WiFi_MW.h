/*******************************************************************************
  File Name:
    WiFi_MW.h

  Summary:
	This file is to process the commands and trigger the corresponding API's in
	WiFi library.
	
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

#ifndef WiFi_MW_H
#define	WiFi_MW_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
    unsigned char             bootMode;         //Save data in Flash is valid or not valid
    unsigned char             ssid[32+1];        // 32 for SSID and one for term character
    unsigned char             type;              // net type: infrastructure/adhoc
    unsigned char             prevSSID[32+1];    // previous SSID we were connected to
    unsigned char             prevWLAN;          // previous WLAN type

    unsigned char             security;          // security type   
    unsigned char             cipher;            //cipher
    unsigned char             defaultWepKey;     // WEP index value
    unsigned char             key[64+1];         // 64-byte key plus term character
    unsigned char             SecurityKeyLength; // number of bytes in security key (can be 0)

} wifiConfigData;

typedef struct {
    unsigned char             ChannelBitmap5Ghz;    // channel bit maps for 5GHZ
    unsigned char             ChannelBitmap2Ghz;    // channel bit maps for 2.4GHZ
    unsigned char             CountryCode[5];       // coutry code
    unsigned char             FreqBand;             // frequency band
    unsigned char             Enable2g2040;         // enable 2.4GHZ 20MHZ and 40MHZ
    
    unsigned char             Txpow_2G_20_US[14];       //Tx power for 2.4GHZ 20MHZ USA 
    unsigned char             Txpow_2G_BB_US[14];       //Tx power for 2.4GHZ BB USA 
    unsigned char             Txpow_2G_40_US[11];       //Tx power for 2.4GHZ 40MHZ USA 
    unsigned char             Txpow_5G_20_US[28];       //Tx power for 5GHZ 20MHZ USA 
    unsigned char             Txpow_5G_40_US[11];       //Tx power for 5GHZ 40MHZ USA 
    unsigned char             Txpow_2G_20_EU[14];       //Tx power for 2.4GHZ 20MHZ Europe 
    unsigned char             Txpow_2G_BB_EU[14];       //Tx power for 2.4GHZ BB Europe 
    unsigned char             Txpow_2G_40_EU[11];       //Tx power for 2.4GHZ 40MHZ Europe 
    unsigned char             Txpow_5G_20_EU[28];       //Tx power for 5GHZ 20MHZ Europe 
    unsigned char             Txpow_5G_40_EU[11];       //Tx power for 5GHZ 40MHZ Europe 
    unsigned char             Txpow_2G_20_JP[14];       //Tx power for 2.4GHZ 20MHZ Japan 
    unsigned char             Txpow_2G_BB_JP[14];       //Tx power for 2.4GHZ BB Japan
    unsigned char             Txpow_2G_40_JP[11];       //Tx power for 2.4GHZ 40MHZ Japan
    unsigned char             Txpow_5G_20_JP[28];       //Tx power for 5GHZ 20MHZ Japan
    unsigned char             Txpow_5G_40_JP[11];       //Tx power for 5GHZ 40MHZ Japan
    unsigned char             Txpow_2G_20_GN[14];       //Tx power for 2.4GHZ 20MHZ General
    unsigned char             Txpow_2G_BB_GN[14];       //Tx power for 2.4GHZ BB General
    unsigned char             Txpow_2G_40_GN[11];       //Tx power for 2.4GHZ 40MHZ General
    unsigned char             Txpow_5G_20_GN[28];       //Tx power for 5GHZ 20MHZ General
    unsigned char             Txpow_5G_40_GN[11];       //Tx power for 5GHZ 40MHZ General
    unsigned char             Txpow_2G_20_CUST[14];     //Tx power for 2.4GHZ 20MHZ customize
    unsigned char             Txpow_2G_BB_CUST[14];     //Tx power for 2.4GHZ BB customize
    unsigned char             Txpow_2G_40_CUST[11];     //Tx power for 2.4GHZ 40MHZ customize
    unsigned char             Txpow_5G_20_CUST[28];     //Tx power for 5GHZ 20MHZ customize
    unsigned char             Txpow_5G_40_CUST[11];     //Tx power for 5GHZ 40MHZ customize
    unsigned char             Txpow_ANT_GAIN_2G;        //Tx power antenna gain for 2.4GHZ
    unsigned char             Txpow_ANT_GAIN_5G;        //Tx power antenna gain for 5GHZ
    
    unsigned char             RxpowAdj_2G_20[14];       //Rx power Adj for 2.4GHZ 20MHZ 
    unsigned char             RxpowAdj_2G_40[11];       //Rx power Adj for 2.4GHZ 40MHZ 
    unsigned char             RxpowAdj_5G_20[28];       //Rx power Adj for 5GHZ 20MHZ 
    unsigned char             RxpowAdj_5G_40[11];       //Rx power Adj for 5GHZ 40MHZ 
} wifiRegdomainConfig;

#define WifiConfig_Valid        0x41 //Magic number used in set/get valid Flash data

//****************************************************************************
/*  
  Function:
    bool wifi_command_process();

  Summary:
	This function process the WiFi commands received from user
	
  Description:
	This API can be used in processing external events from the other
	sources(external)

  Precondition:
    None. 

  Parameters:
    None.	
		
  Returns:
    - On Success - true
    - On Failure - false
    
  Remarks:
    None.
*/
bool wifi_command_process(int argc,char *argv[]);

#ifdef	__cplusplus
}
#endif

#endif	/* WiFi_MW_H */

