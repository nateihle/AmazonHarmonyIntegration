/**
  ECC Crypto Authentication Configuration

  @Company
    Microchip Technology Inc.

  @File Name
    ecc_atca_configure.c

  @Summary
    This file implements basic communication and authentication with the ECC device
	
  @Author
	C40249
*/

/*
    (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/ 

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include "ecc_atca_configure.h"
#include "ecc_asymmetric_app.h"
#include "cryptoauthlib.h"


/***** 508A XML configuration **********************

<ConfigurationZone>
  <SN01>01 23</SN01>
  <SN8>EE</SN8>
  <I2CEnable>69</I2CEnable>
  <I2CAddress>C0</I2CAddress>
  <Reserved Address="17" Size="1">00</Reserved>
  <OTPMode>55</OTPMode>
  <ChipMode>00</ChipMode>
  <SlotConfigurations Size="16">

    <!-- Unique Key -->
    <SlotConfiguration Index="0">8F 8F</SlotConfiguration>

    <!-- PSK -->
    <SlotConfiguration Index="1">8F 8F</SlotConfiguration>

    <!-- Transport Key -->
    <SlotConfiguration Index="2">8F 8F</SlotConfiguration>

    <!-- User Programmable Key -->
    <SlotConfiguration Index="3">8F 42</SlotConfiguration>

    <!-- Private Key -->
    <SlotConfiguration Index="4">8F 0F</SlotConfiguration>

    <!-- ECDH Value -->
    <SlotConfiguration Index="5">C2 8F</SlotConfiguration>
    <SlotConfiguration Index="6">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="7">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="8">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="9">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="A">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="B">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="C">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="D">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="E">0F 0F</SlotConfiguration>
    <SlotConfiguration Index="F">9F 8F</SlotConfiguration>
  </SlotConfigurations>
  <Counter0 Size="8">FF FF FF FF 00 00 00 00</Counter0>
  <Counter1 Size="8">FF FF FF FF 00 00 00 00</Counter1>
  <LastKeyUse Size="16">FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF</LastKeyUse>
  <UserExtra>00</UserExtra>
  <Selector>00</Selector>
  <SlotLocked>FF FF</SlotLocked>
  <Reserved Address="90" Size="2">00 00</Reserved>
  <X509format>00 00 00 00</X509format>
  <KeyConfigurations Size="16">
    <KeyConfiguration Index="0">1E 00</KeyConfiguration>
    <KeyConfiguration Index="1">1E 00</KeyConfiguration>
    <KeyConfiguration Index="2">1E 00</KeyConfiguration>
    <KeyConfiguration Index="3">1C 00</KeyConfiguration>
    <KeyConfiguration Index="4">13 00</KeyConfiguration>
    <KeyConfiguration Index="5">5C 00</KeyConfiguration>
    <KeyConfiguration Index="6">1C 00</KeyConfiguration>
    <KeyConfiguration Index="7">1C 00</KeyConfiguration>
    <KeyConfiguration Index="8">1C 00</KeyConfiguration>
    <KeyConfiguration Index="9">1C 00</KeyConfiguration>
    <KeyConfiguration Index="A">1C 00</KeyConfiguration>
    <KeyConfiguration Index="B">1C 00</KeyConfiguration>
    <KeyConfiguration Index="C">1C 00</KeyConfiguration>
    <KeyConfiguration Index="D">1C 00</KeyConfiguration>
    <KeyConfiguration Index="E">1C 00</KeyConfiguration>
    <KeyConfiguration Index="F">1C 00</KeyConfiguration>
  </KeyConfigurations>
</ConfigurationZone>
*/
 
bool ECC_ATCA_CONFIGURE_WriteConfig(ATCAIfaceCfg *cfg) 
{
	ATCA_STATUS status = ATCA_GEN_FAIL;	
	bool islocked;	
    uint8_t data[128];
    
	//generated C HEX from javascript config tool
	const uint8_t ecc508_config_lab[] = 
    {        
		0x01, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
        0x00, 0x00, 0x00, 0x00, 0xEE, 0x00, 0x69, 0x00,
		cfg->atcai2c.slave_address, 0x00, 0x55, 0x00, 0x8F, 0x8F, 0x8F, 0x8F,  
        0x8F, 0x8F, 0x8F, 0x42, 0x8F, 0x0F, 0xC2, 0x8F,
		0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,  
        0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
		0x0F, 0x0F, 0x9F, 0x8F, 0xFF, 0xFF, 0xFF, 0xFF,  
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
		0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,  
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,  
        0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x1E, 0x00, 0x1E, 0x00, 0x1E, 0x00, 0x1C, 0x00,  
        0x13, 0x00, 0x5C, 0x00, 0x1C, 0x00, 0x1C, 0x00,
		0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00,  
        0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00	
	};		
	
	SYS_PRINT("--Writing Configuration--\r\n");	
    status = atcab_init( cfg );    
	while(status != ATCA_SUCCESS);
	
	status = atcab_read_config_zone((uint8_t*)&data);
	while(status != ATCA_SUCCESS);
	        
    ECC_ASYMMETRIC_APP_PrintBytes((uint8_t*)&data, 128);    
    
    status = atcab_is_locked(LOCK_ZONE_CONFIG, &islocked);
	while(status != ATCA_SUCCESS);	
	if(islocked) 
    {
		SYS_PRINT("ATCA already configured\r\n");
		return false;
	}
	
	status = atcab_is_locked(LOCK_ZONE_DATA, &islocked);	
	while(status != ATCA_SUCCESS);	
	if(islocked) 
    {
		SYS_PRINT("ATCA - Error\r\n");
		while(status != ATCA_SUCCESS);
	}
    
	status = atcab_write_config_zone(ecc508_config_lab);
	while(status != ATCA_SUCCESS);
	SYS_PRINT("Configuration Write Complete\r\n");	
    
    SYS_PRINT("Locking Configuration Zone..\r\n");
	status = atcab_lock_config_zone();
	while(status != ATCA_SUCCESS);	
    
	SYS_PRINT("Configuration Zone Lock Complete\r\n");	
	return true;	
}

void ECC_ATCA_CONFIGURE_WriteData(void) 
{
    uint8_t pubkey[64];
	ATCA_STATUS status = ATCA_GEN_FAIL;		
	const uint8_t key0[] = 
    {		
		0xf0, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x0f,
		
	};

	const uint8_t key1[] = 
    {		
		0xf1, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x1f,
		
	};

	const uint8_t key2[] = 
    {		
		0xf2, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x2f,
		
	};
	
	const uint8_t key3[] = 
    {		
		0xf3, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x3f,
		
	};	
		
	SYS_PRINT("--Writing Data Zone--\r\n");	
	status = atcab_write_bytes_zone(ATCA_ZONE_DATA, 0, 0, key0, 32);
	while(status != ATCA_SUCCESS);
	
	status = atcab_write_bytes_zone(ATCA_ZONE_DATA, 1, 0, key1, 32);
	while(status != ATCA_SUCCESS);
	
	status = atcab_write_bytes_zone(ATCA_ZONE_DATA, 2, 0, key2, 32);
	while(status != ATCA_SUCCESS);
	
	status = atcab_write_bytes_zone(ATCA_ZONE_DATA, 3, 0, key3, 32);
	while(status != ATCA_SUCCESS);	
	
	status = atcab_genkey(4, pubkey);
	while(status != ATCA_SUCCESS);    
	SYS_PRINT("Data Zone Write Complete\r\n");
	
	SYS_PRINT("Locking Data Zone..\r\n");
	status = atcab_lock_data_zone();
	while(status != ATCA_SUCCESS);
	
	SYS_PRINT("Data Zone Lock Complete\r\n");		
}