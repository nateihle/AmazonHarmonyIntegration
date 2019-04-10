/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    ecc_asymmetric_app.c

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
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "ecc_asymmetric_app.h"
#include "ecc_atca_configure.h"

#define ECC_ASYMMETRIC_APP_READ_BUFFER_SIZE                    64
#define ECC_ASYMMETRIC_APP_WRITE_BUFFER_SIZE                   64

#define ECC_ASYMMETRIC_APP_CHECK_STATUS(s)						\
	if(s != ATCA_SUCCESS) {										\
		SYS_PRINT("Error: Line %d in %s\r\n", __LINE__, __FILE__); \
		while(1);												\
	}															\
																\
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
extern DRV_I2C_INIT drvI2C0InitData;
extern DRV_I2C_INIT drvI2C1InitData;
extern SYSTEM_OBJECTS sysObj;

ECC_ASYMMETRIC_APP_DATA ecc_asymmetric_appData;

DRV_I2C_Object i2cObj0 = 
{
    .i2cDriverInstanceIndex = DRV_I2C_INDEX_0,
    .i2cDriverInit = &drvI2C0InitData
};

DRV_I2C_Object i2cObj1 = 
{
    .i2cDriverInstanceIndex = DRV_I2C_INDEX_1,
    .i2cDriverInit = &drvI2C1InitData    
};

ATCAIfaceCfg cfg_ateccx08a_i2c_host =
{
    .iface_type = ATCA_I2C_IFACE,
    .devtype = ATECC608A,
    .atcai2c.slave_address = 0xC0,
    .atcai2c.bus = 2,
    .atcai2c.baud = 100000,
    .wake_delay = 1500,
    .rx_retries = 20,
    .cfg_data = (void *) &i2cObj0    

};

ATCAIfaceCfg cfg_ateccx08a_i2c_remote =
{
    .iface_type = ATCA_I2C_IFACE,
    .devtype = ATECC608A,
    .atcai2c.slave_address = 0xC0,
    .atcai2c.bus = 2,
    .atcai2c.baud = 100000,
    .wake_delay = 1500,
    .rx_retries = 20,
    .cfg_data = (void *) &i2cObj1    
};

typedef struct {
	uint8_t pub_key[64];
} asymm_public_key_t;

asymm_public_key_t key_store[4] =
{
    
}; //cut and paste in remote public key


static char ecc_asymmetric_rdBuf[ECC_ASYMMETRIC_APP_READ_BUFFER_SIZE] SYS_DEBUG_BUFFER_DMA_READY;
static char ecc_asymmetric_tmpWrBuf1[2] SYS_DEBUG_BUFFER_DMA_READY;
static char ecc_asymmetric_tmpWrBuf2[2] SYS_DEBUG_BUFFER_DMA_READY;
static char ecc_asymmetric_tmpWrBuf3[2] SYS_DEBUG_BUFFER_DMA_READY;
static char ecc_asymmetric_tmpWrBuf4[2] SYS_DEBUG_BUFFER_DMA_READY;
static bool ecc_asymmetric_repaintMenu;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
static void _ECC_ASYMMETRIC_APP_ReadComplete (void *handle);
static void _ECC_ASYMMETRIC_APP_WriteComplete (void *handle);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
static void _ECC_ASYMMETRIC_APP_Handle_Authentication (void);
static void _ECC_ASYMMETRIC_APP_Handle_Configuration (void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ECC_ASYMMETRIC_APP_Initialize ( void )

  Remarks:
    See prototype in ecc_asymmetric_app.h.
 */

void ECC_ASYMMETRIC_APP_Initialize ( void )
{
    DRV_I2C_Object *tempObj0, *tempObj1;
    
    /* Place the App state machine in its initial state. */
    ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_INIT;

    ecc_asymmetric_appData.rdComplete = false;
    ecc_asymmetric_appData.wrComplete = false;
    ecc_asymmetric_repaintMenu = false;

    tempObj0 = (DRV_I2C_Object *) cfg_ateccx08a_i2c_host.cfg_data;
    tempObj0->i2cDriverInstance = *(&sysObj.drvI2C0);
    
    tempObj1 = (DRV_I2C_Object *) cfg_ateccx08a_i2c_remote.cfg_data;
    tempObj1->i2cDriverInstance = *(&sysObj.drvI2C1);
    
    BSP_LEDOff(BSP_RGB_LED_RED);
    BSP_LEDOff(BSP_RGB_LED_GREEN);
    BSP_LEDOff(BSP_RGB_LED_BLUE);
    
    BSP_LEDOff(BSP_LED_1);
    BSP_LEDOff(BSP_LED_2);
    BSP_LEDOn(BSP_LED_3);    
}

/******************************************************************************
  Function:
    void ECC_ASYMMETRIC_APP_Tasks ( void )

  Remarks:
    See prototype in ecc_asymmetric_app.h.
 */

void ECC_ASYMMETRIC_APP_Tasks(void)
{
    /* Check the application's current state. */
    switch (ecc_asymmetric_appData.state)
    {
        /* Application's initial state. */
        case ECC_ASYMMETRIC_APP_STATE_INIT:
        {
            bool appInitialized = false;

            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0,
                        _ECC_ASYMMETRIC_APP_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0,
                        _ECC_ASYMMETRIC_APP_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                SYS_PRINT("======================================================\r\n");
                SYS_PRINT("******ATECC608A Security on PIC32MZ EF Curiosity******\r\n");
                SYS_PRINT("CryptoAuthLib: Asymmetric Authentication of a Remote Device\r\n");
                SYS_PRINT("\r\n");
                appInitialized = true;
            }

            if (appInitialized)
            {
                ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_PRINT_MENU;
            }

        }
        break;

        case ECC_ASYMMETRIC_APP_STATE_PRINT_MENU:
        {
            BSP_LEDOff(BSP_LED_1);
            BSP_LEDOff(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);
            SYS_PRINT("Select a mode of operation and press Return Key \r\n");
            SYS_PRINT("1. Configuration Mode\r\n");
            SYS_PRINT("2. Authentication Mode");
            strcpy(ecc_asymmetric_tmpWrBuf1, "\r\n");
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, ecc_asymmetric_tmpWrBuf1, 2);
            ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_WAIT;
        }
        break;

        case ECC_ASYMMETRIC_APP_STATE_MODE_WAIT:
        {
            if (ecc_asymmetric_appData.wrComplete)
            {
                ecc_asymmetric_appData.wrComplete = false;
                ecc_asymmetric_appData.rdComplete = false;
                ecc_asymmetric_appData.bytesRead = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0,
                        STDIN_FILENO, ecc_asymmetric_rdBuf, 2);
            }
            if (ecc_asymmetric_appData.rdComplete)
            {
                if (ecc_asymmetric_repaintMenu == true)
                {
                    ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_PRINT_MENU;
                    ecc_asymmetric_repaintMenu = false;
                }
                else
                {
                    if (ecc_asymmetric_rdBuf[0] == '1')
                    {
                        SYS_PRINT("\r\n------In Configuration Mode------\r\n");
                        ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_CONFIGURATION;
                    } else if (ecc_asymmetric_rdBuf[0] == '2')
                    {
                        SYS_PRINT("\r\n------In Authentication Mode------\r\n");
                        ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_AUTHENTICATION;
                    }
                    else
                    {
                        ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_WAIT;
                        SYS_PRINT("\r\nInvalid Input, Try Again ");
                        strcpy(ecc_asymmetric_tmpWrBuf2, "\r\n");
                        SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO,
                                ecc_asymmetric_tmpWrBuf2, 2);
                    }
                }
                ecc_asymmetric_appData.rdComplete = false;
            }
        }
        break;

        case ECC_ASYMMETRIC_APP_STATE_MODE_CONFIGURATION:
        {
            _ECC_ASYMMETRIC_APP_Handle_Configuration();
            ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_IDLE;
        }
        break;

        case ECC_ASYMMETRIC_APP_STATE_MODE_AUTHENTICATION:
        {
            _ECC_ASYMMETRIC_APP_Handle_Authentication();
            ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_IDLE;
        }
        break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
        }
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
static void _ECC_ASYMMETRIC_APP_Handle_Authentication(void)
{

    volatile ATCA_STATUS status;
    uint8_t nonce[32];
    uint8_t signature[64];
    uint8_t slot = 0;
    uint8_t temp_pubk[64];

    /*Initialize HOST ECC */
    status = atcab_init( &cfg_ateccx08a_i2c_host );
    ECC_ASYMMETRIC_APP_CHECK_STATUS(status);

    /*Generating an NONCE */
    status = atcab_random((uint8_t*)&nonce);
    ECC_ASYMMETRIC_APP_CHECK_STATUS(status);
    SYS_PRINT("Random from host\r\n");
    ECC_ASYMMETRIC_APP_PrintBytes((uint8_t*)&nonce, 32);
    atca_delay_ms(100);
    
    /*Initialize Remote ECC */
    status = atcab_init( &cfg_ateccx08a_i2c_remote );
    ECC_ASYMMETRIC_APP_CHECK_STATUS(status);

    slot=4;

    status = atcab_sign(slot, (const uint8_t*)&nonce, (uint8_t*)&signature);
	ECC_ASYMMETRIC_APP_CHECK_STATUS(status);
	SYS_PRINT("Signature from remote\r\n");
	ECC_ASYMMETRIC_APP_PrintBytes((uint8_t*)&signature, 64);
    atca_delay_ms(100);
    
	status = atcab_get_pubkey(slot, (uint8_t *)&temp_pubk);
	ECC_ASYMMETRIC_APP_CHECK_STATUS(status);
	SYS_PRINT("Remote disposable public key\r\n");
	ECC_ASYMMETRIC_APP_PrintBytes((uint8_t*)&temp_pubk, 64);
    atca_delay_ms(100);

    status = atcab_init( &cfg_ateccx08a_i2c_host );
	ECC_ASYMMETRIC_APP_CHECK_STATUS(status);
	bool verify = false;
	bool key_found = false;
	uint8_t i = 0;

    for(;i < sizeof(key_store)/sizeof(asymm_public_key_t); i++)
    {
		if(memcmp(&key_store[i], &temp_pubk, 64) == 0)
        {
            key_found = true;
        	break;
		}
	}
	if(key_found)
    {
		status = atcab_verify_extern((const uint8_t*)&nonce,
                                    (const uint8_t*)&signature,
                                    (const uint8_t*)key_store[i].pub_key,
                                    &verify);
	}
    else
    {
        SYS_PRINT("Remote's Public Key not found in Host's Database \r\n\r\n");
    }

    if(verify)
    {
        BSP_LEDOn(BSP_LED_2);
        SYS_PRINT("Verified Signature - Authentication Successful!\r\n\r\n");
    }
    else
    {
        BSP_LEDOn(BSP_LED_1);
        SYS_PRINT("Signature not verified - Authentication Failure!\r\n\r\n");
    }

    SYS_PRINT("Press a key followed by 'enter' key to return to Options Menu \r\n");
    strcpy(ecc_asymmetric_tmpWrBuf4,"\r\n");
    SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, ecc_asymmetric_tmpWrBuf4, 2);
    ecc_asymmetric_repaintMenu = true;
}


////////////////////////////////////////////////////////////////////////////////
static void _ECC_ASYMMETRIC_APP_Handle_Configuration(void)
{
    SYS_PRINT("Plug in host Secure 4 Click board in Mikro Bus Interface 1 and press S1\r\n");
    while (BSP_SWITCH_1StateGet() != BSP_SWITCH_STATE_PRESSED);
    if (ECC_ATCA_CONFIGURE_WriteConfig(&cfg_ateccx08a_i2c_host) == true)
    {
        ECC_ATCA_CONFIGURE_WriteData();
        SYS_PRINT("**Host board Configuration Done**\r\n");
    }

    atca_delay_ms(500);

    SYS_PRINT("Plug in remote Secure 4 Click board in Mikro Bus Interface 2 and press S1\r\n");
    while (BSP_SWITCH_1StateGet() != BSP_SWITCH_STATE_PRESSED);
    if (ECC_ATCA_CONFIGURE_WriteConfig(&cfg_ateccx08a_i2c_remote) == true)
    {
        ECC_ATCA_CONFIGURE_WriteData();
        SYS_PRINT("**Remote board Configuration Done**\r\n");
    }

    atca_delay_ms(500);

    SYS_PRINT("Press a key followed by 'enter' key to return to Options Menu \r\n");
    strcpy(ecc_asymmetric_tmpWrBuf3,"\r\n");
    SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, ecc_asymmetric_tmpWrBuf3, 2);
    ecc_asymmetric_repaintMenu = true;
}

////////////////////////////////////////////////////////////////////////////////
static void _ECC_ASYMMETRIC_APP_ReadComplete (void *handle)
{
    ecc_asymmetric_appData.rdComplete = true;

}

////////////////////////////////////////////////////////////////////////////////
static void _ECC_ASYMMETRIC_APP_WriteComplete (void *handle)
{
    if((handle == ecc_asymmetric_tmpWrBuf1 || handle == ecc_asymmetric_tmpWrBuf2)
            && ecc_asymmetric_appData.state == ECC_ASYMMETRIC_APP_STATE_MODE_WAIT)
    {
        ecc_asymmetric_appData.wrComplete = true;
    }
    else if(handle == ecc_asymmetric_tmpWrBuf3 || handle == ecc_asymmetric_tmpWrBuf4)
    {
        ecc_asymmetric_appData.wrComplete = true;
        ecc_asymmetric_appData.state = ECC_ASYMMETRIC_APP_STATE_MODE_WAIT;
    }
    return;
}

////////////////////////////////////////////////////////////////////////////////
void ECC_ASYMMETRIC_APP_PrintBytes(uint8_t * ptr, uint8_t length)
{
    uint8_t i = 0;
    uint8_t line_count = 0;
    for (; i < length; i++)
    {
        SYS_PRINT("0x%02x, ", ptr[i]);
        line_count++;
        if (line_count == 8)
        {
            SYS_PRINT("\r\n");
            line_count = 0;
        }
    }
    SYS_PRINT("\r\n");
}
/*******************************************************************************
 End of File
 */
