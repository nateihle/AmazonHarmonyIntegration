/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "crypto/crypto.h"
#include "system/system.h"
#include "system/int/sys_int.h"
#include "driver/driver_common.h"
#include "system/devcon/sys_devcon.h"
#include "system/console/sys_console.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* Fulfill USB DMA transfer criteria */
#define APP_READ_BUFFER_SIZE                    64
#define APP_WRITE_BUFFER_SIZE                   64

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,

#if defined(RUN_FLASH_TEST)
                APP_STATE_RUN_MD5_BULK,

                APP_STATE_RUN_SHA1_BULK,

                APP_STATE_RUN_SHA256_BULK,

                APP_STATE_RUN_SHA384_BULK,

                APP_STATE_RUN_SHA512_BULK,
#endif
                APP_STATE_RUN_MD5_FEED,

                APP_STATE_RUN_SHA1_FEED,

                APP_STATE_RUN_SHA256_FEED,

                APP_STATE_RUN_SHA384_FEED,

                APP_STATE_RUN_SHA512_FEED,

                APP_STATE_DISPLAY_RESULTS,
                APP_STATE_WAIT_FOR_CONSOLE,
                APP_STATE_CHECK_RESULTS,

                APP_STATE_SPIN

} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    bool wrComplete;

    bool rdComplete;

    uint8_t md5_result[CRYPT_MD5_DIGEST_SIZE];
    uint32_t md5_timing;

    uint8_t sha1_result[CRYPT_SHA_DIGEST_SIZE];
    uint32_t sha1_timing;

    uint8_t sha256_result[CRYPT_SHA256_DIGEST_SIZE];
    uint32_t sha256_timing;

    uint8_t sha384_result[CRYPT_SHA384_DIGEST_SIZE];
    uint32_t sha384_timing;

    uint8_t sha512_result[CRYPT_SHA512_DIGEST_SIZE];
    uint32_t sha512_timing;

    uint8_t md5_feed_result[CRYPT_MD5_DIGEST_SIZE];
    uint32_t md5_feed_timing;

    uint8_t sha1_feed_result[CRYPT_SHA_DIGEST_SIZE];
    uint32_t sha1_feed_timing;

    uint8_t sha256_feed_result[CRYPT_SHA256_DIGEST_SIZE];
    uint32_t sha256_feed_timing;

    uint8_t sha384_feed_result[CRYPT_SHA384_DIGEST_SIZE];
    uint32_t sha384_feed_timing;

    uint8_t sha512_feed_result[CRYPT_SHA512_DIGEST_SIZE];
    uint32_t sha512_feed_timing;
    
    int32_t wallTime;

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

void SYS_Initialize ( void* data );
void SYS_Tasks ( void );
// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern APP_DATA appData;

#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

