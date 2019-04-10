/*******************************************************************************
  MPLAB Harmony Application

  Application Header

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
	Application definitions.

  Description:
	 This file contains the  application definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H

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
#include "system/int/sys_int.h"
#include "driver/nvm/drv_nvm.h"

#define KEEP              __attribute__ ((keep)) __attribute__((address(DRV_NVM_MEDIA_START_ADDRESS)))

/* The size of the media used for this demo. */
#define APP_NVM_MEMORY_AREA_SIZE (DRV_NVM_MEDIA_SIZE * 1024)

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* Enumeration of the Read, write and erase region geometry indices */
typedef enum {

    /* Read region index */
    APP_NVM_READ_REGION_INDEX = 0,

    /* Write region index */
    APP_NVM_WRITE_REGION_INDEX,

    /* Erase region index */
    APP_NVM_ERASE_REGION_INDEX

} APP_NVM_REGION_INDEX;

/* Enumeration listing the Erase operation sub-states */
typedef enum {

    /* Initialize variables for Erase operation */
    APP_ERASE_STATE_INIT = 0,

    /* Issue Erase command */
    APP_ERASE_STATE_ERASE_CMD,

    /* Check if the erase command operation is complete */
    APP_ERASE_STATE_ERASE_CMD_STATUS,

    /* Issue Read command */
    APP_ERASE_STATE_READ_CMD,

    /* Check if the read command operation is complete */
    APP_ERASE_STATE_READ_CMD_STATUS,

    /* Verify that the data is indeed erased */
    APP_ERASE_STATE_VERIFY_DATA,

    /* Erase and verification of data is successful */
    APP_ERASE_STATE_IDLE,

    /* Erase or verification of data is unsuccessful */
    APP_ERASE_STATE_ERROR

} APP_ERASE_STATES;

/* Enumeration listing the Sequential Read Write operation sub-states */
typedef enum {

    /* Initialize the variables for the Sequential
    Read Write operation */
    APP_SEQ_RW_INIT = 0,

    /* Issue Write Command */
    APP_SEQ_RW_WRITE,

    /* Check if the write command is complete */
    APP_SEQ_RW_WRITE_STATUS,

    /* Issue the Read Command */
    APP_SEQ_RW_READ,

    /* Check if the Read Command is complete and also
    verify the data */
    APP_SEQ_RW_READ_STATUS,

    /* Sequential Read Write operation is successful */
    APP_SEQ_RW_IDLE,

    /* Sequential Read Write operation is unsuccessful */
    APP_SEQ_RW_ERROR

} APP_SEQ_RW_STATES;

/* Enumeration listing the Random Read Write operation sub-states */
typedef enum {
    /* Initialize the variables for the Random read write
    operation */
    APP_RANDOM_RW_INIT = 0,

    /* Issue Write command */
    APP_RANDOM_RW_WRITE,

    /* Check if the write command is complete */
    APP_RANDOM_RW_WRITE_STATUS,

    /* Issue read command */
    APP_RANDOM_RW_READ,

    /* Check if the read command is complete */
    APP_RANDOM_RW_READ_STATUS,

    /* Verify the data */
    APP_RANDOM_RW_VERIFY_DATA,

    /* Random Read Write operation is successful */
    APP_RANDOM_RW_IDLE,

    /* Random Read Write operation is unsuccessful */
    APP_RANDOM_RW_ERROR

} APP_RANDOM_RW_STATES;

/* Enumeration listing the EraseWrite operation sub-states */
typedef enum {

    /* Initialize the variables for the EraseWrite operation */
    APP_ERASEWRITE_INIT = 0,

    /* Issue EraseWrite Command */
    APP_ERASEWRITE_ERASEWIRTE,

    /* Check if the EraseWrite command is complete */
    APP_ERASEWRITE_ERASEWIRTE_STATUS,

    /* Issue command to read page one data */
    APP_ERASEWRITE_READ_PAGE_ONE,

    /* Check if the read command is complete */
    APP_ERASEWRITE_READ_PAGE_ONE_STATUS,

    /* Verify page one data */
    APP_ERASEWRITE_VERIFY_PAGE_ONE_DATA,

    /* Issue command to read page two data */
    APP_ERASEWRITE_READ_PAGE_TWO,

    /* Check if the read command is complete */
    APP_ERASEWRITE_READ_PAGE_TWO_STATUS,

    /* Verify page two data */
    APP_ERASEWRITE_VERIFY_PAGE_TWO_DATA,

    /* EraseWrite operation is successful */
    APP_ERASEWRITE_IDLE,

    /* EraseWrite operation is unsuccessful */
    APP_ERASEWRITE_ERROR

} APP_ERASEWRITE_STATES;

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
    /* Open the NVM driver, read the media layout and register for
    the NVM driver events */
    APP_STATE_INIT,

    /* Erase the memory */
    APP_STATE_ERASE_ALL,

    /* Perform sequential read write operations */
    APP_STATE_SEQ_RW,

    /* Perfrom random read write operations */
    APP_STATE_RANDOM_RW,

    /* Perform the erasewrite operations */
    APP_STATE_ERASEWRITE_RW,

    /* Close the NVM driver */
    APP_STATE_CLOSE,

    /* App demonstration is successful */
    APP_STATE_IDLE,

    /* An app error has occurred. App demonstration is unsuccessful */
    APP_STATE_ERROR

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
    /* NVM Driver Handle */
    DRV_HANDLE                  nvmHandle;

    /* NVM Command Handles */
    DRV_NVM_COMMAND_HANDLE      nvmCommandHandle[8];

    /* Erase operation's current state */
    APP_ERASE_STATES            eraseState;

    /* Sequential Read Write operation's current state */
    APP_SEQ_RW_STATES           seqState;

    /* Random Read Write operation's current state */
    APP_RANDOM_RW_STATES        randomState;

    /* EraseWrite operation's current state */
    APP_ERASEWRITE_STATES       eraseWriteState;

    /* Application's current state */
    APP_STATES                  state;

    /* Some of the states have to be repeated as the erase
    operation is done prior to each data read/write operation.
    nextState is used to track the next main state for such
    cases. */
    APP_STATES                  nextState;

    /* Counter to track the number of successful command
    complete events */
    uint8_t                     eventCount;

    /* Counter to track the number of unsuccessful command
    complete events */
    uint8_t                     errorEventCount;

    /* Counter to track the number of reads to be done for the
    random read write operation */
    uint8_t                     randomRWCount;

    /* Read block address */
    uint32_t                    readBlockAddr;

    /* Number of blocks to be read */
    uint32_t                    numReadBlocks;

} APP_DATA;

// *****************************************************************************

/* Driver objects.

  Summary:
    Holds driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.

  Remarks:
    None.
*/

typedef struct
{
	//SYS_MODULE_OBJ   drvObject;

} APP_DRV_OBJECTS;


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
     MPLAB Harmony Demo application initialization routine

  Description:
    This routine initializes Harmony Demo application.  This function opens
    the necessary drivers, initializes the timer and registers the application
    callback with the USART driver.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    APP_Initialize();


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

void APP_Tasks ( void );

void SYS_Initialize ( void* data );
void SYS_Tasks ( void );
// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern APP_DRV_OBJECTS appDrvObject;

extern APP_DATA appData;

#endif /* _APP_HEADER_H */

/*******************************************************************************
 End of File
 */



