/*******************************************************************************
  MPLAB Harmony Application 
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the application logic.
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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "peripheral/bmx/plib_bmx.h"
#include "peripheral/ports/plib_ports.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************


/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData = 
{
    //TODO - Initialize appData structure. 

};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/


APP_DRV_OBJECTS appDrvObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    appData.userPartitionSize = 20 * 1024; // User Mode = 20kb

    /* Ram sizes */
    appData.kernDataRamSize = 64 * 1024; // Kernel Data = 64KB
    appData.kernProgRamSize = 32 * 1024; // Kernel Program = 32KB
    appData.userDataRamSize = 16 * 1024; // User Data = 16KB
    appData.userProgRamSize = 16 * 1024; // User Program = 16KB

    /* Get size of program flash and data ram */
    appData.totalFlashSize = PLIB_BMX_ProgramFlashMemorySizeGet(BMX_ID_0);
    appData.totalRamSize = PLIB_BMX_DataRAMSizeGet(BMX_ID_0);

    /* Setup flash offsets */
    appData.userPartitionOffset = appData.totalFlashSize - appData.userPartitionSize;

    /* Setup ram offsets */
    appData.kernProgOffset = appData.kernDataRamSize;
    appData.userDataOffset = appData.kernProgOffset + appData.kernProgRamSize;
    appData.userProgOffset = appData.userDataOffset + appData.userDataRamSize;

    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
}




/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    /* check the application state*/
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:

            SYS_DEVCON_JTAGDisable();
            appData.state = APP_STATE_MEM_PARTITIONS_SET;
            break;


        case APP_STATE_MEM_PARTITIONS_SET:
            /* Set a user mode partition in flash memory of 20kb, the rest will be kernel mode */
            PLIB_BMX_ProgramFlashPartitionSet(BMX_ID_0, appData.userPartitionOffset);

            /* Set RAM partitions */
            PLIB_BMX_DataRAMPartitionSet(BMX_ID_0, appData.kernProgOffset,
                    appData.userDataOffset, appData.userProgOffset);

            appData.state = APP_STATE_PARTITION_SIZES_GET;
            break;

        case APP_STATE_PARTITION_SIZES_GET:
            /* Get the offsets */
            appData.kernProgOffset = PLIB_BMX_DataRAMKernelProgramOffsetGet(BMX_ID_0);
            appData.userDataOffset = PLIB_BMX_DataRAMUserDataOffsetGet(BMX_ID_0);
            appData.userProgOffset = PLIB_BMX_DataRAMUserProgramOffsetGet(BMX_ID_0);

            /* Check the sizes of each partition, to ensure everything was successful */
            appData.kernDataRamSize = appData.kernProgOffset;
            appData.kernProgRamSize = appData.userDataOffset - appData.kernProgOffset;
            appData.userDataRamSize = appData.userProgOffset - appData.userDataOffset;
            appData.userProgRamSize = appData.totalRamSize - appData.userProgOffset;

            appData.state = APP_STATE_PARTITION_SIZES_VERIFY;
            break;

        case APP_STATE_PARTITION_SIZES_VERIFY:
            /* Verify partition sizes fit RAM */
            if ((appData.kernDataRamSize + appData.kernProgRamSize + appData.userDataRamSize + appData.userProgRamSize) == appData.totalRamSize)
            {
                /* Turn on LEDs */
                PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_A, 0xFF);
    		}

            appData.state = APP_STATE_SPIN;
            break;

        case APP_STATE_SPIN:
            /* Do nothing, but the state will not change. */
            break;
   
        /* The default state should never be executed. */
        default:
            /* TODO: 098. Handle error in application's state machine. */
            break;
	} 
}


/*******************************************************************************
 End of File
 */

