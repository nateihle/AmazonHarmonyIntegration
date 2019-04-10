/*******************************************************************************
  MPLAB Harmony Application 
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    Application Template

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

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
#define SRAM_ADDR_CS0  0xE0000000
#define RAM_SIZE       2*1024*1024

uint32_t loop;
uint16_t loop_16;
uint16_t *addr_16;
uint16_t val_16;

/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData;

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
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */

    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_WRITE;
}

/********************************************************
 * Application switch press routine
 ********************************************************/



/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    uint32_t val = 0;
    uint32_t *addr;

    switch ( appData.state )
    {
        case APP_STATE_WRITE:
            /*Sets EBI address back to the begining*/
            addr = (uint32_t *)SRAM_ADDR_CS0;

            /*this loop writes the data to SRAM*/
            for (loop=0; loop < RAM_SIZE/4; loop++)
            {
                /*writing address of memory location into the memory location*/
                *addr = (uint32_t)addr;
                /*incrementing the address to the next address*/
                addr++;
            }
            appData.state = APP_STATE_READBACK;
            break;

        case APP_STATE_READBACK:
            /*Sets EBI address back to the begining*/
            addr = (uint32_t *)SRAM_ADDR_CS0;

            /*Loop reads back written data*/
            for (loop=0 ; loop < RAM_SIZE/4; loop++)
            {
                val = *addr;
                if (val != (uint32_t)addr)
                {
                    /*If this fails the value read doesn't match the written value*/
                    appData.state = APP_STATE_FAIL;
                    break;
                }
                else
                {
                    appData.state = APP_STATE_DONE;
                }
                addr++;
            }
            break;

        case APP_STATE_DONE:
        {
            BSP_LEDOn(BSP_LED_3);
            /* Test PASSED */
            Nop();
        }
        break;

        case APP_STATE_FAIL:
        {
            BSP_LEDOn(BSP_LED_1);
            /* Test FAILED */
            Nop();
        }
        break;

        default:
            break;
    }
} 

/*******************************************************************************
 End of File
 */

