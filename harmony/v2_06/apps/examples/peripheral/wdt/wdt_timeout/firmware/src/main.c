/*******************************************************************************
  MPLAB Harmony WDT Timeout Example

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    MPLAB Harmony wdt_timeout main function

  Description:
    This example implements a simple WDT timeout handler. The first LED
    is continuously toggled in a loop to show that the WDT is being cleared
    on time and the system is stable. When one of the switches are pressed,
    it triggers an interrupt which contains a blocking loop, causing the WDT to
    timeout and reset the system. The WDT handler then turns on the 3rd LED
    to indicate the WDT timeout event was taken care of.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
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

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{

    /*Call the SYS Init routine. App init routine gets called from this*/
    SYS_Initialize ( NULL );

    while ( true )
    {
        /*Invoke SYS tasks. APP tasks gets called from this*/
        SYS_Tasks ( );

    }

    /* Should not come here during normal operation */
    SYS_ASSERT ( false , "about to exit main" );

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

