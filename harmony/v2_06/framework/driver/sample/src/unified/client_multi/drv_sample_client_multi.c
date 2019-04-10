/*******************************************************************************
  SAMPLE Driver Client Dynamic Handling Interfaces

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_client_multi.c

  Summary:
    SAMPLE Driver Client Dynamic Handling Interfaces

  Description:
    This file implements the functions for the multiple client support.

    While building the driver from source, use this file in the build when
    DRV_SAMPLE_CLIENTS_NUMBER is defined in the system configuration or when
    DRV_SAMPLE_INSTANCES_NUMBER is defined in the system configuration.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include "sample/src/drv_sample_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

DRV_SAMPLE_CLIENT_OBJ   gDrvSAMPLEClientObj[DRV_SAMPLE_INSTANCES_NUMBER] ;

// *****************************************************************************
/* DRV_SAMPLE_CLIENT_OBJ_HANDLE _DRV_SAMPLE_ClientObjectAllocate (SYS_MODULE_INDEX drvIndex )

  Summary:
    Creates a client instance Object and returns it.

  Description:
    Creates a client instance Object and returns it.

  Remarks:
    None
*/

DRV_SAMPLE_CLIENT_OBJ_HANDLE _DRV_SAMPLE_ClientObjectAllocate (SYS_MODULE_INDEX drvIndex )
{
    /* Local Variables */
    _DRV_SAMPLE_IF_MC( DRV_SAMPLE_CLIENT_OBJ_HANDLE hClientObj = (DRV_SAMPLE_CLIENT_OBJ_HANDLE) 0 );
    uint8_t i = 0;

    for (i = 0; i < DRV_SAMPLE_CLIENTS_NUMBER ; i++)
    {
        hClientObj = (i*drvIndex);
        // Return the matching index associated the hardware instance.
        if (!(( _DRV_SAMPLE_CLIENT_OBJ(hClientObj, inUse) == true )))
        {
            _DRV_SAMPLE_CLIENT_OBJ(hClientObj, driverObject) = drvIndex;

            return hClientObj;
        }
    }

    // Returns the available client object
    return -1;
}

/*******************************************************************************
End of File
*/

