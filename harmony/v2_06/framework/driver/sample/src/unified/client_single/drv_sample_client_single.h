/*******************************************************************************
  SAMPLE Driver Build Variant implementation for single open static driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_client_single.h

  Summary:
    SAMPLE Driver Build Variant implementation for single open static driver

  Description:
    This file defines the build variant implementations for the single open
    static driver.
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

#ifndef _DRV_SAMPLE_CLIENT_SINGLE_H
#define _DRV_SAMPLE_CLIENT_SINGLE_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "sample/src/drv_sample_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: Multi-Client Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SAMPLE_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SAMPLE_CLIENT_OBJ(obj,mem)    _DRV_SAMPLE_OBJ_MAKE_NAME(gDrvSAMPLEClientObj).mem


// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC( statement )

  Summary:
    Allows removal of code statements only needed for multi client
    configurations

  Description:
    This macro allows removal of code statements that are only needed for
    multi client configurations.

  Remarks:
    Do not put multiple statements or compound statements within this macro.
    The statement must not include a comma (,).
*/

#define _DRV_SAMPLE_IF_MC(statement)



// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_COMMA(arg)

  Summary:
    Allows removal of multiple function arguments only needed for multi client
    configurations

  Description:
    This macro allows removal of multiple function arguments that are only
    needed for multi client configurations.

  Remarks:
    This macro is only for use on arguments that preceed (or are to the Left of)
    other arguments in a function's formal parameter list because it embeds a
    comma (,) at the end of the argument declaration.
*/

#define _DRV_SAMPLE_IF_MC_COMMA(arg)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_RETURN_TYPE(type)

  Summary:
    Multi client return type

  Description:
    Multi client interface return type, single client voids out and multi client
    returns the client handle
*/

#define _DRV_SAMPLE_IF_MC_RETURN_TYPE(type)   void


// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_RETURN(handle)

  Summary:
    Switches "return(handle)" statements needed in multi client builds to just
    "return" in single client builds.

  Description:
    This macro switches "return(handle)" statements needed in multi client builds
    to  just "return" in single client builds.
*/

#define _DRV_SAMPLE_IF_MC_RETURN(handle)  return


// *****************************************************************************
/* Macro: _DRV_SAMPLE_ClientObjectIsValid( index )

  Summary:
    Verifies that the client object is valid

  Description:
    This macro evaluates (or returns) "true" (non-zero) if the given client
    object is valid or "false" (zero) if the index is invalid.

  Remarks:
    Exactly how this validation is performed depends on how the client objects
    are allocated.  This macro may implement a static test directly, it may call
    another function, or it may always evaluate to a constant.
*/

#define _DRV_SAMPLE_ClientObjectIsValid( index )    (true)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_CLIENT_OBJ_ALLOCATE(index, hClientObj)

  Summary:
    Creates the Client instance

  Description:
    This macro to creates the Client instance.
*/

#define _DRV_SAMPLE_CLIENT_OBJ_ALLOCATE(drvId, hClientObj)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_CLIENT_OBJ_FREE(hClientObj)

  Summary:
    Frees the client instance

  Description:
    This macro frees the client instance object.
*/

#define _DRV_SAMPLE_CLIENT_OBJ_FREE(hClientObj) \
            (_DRV_SAMPLE_CLIENT_OBJ(hClientObj, inUse) = false )


// *****************************************************************************
/* Macro: _DRV_SAMPLE_GetClientObject(handle)

  Summary:
    Gets the client instance

  Description:
    This macro "returns" the client instance handle/pointer, either statically
    or dynamically, from the given client handle.
*/

#define _DRV_SAMPLE_GetClientObject(handle)    (handle)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_MULTI_CLIENT_DOBJ(hClientObj, dObj)

  Summary:
    Gets the driver object from the client instance

  Description:
    This macro gets the driver object from the client instance.
*/

#define _DRV_SAMPLE_MC_DOBJ(hClientObj, dObj)


#endif // _DRV_SAMPLE_CLIENT_SINGLE_H

/*******************************************************************************
 End of File
*/

