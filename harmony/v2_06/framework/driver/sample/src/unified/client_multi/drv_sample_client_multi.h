/*******************************************************************************
  SAMPLE Driver Build Variant implementation for dynamic driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_client_multi.h

  Summary:
    SAMPLE Driver Build Variant implementation for dynamic driver configurations

  Description:
    This file defines the build variant implementations for dynamic driver
    configurations.
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


#ifndef _DRV_SAMPLE_CLIENT_MULTI_H
#define _DRV_SAMPLE_CLIENT_MULTI_H


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

#define _DRV_SAMPLE_CLIENT_OBJ(obj,mem)    gDrvSAMPLEClientObj[obj].mem


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

#define _DRV_SAMPLE_IF_MC(statement)  statement



// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_COMMA( argument )

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

#define _DRV_SAMPLE_IF_MC_COMMA(arg) arg,


// *****************************************************************************
/* Macro: _DRV_SAMPLE_MC_PARAM(param)

  Summary:
    Allows removal of function parameters only needed for multi client configurations

  Description:
    This macro allows removal of function parameters that are only needed for
    multi client configurations.

  Remarks:
    This macro is only for single or Right-most arguments in a function's actual
    parameter list.
*/

#define _DRV_SAMPLE_MC_PARAM(param)                        param


// *****************************************************************************
/* Macro: _DRV_SAMPLE_MC_PARAM_COMMA(param)

  Summary:
    Allows removal of multiple function parameters only needed for multi client
    configurations

  Description:
    This macro allows removal of multiple function parameters that are only
    needed for multi client configurations.

  Remarks:
    This macro is only for use on arguments that preceed (or are to the Left of)
    other arguments in a function's actual parameter list because it embeds a
    comma (,) at the end of the argument declaration.
*/

#define _DRV_SAMPLE_MC_PARAM_COMMA(param)                  param,


// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_RETURN_TYPE(type)

  Summary:
    Multi client return type

  Description:
    Multi client interface return type, single client voids out and multi client
    returns the client handle
*/

#define _DRV_SAMPLE_IF_MC_RETURN_TYPE(type)   type


// *****************************************************************************
/* Macro: _DRV_SAMPLE_IF_MC_RETURN(handle)

  Summary:
    Switches "return(handle)" statements needed in multi client builds to just
    "return" in single client builds.

  Description:
    This macro switches "return(handle)" statements needed in multi client builds
    to  just "return" in single client builds.
*/

#define _DRV_SAMPLE_IF_MC_RETURN(handle)   return(handle)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_MC_Test( condition )

  Summary:
    Returns the condition value

  Description:
    This macro evaluates (or returns) "true" (non-zero) or "false" (zero) based
    on the condition provided.

  Remarks:
    Exactly how this test is performed depends on how condition is framed.

*/

#define _DRV_SAMPLE_MC_Test( condition )   ( condition )


// *****************************************************************************
/* Macro: _DRV_SAMPLE_CLIENT_OBJ(index, hClientObj)

  Summary:
    Macro to create the Client instance

  Description:
    Macro to create the Client instance
*/

#define _DRV_SAMPLE_CLIENT_OBJ_ALLOCATE(drvId, hClientObj)  (hClientObj = drvId)


// *****************************************************************************
/* Macro: _DRV_SAMPLE_CLIENT_OBJ_FREE(hClientObj)

  Summary:
    Frees the client instance

  Description:
    This macro frees the client instance object
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

#define _DRV_SAMPLE_GetClientObject(handle)     (handle)


#endif // _DRV_SAMPLE_CLIENT_MULTI_H

/*******************************************************************************
 End of File
*/

