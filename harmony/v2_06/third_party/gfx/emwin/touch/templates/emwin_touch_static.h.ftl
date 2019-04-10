/*******************************************************************************
  emWin Touch Wrapper Header File

  Company:
    Microchip Technology Inc.

  File Name:
    emwin_touch_static.h

  Summary:
    This header file provides prototypes and definitions for emWin Touch Wrapper.

  Description:

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _EMWIN_TOUCH_STATIC_H_
#define _EMWIN_TOUCH_STATIC_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "GUI.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*

  Summary:

  Description:

  Remarks:

 */
typedef enum
{
    /* 0 degree */
    EMWIN_TOUCH_ORIENTATION_0_DEGREE = 0,

    /* 90 degree */
    EMWIN_TOUCH_ORIENTATION_90_DEGREE,

    /* 180 degree */
    EMWIN_TOUCH_ORIENTATION_180_DEGREE,

    /* 270 degree */
    EMWIN_TOUCH_ORIENTATION_270_DEGREE,

} EMWIN_TOUCH_ORIENTATION;

// *****************************************************************************
/*

  Summary:

  Description:

  Remarks:

 */

typedef struct
{


} EMWIN_TOUCH_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */
 <#if CONFIG_USE_SYS_INPUT == true>
  void emWin_TouchInitialize( );
 </#if>
 <#if CONFIG_USE_SYS_TOUCH_NEEDED>
  void emWin_TouchInitialize( const SYS_MODULE_INIT * const init );
 </#if>
/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_TouchLayerIndexSet( uint32_t layerIndex );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_TouchOrientationSet( EMWIN_TOUCH_ORIENTATION orientation );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_TouchResolutionSet( uint32_t displayWidth, uint32_t displayHeight );

<#if CONFIG_USE_SYS_TOUCH_NEEDED>
/*******************************************************************************

  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_TouchMailBoxCreate( void );
</#if>


/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */
void emWin_TouchTasks(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _EMWIN_TOUCH_STATIC_H_

/*******************************************************************************
 End of File
 */

