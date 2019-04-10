<#include "/utilities/mhc/templates/freemarker_functions.ftl">
<@mhc_expand_list_named name="LIST_APP_FREEMARKER_MACROS"/>
/*******************************************************************************
  Global event source file

  Company:
    Microchip Technology Inc.

  File Name:
    global_event.h

  Summary:
    Global event source file

  Description:
    Global event source file
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "global_event.h"


// *****************************************************************************
// *****************************************************************************
// Section: Data Declarations
// *****************************************************************************
// *****************************************************************************

global_events_t global_events<#if LIST_GLOBAL_EVENT_NAMES?has_content> = {
    <#list LIST_GLOBAL_EVENT_NAMES as x>false<#sep>,
    </#list>};<#else>;</#if>


// *****************************************************************************
// *****************************************************************************
// Section: Global functions.
// *****************************************************************************
// *****************************************************************************

bool global_event_triggered(bool *pEvent)
{
    if (*pEvent)
    {
        *pEvent = false;
        return true;
    }
    
    return false;
}


/*******************************************************************************
 End of File
*/