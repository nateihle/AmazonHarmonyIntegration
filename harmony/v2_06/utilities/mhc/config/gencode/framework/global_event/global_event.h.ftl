<#include "/utilities/mhc/templates/freemarker_functions.ftl">
<@mhc_expand_list_named name="LIST_APP_FREEMARKER_MACROS"/>
/*******************************************************************************
  Global event header file

  Company:
    Microchip Technology Inc.

  File Name:
    global_event.h

  Summary:
    Global event header file

  Description:
    Global event header file
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

#ifndef _GLOBAL_EVENT_H
#define _GLOBAL_EVENT_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

typedef struct {
<#if LIST_GLOBAL_EVENT_NAMES?has_content>
<#list LIST_GLOBAL_EVENT_NAMES as x>
    bool ${("CONFIG_" + x) ? eval};
</#list>
<#else>
    #error "Must define at least one source to trigger an event"
</#if>
} global_events_t;

extern global_events_t global_events;


// *****************************************************************************
// *****************************************************************************
// Section: Global functions.
// *****************************************************************************
// *****************************************************************************

bool global_event_triggered(bool *pEvent);


#endif //#ifndef _GLOBAL_EVENT_H

/*******************************************************************************
 End of File
*/

