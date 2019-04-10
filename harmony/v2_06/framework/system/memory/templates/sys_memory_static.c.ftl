/*******************************************************************************
  Memory System Service Initialization

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_static.c

  Summary:
    This function calls the individual initialization routine(s) in the
	Memory System Service.

  Description:
    This function calls the initialization routine(s) for the Memory System
	Services enabled by MHC.

  Remarks:
    None.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED 'AS IS'  WITHOUT  WARRANTY  OF  ANY  KIND,
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
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "system/memory/sys_memory.h"

<#if CONFIG_USE_SYS_MEMORY_DDR == true>
#include "system/memory/ddr/sys_memory_ddr_static.h"
</#if>
<#if CONFIG_USE_SYS_MEMORY_EBI == true>
#include "system/memory/ebi/sys_memory_ebi_static.h"
</#if>

/*******************************************************************************
  Function:
    void SYS_MEMORY_Initialize(void)

  Summary:
    Initializes the Memory System Service

  Remarks:
 */
void SYS_MEMORY_Initialize(void)
{
<#if CONFIG_USE_SYS_MEMORY_DDR?has_content>
  <#if CONFIG_USE_SYS_MEMORY_DDR == true>
    SYS_MEMORY_DDR_Initialize();
  </#if>
</#if>
<#if CONFIG_USE_SYS_MEMORY_EBI?has_content>
  <#if CONFIG_USE_SYS_MEMORY_EBI == true>
    SYS_MEMORY_EBI_Initialize();
  </#if>
</#if>
}
<#--
/*******************************************************************************
 End of File
*/
-->
