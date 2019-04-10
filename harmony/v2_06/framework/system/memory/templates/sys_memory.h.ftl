<#--
/*******************************************************************************
  Memory System Service Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory.h.ftl

  Summary:
    Memory System Service Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

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


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
  extern "C" {
#endif
// DOM-IGNORE-END

-->
<#if CONFIG_USE_SYS_MEMORY == true>
// *****************************************************************************
/* Memory System Service Configuration Options
*/
<#if CONFIG_USE_SYS_MEMORY_DDR == true>
#define DDR_SIZE				(${CONFIG_SYS_MEMORY_DDR_SIZE} * 1024 *1024)
</#if>

</#if>
<#--


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif

//DOM-IGNORE-END

/*******************************************************************************
 End of File
*/
-->
