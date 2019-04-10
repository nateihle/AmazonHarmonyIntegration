//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

<#if CONFIG_USE_HARMONY_ASSERT_HANDLER?has_content>
<#if CONFIG_USE_HARMONY_ASSERT_HANDLER == true>

// *****************************************************************************
// *****************************************************************************
// Section: Improved assert handling
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"
<#if CONFIG_USE_SYS_CONSOLE_WRITE == true>
#include "system/console/sys_console.h"
</#if>
<#if CONFIG_EXCEPTION_USE_SYS_DEBUG == true>
#include "system/debug/sys_debug.h"
</#if>

<#if CONFIG_USE_SYS_CONSOLE_WRITE == true>
#define MSG_BUFFER_LENGTH 256
char msgBuffer[MSG_BUFFER_LENGTH];
</#if>

// Replacement for built-in _fassert
void __attribute__((noreturn)) _fassert(int          nLineNumber,
                                        const char * sFileName,
                                        const char * sFailedExpression,
                                        const char * sFunction )
{
  <#if     CONFIG_USE_SYS_DEBUG_PRINT   == true>
    SYS_DEBUG_PRINT(SYS_ERROR_FATAL,
                      "ASSERTION '%s' FAILED! File: %s, Line: %d, Function: %s\r\n",
                       sFailedExpression,sFileName,nLineNumber,sFunction);
  <#elseif CONFIG_USE_SYS_CONSOLE_WRITE == true>
    sprintf(msgBuffer,"ASSERTION '%s' FAILED! File: %s, Line: %d, Function: %s\r\n",
                       sFailedExpression,sFileName,nLineNumber,sFunction);
    SYS_CONSOLE_Write( SYS_CONSOLE_INDEX_0, STDOUT_FILENO, msgBuffer, strlen(msgBuffer) );
  </#if>

    while(1) {
      <#if CONFIG_ASSERT_BREAKPOINT == true>
        SYS_DEBUG_BreakPoint();
      <#else>
      /* TODO:  Insert exception handling code. */
      </#if>
    }
}
</#if>
</#if>
