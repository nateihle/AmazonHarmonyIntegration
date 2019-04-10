//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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
// DOM-IGNORE-END
#ifndef _CLASSB_H_
#define _CLASSB_H_
/****************************************************************************
  Enumeration:
    CLASSBRESULT

  Description:
    This enumeration is used by the class B test functions to return the results:

    CLASSB_TEST_PASS    - the test finished successfully,
    CLASSB_TEST_FAIL    - the test is failed,
    CLASSB_TEST_TIMEOUT - the test is failed because a timeout was detected,
    CLASSB_TEST_INPROGRESS - the test is still in progress.

  ***************************************************************************/
typedef enum
{
  CLASSB_TEST_PASS = 0,
  CLASSB_TEST_FAIL,
  CLASSB_TEST_TIMEOUT,
  CLASSB_TEST_INPROGRESS
} CLASSBRESULT;
#include <peripheral/peripheral.h>
#include <xc.h>
#include <stdint.h>

<#if CONFIG_USE_CLASSB_CLOCK_LINE_FREQUENCY_TEST>
<#include "/framework/classb/templates/CLASSB_ClockLineFrequencyTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_CLOCK_TEST>
<#include "/framework/classb/templates/CLASSB_ClockTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_CPU_PC_TEST>
<#include "/framework/classb/templates/CLASSB_CPUPCTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_CPU_REGISTERS_TEST>
<#include "/framework/classb/templates/CLASSB_CPURegistersTest.h.ftl">
</#if>

<#if CONFIG_USE_CLASSB_FLASH_TEST>
<#include "/framework/classb/templates/CLASSB_CRCByte.h.ftl">
<#include "/framework/classb/templates/CLASSB_CRCFlashTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_RAM_NON_DESTRUCT_TEST!false>
#define CLASSB_RAM_TEST_CYCLE_SIZE ${CONFIG_CLASSB_RAM_TEST_CYCLE_SIZE}
</#if>
<#if CONFIG_USE_CLASSB_RAM_CHECKERBOARD_TEST>
<#include "/framework/classb/templates/CLASSB_RAMCheckerBoardTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_RAM_MARCHB_TEST>
<#include "/framework/classb/templates/CLASSB_RAMMarchBTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_RAM_MARCHC_STACK_TEST>
<#include "/framework/classb/templates/CLASSB_RAMMarchCStackTest.h.ftl">
</#if>
<#if CONFIG_USE_CLASSB_RAM_MARCHC_TEST>
<#include "/framework/classb/templates/CLASSB_RAMMarchCTest.h.ftl">
</#if>
#endif