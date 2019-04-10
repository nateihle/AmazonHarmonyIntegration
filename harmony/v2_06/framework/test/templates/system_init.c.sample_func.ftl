<#--
/*******************************************************************************
  MPLAB Harmony Test Harness Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    system_init.c.sample_func.ftl

  Summary:
    MPLAB Harmony Sample Functinal Test Freemarker Template File for 
    system_init.c

  Description:
    This file defines the template for the system configuration entries 
    inserted into the system_init.c file when the sample functional test is 
    used.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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
-->
/* Sample Functional Test Initialization Data */
TEST_SAMPLE_FUNCTIONAL_INIT_DATA testSampleInit = 
{
    .modulesInitData =
    {<#--  CONFIG_TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_<n> limited by test_sample_func_idx.hconfig.ftl" to 2 instances-->
      <#if CONFIG_TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_0 == true>
        &${CONFIG_TEST_SAMPLE_LIB_INIT_DATA_IDX0}</#if><#if CONFIG_TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_1 == true>,
        &${CONFIG_TEST_SAMPLE_LIB_INIT_DATA_IDX1}</#if>
    }
};

<#--
/*******************************************************************************
 End of File
*/
-->
