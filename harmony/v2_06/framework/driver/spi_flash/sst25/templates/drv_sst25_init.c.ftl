<#--
/*******************************************************************************
  USART Driver Initialization File

  File Name:
    drv_sst25_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
<#-- Instance 0 -->
<#if CONFIG_DRV_SST25_INST_IDX0 == true>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "DYNAMIC">
    sysObj.drvSst25Obj0 = DRV_SST25_Initialize(DRV_SST25_INDEX_0, (SYS_MODULE_INIT *)&drvSst25Obj0InitData);
</#if>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "STATIC">
    DRV_SST250_Initialize();
</#if>
</#if>

<#-- Instance 1 -->
<#if CONFIG_DRV_SST25_INST_IDX1 == true>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "DYNAMIC">
    sysObj.drvSst25Obj1 = DRV_SST25_Initialize(DRV_SST25_INDEX_1, (SYS_MODULE_INIT *)&drvSst25Obj1InitData);
</#if>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "STATIC">
    DRV_SST251_Initialize();
</#if>
</#if>

<#-- Instance 2 -->
<#if CONFIG_DRV_SST25_INST_IDX2 == true>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "DYNAMIC">
    sysObj.drvSst25Obj2 = DRV_SST25_Initialize(DRV_SST25_INDEX_2, (SYS_MODULE_INIT *)&drvSst25Obj2InitData);
</#if>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "STATIC">
    DRV_SST252_Initialize();
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
