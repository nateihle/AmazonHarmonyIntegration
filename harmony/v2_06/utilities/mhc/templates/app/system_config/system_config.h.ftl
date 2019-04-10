<#include "/utilities/mhc/templates/freemarker_functions.ftl">
<#include "/bsp/templates/bsp_freemarker_functions.ftl">
<@mhc_expand_list_named name="LIST_APP_FREEMARKER_MACROS"/>
/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/
<#if LIST_SYSTEM_CONFIG_H_GLOBAL_INCLUDES?has_content>
<@mhc_expand_list list=LIST_SYSTEM_CONFIG_H_GLOBAL_INCLUDES/>
</#if>


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
<#if LIST_SYSTEM_CONFIG_H_SYSTEM_SERVICE_CONFIGURATION?has_content>
<@mhc_expand_list list=LIST_SYSTEM_CONFIG_H_SYSTEM_SERVICE_CONFIGURATION/>
</#if>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
<#if LIST_SYSTEM_CONFIG_H_DRIVER_CONFIGURATION?has_content>
<@mhc_expand_list list=LIST_SYSTEM_CONFIG_H_DRIVER_CONFIGURATION/>
</#if>

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************
<#if LIST_SYSTEM_CONFIG_H_MIDDLEWARE_CONFIGURATION?has_content>
<@mhc_expand_list list=LIST_SYSTEM_CONFIG_H_MIDDLEWARE_CONFIGURATION/>
</#if>


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
<#if CONFIG_PIC32MX || CONFIG_PIC32MZ || CONFIG_PIC32WK || CONFIG_PIC32MK>
<#include "/bsp/templates/bsp_pins_config.h.ftl">
</#if>
<#if CONFIG_USE_APP_CONFIG?has_content>
<#if CONFIG_USE_APP_CONFIG == true>
<#if LIST_SYSTEM_CONFIG_H_APPLICATION_CONFIGURATION?has_content>
<@mhc_expand_list list=LIST_SYSTEM_CONFIG_H_APPLICATION_CONFIGURATION/>
</#if>
<#include "${CONFIG_APP_DIRECTORY}/firmware/templates/system_config.h.ftl">
</#if>
</#if>
<#if CONFIG_USE_BLUETOOTH_LIBRARIES == true>
<#include "/framework/bluetooth/templates/bt_config.h.ftl">
</#if>
<#if CONFIG_USE_ENCODER == true>
<#include "/framework/encoder/templates/audio_encoder_config.h.ftl">
</#if>
<#if CONFIG_USE_DECODER == true>
<#include "/framework/decoder/templates/audio_decoders_config.h.ftl">
</#if>
<#if CONFIG_APP_NAME_0?has_content>
<#assign HCONFIG_APP_INSTANCE=0>
<#assign APP_NAME = CONFIG_APP_NAME_0>

/*** Application Instance 0 Configuration ***/
<@mhc_expand_list_named name="LIST_APP0_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_1?has_content>
<#assign HCONFIG_APP_INSTANCE=1>
<#assign APP_NAME = CONFIG_APP_NAME_1>

/*** Application Instance 1 Configuration ***/
<@mhc_expand_list_named name="LIST_APP1_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_2?has_content>
<#assign HCONFIG_APP_INSTANCE=2>
<#assign APP_NAME = CONFIG_APP_NAME_2>

/*** Application Instance 2 Configuration ***/
<@mhc_expand_list_named name="LIST_APP2_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_3?has_content>
<#assign HCONFIG_APP_INSTANCE=3>
<#assign APP_NAME = CONFIG_APP_NAME_3>

/*** Application Instance 3 Configuration ***/
<@mhc_expand_list_named name="LIST_APP3_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_4?has_content>
<#assign HCONFIG_APP_INSTANCE=4>
<#assign APP_NAME = CONFIG_APP_NAME_4>

/*** Application Instance 4 Configuration ***/
<@mhc_expand_list_named name="LIST_APP4_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_5?has_content>
<#assign HCONFIG_APP_INSTANCE=5>
<#assign APP_NAME = CONFIG_APP_NAME_5>

/*** Application Instance 5 Configuration ***/
<@mhc_expand_list_named name="LIST_APP5_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_6?has_content>
<#assign HCONFIG_APP_INSTANCE=6>
<#assign APP_NAME = CONFIG_APP_NAME_6>

/*** Application Instance 6 Configuration ***/
<@mhc_expand_list_named name="LIST_APP6_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_7?has_content>
<#assign HCONFIG_APP_INSTANCE=7>
<#assign APP_NAME = CONFIG_APP_NAME_7>

/*** Application Instance 7 Configuration ***/
<@mhc_expand_list_named name="LIST_APP7_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_8?has_content>
<#assign HCONFIG_APP_INSTANCE=8>
<#assign APP_NAME = CONFIG_APP_NAME_8>

/*** Application Instance 8 Configuration ***/
<@mhc_expand_list_named name="LIST_APP8_H_CONSTANTS"/>
</#if>
<#if CONFIG_APP_NAME_9?has_content>
<#assign HCONFIG_APP_INSTANCE=9>
<#assign APP_NAME = CONFIG_APP_NAME_9>

/*** Application Instance 9 Configuration ***/
<@mhc_expand_list_named name="LIST_APP9_H_CONSTANTS"/>
</#if>

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/
