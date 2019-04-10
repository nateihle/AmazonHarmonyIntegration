/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_config.h

  Summary:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#ifndef _LIBARIA_CONFIG_H
#define _LIBARIA_CONFIG_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#ifndef LIB_EXPORT
#define LIB_EXPORT
#endif

/* library configuration flags */
<#if CONFIG_LIBARIA_MAX_LAYERS?? == false>
#define LA_MAX_LAYERS     1
<#else>
#define LA_MAX_LAYERS     ${CONFIG_LIBARIA_MAX_LAYERS}
</#if>

// widget inclusion
#define LA_ARC_WIDGET_ENABLED              <#if CONFIG_LIBARIA_WIDGET_ARC_ENABLED == true>1<#else>0</#if>
#define LA_BAR_GRAPH_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_BAR_GRAPH_ENABLED == true>1<#else>0</#if>
#define LA_BUTTON_WIDGET_ENABLED           <#if CONFIG_LIBARIA_WIDGET_BUTTON_ENABLED == true>1<#else>0</#if>
#define LA_CHECKBOX_WIDGET_ENABLED         <#if CONFIG_LIBARIA_WIDGET_CHECKBOX_ENABLED == true>1<#else>0</#if>
#define LA_CIRCLE_WIDGET_ENABLED           <#if CONFIG_LIBARIA_WIDGET_CIRCLE_ENABLED == true>1<#else>0</#if>
#define LA_CIRCULAR_GAUGE_WIDGET_ENABLED   <#if CONFIG_LIBARIA_WIDGET_CIRCULAR_GAUGE_ENABLED == true>1<#else>0</#if>
#define LA_CIRCULAR_SLIDER_WIDGET_ENABLED  <#if CONFIG_LIBARIA_WIDGET_CIRCULAR_SLIDER_ENABLED == true>1<#else>0</#if>
#define LA_DRAWSURFACE_WIDGET_ENABLED      <#if CONFIG_LIBARIA_WIDGET_DRAWSURFACE_ENABLED == true>1<#else>0</#if>
#define LA_IMAGE_WIDGET_ENABLED            <#if CONFIG_LIBARIA_WIDGET_IMAGE_ENABLED == true>1<#else>0</#if>
#define LA_IMAGEPLUS_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_IMAGEPLUS_ENABLED == true>1<#else>0</#if>
#define LA_IMAGESEQUENCE_WIDGET_ENABLED    <#if CONFIG_LIBARIA_WIDGET_IMAGESEQUENCE_ENABLED == true>1<#else>0</#if>
#define LA_GRADIENT_WIDGET_ENABLED         <#if CONFIG_LIBARIA_WIDGET_GRADIENT_ENABLED == true>1<#else>0</#if>
#define LA_GROUPBOX_WIDGET_ENABLED         <#if CONFIG_LIBARIA_WIDGET_GROUPBOX_ENABLED == true>1<#else>0</#if>
#define LA_KEYPAD_WIDGET_ENABLED           <#if CONFIG_LIBARIA_WIDGET_KEYPAD_ENABLED == true>1<#else>0</#if>
#define LA_LABEL_WIDGET_ENABLED            <#if CONFIG_LIBARIA_WIDGET_LABEL_ENABLED == true>1<#else>0</#if>
#define LA_LINE_WIDGET_ENABLED             <#if CONFIG_LIBARIA_WIDGET_LINE_ENABLED == true>1<#else>0</#if>
#define LA_LINE_GRAPH_WIDGET_ENABLED       <#if CONFIG_LIBARIA_WIDGET_LINE_GRAPH_ENABLED == true>1<#else>0</#if>
#define LA_LIST_WIDGET_ENABLED             <#if CONFIG_LIBARIA_WIDGET_LIST_ENABLED == true>1<#else>0</#if>
#define LA_LISTWHEEL_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_LISTWHEEL_ENABLED == true>1<#else>0</#if>
#define LA_PIE_CHART_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_PIE_CHART_ENABLED == true>1<#else>0</#if>
#define LA_PROGRESSBAR_WIDGET_ENABLED      <#if CONFIG_LIBARIA_WIDGET_PROGRESSBAR_ENABLED == true>1<#else>0</#if>
#define LA_RADIAL_MENU_WIDGET_ENABLED      <#if CONFIG_LIBARIA_WIDGET_RADIAL_MENU_ENABLED == true>1<#else>0</#if>
#define LA_RADIOBUTTON_WIDGET_ENABLED      <#if CONFIG_LIBARIA_WIDGET_RADIOBUTTON_ENABLED == true>1<#else>0</#if>
#define LA_RECTANGLE_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_RECTANGLE_ENABLED == true>1<#else>0</#if>
#define LA_SCROLLBAR_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_SCROLLBAR_ENABLED == true>1<#else>0</#if>
#define LA_SLIDER_WIDGET_ENABLED           <#if CONFIG_LIBARIA_WIDGET_SLIDER_ENABLED == true>1<#else>0</#if>
#define LA_TEXTFIELD_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_TEXTFIELD_ENABLED == true>1<#else>0</#if>
#define LA_TOUCHTEST_WIDGET_ENABLED        <#if CONFIG_LIBARIA_WIDGET_TOUCHTEST_ENABLED == true>1<#else>0</#if>
#define LA_WINDOW_WIDGET_ENABLED           <#if CONFIG_LIBARIA_WIDGET_WINDOW_ENABLED == true>1<#else>0</#if>

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_EVENTS_H
/*******************************************************************************
 End of File
*/
