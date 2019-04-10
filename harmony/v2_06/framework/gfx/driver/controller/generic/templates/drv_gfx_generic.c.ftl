/*******************************************************************************
  MPLAB Harmony Generated Driver Implementation File

  File Name:
    drv_gfx_generic.c

  Summary:
    Build-time generated driver.

  Description:
    This is a hardware-agnostic generated driver.  It can be used as a starting
	point to create a hardware specific driver.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#include "framework/gfx/driver/controller/generic/drv_gfx_generic.h"

#include <xc.h>
#include <sys/attribs.h>

// include any peripheral header files

// define any custom values
#define LAYER_COUNT     1

uint32_t state;

#define DISPLAY_WIDTH   ${CONFIG_DRV_GFX_DISPLAY_WIDTH}
#define DISPLAY_HEIGHT  ${CONFIG_DRV_GFX_DISPLAY_HEIGHT}

const char* DRIVER_NAME = "Generic";

<#if CONFIG_GFX_DRV_GENERIC_STATIC_FRAME_BUFFER>
<#if CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_GS_8">
<#assign COLOR_MASK="GFX_COLOR_MASK_GS_8">
<#assign BPP="1">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_RGB_332">
<#assign COLOR_MASK="GFX_COLOR_MODE_RGB_332">
<#assign BPP="1">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_RGB_565">
<#assign COLOR_MASK="GFX_COLOR_MODE_RGB_565">
<#assign BPP="2">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_RGBA_5551">
<#assign COLOR_MASK="GFX_COLOR_MODE_RGBA_5551">
<#assign BPP="2">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_RGB_888">
<#assign COLOR_MASK="GFX_COLOR_MODE_RGB_888">
<#assign BPP="3">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_RGBA_8888">
<#assign COLOR_MASK="GFX_COLOR_MODE_RGBA_8888">
<#assign BPP="4">
<#elseif CONFIG_GFX_DRV_GENERIC_COLOR_MODE = "GFX_COLOR_MODE_ARGB_8888">
<#assign COLOR_MASK="GFX_COLOR_MODE_ARGB_8888">
<#assign BPP="4">
</#if>
static uint32_t supported_color_formats = ${COLOR_MASK};

uint8_t __attribute__((coherent, aligned(32))) frameBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT * ${BPP}];
<#else>
static uint32_t supported_color_formats = GFX_COLOR_MASK_ALL;
</#if>

/**** Hardware Abstraction Interfaces ****/
enum
{
    INIT = 0,
    RUN
};

// function that returns the information for this driver
GFX_Result driverGenericInfoGet(GFX_DriverInfo* info)
{
	if(info == NULL)
        return GFX_FAILURE;

	// populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supported_color_formats;
    info->layer_count = LAYER_COUNT;
    
    return GFX_SUCCESS;
}

static GFX_Result genericUpdate()
{
    GFX_Context* context = GFX_ActiveContext();
   
    if(context == NULL)
        return GFX_FAILURE;
    
    if(state == INIT)
    {
        // perform driver initialization here
        
        state = RUN;
    }
    
    return GFX_SUCCESS;
}

static void genericDestroy(GFX_Context* context)
{	
	// driver specific shutdown tasks
	if(context->driver_data != GFX_NULL)
	{
		context->memory.free(context->driver_data);
		context->driver_data = GFX_NULL;
	}
	
	// general default shutdown
	defDestroy(context);
}

<#if CONFIG_GFX_DRV_GENERIC_STATIC_FRAME_BUFFER>
static GFX_Result layerBufferCountSet(uint32_t count)
{
    count = count;
        
    return GFX_FAILURE;
}

static GFX_Result layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
    idx = 0;
    address = address;
    
    return GFX_FAILURE;
}

static GFX_Result layerBufferAllocate(uint32_t idx)
{
    idx = 0;
    
    return GFX_FAILURE;
}
</#if>

static GFX_Result genericInitialize(GFX_Context* context)
{
<#if CONFIG_GFX_DRV_GENERIC_STATIC_FRAME_BUFFER>
	int i;

</#if>
	// general default initialization
	if(defInitialize(context) == GFX_FAILURE)
		return GFX_FAILURE;	
	
<#if CONFIG_GFX_DRV_GENERIC_STATIC_FRAME_BUFFER>
	// driver specific initialization tasks	
	// initialize all layer color modes
    for(i = 0; i < LAYER_COUNT; i++)
    {   
		context->layer.layers[i].buffer_count = 1;
		
		GFX_PixelBufferCreate(DISPLAY_WIDTH,
							  DISPLAY_HEIGHT,
							  ${CONFIG_GFX_DRV_GENERIC_COLOR_MODE},
							  frameBuffer,
							  &context->layer.layers[i].buffers[0].pb);
	
		// mark buffer state as managed so that it can't be freed
		// by anything but the driver
		context->layer.layers[i].buffers[0].state = GFX_BS_MANAGED;
	}
</#if>
	
	return GFX_SUCCESS;
}

// function that initialized the driver context
GFX_Result driverGenericContextInitialize(GFX_Context* context)
{
	// set driver-specific data initialization function address
	context->hal.initialize = &genericInitialize; 	
    
	// override essential hal functions
	context->hal.destroy = &genericDestroy;
	context->hal.update = &genericUpdate;
	
	// override default HAL functions with driver specific implementations
	// any HAL function can be overridden by a driver.  care must be taken
	// that essential functionality is reimplemented by the new function
	// or that the default implementation is called by the new function
    
<#if CONFIG_GFX_DRV_GENERIC_STATIC_FRAME_BUFFER>
	// the application should not be allowed to free or modify
	// a static frame buffer, override these functions
	context->hal.layerBufferCountSet = &layerBufferCountSet;
    context->hal.layerBufferAddressSet = &layerBufferAddressSet;
    context->hal.layerBufferAllocate = &layerBufferAllocate;
</#if>
	
	return GFX_SUCCESS;
}

/**** End Hardware Abstraction Interfaces ****/
