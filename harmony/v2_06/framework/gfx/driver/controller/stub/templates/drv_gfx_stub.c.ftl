/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_stub.c

  Summary:

  Description:
    
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#define MAX_LAYER_COUNT 1

static uint32_t supportedColorModes = GFX_COLOR_MASK_${CONFIG_DRV_GFX_STUB_COLOR_MODE};
static GFX_ColorMode colorMode = GFX_COLOR_MODE_${CONFIG_DRV_GFX_STUB_COLOR_MODE};

const char* DRIVER_NAME = "Stub";

enum
{
    INITITIALIZE = 0,
    RUN
};

static GFX_Result update()
{   
    return GFX_SUCCESS;
}

static GFX_Result initialize(GFX_Context* context)
{
	uint32_t i;
	
	// general default initialization
	if(defInitialize(context) == GFX_FAILURE)
		return GFX_FAILURE;
		
	// override default HAL functions with driver specific implementations
    context->hal.update = &update;
    context->hal.destroy = &destroy;

	// driver specific initialization tasks	
	// initialize all layer color modes
    for(i = 0; i < context->layer.count; i++)
        context->layer.layers[i].color_mode = colorMode;

	return GFX_SUCCESS;
}

static void destroy(GFX_Context* context)
{		
	// general default shutdown
	defDestroy(context);
}

// function that returns the information for this driver
GFX_Result driverStubInfoGet(GFX_DriverInfo* info)
{
	if(info == NULL)
        return GFX_FAILURE;

	// populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supportedColorModes;
    info->layer_count = MAX_LAYER_COUNT;
    
    return GFX_SUCCESS;
}

// function that initializes the driver context
GFX_Result driverStubContextInitialize(GFX_Context* context)
{	
	// set driver-specific data initialization function address
	context->hal.initialize = &initialize; 
	
	// set driver-specific destroy function address
    context->hal.destroy = &destroy;
	
	return GFX_SUCCESS;
}