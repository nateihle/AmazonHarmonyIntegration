#include "gfx/hal/inc/gfx_driver_interface.h"

GFX_Result ILI9488_InfoGet(GFX_DriverInfo* info);
GFX_Result ILI9488_ContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeDriverList()
{
	GFX_DriverInterfaces[0].infoGet = &ILI9488_InfoGet;
    GFX_DriverInterfaces[0].contextInitialize = &ILI9488_ContextInitialize;

    return GFX_SUCCESS;
}
