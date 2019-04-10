#include "gfx/hal/inc/gfx_driver_interface.h"

GFX_Result driverS1D13517InfoGet(GFX_DriverInfo* info);
GFX_Result driverS1D13517ContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeDriverList()
{
	GFX_DriverInterfaces[0].infoGet = &driverS1D13517InfoGet;
    GFX_DriverInterfaces[0].contextInitialize = &driverS1D13517ContextInitialize;

    return GFX_SUCCESS;
}
