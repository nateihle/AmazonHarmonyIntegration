#include "gfx/hal/inc/gfx_driver_interface.h"

GFX_Result driverOTM2201AInfoGet(GFX_DriverInfo* info);
GFX_Result driverOTM2201AContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeDriverList()
{
	GFX_DriverInterfaces[0].infoGet = &driverOTM2201AInfoGet;
    GFX_DriverInterfaces[0].contextInitialize = &driverOTM2201AContextInitialize;

    return GFX_SUCCESS;
}
