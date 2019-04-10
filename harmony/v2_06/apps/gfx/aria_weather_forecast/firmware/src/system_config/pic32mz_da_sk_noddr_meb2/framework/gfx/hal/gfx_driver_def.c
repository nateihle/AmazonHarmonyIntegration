#include "gfx/hal/inc/gfx_driver_interface.h"

GFX_Result driverGLCDInfoGet(GFX_DriverInfo* info);
GFX_Result driverGLCDContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeDriverList()
{
    GFX_DriverInterfaces[0].infoGet = &driverGLCDInfoGet;
    GFX_DriverInterfaces[0].contextInitialize = &driverGLCDContextInitialize;

    return GFX_SUCCESS;
}
