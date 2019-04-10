#include "gfx/hal/inc/gfx_driver_interface.h"

GFX_Result driverSSD1963InfoGet(GFX_DriverInfo* info);
GFX_Result driverSSD1963ContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeDriverList()
{
    GFX_DriverInterfaces[0].infoGet = &driverSSD1963InfoGet;
    GFX_DriverInterfaces[0].contextInitialize = &driverSSD1963ContextInitialize;

    return GFX_SUCCESS;
}
