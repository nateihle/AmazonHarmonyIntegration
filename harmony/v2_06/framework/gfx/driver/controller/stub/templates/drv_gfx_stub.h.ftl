#ifndef DRV_GFX_STUB_H
#define DRV_GFX_STUB_H

#include "gfx/hal/inc/gfx_common.h"
#include "gfx/hal/inc/gfx_driver_interface.h"

#ifdef __cplusplus
    extern "C" {
#endif

GFX_Result driverStubInfoGet(GFX_DriverInfo* info);
GFX_Result driverStubContextInitialize(GFX_Context* context);

#ifdef __cplusplus
    }
#endif

#endif /* DRV_GFX_STUB_H */