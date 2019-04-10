#include "gfx/hal/inc/gfx_processor_interface.h"

GFX_Result procNANO2DInfoGet(GFX_ProcessorInfo* info);
GFX_Result procNANO2DContextInitialize(GFX_Context* context);

GFX_Result GFX_InitializeProcessorList()
{
    GFX_ProcessorInterfaces[0].infoGet = &procNANO2DInfoGet;
    GFX_ProcessorInterfaces[0].contextInitialize = &procNANO2DContextInitialize;

    return GFX_SUCCESS;
}