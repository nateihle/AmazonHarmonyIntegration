<#macro DRV_GFX_NANO2D_PROCESSOR_DECLARATION IDX>
GFX_Result procNANO2DInfoGet(GFX_ProcessorInfo* info);
GFX_Result procNANO2DContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_NANO2D_PROCESSOR_DEFINITION
	ID
	IDX>
    GFX_ProcessorInterfaces[${ID}].infoGet = &procNANO2DInfoGet;
    GFX_ProcessorInterfaces[${ID}].contextInitialize = &procNANO2DContextInitialize;
</#macro>
