<#macro DRV_GFX_ILI9488_DRIVER_DECLARATION IDX>
GFX_Result ILI9488_InfoGet(GFX_DriverInfo* info);
GFX_Result ILI9488_ContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_ILI9488_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &ILI9488_InfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &ILI9488_ContextInitialize;
</#macro>