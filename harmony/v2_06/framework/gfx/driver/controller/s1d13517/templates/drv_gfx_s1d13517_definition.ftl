<#macro DRV_GFX_S1D13517_DRIVER_DECLARATION IDX>
GFX_Result driverS1D13517InfoGet(GFX_DriverInfo* info);
GFX_Result driverS1D13517ContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_S1D13517_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverS1D13517InfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverS1D13517ContextInitialize;
</#macro>