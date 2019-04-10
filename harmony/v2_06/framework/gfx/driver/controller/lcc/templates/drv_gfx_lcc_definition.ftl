<#macro DRV_GFX_LCC_DRIVER_DECLARATION IDX>
GFX_Result driverLCCInfoGet(GFX_DriverInfo* info);
GFX_Result driverLCCContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_LCC_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverLCCInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverLCCContextInitialize;
</#macro>