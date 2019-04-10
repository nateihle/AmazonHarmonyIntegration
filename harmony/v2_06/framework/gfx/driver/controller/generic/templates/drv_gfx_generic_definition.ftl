<#macro DRV_GFX_GENERIC_DRIVER_DECLARATION IDX>
GFX_Result driverGenericInfoGet(GFX_DriverInfo* info);
GFX_Result driverGenericContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_GENERIC_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverGenericInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverGenericContextInitialize;
</#macro>