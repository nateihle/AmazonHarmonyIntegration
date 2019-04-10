<#macro DRV_GFX_OTM2201A_DRIVER_DECLARATION IDX>
GFX_Result driverOTM2201AInfoGet(GFX_DriverInfo* info);
GFX_Result driverOTM2201AContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_OTM2201A_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverOTM2201AInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverOTM2201AContextInitialize;
</#macro>