<#macro DRV_GFX_STUB_DRIVER_DECLARATION IDX>
GFX_Result driverStubInfoGet(GFX_DriverInfo* info);
GFX_Result driverStubContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_STUB_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverStubInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverStubContextInitialize;
</#macro>