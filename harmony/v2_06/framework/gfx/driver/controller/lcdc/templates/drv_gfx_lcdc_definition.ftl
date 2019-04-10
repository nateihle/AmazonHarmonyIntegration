<#macro DRV_GFX_LCDC_DRIVER_DECLARATION IDX>
GFX_Result driverLcdcInfoGet(GFX_DriverInfo* info);
GFX_Result driverLcdcContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_LCDC_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverLcdcInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverLcdcContextInitialize;
</#macro>