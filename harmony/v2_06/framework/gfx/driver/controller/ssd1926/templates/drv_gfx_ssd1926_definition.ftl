<#macro DRV_GFX_SSD1926_DRIVER_DECLARATION IDX>
GFX_Result driverSSD1926InfoGet(GFX_DriverInfo* info);
GFX_Result driverSSD1926ContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_SSD1926_DRIVER_DEFINITION
	ID
	IDX>
	GFX_DriverInterfaces[${ID}].infoGet = &driverSSD1926InfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverSSD1926ContextInitialize;
</#macro>