<#macro DRV_GFX_GLCD_DRIVER_DECLARATION IDX>
GFX_Result driverGLCDInfoGet(GFX_DriverInfo* info);
GFX_Result driverGLCDContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_GLCD_DRIVER_DEFINITION
	ID
	IDX>
    GFX_DriverInterfaces[${ID}].infoGet = &driverGLCDInfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverGLCDContextInitialize;
</#macro>
