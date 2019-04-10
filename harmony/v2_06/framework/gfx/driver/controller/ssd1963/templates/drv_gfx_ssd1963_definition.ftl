<#macro DRV_GFX_SSD1963_DRIVER_DECLARATION IDX>
GFX_Result driverSSD1963InfoGet(GFX_DriverInfo* info);
GFX_Result driverSSD1963ContextInitialize(GFX_Context* context);
</#macro>

<#macro DRV_GFX_SSD1963_DRIVER_DEFINITION
    ID
    IDX>
    GFX_DriverInterfaces[${ID}].infoGet = &driverSSD1963InfoGet;
    GFX_DriverInterfaces[${ID}].contextInitialize = &driverSSD1963ContextInitialize;
</#macro>