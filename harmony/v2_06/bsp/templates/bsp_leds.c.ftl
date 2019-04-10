<#if (LED_Name_List?size > 0)>

// *****************************************************************************
/* Data Structure: 
    led_port_channel_map[]

  Summary:
    Maps each led to its port channel
  
  Description:
    The led_port_channel_map array, indexed by BSP_LED, maps each led to its 
    port channel.

  Remarks:
    Refer to bsp.h for usage information.
*/
static const PORTS_CHANNEL led_port_channel_map[] =
{
<#list LED_PortChannel_List as ledChannel>
    PORT_CHANNEL_${ledChannel}<#sep>,</#sep>
</#list>
};

// *****************************************************************************
/* Data Structure: 
    led_port_bit_pos_map[]

  Summary:
    Maps each led to its port bit position
  
  Description:
    The led_port_bit_pos_map array, indexed by BSP_LED, maps each led to its port 
    bit position.

  Remarks:
    Refer to bsp.h for usage information.
*/
static const PORTS_BIT_POS led_port_bit_pos_map[] =
{
<#list LED_PortPin_List as ledPinPos>
    PORTS_BIT_POS_${ledPinPos}<#sep>,</#sep>
</#list>
};

// *****************************************************************************
/* Data Structure: 
    led_active_level_map[]

  Summary:
    Maps each led to its active level
  
  Description:
    The led_active_level_map array, indexed by BSP_LED, maps each led to its active 
    level.

  Remarks:
    Refer to bsp.h for usage information.
*/
static const BSP_LED_ACTIVE_LEVEL led_active_level_map[] =
{
<#list LED_ActiveLevel_List as ledActiveLevel>
<#if ledActiveLevel = "High">   BSP_LED_ACTIVE_HIGH<#else>   BSP_LED_ACTIVE_LOW</#if><#sep>,</#sep>
</#list>
};

// *****************************************************************************
/* Function: 
    void BSP_LEDStateSet(BSP_LED led, BSP_LED_STATE state);

  Summary:
    Controls the state of the LED.
  
  Description:
    This function allows the application to specify the state of the LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDStateSet(BSP_LED led, BSP_LED_STATE state)
{
    /* Set the state of the LED */
    if(led_active_level_map[led] == BSP_LED_ACTIVE_HIGH)
    {
        PLIB_PORTS_PinWrite (PORTS_ID_0 , led_port_channel_map[led], led_port_bit_pos_map[led], 
							 (BSP_LED_STATE_ON == state ? true : false));
    }
    else
    {
        PLIB_PORTS_PinWrite (PORTS_ID_0 , led_port_channel_map[led], led_port_bit_pos_map[led], 
							 (BSP_LED_STATE_ON == state ? false : true));
    }
}

// *****************************************************************************
/* Function: 
    void BSP_LEDToggle(BSP_LED led);

  Summary:
    Toggles the state of the LED between BSP_LED_STATE_ON and BSP_LED_STATE_OFF.
  
  Description:
    This function toggles the state of the LED between BSP_LED_STATE_ON and
    BSP_LED_STATE_OFF.

  Remarks:
    Refer to bsp.h for usage information.
*/    

void BSP_LEDToggle(BSP_LED led)
{
    PLIB_PORTS_PinToggle(PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led] );
}

// *****************************************************************************
/* Function: 
    BSP_LED_STATE BSP_LEDStateGet(BSP_LED led);

  Summary:
    Returns the present state of the LED.
  
  Description:
    This function returns the present state of the LED.

  Remarks:
    Refer to bsp.h for usage information.
*/    

BSP_LED_STATE BSP_LEDStateGet (BSP_LED led)
{
    bool value;

    /* Get LED Status */
    value = PLIB_PORTS_PinGetLatched (PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led]);

    if(led_active_level_map[led] == BSP_LED_ACTIVE_LOW)
    {
        value = !value;
    }

    return (value ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOn(BSP_LED led);

  Summary:
    Switches ON the specified LED.
  
  Description:
    This function switches ON the specified LED.

  Remarks:
    Refer to bsp.h for usage information.
*/

void BSP_LEDOn(BSP_LED led)
{
    if(led_active_level_map[led] == BSP_LED_ACTIVE_HIGH)
    {
        PLIB_PORTS_PinSet( PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led] );
    }
    else
    {
        PLIB_PORTS_PinClear( PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led] );
    }
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOff(BSP_LED led);

  Summary:
    Switches Off the specified LED.
  
  Description:
    This function switches Off the specified LED.

  Remarks:
    Refer to bsp.h for usage information.
*/

void BSP_LEDOff(BSP_LED led)
{
    if(led_active_level_map[led] == BSP_LED_ACTIVE_HIGH)
    {
        PLIB_PORTS_PinClear( PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led] );
    }
    else
    {
        PLIB_PORTS_PinSet( PORTS_ID_0, led_port_channel_map[led], led_port_bit_pos_map[led] );
    }
}
</#if>
