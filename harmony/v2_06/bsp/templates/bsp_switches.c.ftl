<#if (Switch_Name_List?size > 0)>

// *****************************************************************************
/* Data Structure: 
    switch_port_channel_map[]

  Summary:
    Maps each switch to its port channel
  
  Description:
    The switch_port_channel_map array, indexed by BSP_SWITCH, maps each switch 
    to its port channel.

  Remarks:
    Refer to bsp.h for usage information.
*/
static const PORTS_CHANNEL switch_port_channel_map[] =
{
<#list Switch_PortChannel_List as switchChannel>
    PORT_CHANNEL_${switchChannel}<#sep>,</#sep>
</#list>
};

// *****************************************************************************
/* Data Structure: 
    switch_port_bit_pos_map[]

  Summary:
    Maps each switch to its port bit position
  
  Description:
    The switch_port_bit_pos_map array, indexed by BSP_SWITCH, maps each switch to its port bit position

  Remarks:
    Refer to bsp.h for usage information.
*/
static const PORTS_BIT_POS switch_port_bit_pos_map[] =
{
<#list Switch_PortPin_List as switchPinPos>
    PORTS_BIT_POS_${switchPinPos}<#sep>,</#sep>
</#list>
};


// *****************************************************************************
/* Function: 
    void BSP_SwitchStateGet(BSP_SWITCH switch);

  Summary:
    Returns the present state (pressed or not pressed) of the specified switch.
  
  Description:
    This function returns the present state (pressed or not pressed) of the
    specified switch.

  Remarks:
    Refer to bsp.h for usage information.
*/

BSP_SWITCH_STATE BSP_SwitchStateGet( BSP_SWITCH bspSwitch )
{
    return ( PLIB_PORTS_PinGet(PORTS_ID_0, switch_port_channel_map[bspSwitch], switch_port_bit_pos_map[bspSwitch]) );
}
</#if>
