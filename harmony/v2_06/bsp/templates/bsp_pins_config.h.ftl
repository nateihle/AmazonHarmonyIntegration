<#--
/*******************************************************************************
  BSP Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
   bsp_pins_config.h.ftl

  Summary:
    BSP Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->
/*** Application Defined Pins ***/
<#if (LED_Name_List?size > 0)>
<#list LED_Name_List as ledName>
<#list LED_PortChannel_List as ledChannel>
<#list LED_PortPin_List as ledPinPos>
<#list LED_ActiveLevel_List as ledActiveLevel>
<#if ledName?counter == ledChannel?counter><#if ledName?counter == ledPinPos?counter><#if ledName?counter == ledActiveLevel?counter>

/*** Functions for ${ledName} pin ***/
#define ${ledName}Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
<#if ledActiveLevel == "High">
#define ${ledName}On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
#define ${ledName}Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
#define ${ledName}StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
<#else>
#define ${ledName}On() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
#define ${ledName}Off() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})
#define ${ledName}StateGet() (!(PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_${ledChannel}, PORTS_BIT_POS_${ledPinPos})))
</#if>
</#if></#if></#if>
</#list>
</#list>
</#list>
</#list>
</#if>
<#if (Switch_Name_List?size > 0)>
<#list Switch_Name_List as SwitchName>
<#list Switch_PortChannel_List as SwitchChannel>
<#list Switch_PortPin_List as SwitchPinPos>
<#if SwitchName?counter == SwitchChannel?counter><#if SwitchName?counter == SwitchPinPos?counter>

/*** Functions for ${SwitchName} pin ***/
#define ${SwitchName}StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_${SwitchChannel}, PORTS_BIT_POS_${SwitchPinPos})
</#if></#if>
</#list>
</#list>
</#list>
</#if>
<#if (GPIO_OUT_Name_List?size > 0)>
<#list GPIO_OUT_Name_List as gpio_outName>
<#list GPIO_OUT_PortChannel_List as gpio_outChannel>
<#list GPIO_OUT_PortPin_List as gpio_outPinPos>
<#if gpio_outName?counter == gpio_outChannel?counter><#if gpio_outName?counter == gpio_outPinPos?counter>

/*** Functions for ${gpio_outName} pin ***/
#define ${gpio_outName}Toggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_${gpio_outChannel}, PORTS_BIT_POS_${gpio_outPinPos})
#define ${gpio_outName}On() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_${gpio_outChannel}, PORTS_BIT_POS_${gpio_outPinPos})
#define ${gpio_outName}Off() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_${gpio_outChannel}, PORTS_BIT_POS_${gpio_outPinPos})
#define ${gpio_outName}StateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_${gpio_outChannel}, PORTS_BIT_POS_${gpio_outPinPos})
#define ${gpio_outName}StateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_${gpio_outChannel}, PORTS_BIT_POS_${gpio_outPinPos}, Value)
</#if></#if>
</#list>
</#list>
</#list>
</#if>
<#if (GPIO_IN_Name_List?size > 0)>
<#list GPIO_IN_Name_List as gpio_inName>
<#list GPIO_IN_PortChannel_List as  gpio_inChannel>
<#list GPIO_IN_PortPin_List as  gpio_inPinPos>
<#if  gpio_inName?counter ==  gpio_inChannel?counter><#if  gpio_inName?counter ==  gpio_inPinPos?counter>

/*** Functions for ${ gpio_inName} pin ***/
#define ${ gpio_inName}StateGet() PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_${ gpio_inChannel}, PORTS_BIT_POS_${ gpio_inPinPos})
</#if></#if>
</#list>
</#list>
</#list>
</#if>
<#if (GPIO_Name_List?size > 0)>
<#list GPIO_Name_List as gpioName>
<#list GPIO_PortChannel_List as  gpioChannel>
<#list GPIO_PortPin_List as  gpioPinPos>
<#if  gpioName?counter ==  gpioChannel?counter><#if  gpioName?counter ==  gpioPinPos?counter>

/*** Functions for ${ gpioName} pin ***/
#define ${ gpioName}_PORT PORT_CHANNEL_${ gpioChannel}
#define ${ gpioName}_PIN PORTS_BIT_POS_${ gpioPinPos}
#define ${ gpioName}_PIN_MASK (0x1 << ${ gpioPinPos})
</#if></#if>
</#list>
</#list>
</#list>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->

