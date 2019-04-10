<#--
/*******************************************************************************
  Wi-Fi Driver Initialization File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_wifi_init.c.ftl

  Summary:
    This file contains source code necessary to initialize the Wi-Fi driver.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the sysObj structure that
    contains the object handles to all the MPLAB Harmony module objects in
    the system.
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
<#if CONFIG_USE_DRV_WIFI_WK!false == true>
    IPC21bits.RFMACIP = 1;
    IPC21bits.RFMACIS = 0;

    IPC21bits.RFTM0IP = 1;
    IPC21bits.RFTM0IS = 0;

    MAC_Interrupt_enable();
</#if><#-- CONFIG_USE_DRV_WIFI_WK!false == true -->
<#if CONFIG_BSP_PIC32MX_ETH_SK!false || CONFIG_BSP_PIC32MX_ETH_SK2!false>
<#if CONFIG_DRV_WIFI_DEVICE == "MRF24WN">
    // On PIC32MX ESK, when CN9 (Pin G7) is used as external interrupt, read its value directly/indirectly
    // basing on the value of WF_INT_PORT_CHANNEL_READ and WF_INT_BIT_POS_READ.
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,
        SYS_PORTS_DIRECTION_INPUT,
        WF_INT_PORT_CHANNEL_READ,
        WF_INT_BIT_POS_READ);
<#elseif CONFIG_DRV_WIFI_DEVICE == "WINC1500">
    // On PIC32MX ESK, when CN9 (Pin G7) is used as external interrupt, read its value directly/indirectly
    // basing on the value of WF_INT_PORT_CHANNEL_READ and WF_INT_BIT_POS_READ.
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,
        SYS_PORTS_DIRECTION_INPUT,
        WDRV_IRQN_PORT_CHANNEL_READ,
        WDRV_IRQN_BIT_POS_READ);
<#elseif CONFIG_DRV_WIFI_DEVICE == "WILC1000">
    // On PIC32MX ESK, when CN9 (Pin G7) is used as external interrupt, read its value directly/indirectly
    // basing on the value of WF_INT_PORT_CHANNEL_READ and WF_INT_BIT_POS_READ.
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,
        SYS_PORTS_DIRECTION_INPUT,
        WDRV_IRQN_PORT_CHANNEL_READ,
        WDRV_IRQN_BIT_POS_READ);
</#if><#-- outer CONFIG_DRV_WIFI_DEVICE -->
</#if><#-- CONFIG_BSP_PIC32MX_ETH_SK!false || CONFIG_BSP_PIC32MX_ETH_SK2!false -->
<#--
/*******************************************************************************
 End of File
 */
-->
