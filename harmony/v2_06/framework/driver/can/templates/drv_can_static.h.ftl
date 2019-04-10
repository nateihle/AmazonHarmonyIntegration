/*******************************************************************************
  CAN Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_static.h

  Summary:
    CAN driver interface declarations for the static single instance driver.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the CAN driver.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_CAN_STATIC_H
#define _DRV_CAN_STATIC_H

#include <sys/kmem.h>
#include "peripheral/can/plib_can.h"
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID0 == true || CONFIG_DRV_CAN_INTERRUPT_MODE_ID1 == true || CONFIG_DRV_CAN_INTERRUPT_MODE_ID2 == true || CONFIG_DRV_CAN_INTERRUPT_MODE_ID3 == true>
#include "peripheral/int/plib_int.h"
</#if>

#ifdef __cplusplus
    extern "C" {
#endif

<#macro DRV_CAN_STATIC_API DRV_INSTANCE>
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance ${DRV_INSTANCE} for the static driver
// *****************************************************************************
// *****************************************************************************
void DRV_CAN${DRV_INSTANCE}_Initialize(void);
void DRV_CAN${DRV_INSTANCE}_Deinitialize(void);
void DRV_CAN${DRV_INSTANCE}_Open(void);
void DRV_CAN${DRV_INSTANCE}_Close(void);
bool DRV_CAN${DRV_INSTANCE}_ChannelMessageTransmit(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message);
bool DRV_CAN${DRV_INSTANCE}_ChannelMessageReceive(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message);
</#macro>

<#if CONFIG_DRV_CAN_INST_IDX0!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="0"/>
</#if>

<#if CONFIG_DRV_CAN_INST_IDX1!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="1"/>
</#if>

<#if CONFIG_DRV_CAN_INST_IDX2!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="2"/>
</#if>

<#if CONFIG_DRV_CAN_INST_IDX3!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="3"/>
</#if>

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_CAN_STATIC_H

/*******************************************************************************
 End of File
*/
