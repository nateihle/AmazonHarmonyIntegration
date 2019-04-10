/*******************************************************************************
  CAN Driver Dynamic to Static mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_mapping.c

  Summary:
    Source code for the CAN driver dynamic APIs to static API mapping.

  Description:
    This file contains code that maps dynamic APIs to static whenever
    the static mode of the driver is selected..

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"


SYS_MODULE_OBJ DRV_CAN_Initialize(const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init)
{
    SYS_MODULE_OBJ returnValue;

    switch(index)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            returnValue = DRV_CAN_INDEX_0;
            DRV_CAN0_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            returnValue = DRV_CAN_INDEX_1;
            DRV_CAN1_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            returnValue = DRV_CAN_INDEX_2;
            DRV_CAN2_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            returnValue = DRV_CAN_INDEX_3;
            DRV_CAN3_Initialize();
            break;
        }
</#if>
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }

    return returnValue;
}



void DRV_CAN_Deinitialize(SYS_MODULE_OBJ object)
{
    switch(object)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            DRV_CAN0_Deinitialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            DRV_CAN1_Deinitialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            DRV_CAN2_Deinitialize();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            DRV_CAN3_Deinitialize();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}



/* client interface */
DRV_HANDLE DRV_CAN_Open(const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent)
{
    DRV_HANDLE returnValue;

    switch(index)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            returnValue = DRV_CAN_INDEX_0;
            DRV_CAN0_Open();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            returnValue = DRV_CAN_INDEX_1;
            DRV_CAN1_Open();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            returnValue = DRV_CAN_INDEX_2;
            DRV_CAN2_Open();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            returnValue = DRV_CAN_INDEX_3;
            DRV_CAN3_Open();
            break;
        }
</#if>
        default:
        {
            returnValue = DRV_HANDLE_INVALID;
            break;
        }
    }

    return returnValue;
}



void DRV_CAN_Close(const DRV_HANDLE handle)
{
    switch(handle)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            DRV_CAN0_Close();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            DRV_CAN0_Close();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            DRV_CAN2_Close();
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            DRV_CAN3_Close();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}



bool DRV_CAN_ChannelMessageTransmit(const DRV_HANDLE handle,
        CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{
    bool returnValue = false;

    switch(handle)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            returnValue = DRV_CAN0_ChannelMessageTransmit(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            returnValue = DRV_CAN1_ChannelMessageTransmit(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            returnValue = DRV_CAN2_ChannelMessageTransmit(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            returnValue = DRV_CAN3_ChannelMessageTransmit(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
        default:
        {
            break;
        }
    }

    return returnValue;
}



bool DRV_CAN_ChannelMessageReceive(const DRV_HANDLE handle,
        CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{
    bool returnValue = false;

    switch(handle)
    {
<#if CONFIG_DRV_CAN_INST_IDX0!false>
        case DRV_CAN_INDEX_0:
        {
            returnValue = DRV_CAN0_ChannelMessageReceive(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
        case DRV_CAN_INDEX_1:
        {
            returnValue = DRV_CAN1_ChannelMessageReceive(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
        case DRV_CAN_INDEX_2:
        {
            returnValue = DRV_CAN2_ChannelMessageReceive(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
        case DRV_CAN_INDEX_3:
        {
            returnValue = DRV_CAN3_ChannelMessageReceive(channelNum, address,
                    DLC, message);
            break;
        }
</#if>
        default:
        {
            break;
        }
    }

    return returnValue;
}



/*******************************************************************************
 End of File
*/
