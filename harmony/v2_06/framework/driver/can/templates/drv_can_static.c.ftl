/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_static.c

  Summary:
    CAN driver implementation for the static single instance driver.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers.

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

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "driver/can/drv_can.h"

<#macro DRV_CAN_STATIC_FUNCTIONS DRV_INSTANCE CAN_INSTANCE CAN_SYNC CAN_TSPROP CAN_BR_PRESCALE CAN_TSPHASE1
CAN_TSPHASE2 CAN_OPMODE CAN_INT_SRC CAN_INT_VEC CAN_INT_PRI CAN_INT_SPRI DRV_CAN_CHANNELS_NUMBER CAN_MASK0 CAN_MASK1 CAN_MASK2 CAN_MASK3 CAN_INTERRUPT_MODE>

static CAN_TX_MSG_BUFFER  *drv_Message${DRV_INSTANCE};
static CAN_TX_MSG_BUFFER <#if CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true || CONFIG_PIC32MK == true>__attribute__((coherent, aligned(16)))</#if> can_message_buffer${DRV_INSTANCE}[${DRV_CAN_CHANNELS_NUMBER}*2*16];


// *****************************************************************************
// *****************************************************************************
// Section: Instance ${DRV_INSTANCE} static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CAN${DRV_INSTANCE}_Initialize(void)
{

    /* Switch the CAN module ON */
    PLIB_CAN_Enable(${CAN_INSTANCE});

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(${CAN_INSTANCE}, CAN_CONFIGURATION_MODE);
    while(PLIB_CAN_OperationModeGet(${CAN_INSTANCE}) != CAN_CONFIGURATION_MODE);

    PLIB_CAN_PhaseSegment2LengthFreelyProgrammableEnable(${CAN_INSTANCE});

    //Set the Baud rate to ${CONFIG_DRV_CAN_BAUD_RATE_IDX0} kbps
    PLIB_CAN_PropagationTimeSegmentSet(${CAN_INSTANCE}, ${CAN_TSPROP}-1);
    PLIB_CAN_PhaseSegment1LengthSet(${CAN_INSTANCE}, ${CAN_TSPHASE1}-1);
    PLIB_CAN_PhaseSegment2LengthSet(${CAN_INSTANCE}, ${CAN_TSPHASE2}-1);
    PLIB_CAN_SyncJumpWidthSet(${CAN_INSTANCE}, ${CAN_SYNC}-1);
    PLIB_CAN_BaudRatePrescaleSet(${CAN_INSTANCE}, ${CAN_BR_PRESCALE}); // set to 1 higher then ECAN tool


    /* Assign the buffer area to the CAN module.
      In this case assign enough memory for 2
      channels, each with 8 message buffers.*/
    PLIB_CAN_MemoryBufferAssign(${CAN_INSTANCE}, can_message_buffer${DRV_INSTANCE});

<#if DRV_INSTANCE == "0">
 <#if CONFIG_DRV_CAN_CHANNEL_IDX0 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX0} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX0?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX0 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX0}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX0}, CAN_CHANNEL0);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX0 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX0});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX1 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX1} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX1?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX1 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX1}, CAN_CHANNEL1);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX1 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX1});
     </#if>
     </#if>
<#if CONFIG_DRV_CAN_CHANNEL_IDX2 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX2} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX2?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX2 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX2}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX2}, CAN_CHANNEL2);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX2 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX2});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX3 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX3} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX3?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX3 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX3}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX3}, CAN_CHANNEL3);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX3 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX3});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX4 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX4} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX4?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX4 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX4}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX4}, CAN_CHANNEL4);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX4 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX4});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX5 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX5} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX5?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX5 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX5}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX5}, CAN_CHANNEL5);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX5 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX5});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX6 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX6} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX6?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX6 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX6}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX6}, CAN_CHANNEL6);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX6 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX6});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX7 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX7} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX7?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX7 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX7}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX7}, CAN_CHANNEL7);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX7 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX7});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX8 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX8} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX8?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX8 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX8}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX8}, CAN_CHANNEL8);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX8 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX8});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX9 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX9} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX9?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX9 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX9}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX9}, CAN_CHANNEL9);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX9 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX9});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX10 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX10} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX10?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX10 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX10}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX10}, CAN_CHANNEL10);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX10 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX10});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX11 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX11} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX11?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX11 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX11}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX11}, CAN_CHANNEL11);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX11 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX11});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX12 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX12} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX12?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX12 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX12}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX12}, CAN_CHANNEL12);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX12 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX12});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX13 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX13} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX13?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX13 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX13}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX13}, CAN_CHANNEL13);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX13 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX13});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX14 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX14} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX14?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX14 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX14}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX14}, CAN_CHANNEL14);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX14 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX14});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX15 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX15} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX15?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX15 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX15}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX15}, CAN_CHANNEL15);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX15 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX15});
     </#if>
     </#if>
<#if CONFIG_PIC32MK == false>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX16 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX16} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX16} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX16?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELSize_IDX16}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX16}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELSize_IDX16}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX16});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX16 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX16}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX16}, CAN_CHANNEL16);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX16 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX16});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX17 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX17} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX17} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX17?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELSize_IDX17}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX17}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELSize_IDX17}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX17});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX17 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX17}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX17}, CAN_CHANNEL17);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX17 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX17});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX18 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX18} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX18} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX18?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELSize_IDX18}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELSize_IDX18}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX18});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX18 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX17}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX18}, CAN_CHANNEL18);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX18 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX18});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX19 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX19} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX19} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX19?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX19}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX19}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL19, ${CONFIG_DRV_CAN_CHANNELSize_IDX19}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX19});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX19 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX19}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX19}, CAN_CHANNEL19);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX19 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL19, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX19});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX20 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX20} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX20} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX20?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELSize_IDX20}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX20}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELSize_IDX20}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX20});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX20 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX20}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX20}, CAN_CHANNEL20);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX20 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX20});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX21 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX21} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX21} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX21?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELSize_IDX21}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELSize_IDX21}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX21});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX21 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX21}, CAN_CHANNEL21);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX21 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX21});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX22 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX22} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX22} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX22?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX22}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX22}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL22, ${CONFIG_DRV_CAN_CHANNELSize_IDX22}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX22});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX22 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX22}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX22}, CAN_CHANNEL22);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX22 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL22, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX22});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX23 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX23} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX23} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX23?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELSize_IDX23}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX23}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELSize_IDX23}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX23});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX23 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX23}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX23}, CAN_CHANNEL23);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX23 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX23});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX24 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX24} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX24} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX24?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELSize_IDX24}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX24}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELSize_IDX24}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX24});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX24 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX24}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX24}, CAN_CHANNEL24);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX24 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX24});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX25 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX25} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX25} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX25?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELSize_IDX25}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX25}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELSize_IDX25}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX25});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX25 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX25}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX25}, CAN_CHANNEL25);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX25 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX25});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX26 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX26} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX26} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX26?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELSize_IDX26}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX26}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELSize_IDX26}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX26});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX26 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX26}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX26}, CAN_CHANNEL26);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX26 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX26});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX27 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX27} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX27} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX27?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELSize_IDX27}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX27}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELSize_IDX27}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX27});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX27 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX27}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX27}, CAN_CHANNEL27);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX27 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX27});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX28 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX28} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX28} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX28?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELSize_IDX28}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELSize_IDX28}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX28});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX28 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX28}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX28}, CAN_CHANNEL28);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX28 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX28});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX29 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX29} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX29} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX29?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELSize_IDX29}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX29}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELSize_IDX29}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX29});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX29 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX29}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX29}, CAN_CHANNEL29);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX29 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX29});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX30 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX30} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX30} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX30?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX30}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX30}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL30, ${CONFIG_DRV_CAN_CHANNELSize_IDX30}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX30});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX30 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX30}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX30}, CAN_CHANNEL30);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX30 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL30, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX30});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_IDX31 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX31} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX31} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX31?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELSize_IDX31}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX31}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELSize_IDX31}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX31});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX31 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX31}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX31}, CAN_CHANNEL31);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX31 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX31});
     </#if>
     </#if></#if></#if>
     <#if DRV_INSTANCE == "1">
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX0 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX0} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX0} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX0?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize1_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize1_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX0});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX0 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX0}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX0}, CAN_CHANNEL0);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX0 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX0});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX1 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX1} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX1?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize1_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize1_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX1 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX1}, CAN_CHANNEL1);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX1 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX1});
     </#if>
     </#if>
<#if CONFIG_DRV_CAN_CHANNEL1_IDX2 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX2} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX2} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX2?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize1_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX2}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize1_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX2});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX2 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX2}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX2}, CAN_CHANNEL2);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX2 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX2});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX3 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX3} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX3} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX3?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize1_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX3}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELSize1_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX3});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX3 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX3}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX3}, CAN_CHANNEL3);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX3 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX3});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX4 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX4} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX4} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX4?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize1_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX4}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize1_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX4});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX4 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX4}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX4}, CAN_CHANNEL4);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX4 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX4});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX5 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX5} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX5} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX5?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize1_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX5}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize1_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX5});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX5 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX5}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX5}, CAN_CHANNEL5);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX5 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX5});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX6 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX6} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX6} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX6?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize1_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX6}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize1_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX6});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX6 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX6}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX6}, CAN_CHANNEL6);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX6 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX6});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX7 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX7} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX7} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX7?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize1_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize1_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX7});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX7 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX7}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX7}, CAN_CHANNEL7);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX7 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX7});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX8 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX8} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX8} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX8?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize1_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX8}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize1_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX8});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX8 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX8}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX8}, CAN_CHANNEL8);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX8 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX8});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX9 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX9} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX9} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX9?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize1_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX9}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize1_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX9});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX9 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX9}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX9}, CAN_CHANNEL9);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX9 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX9});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX10 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX10} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX10} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX10?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize1_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX10}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize1_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX10});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX10 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX10}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX10}, CAN_CHANNEL10);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX10 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX10});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX11 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX11} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX11} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX11?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize1_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize1_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX11});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX11 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX11}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX11}, CAN_CHANNEL11);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX11 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX11});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX12 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX12} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX12} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX12?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize1_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX12}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize1_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX12});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX12 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX12}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX12}, CAN_CHANNEL12);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX12 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX12});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX13 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX13} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX13} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX13?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize1_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX13}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize1_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX13});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX13 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX13}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX13}, CAN_CHANNEL13);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX13 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX13});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX14 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX14} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX14} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX14?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize1_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX14}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize1_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX14});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX14 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX14}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX14}, CAN_CHANNEL14);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX14 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX14});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX15 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX15} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX15} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX15?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize1_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX15}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize1_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX15});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX15 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX15}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX15}, CAN_CHANNEL15);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX15 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX15});
     </#if>
     </#if>
<#if CONFIG_PIC32MK == false>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX16 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX16} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX16} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX16?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELSize1_IDX16}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX16}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELSize1_IDX16}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX16});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX16 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX16}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX16}, CAN_CHANNEL16);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX16 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL16, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX16});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX17 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX17} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX17} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX17?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELSize1_IDX17}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX17}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELSize1_IDX17}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX17});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX17 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX17}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX17}, CAN_CHANNEL17);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX17 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL17, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX17});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX18 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX18} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX18} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX18?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELSize1_IDX18}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELSize1_IDX18}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX18});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX18 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX17}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX18}, CAN_CHANNEL18);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX18 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL18, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX18});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX19 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX19} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX19} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX19?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize1_IDX19}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX19}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL19, ${CONFIG_DRV_CAN_CHANNELSize1_IDX19}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX19});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX19 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX19}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX19}, CAN_CHANNEL19);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX19 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL19, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX19});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX20 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX20} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX20} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX20?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELSize1_IDX20}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX20}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELSize1_IDX20}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX20});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX20 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX20}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX20}, CAN_CHANNEL20);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX20 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL20, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX20});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX21 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX21} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX21} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX21?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELSize1_IDX21}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELSize1_IDX21}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX21});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX21 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX21}, CAN_CHANNEL21);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX21 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL21, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX21});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX22 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX22} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX22} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX22?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize1_IDX22}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX22}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL22, ${CONFIG_DRV_CAN_CHANNELSize1_IDX22}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX22});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX22 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX22}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX22}, CAN_CHANNEL22);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX22 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL22, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX22});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX23 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX23} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX23} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX23?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELSize1_IDX23}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX23}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELSize1_IDX23}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX23});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX23 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX23}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX23}, CAN_CHANNEL23);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX23 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL23, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX23});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX24 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX24} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX24} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX24?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELSize1_IDX24}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX24}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELSize1_IDX24}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX24});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX24 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX24}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX24}, CAN_CHANNEL24);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX24 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL24, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX24});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX25 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX25} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX25} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX25?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELSize1_IDX25}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX25}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELSize1_IDX25}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX25});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX25 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX25}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX25}, CAN_CHANNEL25);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX25 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL25, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX25});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX26 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX26} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX26} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX26?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELSize1_IDX26}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX26}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELSize1_IDX26}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX26});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX26 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX26}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX26}, CAN_CHANNEL26);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX26 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL26, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX26});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX27 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX27} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX27} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX27?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELSize1_IDX27}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX27}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELSize1_IDX27}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX27});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX27 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX27}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX27}, CAN_CHANNEL27);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX27 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL27, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX27});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX28 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX28} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX28} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX28?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELSize1_IDX28}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELSize1_IDX28}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX28});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX28 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX28}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX28}, CAN_CHANNEL28);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX28 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL28, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX28});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX29 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX29} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX29} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX29?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELSize1_IDX29}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX29}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELSize1_IDX29}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX29});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX29 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX29}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX29}, CAN_CHANNEL29);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX29 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL29, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX29});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX30 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX30} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX30} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX30?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize1_IDX30}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX30}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL30, ${CONFIG_DRV_CAN_CHANNELSize1_IDX30}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX30});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX30 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX30}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX30}, CAN_CHANNEL30);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX30 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL30, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX30});
     </#if>
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL1_IDX31 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX31} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize1_IDX31} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE1_IDX31?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELSize1_IDX31}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX31}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELSize1_IDX31}, ${CONFIG_DRV_CAN_CHANNELTYPE1_IDX31});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK1_IDX31 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER1_IDX31}, ${CONFIG_DRV_CAN_CHANNEL_MASK1_IDX31}, CAN_CHANNEL31);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS1_IDX31 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL31, ${CONFIG_DRV_CAN_CHANNELEVENT1_IDX31});
     </#if>
     </#if></#if></#if>
<#if DRV_INSTANCE == "2">
 <#if CONFIG_DRV_CAN_CHANNEL_IDX0 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX0} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX0?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX0 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX0}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX0}, CAN_CHANNEL0);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX0 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX0});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX1 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX1} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX1?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX1 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX1}, CAN_CHANNEL1);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX1 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX1});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX2 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX2} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX2?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX2 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX2}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX2}, CAN_CHANNEL2);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX2 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX2});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX3 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX3} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX3?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX3 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX3}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX3}, CAN_CHANNEL3);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX3 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX3});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX4 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX4} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX4?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX4 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX4}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX4}, CAN_CHANNEL4);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX4 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX4});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX5 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX5} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX5?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX5 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX5}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX5}, CAN_CHANNEL5);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX5 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX5});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX6 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX6} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX6?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX6 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX6}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX6}, CAN_CHANNEL6);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX6 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX6});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX7 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX7} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX7?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX7 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX7}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX7}, CAN_CHANNEL7);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX7 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX7});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX8 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX8} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX8?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX8 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX8}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX8}, CAN_CHANNEL8);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX8 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX8});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX9 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX9} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX9?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX9 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX9}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX9}, CAN_CHANNEL9);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX9 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX9});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX10 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX10} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX10?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX10 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX10}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX10}, CAN_CHANNEL10);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX10 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX10});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX11 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX11} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX11?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX11 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX11}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX11}, CAN_CHANNEL11);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX11 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX11});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX12 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX12} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX12?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX12 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX12}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX12}, CAN_CHANNEL12);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX12 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX12});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX13 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX13} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX13?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX13 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX13}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX13}, CAN_CHANNEL13);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX13 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX13});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX14 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX14} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX14?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX14 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX14}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX14}, CAN_CHANNEL14);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX14 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX14});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX15 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX15} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX15?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX15 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX15}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX15}, CAN_CHANNEL15);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX15 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX15});
     </#if>
 </#if>
</#if>
<#if DRV_INSTANCE == "3">
 <#if CONFIG_DRV_CAN_CHANNEL_IDX0 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX0} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX0?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELSize_IDX0}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX0});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX0 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX0}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX0}, CAN_CHANNEL0);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX0 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL0, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX0});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX1 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX1} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX1?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX1}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX1 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX1}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX1}, CAN_CHANNEL1);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX1 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX1});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX2 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX2} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX2?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELSize_IDX2}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX2});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX2 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX2}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX2}, CAN_CHANNEL2);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX2 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL2, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX2});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX3 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX3} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX3?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL1, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELSize_IDX3}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX3});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX3 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX3}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX3}, CAN_CHANNEL3);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX3 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL3, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX3});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX4 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX4} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX4?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELSize_IDX4}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX4});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX4 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX4}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX4}, CAN_CHANNEL4);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX4 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL4, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX4});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX5 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX5} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX5?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELSize_IDX5}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX5});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX5 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX5}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX5}, CAN_CHANNEL5);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX5 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL5, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX5});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX6 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX6} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX6?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELSize_IDX6}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX6});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX6 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX6}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX6}, CAN_CHANNEL6);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX6 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL6, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX6});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX7 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX7} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX7?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELSize_IDX7}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX7});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX7 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX7}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX7}, CAN_CHANNEL7);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX7 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL7, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX7});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX8 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX8} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX8?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELSize_IDX8}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX8});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX8 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX8}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX8}, CAN_CHANNEL8);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX8 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL8, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX8});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX9 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX9} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX9?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELSize_IDX9}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX9});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX9 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX9}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX9}, CAN_CHANNEL9);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX9 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL9, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX9});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX10 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX10} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX10?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELSize_IDX10}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX10});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX10 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX10}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX10}, CAN_CHANNEL10);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX10 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL10, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX10});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX11 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX11} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX11?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX1}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELSize_IDX11}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX11});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX11 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX11}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX11}, CAN_CHANNEL11);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX11 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL11, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX11});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX12 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX12} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX12?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELSize_IDX12}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX12});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX12 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX12}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX12}, CAN_CHANNEL12);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX12 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL12, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX12});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX13 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX13} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX13?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELSize_IDX13}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX13});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX13 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX13}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX13}, CAN_CHANNEL13);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX13 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL13, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX13});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX14 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX14} message buffer, and assign low medium priority for transmissions. */
    <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX14?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELSize_IDX14}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX14});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX14 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX14}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX14}, CAN_CHANNEL14);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX14 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL14, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX14});
     </#if>
 </#if>
 <#if CONFIG_DRV_CAN_CHANNEL_IDX15 == true>
    /* Configure ${CAN_INSTANCE} Channel for ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15} operation. Allocate ${CONFIG_DRV_CAN_CHANNELSize_IDX15} message buffer, and assign low medium priority for transmissions. */
     <#if CONFIG_DRV_CAN_CHANNELTYPE_IDX15?contains("CAN_TX")>
    PLIB_CAN_ChannelForTransmitSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15}, CAN_LOW_MEDIUM_PRIORITY);
     <#else>
    PLIB_CAN_ChannelForReceiveSet(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELSize_IDX15}, ${CONFIG_DRV_CAN_CHANNELTYPE_IDX15});
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_FILTERMASK_IDX15 == true>
    PLIB_CAN_FilterToChannelLink(${CAN_INSTANCE}, ${CONFIG_DRV_CAN_CHANNEL_FILTER_IDX15}, ${CONFIG_DRV_CAN_CHANNEL_MASK_IDX15}, CAN_CHANNEL15);
     </#if>
     <#if CONFIG_DRV_CAN_CHANNEL_USEEVENTS_IDX15 == true>
    PLIB_CAN_ChannelEventEnable(${CAN_INSTANCE}, CAN_CHANNEL15, ${CONFIG_DRV_CAN_CHANNELEVENT_IDX15});
     </#if>
 </#if>
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX0 == true || CONFIG_DRV_CAN_FILT1_IDX0 == true || CONFIG_DRV_CAN_FILT2_IDX0 == true || CONFIG_DRV_CAN_FILT3_IDX0 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX0 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER0, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX0}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX0});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX0 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER0, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX0}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX0});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX0 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER0, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX0}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX0});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX0 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER0, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX0}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX0});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER0);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX1 == true || CONFIG_DRV_CAN_FILT1_IDX1  == true || CONFIG_DRV_CAN_FILT2_IDX1 == true || CONFIG_DRV_CAN_FILT3_IDX1 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX1 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER1, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX1}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX1});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX1 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER1, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX1}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX1})
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX1 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER1, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX1}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX1});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX1 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER1, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX1}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX1});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER1);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX2 == true || CONFIG_DRV_CAN_FILT1_IDX2 == true || CONFIG_DRV_CAN_FILT2_IDX2 == true || CONFIG_DRV_CAN_FILT3_IDX2 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX2 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER2, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX2}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX2});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX2 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER2, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX2}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX2});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX2 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER2, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX2}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX2});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX2 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER2, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX2}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX2});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER2);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX3 == true || CONFIG_DRV_CAN_FILT1_IDX3 == true || CONFIG_DRV_CAN_FILT2_IDX3 == true || CONFIG_DRV_CAN_FILT3_IDX3 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX3 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER3, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX3}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX3});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX3 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER3, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX3}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX3});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX3 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER3, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX3}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX3});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX3 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER3, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX3}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX3});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER3);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX4 == true || CONFIG_DRV_CAN_FILT1_IDX4 == true || CONFIG_DRV_CAN_FILT2_IDX4 == true || CONFIG_DRV_CAN_FILT3_IDX4 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX4 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER4, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX4}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX4});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX4 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER4, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX4}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX4});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX4 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER4, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX4}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX4});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX4 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER4, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX4}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX4});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER4);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX5 == true || CONFIG_DRV_CAN_FILT1_IDX5 == true || CONFIG_DRV_CAN_FILT2_IDX5 == true || CONFIG_DRV_CAN_FILT3_IDX5 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX5 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER5, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX5}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX5});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX5 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER5, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX5}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX5});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX5 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER5, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX5}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX5});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX5 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER5, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX5}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX5});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER5);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX6 == true || CONFIG_DRV_CAN_FILT1_IDX6 == true || CONFIG_DRV_CAN_FILT2_IDX6 == true || CONFIG_DRV_CAN_FILT3_IDX6 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX6 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER6, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX6}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX6});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX6 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER6, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX6}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX6});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX6 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER6, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX6}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX6});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX6 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER6, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX6}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX6});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER6);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX7 == true || CONFIG_DRV_CAN_FILT1_IDX7 == true || CONFIG_DRV_CAN_FILT2_IDX7 == true || CONFIG_DRV_CAN_FILT3_IDX7 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX7 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER7, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX7}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX7});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX7 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER7, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX7}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX7});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX7 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER7, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX7}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX7});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX7 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER7, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX7}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX7});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER7);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX8 == true || CONFIG_DRV_CAN_FILT1_IDX8 == true || CONFIG_DRV_CAN_FILT2_IDX8 == true || CONFIG_DRV_CAN_FILT3_IDX8 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX8 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER8, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX8}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX8});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX8 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER8, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX8}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX8});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX8 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER8, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX8}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX8});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX8 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER8, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX8}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX8});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER8);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX9 == true || CONFIG_DRV_CAN_FILT1_IDX9 == true || CONFIG_DRV_CAN_FILT2_IDX9 == true || CONFIG_DRV_CAN_FILT3_IDX9 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX9 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER9, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX9}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX9});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX9 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER9, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX9}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX9});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX9 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER9, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX9}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX9});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX9 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER9, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX9}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX9});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER9);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX10 == true || CONFIG_DRV_CAN_FILT1_IDX10 == true || CONFIG_DRV_CAN_FILT2_IDX10 == true || CONFIG_DRV_CAN_FILT3_IDX10 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX10 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER10, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX10}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX10});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX10 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER10, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX10}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX10});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX10 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER10, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX10}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX10});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX10 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER10, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX10}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX10});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER10);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX11 == true || CONFIG_DRV_CAN_FILT1_IDX11 == true || CONFIG_DRV_CAN_FILT2_IDX11 == true || CONFIG_DRV_CAN_FILT3_IDX11 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX11 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER11, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX11}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX11});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX11 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER11, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX11}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX11});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX11 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER11, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX11}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX11});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX11 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER11, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX11}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX11});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER11);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX12 == true || CONFIG_DRV_CAN_FILT1_IDX12 == true || CONFIG_DRV_CAN_FILT2_IDX12 == true || CONFIG_DRV_CAN_FILT3_IDX12 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX12 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER12, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX12}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX12});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX12 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER12, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX12}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX12});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX12 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER12, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX12}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX12});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX12 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER12, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX12}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX12});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER12);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX13 == true || CONFIG_DRV_CAN_FILT1_IDX13 == true || CONFIG_DRV_CAN_FILT2_IDX13 == true || CONFIG_DRV_CAN_FILT3_IDX13 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX13 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER13, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX13}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX13});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX13 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER13, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX13}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX13});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX13 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER13, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX13}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX13});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX13 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER13, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX13}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX13});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER13);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX14 == true || CONFIG_DRV_CAN_FILT1_IDX14 == true || CONFIG_DRV_CAN_FILT2_IDX14 == true || CONFIG_DRV_CAN_FILT3_IDX14 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX14 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER14, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX14}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX14});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX14 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER14, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX14}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX14});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX14 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER14, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX14}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX14});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX14 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER14, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX14}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX14});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER14);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX15 == true || CONFIG_DRV_CAN_FILT1_IDX15 == true || CONFIG_DRV_CAN_FILT2_IDX15 == true || CONFIG_DRV_CAN_FILT3_IDX15 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX15 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER15, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX15}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX15});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX15 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER15, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX15}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX15});
 </#if>
 <#if CONFIG_DRV_CAN_FILT2_IDX15 == true && DRV_INSTANCE == "2">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER15, ${CONFIG_DRV_CAN_FILTERACCEPTANCE2_IDX15}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER2_IDX15});
 </#if>
 <#if CONFIG_DRV_CAN_FILT3_IDX15 == true && DRV_INSTANCE == "3">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER15, ${CONFIG_DRV_CAN_FILTERACCEPTANCE3_IDX15}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER3_IDX15});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER15);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX16 == true || CONFIG_DRV_CAN_FILT1_IDX16 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX16 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER16, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX16}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX16});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX16 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER16, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX16}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX16});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER16);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX17 == true || CONFIG_DRV_CAN_FILT1_IDX17 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX17 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER17, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX17}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX17});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX17 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER17, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX17}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX17});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER17);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX18 == true || CONFIG_DRV_CAN_FILT1_IDX18 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX18 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER18, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX18}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX18});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX18 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER18, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX18}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX18});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER18);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX19 == true || CONFIG_DRV_CAN_FILT1_IDX19 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX19 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER19, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX19}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX19});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX19 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER19, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX19}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX19});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER19);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX20 == true || CONFIG_DRV_CAN_FILT1_IDX20 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX20 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER20, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX20}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX20});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX20 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER20, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX20}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX20});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER20);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX21 == true || CONFIG_DRV_CAN_FILT1_IDX21 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX21 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER21, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX21}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX21});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX21 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER21, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX21}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX21});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER21);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX22 == true || CONFIG_DRV_CAN_FILT1_IDX22 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX22 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER22, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX22}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX22});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX22 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER22, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX22}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX22});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER22);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX23 == true || CONFIG_DRV_CAN_FILT1_IDX23 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX23 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER23, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX23}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX23});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX23 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER23, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX23}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX23});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER23);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX24 == true || CONFIG_DRV_CAN_FILT1_IDX24 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX24 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER24, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX24}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX24});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX24 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER24, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX24}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX24});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER24);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX25 == true || CONFIG_DRV_CAN_FILT1_IDX25 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX25 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER25, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX25}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX25});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX25 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER25, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX25}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX25});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER25);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX26 == true || CONFIG_DRV_CAN_FILT1_IDX26 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX26 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER26, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX26}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX26});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX26 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER26, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX26}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX26});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER26);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX27 == true || CONFIG_DRV_CAN_FILT1_IDX27 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX27 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER27, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX27}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX27});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX27 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER27, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX27}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX27});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER27);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX28 == true || CONFIG_DRV_CAN_FILT1_IDX28 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX28 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER28, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX28}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX28});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX28 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER28, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX28}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX28});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER28);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX29 == true || CONFIG_DRV_CAN_FILT1_IDX29 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX29 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER29, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX29}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX29});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX29 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER29, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX29}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX29});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER29);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX30 == true || CONFIG_DRV_CAN_FILT1_IDX30 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX30 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER30, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX30}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX30});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX30 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER30, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX30}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX30});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER30);
</#if>
<#if CONFIG_DRV_CAN_FILT_IDX31 == true || CONFIG_DRV_CAN_FILT1_IDX31 == true>
 <#if CONFIG_DRV_CAN_FILT_IDX31 == true && DRV_INSTANCE == "0">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER31, ${CONFIG_DRV_CAN_FILTERACCEPTANCE_IDX31}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER_IDX31});
 </#if>
 <#if CONFIG_DRV_CAN_FILT1_IDX31 == true && DRV_INSTANCE == "1">
    PLIB_CAN_FilterConfigure(${CAN_INSTANCE}, CAN_FILTER31, ${CONFIG_DRV_CAN_FILTERACCEPTANCE1_IDX31}, ${CONFIG_DRV_CAN_FILTERIDENTIFIER1_IDX31});
 </#if>
    PLIB_CAN_FilterEnable(${CAN_INSTANCE}, CAN_FILTER31);
</#if>

<#if CAN_MASK0 == true>
 <#if DRV_INSTANCE == "0">
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK0, ${CONFIG_DRV_CAN_MASKACCEPTANCE_IDX0}, ${CONFIG_DRV_CAN_MASKIDENTIFIER_IDX0}, ${CONFIG_DRV_CAN_MASKTYPE_IDX0});
 <#else>
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK0, ${CONFIG_DRV_CAN_MASKACCEPTANCE1_IDX0}, ${CONFIG_DRV_CAN_MASKIDENTIFIER1_IDX0}, ${CONFIG_DRV_CAN_MASKTYPE1_IDX0});
 </#if>
</#if>
<#if CAN_MASK1 == true>
 <#if DRV_INSTANCE == "0">
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK1, ${CONFIG_DRV_CAN_MASKACCEPTANCE_IDX1}, ${CONFIG_DRV_CAN_MASKIDENTIFIER_IDX1}, ${CONFIG_DRV_CAN_MASKTYPE_IDX1});
 <#else>
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK1, ${CONFIG_DRV_CAN_MASKACCEPTANCE1_IDX1}, ${CONFIG_DRV_CAN_MASKIDENTIFIER1_IDX1}, ${CONFIG_DRV_CAN_MASKTYPE1_IDX1});
 </#if>
</#if>
<#if CAN_MASK2 == true>
 <#if DRV_INSTANCE == "0">
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK2, ${CONFIG_DRV_CAN_MASKACCEPTANCE_IDX2}, ${CONFIG_DRV_CAN_MASKIDENTIFIER_IDX2}, ${CONFIG_DRV_CAN_MASKTYPE_IDX2});
 <#else>
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK2, ${CONFIG_DRV_CAN_MASKACCEPTANCE1_IDX2}, ${CONFIG_DRV_CAN_MASKIDENTIFIER_IDX2}, ${CONFIG_DRV_CAN_MASKTYPE1_IDX2});
 </#if>
</#if>
<#if CAN_MASK3 == true>
 <#if DRV_INSTANCE == "0">
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK3, ${CONFIG_DRV_CAN_MASKACCEPTANCE_IDX3}, ${CONFIG_DRV_CAN_MASKIDENTIFIER_IDX3}, ${CONFIG_DRV_CAN_MASKTYPE_IDX3});
 <#else>
    PLIB_CAN_FilterMaskConfigure(${CAN_INSTANCE}, CAN_FILTER_MASK3, ${CONFIG_DRV_CAN_MASKACCEPTANCE1_IDX3}, ${CONFIG_DRV_CAN_MASKIDENTIFIER1_IDX3}, ${CONFIG_DRV_CAN_MASKTYPE1_IDX3});
 </#if>
</#if>

    /* Switch the CAN module to Normal mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(${CAN_INSTANCE}, ${CAN_OPMODE});
    while(PLIB_CAN_OperationModeGet(${CAN_INSTANCE}) != ${CAN_OPMODE});

    <#if CAN_INTERRUPT_MODE == true>
    <#if DRV_INSTANCE == "0">
    PLIB_CAN_ModuleEventEnable(${CAN_INSTANCE} , 0<#if CONFIG_DRV_CAN_TX_EVENT_IDX0==true>|CAN_TX_EVENT</#if><#if CONFIG_DRV_CAN_RX_EVENT_IDX0==true>|CAN_RX_EVENT</#if><#if CONFIG_DRV_CAN_TSOVERFLOW_EVENT_IDX0==true>|CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_MODECHANGE_EVENT_IDX0==true>|CAN_OPERATION_MODE_CHANGE_EVENT</#if><#if CONFIG_DRV_CAN_RXOVERFLOW_EVENT_IDX0==true>|CAN_RX_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_SYSERROR_EVENT_IDX0==true>|CAN_SYSTEM_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_BUSERROR_EVENT_IDX0==true>|CAN_BUS_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_WAKEUP_EVENT_IDX0 == true>|CAN_BUS_ACTIVITY_WAKEUP_EVENT</#if><#if CONFIG_DRV_CAN_INVALIDRX_IDX0==true>|CAN_INVALID_RX_MESSAGE_EVENT</#if><#if CONFIG_DRV_CAN_ALLEVENTS_IDX0==true>|CAN_All_EVENTS</#if>);</#if>
    <#if DRV_INSTANCE == "1">
    PLIB_CAN_ModuleEventEnable(${CAN_INSTANCE} , 0<#if CONFIG_DRV_CAN_TX_EVENT_IDX1==true>|CAN_TX_EVENT</#if><#if CONFIG_DRV_CAN_RX_EVENT_IDX1==true>|CAN_RX_EVENT</#if><#if CONFIG_DRV_CAN_TSOVERFLOW_EVENT_IDX1==true>|CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_MODECHANGE_EVENT_IDX1==true>|CAN_OPERATION_MODE_CHANGE_EVENT</#if><#if CONFIG_DRV_CAN_RXOVERFLOW_EVENT_IDX1==true>|CAN_RX_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_SYSERROR_EVENT_IDX1==true>|CAN_SYSTEM_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_BUSERROR_EVENT_IDX1==true>|CAN_BUS_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_WAKEUP_EVENT_IDX1==true>|CAN_BUS_ACTIVITY_WAKEUP_EVENT</#if><#if CONFIG_DRV_CAN_INVALIDRX_IDX1==true>|CAN_INVALID_RX_MESSAGE_EVENT</#if><#if CONFIG_DRV_CAN_ALLEVENTS_IDX1==true>|CAN_All_EVENTS</#if>);</#if>
    <#if DRV_INSTANCE == "2">
    PLIB_CAN_ModuleEventEnable(${CAN_INSTANCE} , 0<#if CONFIG_DRV_CAN_TX_EVENT_IDX2==true>|CAN_TX_EVENT</#if><#if CONFIG_DRV_CAN_RX_EVENT_IDX2==true>|CAN_RX_EVENT</#if><#if CONFIG_DRV_CAN_TSOVERFLOW_EVENT_IDX2==true>|CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_MODECHANGE_EVENT_IDX2==true>|CAN_OPERATION_MODE_CHANGE_EVENT</#if><#if CONFIG_DRV_CAN_RXOVERFLOW_EVENT_IDX2==true>|CAN_RX_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_SYSERROR_EVENT_IDX2==true>|CAN_SYSTEM_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_BUSERROR_EVENT_IDX2==true>|CAN_BUS_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_WAKEUP_EVENT_IDX2==true>|CAN_BUS_ACTIVITY_WAKEUP_EVENT</#if><#if CONFIG_DRV_CAN_INVALIDRX_IDX2==true>|CAN_INVALID_RX_MESSAGE_EVENT</#if><#if CONFIG_DRV_CAN_ALLEVENTS_IDX2==true>|CAN_All_EVENTS</#if>);</#if>
    <#if DRV_INSTANCE == "3">
    PLIB_CAN_ModuleEventEnable(${CAN_INSTANCE} , 0<#if CONFIG_DRV_CAN_TX_EVENT_IDX3==true>|CAN_TX_EVENT</#if><#if CONFIG_DRV_CAN_RX_EVENT_IDX3==true>|CAN_RX_EVENT</#if><#if CONFIG_DRV_CAN_TSOVERFLOW_EVENT_IDX3==true>|CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_MODECHANGE_EVENT_IDX3==true>|CAN_OPERATION_MODE_CHANGE_EVENT</#if><#if CONFIG_DRV_CAN_RXOVERFLOW_EVENT_IDX3==true>|CAN_RX_OVERFLOW_EVENT</#if><#if CONFIG_DRV_CAN_SYSERROR_EVENT_IDX3==true>|CAN_SYSTEM_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_BUSERROR_EVENT_IDX3==true>|CAN_BUS_ERROR_EVENT</#if><#if CONFIG_DRV_CAN_WAKEUP_EVENT_IDX3==true>|CAN_BUS_ACTIVITY_WAKEUP_EVENT</#if><#if CONFIG_DRV_CAN_INVALIDRX_IDX3==true>|CAN_INVALID_RX_MESSAGE_EVENT</#if><#if CONFIG_DRV_CAN_ALLEVENTS_IDX3==true>|CAN_All_EVENTS</#if>);</#if>

    /* Setup ${CAN_INSTANCE} Interrupt */
    PLIB_INT_SourceEnable(INT_ID_0,${CAN_INT_SRC});
    PLIB_INT_VectorPrioritySet(INT_ID_0,${CAN_INT_VEC}, ${CAN_INT_PRI});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0,${CAN_INT_VEC}, ${CAN_INT_SPRI});
    </#if>
}

void DRV_CAN${DRV_INSTANCE}_Deinitialize(void)
{

    /* Switch the CAN module to Disable mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(${CAN_INSTANCE}, CAN_DISABLE_MODE);
    while(PLIB_CAN_OperationModeGet(${CAN_INSTANCE}) != CAN_DISABLE_MODE);

}

void DRV_CAN${DRV_INSTANCE}_Open(void)
{
   /* Switch the CAN module ON */
    PLIB_CAN_Enable(${CAN_INSTANCE});
}

void DRV_CAN${DRV_INSTANCE}_Close(void)
{
   /* Switch the CAN module OFF */
    PLIB_CAN_Disable(${CAN_INSTANCE});
}

bool DRV_CAN${DRV_INSTANCE}_ChannelMessageTransmit(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{

    int count = 0;
    if ( (PLIB_CAN_ChannelEventGet(${CAN_INSTANCE}, channelNum) & CAN_TX_CHANNEL_NOT_FULL) == CAN_TX_CHANNEL_NOT_FULL)
    {
        //drv_Message${DRV_INSTANCE} = (CAN_TX_MSG_BUFFER *) &can_message_buffer${DRV_INSTANCE}[channelNum];
        drv_Message${DRV_INSTANCE} = PLIB_CAN_TransmitBufferGet(${CAN_INSTANCE}, channelNum);

        /* Check the address whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if(address > 0x7FF)
        {
            drv_Message${DRV_INSTANCE}->msgSID.sid     = (address>>18);
            drv_Message${DRV_INSTANCE}->msgEID.eid     = (0x3FFFF)&(address);
            drv_Message${DRV_INSTANCE}->msgEID.ide     = 1;
        }
        else
        {
            drv_Message${DRV_INSTANCE}->msgSID.sid     = address;
            drv_Message${DRV_INSTANCE}->msgEID.eid     = 0;
            drv_Message${DRV_INSTANCE}->msgEID.ide     = 0;
        }
        if (DLC > 8)
        {
            DLC = 8;
        }

        drv_Message${DRV_INSTANCE}->msgEID.data_length_code     = DLC;
        while(count < DLC)
        {
            drv_Message${DRV_INSTANCE}->data[count++] = *message++;
        }

        //Update CAN module and then transmit data on the bus;
       PLIB_CAN_ChannelUpdate(${CAN_INSTANCE}, channelNum);
       PLIB_CAN_TransmitChannelFlush(${CAN_INSTANCE}, channelNum);
        return(true);
    }
    return (false);
}

bool DRV_CAN${DRV_INSTANCE}_ChannelMessageReceive(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{
    int EchoDLC = 0;
    CAN_RX_MSG_BUFFER *RxMessage;
    uint32_t tempAddress;
    bool readStatus;
    CAN_CHANNEL_EVENT ChannelEvent;

    /* Get the channel RX status */
    ChannelEvent = PLIB_CAN_ChannelEventGet( ${CAN_INSTANCE} , channelNum );

    /* Check if there is a message available to read. */
    if( (ChannelEvent & CAN_RX_CHANNEL_NOT_EMPTY) == CAN_RX_CHANNEL_NOT_EMPTY )
    {
        /* There is a message available in the Channel FIFO. */

        /* Get a pointer to RX message buffer */
        RxMessage = (CAN_RX_MSG_BUFFER *)PLIB_CAN_ReceivedMessageGet(${CAN_INSTANCE}, channelNum);

        /* Process the message fields */

        /* Check if it's a extended message type */
        if(RxMessage->msgEID.ide)
        {
            tempAddress = (RxMessage->msgSID.sid << 18);
            tempAddress |= ((0x3FFFF)&(RxMessage->msgEID.eid));
        }
        else
        {
            tempAddress = RxMessage->msgSID.sid;
        }


        if (RxMessage->msgEID.data_length_code > 0)
        {

            if(tempAddress == address)
            {
                while(EchoDLC < RxMessage->msgEID.data_length_code)
                {
                     *message++ = RxMessage->data[EchoDLC++];
                }
            }
        }

        /* Message processing is done, update the message buffer pointer. */
        PLIB_CAN_ChannelUpdate(${CAN_INSTANCE}, channelNum);

        /* Message is processed successfully, so return true */
        readStatus = true;
    }
    else
    {
        /* There is no message to read ,so return false */
        readStatus = false;
    }

    return readStatus;
}

</#macro>
<#if CONFIG_DRV_CAN_INST_IDX0!false>
<@DRV_CAN_STATIC_FUNCTIONS DRV_INSTANCE="0" CAN_INSTANCE=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX0
CAN_SYNC=CONFIG_DRV_CAN_TS_SYNC_IDX0 CAN_TSPROP=CONFIG_DRV_CAN_TS_PROP_IDX0
CAN_BR_PRESCALE=CONFIG_DRV_CAN_BAUD_RATE_PRESCALER_IDX0 CAN_TSPHASE1=CONFIG_DRV_CAN_TS_PHASE1_IDX0
CAN_TSPHASE2=CONFIG_DRV_CAN_TS_PHASE2_IDX0 CAN_OPMODE= CONFIG_DRV_CAN_OPERATION_MODE_IDX0
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX0 CAN_INT_VEC=CONFIG_DRV_CAN_INTERRUPT_VECTOR_IDX0
CAN_INT_PRI=CONFIG_DRV_CAN_INT_PRIORITY_IDX0 CAN_INT_SPRI=CONFIG_DRV_CAN_INT_SUB_PRIORITY_IDX0 DRV_CAN_CHANNELS_NUMBER=CONFIG_DRV_CAN_CHANNELS_NUMBER0
CAN_MASK0=CONFIG_DRV_CAN_MASK_IDX0 CAN_MASK1=CONFIG_DRV_CAN_MASK_IDX1 CAN_MASK2=CONFIG_DRV_CAN_MASK_IDX2 CAN_MASK3=CONFIG_DRV_CAN_MASK_IDX3
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID0/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
<@DRV_CAN_STATIC_FUNCTIONS DRV_INSTANCE="1" CAN_INSTANCE=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX1
CAN_SYNC=CONFIG_DRV_CAN_TS_SYNC_IDX1 CAN_TSPROP=CONFIG_DRV_CAN_TS_PROP_IDX1
CAN_BR_PRESCALE=CONFIG_DRV_CAN_BAUD_RATE_PRESCALER_IDX1 CAN_TSPHASE1=CONFIG_DRV_CAN_TS_PHASE1_IDX1
CAN_TSPHASE2=CONFIG_DRV_CAN_TS_PHASE2_IDX1 CAN_OPMODE= CONFIG_DRV_CAN_OPERATION_MODE_IDX1
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX1 CAN_INT_VEC=CONFIG_DRV_CAN_INTERRUPT_VECTOR_IDX1
CAN_INT_PRI=CONFIG_DRV_CAN_INT_PRIORITY_IDX1 CAN_INT_SPRI=CONFIG_DRV_CAN_INT_SUB_PRIORITY_IDX1 DRV_CAN_CHANNELS_NUMBER=CONFIG_DRV_CAN_CHANNELS_NUMBER1
CAN_MASK0=CONFIG_DRV_CAN_MASK1_IDX0 CAN_MASK1=CONFIG_DRV_CAN_MASK1_IDX1 CAN_MASK2=CONFIG_DRV_CAN_MASK1_IDX2 CAN_MASK3=CONFIG_DRV_CAN_MASK1_IDX3
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID1/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
<@DRV_CAN_STATIC_FUNCTIONS DRV_INSTANCE="2" CAN_INSTANCE=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX2
CAN_SYNC=CONFIG_DRV_CAN_TS_SYNC_IDX2 CAN_TSPROP=CONFIG_DRV_CAN_TS_PROP_IDX2
CAN_BR_PRESCALE=CONFIG_DRV_CAN_BAUD_RATE_PRESCALER_IDX2 CAN_TSPHASE1=CONFIG_DRV_CAN_TS_PHASE1_IDX2
CAN_TSPHASE2=CONFIG_DRV_CAN_TS_PHASE2_IDX2 CAN_OPMODE= CONFIG_DRV_CAN_OPERATION_MODE_IDX2
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX2 CAN_INT_VEC=CONFIG_DRV_CAN_INTERRUPT_VECTOR_IDX2
CAN_INT_PRI=CONFIG_DRV_CAN_INT_PRIORITY_IDX2 CAN_INT_SPRI=CONFIG_DRV_CAN_INT_SUB_PRIORITY_IDX2 DRV_CAN_CHANNELS_NUMBER=CONFIG_DRV_CAN_CHANNELS_NUMBER2
CAN_MASK0=CONFIG_DRV_CAN_MASK2_IDX0 CAN_MASK1=CONFIG_DRV_CAN_MASK2_IDX1 CAN_MASK2=CONFIG_DRV_CAN_MASK2_IDX2 CAN_MASK3=CONFIG_DRV_CAN_MASK2_IDX3
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID2/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
<@DRV_CAN_STATIC_FUNCTIONS DRV_INSTANCE="3" CAN_INSTANCE=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX3
CAN_SYNC=CONFIG_DRV_CAN_TS_SYNC_IDX3 CAN_TSPROP=CONFIG_DRV_CAN_TS_PROP_IDX3
CAN_BR_PRESCALE=CONFIG_DRV_CAN_BAUD_RATE_PRESCALER_IDX3 CAN_TSPHASE1=CONFIG_DRV_CAN_TS_PHASE1_IDX3
CAN_TSPHASE2=CONFIG_DRV_CAN_TS_PHASE2_IDX3 CAN_OPMODE= CONFIG_DRV_CAN_OPERATION_MODE_IDX3
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX3 CAN_INT_VEC=CONFIG_DRV_CAN_INTERRUPT_VECTOR_IDX3
CAN_INT_PRI=CONFIG_DRV_CAN_INT_PRIORITY_IDX3 CAN_INT_SPRI=CONFIG_DRV_CAN_INT_SUB_PRIORITY_IDX3 DRV_CAN_CHANNELS_NUMBER=CONFIG_DRV_CAN_CHANNELS_NUMBER3
CAN_MASK0=CONFIG_DRV_CAN_MASK3_IDX0 CAN_MASK1=CONFIG_DRV_CAN_MASK3_IDX1 CAN_MASK2=CONFIG_DRV_CAN_MASK3_IDX2 CAN_MASK3=CONFIG_DRV_CAN_MASK3_IDX3
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID3/>
</#if>
/*******************************************************************************
 End of File
*/