<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
 // <editor-fold defaultstate="collapsed" desc="DRV_I2S Initialization Data">
/*** I2S Driver Initialization Data ***/
<#-- Instance 0 -->
<#if CONFIG_DRV_I2S_INST_IDX0 == true>
const DRV_I2S_INIT drvI2S0InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX0?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX0,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX0?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX0, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX0?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX0,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX0?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX0,
</#if>
<#if CONFIG_DRV_I2S_BAUD_RATE?has_content>
    .baud = DRV_I2S_BAUD_RATE,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX0?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX0,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX0?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX0,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX0?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX0,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX0?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX0,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX0?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX0,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX0?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX0,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX0?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX0,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX0?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX0,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX0?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX0,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX0?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX0,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX0?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX0,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX0?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX0,    
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX0,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX0,
</#if>   
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX0?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX0,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX0?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX0,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX0?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX0,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX0,
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX0,
</#if>   
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>

<#-- Instance 1 -->
<#if CONFIG_DRV_I2S_INST_IDX1 == true>
const DRV_I2S_INIT drvI2S1InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX1?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX1,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX1?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX1, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX1?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX1,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX1?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX1,
</#if>
<#if CONFIG_BAUD_RATE_IDX1?has_content>
    .baud = BAUD_RATE_IDX1,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX1?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX1,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX1?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX1,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX1?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX1,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX1?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX1,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX1?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX1,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX1?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX1,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX1?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX1,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX1?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX1,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX1?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX1,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX1?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX1,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX1?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX1,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX1?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX1,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX1?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX1,
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX1,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX1,
</#if>      
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX1?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX1,    
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX1?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX1,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX1?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX1,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX1,
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX1,
</#if>       
    
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>

<#-- Instance 2 -->
<#if CONFIG_DRV_I2S_INST_IDX2 == true>
const DRV_I2S_INIT drvI2S2InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX2?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX2,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX2?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX2, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX2?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX2,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX2?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX2,
</#if>
<#if CONFIG_BAUD_RATE_IDX2?has_content>
    .baud = BAUD_RATE_IDX2,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX2?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX2,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX2?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX2,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX2?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX2,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX2?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX2,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX2?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX2,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX2?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX2,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX2?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX2,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX2?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX2,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX2?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX2,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX2?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX2,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX2?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX2,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX2?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX2,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX2?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX2,
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX2,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX2,
</#if>    
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX2?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX2,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX2?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX2,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX2?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX2,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX2,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX2,
</#if>       
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>

<#-- Instance 3 -->
<#if CONFIG_DRV_I2S_INST_IDX3 == true>
const DRV_I2S_INIT drvI2S3InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX3?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX3,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX3?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX3, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX3?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX3,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX3?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX3,
</#if>
<#if CONFIG_BAUD_RATE_IDX3?has_content>
    .baud = BAUD_RATE_IDX3,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX3?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX3,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX3?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX3,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX3?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX3,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX3?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX3,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX3?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX3,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX3?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX3,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX3?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX3,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX3?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX3,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX3?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX3,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX3?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX3,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX3?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX3,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX3?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX3,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX3?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX3,
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX3,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX3,
</#if>     
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX3?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX3,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX3?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX3,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX3?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX3,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX3,
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX3,
</#if>      
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>

<#-- Instance 4 -->
<#if CONFIG_DRV_I2S_INST_IDX4 == true>
const DRV_I2S_INIT drvI2S4InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX4?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX4,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX4?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX4, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX4?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX4,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX4?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX4,
</#if>
<#if CONFIG_BAUD_RATE_IDX4?has_content>
    .baud = BAUD_RATE_IDX4,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX4?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX4,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX4?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX4,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX4?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX4,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX4?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX4,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX4?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX4,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX4?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX4,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX4?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX4,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX4?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX4,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX4?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX4,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX4?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX4,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX4?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX4,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX4?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX4,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX4?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX4,
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX4,  
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX4,
</#if>     
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX4?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX4,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX4?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX4,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX4?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX4,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX4,
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX4,
</#if>     
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>

<#-- Instance 5 -->
<#if CONFIG_DRV_I2S_INST_IDX5 == true>
const DRV_I2S_INIT drvI2S5InitData =
{
<#if CONFIG_DRV_I2S_POWER_STATE_IDX5?has_content>
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX5,
</#if>
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX5?has_content>
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX5, 
</#if>
<#if CONFIG_DRV_I2S_USAGE_MODE_IDX5?has_content>
    .usageMode = DRV_I2S_USAGE_MODE_IDX5,
</#if>
<#if CONFIG_SPI_BAUD_RATE_CLK_IDX5?has_content>
    .baudClock = SPI_BAUD_RATE_CLK_IDX5,
</#if>
<#if CONFIG_BAUD_RATE_IDX5?has_content>
    .baud = BAUD_RATE_IDX5,
</#if>
<#if CONFIG_DRV_I2S_CLK_MODE_IDX0?has_content>
    .clockMode = DRV_I2S_CLK_MODE_IDX5,
</#if>
<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX5?has_content>
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX5,
</#if>
<#if CONFIG_SPI_AUDIO_TRANSMIT_MODE_IDX5?has_content>
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX5,
</#if>
<#if CONFIG_SPI_INPUT_SAMPLING_PHASE_IDX5?has_content>
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX5,
</#if>
<#if CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX5?has_content>
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX5,
</#if>
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX5?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX5,
</#if>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX5?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX5,
</#if>
<#if CONFIG_DRV_I2S_ERR_INT_SRC_IDX5?has_content>
    .errorInterruptSource = DRV_I2S_ERR_INT_SRC_IDX5,
</#if>
</#if>
<#if CONFIG_QUEUE_SIZE_TX_IDX5?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX5,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX5?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX5,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHANNEL_IDX5?has_content>
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX5,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX5?has_content>
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX5,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_TX_INT_SRC_IDX5?has_content>
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX5,
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX5,    
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX5,
</#if>     
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_TRANSMIT_DMA == false>
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHANNEL_IDX5?has_content>
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX5,
</#if>
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
<#if CONFIG_DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX5?has_content>
    .dmaChaningChannelReceive = DRV_I2S_RX_DMA_CHAINING_CHANNEL_IDX5,
</#if>
</#if>
<#if CONFIG_DRV_I2S_DMA_INTERRUPTS_ENABLE == true>
<#if CONFIG_DRV_I2S_RX_INT_SRC_IDX5?has_content>
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX5,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX5,
<#if CONFIG_DRV_I2S_DMA_USE_CHANNEL_CHAINING == true>
    .dmaInterruptChainingReceiveSource = DRV_I2S_RX_DMA_CHAINING_SOURCE_IDX5,
</#if>     
</#if>
</#if>
</#if>
<#if CONFIG_DRV_I2S_SUPPORT_RECEIVE_DMA == false>
    .dmaChannelReceive = DMA_CHANNEL_NONE,
</#if>
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
