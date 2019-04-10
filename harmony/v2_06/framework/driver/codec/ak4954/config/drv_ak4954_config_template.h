/*******************************************************************************
 AK4954 Codec Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak4954_config_template.h

  Summary:
    AK4954 Codec Driver Configuration Template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

*******************************************************************************/

// DOM-IGNORE-BEGIN
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
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRV_AK4954_CONFIG_TEMPLATE_H
#define _DRV_AK4954_CONFIG_TEMPLATE_H

//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4954 driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported

  Description:
    Sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    AK4954 CODEC modules that are needed by the application. Hardware Instance
    support consumes RAM memory space.
    If this macro is not defined, then the driver will be built statically.

  Remarks:
    None.
*/

#define DRV_AK4954_INSTANCES_NUMBER

// *****************************************************************************
// *****************************************************************************
// Section: Client Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4954 Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description:
    Sets up the maximum number of clients that can be connected to any hardware
    instance. Typically only one client could be connected to one hardware instance.
    This value represents the total number of clients to be supported
    across all hardware instances. Therefore, if there are five AK4954 hardware
	interfaces, this number will be 5.

  Remarks:
    None.
*/

#define DRV_AK4954_CLIENTS_NUMBER					DRV_AK4954_INSTANCES_NUMBER


// *****************************************************************************
/* AK4954 Driver Buffer Queue Entries

  Summary:
    Number of entries of all queues in all instances of the driver.

  Description:
    This macro defined the number of entries of all queues in all instances of
    the driver.

    Each hardware instance supports a buffer queue for transmit
    operations. The size of queue is specified either in driver
    initialization (for dynamic build) or by macros (for static build). The
    hardware instance transmit buffer queue will queue transmit buffers
    submitted by the DRV_AK4954_BufferAddWrite function.

    A buffer queue will contains buffer queue entries, each related to a
    BufferAdd request.  This configuration macro defines total number of buffer
    entries that will be available for use between all AK4954 driver hardware
    instances. The buffer queue entries are allocated to individual hardware
    instances as requested by hardware instances. Once the request is processed,
    the buffer queue entry is free for use by other hardware instances.

    The total number of buffer entries in the system determines the ability of
    the driver to service non blocking write requests. If a free buffer
    entry is not available, the driver will not add the request and will return
    an invalid buffer handle. More the number of buffer entries, greater the
    ability of the driver to service and add requests to its queue. A hardware
    instance additionally can queue up as many buffer entries as  specified by
    its transmit buffer queue size.

    As an example, consider the case of static single client driver application
    where full duplex non blocking operation is desired without queuing, the
    minimum transmit queue depth and minimum receive queue depth should be 1.
    Hence the total number of buffer entries should be 2.

    As an example, consider the case of a dynamic driver (say two instances) where
    instance one will queue up to three write requests and up to two read requests, and
    instance two will queue up to two write requests and up to six read requests, the
    value of this macro should be 13 (2 + 3 + 2 + 6).

*/
#define  DRV_AK4954_QUEUE_DEPTH_COMBINED


// *****************************************************************************
// *****************************************************************************
// Section: CODEC Specific Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4954 Data Interface Master Clock Speed configuration

  Summary:
    Indicate the input clock frequency to generate the MCLK to codec.

  Description:
    Indicate the input clock frequency to generate the MCLK to codec.

  Remarks:
    None.
*/
#define DRV_AK4954_MCLK_SOURCE


// *****************************************************************************
/* AK4954 Input reference clock

  Summary:
    Identifies the input REFCLOCK source to generate the MCLK to codec.

  Description:
    Identifies the input REFCLOCK source to generate the MCLK to codec.

  Remarks:
    None.
*/
#define DRV_AK4954_INPUT_REFCLOCK



// *****************************************************************************
/* AK4954 MCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the MCLK to LRCK Ratio to Generate Audio Stream for specified sampling frequency

  Description:
    Sets up the MCLK to LRCK Ratio to Generate Audio Stream for specified sampling frequency
    I2S sampling frequency

    Supported MCLK to Sampling frequency Ratios are as below
    256fs, 384fs, 512fs, 768fs or 1152fs 
  Remarks:
    None
*/
#define DRV_AK4954_MCLK_SAMPLE_FREQ_MULTPLIER

// *****************************************************************************
/* AK4954 BCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the BCLK to LRCK Ratio to Generate Audio Stream for specified sampling frequency

  Description:
    Sets up the BCLK to LRCK Ratio to Generate Audio Stream for specified sampling frequency

    Following BCLK to LRCK ratios are supported
    16bit data 16 bit channel :- 32fs, hence divisor would be 8
	16bit data 32 bit channel :- 64fs, hence divisor would be 4	

  Remarks:
    None.
*/
#define DRV_AK4954_BCLK_BIT_CLK_DIVISOR
// *****************************************************************************
/* AK4954 Delay Initialization

  Summary:
    Indicates whether the initilization of the AK4954 codec should be delayed.

  Description:
  	If the AK4954 Codec shares its RESET pin with another peripheral, such as
    a Bluetooth module, then this define should be true, in order to indicate
    the AK4954 Codec should starts its initialization only after the other
    peripheral has completed theirs.  It is set in the MHC menu with the checkbox:
    "Delay driver initialization (due to shared RESET pin)"

  Remarks:
    This needs to be set, for example, in the case where the AK4954 and the
    BM64 share a common PDN (power down) or RESET pin on the PIC32 Bluetooth
    Audio Development Kit (BTADK).

*/
#define DRV_AK4954_DELAY_INITIALIZATION

#endif // #ifndef _DRV_AK4954_CONFIG_TEMPLATE_H
/*******************************************************************************
 End of File
*/
