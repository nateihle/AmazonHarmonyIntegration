/*******************************************************************************
 AK7755 Codec Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak7755_config_template.h

  Summary:
    AK7755 Codec Driver configuration template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_AK7755_CONFIG_TEMPLATE_H
#define _DRV_AK7755_CONFIG_TEMPLATE_H

//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK7755 driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported

  Description:
    This macro sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    AK7755 Codec modules that are needed by the application. Hardware Instance
    support consumes RAM memory space.
    If this macro is not defined, the driver will be built statically.

  Remarks:
    None.
*/

#define DRV_AK7755_INSTANCES_NUMBER

// *****************************************************************************
// *****************************************************************************
// Section: Client Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK7755 Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description:
    This macro sets up the maximum number of clients that can be connected to any hardware
    instance. Typically only one client could be connected to one hardware instance.
    This value represents the total number of clients to be supported
    across all hardware instances. Therefore, if there are five AK7755 hardware
	interfaces, this number will be 5.

  Remarks:
    None.
*/

#define DRV_AK7755_CLIENTS_NUMBER					DRV_AK7755_INSTANCES_NUMBER


// *****************************************************************************
// *****************************************************************************
// Section: CODEC Specific Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK7755 Data Interface Master Clock Speed configuration

  Summary:
    Indicates the input clock frequency to generate the MCLK to the codec.

  Description:
    This macro indicates the input clock frequency to generate the MCLK to the codec.

  Remarks:
    None.
*/
#define DRV_AK7755_MCLK_SOURCE


// *****************************************************************************
/* AK7755 Input reference clock

  Summary:
    Identifies the input REFCLOCK source to generate the MCLK to the codec.

  Description:
    This macro identifies the input REFCLOCK source to generate the MCLK to the codec.

  Remarks:
    None.
*/
#define DRV_AK7755_INPUT_REFCLOCK


// *****************************************************************************
/* AK7755 MCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the MCLK to LRCK ratio to generate the audio stream for the specified 
    sampling frequency.

  Description:
    This macro sets up the MCLK to LRCK ratio to generate the audio stream for 
    the specified I2S sampling frequency. 

    The supported MCLK to sampling frequency ratios are as follows:
    * 256 fs
    * 384 fs
    * 512 fs
    * 768 fs
    * 1152 fs
    
  Remarks:
    None.
*/
#define DRV_AK7755_MCLK_SAMPLE_FREQ_MULTPLIER

// *****************************************************************************
/* AK7755 BCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the BCLK to LRCK ratio to generate the audio stream for the specified 
    sampling frequency.

  Description:
    This macro sets up the BCLK to LRCK ratio to generate the audio stream for 
    the specified sampling frequency.

    The following BCLK to LRCK ratios are supported:
    * 16-bit data 16-bit channel: 32 fs; therefore, the divisor would be 8
	* 16-bit data 32-bit channel: 64 fs; therefore, the divisor would be 4	

  Remarks:
    None.
*/
#define DRV_AK7755_BCLK_BIT_CLK_DIVISOR

#endif // #ifndef _DRV_AK7755_CONFIG_TEMPLATE_H
/*******************************************************************************
 End of File
*/
