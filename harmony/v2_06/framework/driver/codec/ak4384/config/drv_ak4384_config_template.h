/*******************************************************************************
 AK4384 Codec Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak4384_config_template.h

  Summary:
    AK4384 Codec Driver Configuration Template.

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

#ifndef _DRV_AK4384_CONFIG_TEMPLATE_H
#define _DRV_AK4384_CONFIG_TEMPLATE_H

//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4384 driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported

  Description:
    Sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    AK4384 CODEC modules that are needed by the application. Hardware Instance
    support consumes RAM memory space.
    If this macro is not defined, then the driver will be built statically.

  Remarks:
    None.
*/

#define DRV_AK4384_INSTANCES_NUMBER

// *****************************************************************************
// *****************************************************************************
// Section: Client Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4384 Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description:
    Sets up the maximum number of clients that can be connected to any hardware
    instance. Typically only one client could be connected to one hardware instance.
    This value represents the total number of clients to be supported
    across all hardware instances. Therefore, if there are five AK4384 hardware
	interfaces, this number will be 5.

  Remarks:
    None.
*/

#define DRV_AK4384_CLIENTS_NUMBER					DRV_AK4384_INSTANCES_NUMBER


// *****************************************************************************
// *****************************************************************************
// Section: CODEC Specific Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AK4384 Input reference clock

  Summary:
    Identifies the input REFCLOCK source to generate the MCLK to codec.

  Description:
    Identifies the input REFCLOCK source to generate the MCLK to codec.

  Remarks:
    None.
*/
#define DRV_AK4384_INPUT_REFCLOCK



// *****************************************************************************
/* AK4384 MCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the MCLK to LRCK Ratio to Generate Audio Stream for 32, 44.1 and 48K
    sampling frequency

  Description:
    Sets up the MCLK to LRCK Ratio to Generate Audio Stream for 32, 44.1, and 48K
    I2S sampling frequency

    Supported MCLK to LRCK Ratios are as below
    256fs, 384fs, 512fs, 768fs or 1152fs [Normal Speed Mode(8kHz~48kHz)]
    128fs, 192fs, 256fs or 384fs [Double Speed Mode(60kHz~96kHz)]
    128fs, 192fs [Quad Speed Mode(120kHz~192kHz)]
  Remarks:
    None
*/
#define DRV_AK4384_MCLK_SAMPLE_FREQ_MULTPLIER

// *****************************************************************************
/* AK4384 BCLK to LRCK Ratio to Generate Audio Stream

  Summary:
    Sets up the BCLK to LRCK Ratio to Generate Audio Stream for 32, 44.1, and 48K
    sampling frequency

  Description:
    Sets up the BCLK to LRCK Ratio to Generate Audio Stream for 32, 44.1 and 48K
    I2S sampling frequency

    Following BCLK to LRCK ratios are supported
    16bit LSB Justified  >=32fs
    20bit LSB Justified  >=40fs
    24bit MSB Justified  >=48fs
    24bit I2S Compatible >=48fs
    24bit LSB Justified  >=48fs
	
	Typical values for the divisor are 1,2,4 and 8

  Remarks:
    None.
*/
#define DRV_AK4384_BCLK_BIT_CLK_DIVISOR


// *****************************************************************************
/* AK4384 Control Interface Clock Speed configuration

  Summary:
    Sets up clock frequency for the control interface (SPI)

  Description:
    Sets up clock frequency for the control interface (SPI).
    The maximum value supported is 5MHZ.

  Remarks:
    1. This Macro is useful only when a hardware SPI module is not available(used)
       or a virtual SPI driver is not available(used) for the control interface
       to the AK4384 CODEC.
    2. This constant needs to defined only for a bit banged implementation of control
       interface with in the driver.
*/
#define DRV_AK4384_CONTROL_CLOCK


// *****************************************************************************
/* AK4384 Timer Module Index

  Summary:
    Identifies the Timer Module Index for custom virtual SPI driver implementation.

  Description:
    Identifies the Timer Module Index for custom virtual SPI driver implementation.
    The AK4384 uses SPI protocol for control interface. The Timer Module Index
    is needed by AK4384 driver to implement a virtual SPI driver for control
    command exchange with the AK4384 CODEC.

  Remarks:
    1. This Macro is useful only when a hardware SPI module is not available(used)
       or a virtual SPI driver is not available(used) for the control interface
       to the AK4384 CODEC.
    2. This constant needs to defined only for a bit banged implementation of control
       interface with in the driver.
*/
#define DRV_AK4384_TIMER_DRIVER_MODULE_INDEX


// *****************************************************************************
/* AK4384 Timer Period

  Summary:
    Identifies the period for the bit bang timer.

  Description:
  	Identifies the period for the bit bang timer after which the timer interrupt
  	should occur.
    The value assigned should align with the expected control interface
    clock defined by AK4384_CONTROL_CLOCK.

  Remarks:
    1. This Macro is useful only when a hardware SPI module is not available(used)
       or a virtual SPI driver is not available(used) for the control interface
       to the AK4384 CODEC.
    2. This constant needs to defined only for a bit banged implementation of control
       interface with in the driver.
*/
#define DRV_AK4384_TIMER_PERIOD

// *****************************************************************************
/* AK4384 Delay Initialization

  Summary:
    Indicates whether the initilization of the AK4384 codec should be delayed.

  Description:
  	If the AK4384 Codec shares its RESET pin with another peripheral, such as
    a Bluetooth module, then this define should be true, in order to indicate
    the AK4384 Codec should starts its initialization only after the other
    peripheral has completed theirs.  It is set in the MHC menu with the checkbox:
    "Delay driver initialization (due to shared RESET pin)"

  Remarks:
    This needs to be set, for example, in the case where the AK4384 and the
    BM64 share a common PDN (power down) or RESET pin on the PIC32 Bluetooth
    Audio Development Kit (BTADK).

*/
#define DRV_AK4384_DELAY_INITIALIZATION

#endif // #ifndef _DRV_AK4384_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/
