/*******************************************************************************
 WM8904 Codec Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_wm8904_config_template.h

  Summary:
    WM8904 Codec Driver Configuration Template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_WM8904_CONFIG_TEMPLATE_H
#define _DRV_WM8904_CONFIG_TEMPLATE_H

//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* WM8904 driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported

  Description:
    Sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    WM8904 Codec modules that are needed by an application, namely one.

  Remarks:
    None.
*/

#define DRV_WM8904_INSTANCES_NUMBER

// *****************************************************************************
// *****************************************************************************
// Section: Client Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* WM8904 Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description:
    Sets up the maximum number of clients that can be connected to any hardware
    instance. Typically only one client could be connected to one hardware instance.
    This value represents the total number of clients to be supported
    across all hardware instances.

  Remarks:
    None.
*/

#define DRV_WM8904_CLIENTS_NUMBER


// *****************************************************************************
// *****************************************************************************
// Section: Codec Specific Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* WM8904 Codec Master/Slave Mode

  Summary:
    Specifies if codec is in Master or Slave mode.

  Description:
    Indicates whether the codec is to be  operating in a Master mode (generating word
    and bit clock as outputs) or Slave mode receiving word and bit clock as inputs).

  Remarks:
    Only Master mode is supported at this time.
*/
#define DRV_CODEC_WM8904_MODE


// *****************************************************************************
/* WM8904 Baud Rate

  Summary:
    Specifies the initial baud rate for the codec.

  Description:
    Sets the initial baud rate (sampling rate) for the codec. Typical values
    are 8000, 16000, 44100, 48000, 88200 and 96000.

  Remarks:
    None.
*/
#define DRV_WM8904_BAUD_RATE


// *****************************************************************************
/* WM8904 Audio Data Format

  Summary:
    Specifies the audio data format for the codec.

  Description:
    Sets up the length of each sample plus the format (I2S or left-justified) for
    the audio.

    Valid choices are:
    "DATA_16_BIT_LEFT_JUSTIFIED"
    "DATA_16_BIT_I2S"
    "DATA_32_BIT_LEFT_JUSTIFIED"
    "DATA_32_BIT_I2S"

  Remarks:
    If 24-bit audio is needed, it should be sent, left-justified, in a 32-bit format.
*/
#define DRV_WM8904_AUDIO_DATA_FORMAT

// *****************************************************************************
/* WM8904 Volume

  Summary:
    Specifies the initial volume level.

  Description:
    Sets the initial volume level, in the range 0-255.

  Remarks:
    The value is mapped to an internal WM8904 volume level in the range 0-192 using a
    logarithmic table so the input scale appears linear (128 is half volume).
*/
#define DRV_WM8904_VOLUME

// *****************************************************************************
/* WM8904 Microphone Enable

  Summary:
    Specifies whether to enable the microphone input.

  Description:
    Indicates whether the ADC inputs for the two microphone channels (L-R) should be
    enabled.

  Remarks:
    None.
*/
#define DRV_WM8904_ENABLE_MIC_INPUT

#endif // #ifndef _DRV_WM8904_CONFIG_TEMPLATE_H
/*******************************************************************************
 End of File
*/
