/*******************************************************************************
  MPLAB Harmony Graphics Object Library Asset Source File

  File Name:
    gfx_resources_ext.c

  Summary:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

  Description:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

    Created with MPLAB Harmony Version 2.03
*******************************************************************************/
// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#include "gfx/gfx_resources_ext.h"

/*****************************************************************************
 Asset Manifest
 -------------------
 
 Images
 -------
	MCHP_LOGO
	NO_USB_DISK_RED
	USB_DISK_GREEN

 Palettes
 -------
	MCHP_LOGO_palette
	NO_USB_DISK_RED_palette
	USB_DISK_GREEN_palette

 Fonts
 -------
    Arial12pt
    Arial14pt


 String Table
 -------
    stringTable
 
 *****************************************************************************/

/*****************************************************************************
 * SECTION:  Images
 *****************************************************************************/
/****** MCHP_LOGO ******/
// INDEX_8
const uint8_t MCHP_LOGO_data[2202] =
{
    0x01,0x01,0x0C,0x05,0x01,0x03,0x02,0x16,0x01,0x1C,0x02,0x16,0x01,0x04,0x70,
    0x05,0x01,0x03,0x01,0x2F,0x01,0x5C,0x01,0x76,0x01,0x8E,0x01,0x99,0x01,0x9E,
    0x01,0x99,0x01,0x8E,0x01,0x7C,0x01,0x62,0x01,0x30,0x01,0x04,0x02,0x05,0x01,
    0x06,0x01,0x10,0x67,0x05,0x01,0x04,0x01,0x45,0x01,0x83,0x01,0x99,0x09,0x9E,
    0x01,0x99,0x01,0x7D,0x01,0x0C,0x01,0x05,0x01,0x46,0x01,0x69,0x01,0x43,0x65,
    0x05,0x01,0x21,0x01,0x7C,0x0E,0x9E,0x01,0x5B,0x01,0x05,0x01,0x4C,0x01,0x74,
    0x01,0x49,0x63,0x05,0x01,0x00,0x01,0x40,0x01,0x92,0x0F,0x9E,0x01,0x92,0x01,
    0x27,0x01,0x18,0x01,0x2E,0x01,0x10,0x62,0x05,0x01,0x00,0x01,0x3E,0x01,0x92,
    0x11,0x9E,0x01,0x7C,0x01,0x02,0x0B,0x05,0x01,0x06,0x01,0x0F,0x08,0x05,0x01,
    0x0D,0x01,0x03,0x4D,0x05,0x01,0x31,0x01,0x99,0x04,0x9E,0x01,0xA4,0x01,0xBE,
    0x01,0xB1,0x07,0x9E,0x01,0xA5,0x01,0xBD,0x01,0xAE,0x02,0x9E,0x01,0x4B,0x0A,
    0x05,0x01,0x1F,0x01,0x97,0x01,0xE3,0x01,0x65,0x01,0x06,0x04,0x05,0x01,0x06,
    0x01,0x72,0x01,0xE3,0x01,0x8F,0x01,0x10,0x4B,0x05,0x01,0x01,0x01,0x87,0x04,
    0x9E,0x01,0x9F,0x01,0xC8,0x01,0xE3,0x01,0xE0,0x01,0xAC,0x05,0x9E,0x01,0xA0,
    0x01,0xC7,0x01,0xE3,0x01,0xDC,0x01,0xAA,0x01,0x9E,0x01,0x92,0x01,0x1B,0x09,
    0x05,0x01,0x48,0x02,0xE3,0x01,0xCF,0x01,0x1E,0x04,0x05,0x01,0x2C,0x01,0xCF,
    0x02,0xE3,0x01,0x35,0x4B,0x05,0x01,0x56,0x05,0x9E,0x01,0xB8,0x03,0xE3,0x01,
    0xD1,0x01,0xA0,0x04,0x9E,0x01,0xB9,0x01,0xE2,0x02,0xE3,0x01,0xCA,0x01,0xA0,
    0x01,0x9E,0x01,0x6E,0x09,0x05,0x01,0x59,0x03,0xE3,0x01,0x5F,0x04,0x05,0x01,
    0x6C,0x03,0xE3,0x01,0x4E,0x4A,0x05,0x01,0x15,0x01,0x8E,0x04,0x9E,0x01,0xAA,
    0x01,0xDD,0x04,0xE3,0x01,0xBB,0x03,0x9E,0x01,0xAA,0x01,0xDF,0x04,0xE3,0x01,
    0xB8,0x02,0x9E,0x01,0x3F,0x08,0x05,0x01,0x70,0x03,0xE3,0x01,0x9B,0x01,0x06,
    0x02,0x05,0x01,0x19,0x01,0x9C,0x03,0xE3,0x01,0x60,0x01,0x05,0x01,0x19,0x01,
    0x80,0x01,0x85,0x01,0x32,0x01,0x05,0x01,0x2D,0x01,0x7F,0x01,0x97,0x04,0x98,
    0x01,0x94,0x01,0x47,0x01,0x06,0x01,0x6A,0x06,0x85,0x01,0x81,0x01,0x4C,0x01,
    0x06,0x01,0x05,0x01,0x2C,0x01,0x79,0x01,0x97,0x03,0x98,0x01,0x97,0x01,0x84,
    0x01,0x3C,0x02,0x05,0x01,0x2C,0x01,0x7A,0x01,0x97,0x04,0x98,0x01,0x90,0x01,
    0x4E,0x01,0x05,0x01,0x72,0x01,0x85,0x01,0x4D,0x03,0x05,0x01,0x0F,0x01,0x79,
    0x01,0x85,0x01,0x2B,0x01,0x2A,0x02,0x85,0x01,0x2C,0x01,0x41,0x07,0x85,0x01,
    0x64,0x01,0x10,0x02,0x05,0x01,0x45,0x04,0x9E,0x01,0xA0,0x01,0xCB,0x05,0xE3,
    0x01,0xDC,0x01,0xA9,0x01,0x9E,0x01,0xA0,0x01,0xCD,0x05,0xE3,0x01,0xDB,0x01,
    0xA6,0x01,0x9E,0x01,0x8E,0x01,0x09,0x07,0x05,0x01,0x80,0x03,0xE3,0x01,0xE2,
    0x01,0x44,0x02,0x05,0x01,0x4D,0x04,0xE3,0x01,0x6C,0x01,0x05,0x01,0x20,0x01,
    0xB5,0x01,0xE3,0x01,0x42,0x01,0x0E,0x01,0x9D,0x07,0xE3,0x01,0x8F,0x01,0x06,
    0x01,0x8B,0x07,0xE3,0x01,0xCE,0x01,0x26,0x01,0x07,0x01,0x9C,0x07,0xE3,0x01,
    0xCE,0x01,0x1E,0x01,0x19,0x01,0x95,0x07,0xE3,0x01,0x8D,0x01,0x06,0x01,0x90,
    0x01,0xE3,0x01,0x64,0x03,0x05,0x01,0x11,0x01,0x9C,0x01,0xE3,0x02,0x3B,0x01,
    0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x08,0xE3,0x01,0x6B,0x02,0x05,0x01,0x6E,
    0x05,0x9E,0x01,0xBE,0x06,0xE3,0x01,0xC9,0x02,0x9F,0x01,0xC1,0x06,0xE3,0x01,
    0xC4,0x02,0x9E,0x01,0x66,0x07,0x05,0x01,0x8D,0x01,0xE3,0x01,0x9C,0x01,0xCF,
    0x01,0xE3,0x01,0x79,0x01,0x00,0x01,0x07,0x01,0x86,0x01,0xE3,0x01,0xD0,0x01,
    0x94,0x01,0xE3,0x01,0x7A,0x01,0x05,0x01,0x20,0x01,0xB5,0x01,0xE3,0x01,0x42,
    0x01,0x3C,0x08,0xE3,0x01,0x97,0x01,0x06,0x01,0x8B,0x01,0xE3,0x01,0xB5,0x03,
    0x9B,0x01,0xB5,0x02,0xE3,0x01,0x47,0x01,0x34,0x02,0xE3,0x01,0xE2,0x03,0x9B,
    0x01,0xCF,0x02,0xE3,0x01,0x54,0x01,0x3C,0x08,0xE3,0x01,0x8F,0x01,0x06,0x01,
    0x90,0x01,0xE3,0x01,0x64,0x03,0x05,0x01,0x11,0x01,0x9C,0x01,0xE3,0x02,0x3B,
    0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x01,0xE3,0x01,0xD0,0x03,0x9B,0x01,
    0x9C,0x02,0xE3,0x01,0x8F,0x01,0x0F,0x01,0x0C,0x01,0x87,0x03,0x9E,0x01,0xA5,
    0x01,0x9E,0x01,0xA2,0x01,0xD3,0x06,0xE3,0x01,0xB8,0x01,0x9E,0x01,0xA3,0x01,
    0xD7,0x05,0xE3,0x01,0xE1,0x01,0xB6,0x01,0x9E,0x01,0x99,0x01,0x37,0x05,0x05,
    0x01,0x10,0x01,0x94,0x01,0xE3,0x01,0x74,0x01,0x85,0x01,0xE3,0x01,0xB4,0x01,
    0x25,0x01,0x24,0x02,0xE3,0x01,0x7B,0x01,0x75,0x01,0xE3,0x01,0x8F,0x01,0x05,
    0x01,0x20,0x01,0xB5,0x01,0xE3,0x01,0x42,0x01,0x55,0x01,0xE3,0x01,0xD0,0x01,
    0x2C,0x07,0x06,0x01,0x8B,0x01,0xE3,0x01,0x5E,0x03,0x06,0x01,0x0F,0x01,0x98,
    0x01,0xE3,0x01,0x47,0x01,0x49,0x01,0xE3,0x01,0xE2,0x01,0x35,0x03,0x06,0x01,
    0x23,0x01,0xB4,0x01,0xE3,0x01,0x5D,0x01,0x42,0x01,0xE3,0x01,0xCF,0x01,0x3B,
    0x06,0x06,0x01,0x05,0x01,0x90,0x01,0xE3,0x01,0x6C,0x03,0x19,0x01,0x25,0x01,
    0xB3,0x01,0xE3,0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x01,0xE3,
    0x01,0x90,0x04,0x06,0x01,0x59,0x01,0xE3,0x01,0x9D,0x01,0x18,0x01,0x16,0x01,
    0x92,0x02,0x9E,0x01,0xA7,0x01,0xD5,0x01,0xAA,0x01,0x9E,0x01,0xAF,0x01,0xE1,
    0x05,0xE3,0x01,0xD9,0x01,0xA6,0x01,0x9E,0x01,0xB6,0x01,0xE2,0x05,0xE3,0x01,
    0xD5,0x01,0xA5,0x01,0x9E,0x01,0x83,0x01,0x0A,0x04,0x05,0x01,0x1E,0x01,0x9C,
    0x01,0xE3,0x01,0x63,0x01,0x4D,0x02,0xE3,0x01,0x59,0x01,0x6C,0x02,0xE3,0x01,
    0x43,0x01,0x72,0x01,0xE3,0x01,0xB4,0x01,0x05,0x01,0x20,0x01,0xB5,0x01,0xE3,
    0x01,0x42,0x01,0x5A,0x01,0xE3,0x01,0x95,0x01,0x11,0x06,0x05,0x01,0x06,0x01,
    0x8B,0x01,0xE3,0x01,0x5E,0x04,0x05,0x01,0x95,0x01,0xE3,0x01,0x47,0x01,0x55,
    0x01,0xE3,0x01,0x9C,0x01,0x19,0x03,0x05,0x01,0x08,0x01,0x8B,0x01,0xE3,0x01,
    0x63,0x01,0x42,0x01,0xE3,0x01,0xB3,0x01,0x0E,0x07,0x05,0x01,0x90,0x01,0xE3,
    0x01,0xCF,0x03,0xB3,0x01,0xB4,0x02,0xE3,0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,
    0x3B,0x01,0x52,0x01,0xE3,0x01,0x90,0x04,0x05,0x01,0x4F,0x01,0xE3,0x01,0x9D,
    0x01,0x18,0x01,0x1C,0x02,0x9E,0x01,0xA0,0x01,0xC5,0x01,0xE3,0x01,0xCD,0x01,
    0xA0,0x01,0x9E,0x01,0xC3,0x06,0xE3,0x01,0xC3,0x01,0x9F,0x01,0xA0,0x01,0xC7,
    0x06,0xE3,0x01,0xBF,0x02,0x9E,0x01,0x56,0x04,0x05,0x01,0x26,0x01,0xCF,0x01,
    0xE3,0x01,0x53,0x01,0x18,0x01,0xB4,0x01,0xE3,0x01,0x95,0x01,0xB5,0x01,0xE3,
    0x01,0x97,0x01,0x10,0x01,0x60,0x01,0xE3,0x01,0xCF,0x01,0x0D,0x01,0x20,0x01,
    0xB5,0x01,0xE3,0x01,0x42,0x01,0x5A,0x01,0xE3,0x01,0x90,0x01,0x10,0x06,0x05,
    0x01,0x06,0x01,0x8B,0x01,0xE3,0x01,0x90,0x03,0x72,0x01,0x79,0x01,0xCF,0x01,
    0xE2,0x01,0x3C,0x01,0x55,0x01,0xE3,0x01,0x95,0x01,0x12,0x03,0x05,0x01,0x06,
    0x01,0x84,0x01,0xE3,0x01,0x63,0x01,0x42,0x01,0xE3,0x01,0xB3,0x01,0x0E,0x07,
    0x05,0x01,0x90,0x08,0xE3,0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,
    0x01,0xE3,0x01,0x9D,0x04,0x5E,0x01,0x90,0x01,0xE3,0x01,0x97,0x01,0x11,0x01,
    0x1C,0x01,0x99,0x01,0x9E,0x01,0xB9,0x03,0xE3,0x01,0xBA,0x01,0x9E,0x01,0xA4,
    0x01,0xD7,0x05,0xE3,0x01,0xE1,0x01,0xB1,0x01,0x9E,0x01,0xA8,0x01,0xDA,0x05,
    0xE3,0x01,0xE0,0x01,0xAD,0x01,0x9E,0x01,0x99,0x01,0x22,0x03,0x05,0x01,0x2C,
    0x02,0xE3,0x01,0x4A,0x01,0x05,0x01,0x71,0x04,0xE3,0x01,0x63,0x01,0x05,0x01,
    0x4F,0x01,0xE3,0x01,0xD0,0x01,0x23,0x01,0x20,0x01,0xB5,0x01,0xE3,0x01,0x42,
    0x01,0x5A,0x01,0xE3,0x01,0x95,0x01,0x11,0x06,0x05,0x01,0x06,0x01,0x8B,0x07,
    0xE3,0x01,0x80,0x01,0x06,0x01,0x55,0x01,0xE3,0x01,0x9B,0x01,0x19,0x03,0x05,
    0x01,0x08,0x01,0x8B,0x01,0xE3,0x01,0x63,0x01,0x42,0x01,0xE3,0x01,0xB3,0x01,
    0x0E,0x07,0x05,0x01,0x90,0x01,0xE3,0x01,0xB4,0x04,0x98,0x01,0xE2,0x01,0xE3,
    0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x08,0xE3,0x01,0x7F,0x01,
    0x06,0x01,0x16,0x01,0x8E,0x01,0xA9,0x01,0xDE,0x03,0xE3,0x01,0xDB,0x01,0xA9,
    0x01,0x9E,0x01,0xB7,0x01,0xE0,0x05,0xE3,0x01,0xD6,0x01,0xA2,0x01,0x9E,0x01,
    0xB9,0x06,0xE3,0x01,0xD2,0x01,0xA1,0x01,0x9E,0x01,0x45,0x03,0x05,0x01,0x42,
    0x01,0xE3,0x01,0xE2,0x01,0x3D,0x01,0x05,0x01,0x32,0x03,0xE3,0x01,0xCE,0x01,
    0x2C,0x01,0x05,0x01,0x43,0x01,0xE3,0x01,0xE2,0x01,0x36,0x01,0x20,0x01,0xB5,
    0x01,0xE3,0x01,0x42,0x01,0x55,0x01,0xE3,0x01,0xCF,0x01,0x2C,0x06,0x0E,0x01,
    0x06,0x01,0x8B,0x01,0xE3,0x01,0xCF,0x03,0xB4,0x01,0xCF,0x01,0xE3,0x01,0xCF,
    0x01,0x2D,0x01,0x49,0x01,0xE3,0x01,0xD0,0x01,0x35,0x03,0x0E,0x01,0x26,0x01,
    0xB3,0x01,0xE3,0x01,0x5F,0x01,0x42,0x01,0xE3,0x01,0xCE,0x01,0x33,0x07,0x05,
    0x01,0x90,0x01,0xE3,0x01,0x64,0x03,0x05,0x01,0x11,0x01,0x9C,0x01,0xE3,0x02,
    0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x07,0xE3,0x01,0x9D,0x01,0x39,
    0x01,0x05,0x01,0x03,0x01,0x7E,0x01,0xD1,0x05,0xE3,0x01,0xC6,0x01,0x9E,0x01,
    0xA0,0x01,0xC8,0x06,0xE3,0x01,0xBE,0x01,0x9E,0x01,0xA0,0x01,0xCD,0x06,0xE3,
    0x01,0xBA,0x01,0x9E,0x01,0x27,0x03,0x05,0x01,0x58,0x01,0xE3,0x01,0xCF,0x01,
    0x2D,0x01,0x05,0x01,0x07,0x01,0x90,0x02,0xE3,0x01,0x85,0x02,0x05,0x01,0x33,
    0x02,0xE3,0x01,0x4A,0x01,0x20,0x01,0xB5,0x01,0xE3,0x01,0x42,0x01,0x3C,0x02,
    0xE3,0x01,0xB4,0x05,0x9B,0x01,0x97,0x01,0x06,0x01,0x8B,0x01,0xE3,0x01,0x65,
    0x03,0x1A,0x01,0x34,0x01,0xB4,0x01,0xE3,0x01,0x47,0x01,0x36,0x02,0xE3,0x01,
    0xB4,0x03,0x9B,0x01,0xB3,0x02,0xE3,0x01,0x54,0x01,0x3B,0x02,0xE3,0x01,0xB4,
    0x01,0x95,0x04,0x96,0x01,0x85,0x01,0x18,0x01,0x90,0x01,0xE3,0x01,0x64,0x03,
    0x05,0x01,0x11,0x01,0x9C,0x01,0xE3,0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,
    0x01,0x52,0x01,0xE3,0x01,0x91,0x05,0x25,0x01,0x19,0x03,0x05,0x01,0x68,0x06,
    0xE3,0x01,0xCC,0x01,0x9F,0x01,0x9E,0x01,0xA6,0x01,0xDC,0x05,0xE3,0x01,0xE0,
    0x01,0xAB,0x01,0x9E,0x01,0xAA,0x01,0xDE,0x05,0xE3,0x01,0xDC,0x01,0x89,0x04,
    0x05,0x01,0x6C,0x01,0xE3,0x01,0xCF,0x01,0x1E,0x02,0x05,0x01,0x48,0x02,0xB3,
    0x01,0x3C,0x02,0x05,0x01,0x2D,0x02,0xE3,0x01,0x53,0x01,0x20,0x01,0xB5,0x01,
    0xE3,0x01,0x42,0x01,0x0F,0x01,0x9C,0x07,0xE3,0x01,0x9D,0x01,0x06,0x01,0x8B,
    0x01,0xE3,0x01,0x5E,0x04,0x05,0x01,0x95,0x01,0xE3,0x01,0x47,0x01,0x05,0x01,
    0x9B,0x07,0xE3,0x01,0xCE,0x01,0x17,0x01,0x18,0x01,0x95,0x07,0xE3,0x01,0x94,
    0x01,0x10,0x01,0x90,0x01,0xE3,0x01,0x64,0x03,0x05,0x01,0x11,0x01,0x9C,0x01,
    0xE3,0x02,0x3B,0x01,0xE3,0x01,0xE2,0x01,0x3B,0x01,0x52,0x01,0xE3,0x01,0x90,
    0x09,0x05,0x01,0x36,0x01,0xCF,0x04,0xE3,0x01,0xDE,0x01,0xAD,0x03,0x9E,0x01,
    0xB9,0x05,0xE3,0x01,0xE0,0x01,0xAF,0x02,0x9E,0x01,0xBD,0x06,0xE3,0x01,0x51,
    0x04,0x05,0x01,0x5A,0x01,0x81,0x01,0x7B,0x01,0x0E,0x03,0x05,0x02,0x20,0x03,
    0x05,0x01,0x1F,0x01,0x7B,0x01,0x81,0x01,0x4D,0x01,0x19,0x01,0x7B,0x01,0x81,
    0x01,0x34,0x01,0x05,0x01,0x2C,0x01,0x79,0x05,0x86,0x01,0x85,0x01,0x49,0x01,
    0x06,0x01,0x6B,0x01,0x81,0x01,0x48,0x04,0x05,0x01,0x73,0x01,0x81,0x01,0x3A,
    0x01,0x05,0x01,0x2B,0x01,0x79,0x05,0x86,0x01,0x80,0x01,0x3C,0x02,0x05,0x01,
    0x35,0x01,0x80,0x01,0x90,0x04,0x94,0x01,0x8C,0x01,0x4D,0x01,0x00,0x01,0x72,
    0x01,0x81,0x01,0x4A,0x03,0x05,0x01,0x0F,0x01,0x75,0x01,0x81,0x01,0x2B,0x01,
    0x2C,0x02,0x81,0x01,0x2C,0x01,0x41,0x01,0x81,0x01,0x6D,0x09,0x05,0x01,0x06,
    0x01,0x8C,0x04,0xE3,0x01,0xBA,0x04,0x9E,0x01,0xA1,0x01,0xCC,0x04,0xE3,0x01,
    0xC0,0x03,0x9E,0x01,0xA3,0x01,0xD4,0x04,0xE3,0x01,0x9C,0x01,0x1D,0x1B,0x05,
    0x06,0x06,0x0F,0x05,0x05,0x06,0x05,0x05,0x01,0x06,0x01,0x10,0x04,0x11,0x01,
    0x08,0x1D,0x05,0x01,0x35,0x01,0xE2,0x02,0xE3,0x01,0xC8,0x01,0xA0,0x05,0x9E,
    0x01,0xAA,0x01,0xDD,0x02,0xE3,0x01,0xD4,0x01,0xA0,0x04,0x9E,0x01,0xAF,0x01,
    0xE0,0x03,0xE3,0x01,0x59,0x60,0x05,0x01,0x70,0x01,0xE3,0x01,0xDA,0x01,0xA6,
    0x07,0x9E,0x01,0xBD,0x01,0xE3,0x01,0xDD,0x01,0xAD,0x06,0x9E,0x01,0xC2,0x02,
    0xE3,0x01,0x8B,0x01,0x0F,0x60,0x05,0x01,0x12,0x01,0x8A,0x01,0xB2,0x08,0x9E,
    0x01,0xA0,0x01,0xBC,0x01,0xB0,0x07,0x9E,0x01,0xA5,0x01,0xD8,0x01,0xB3,0x01,
    0x24,0x62,0x05,0x01,0x14,0x01,0x83,0x13,0x9E,0x01,0x93,0x01,0x35,0x64,0x05,
    0x01,0x13,0x01,0x77,0x11,0x9E,0x01,0x82,0x01,0x29,0x66,0x05,0x01,0x0C,0x01,
    0x50,0x01,0x92,0x0D,0x9E,0x01,0x99,0x01,0x61,0x01,0x09,0x69,0x05,0x01,0x09,
    0x01,0x56,0x01,0x88,0x01,0x99,0x07,0x9E,0x01,0x9A,0x01,0x8E,0x01,0x61,0x01,
    0x22,0x6D,0x05,0x01,0x0B,0x01,0x28,0x01,0x50,0x01,0x67,0x01,0x78,0x01,0x77,
    0x01,0x78,0x01,0x6F,0x01,0x57,0x01,0x38,0x01,0x0A,0xDF,0x05,
};
		
GFXU_ImageAsset MCHP_LOGO =
{
	{
        GFXU_ASSET_TYPE_IMAGE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)MCHP_LOGO_data, // data address pointer
	    2202, // data size
    },	
	GFXU_IMAGE_FORMAT_RAW, // image format type
	122, // image width
	30, // image height
	GFX_COLOR_MODE_INDEX_8, // image color mode
	GFXU_IMAGE_COMPRESSION_RLE, // image compression type (raw only)
	GFX_TRUE, // image mask enable
	0x2167, // image mask color
	&MCHP_LOGO_palette // image palette
};
		
/****** NO_USB_DISK_RED ******/
// INDEX_4
const uint8_t NO_USB_DISK_RED_data[218] =
{
    0x01,0x01,0x2B,0x00,0x0E,0x11,0x05,0x00,0x01,0x11,0x0D,0x22,0x01,0x11,0x02,
    0x00,0x03,0x11,0x01,0x22,0x0C,0x11,0x01,0x22,0x01,0x10,0x01,0x00,0x01,0x01,
    0x02,0x22,0x01,0x12,0x01,0x13,0x0B,0x33,0x01,0x31,0x01,0x21,0x02,0x00,0x02,
    0x12,0x02,0x21,0x0C,0x33,0x01,0x12,0x01,0x10,0x01,0x00,0x01,0x01,0x02,0x22,
    0x01,0x12,0x01,0x13,0x04,0x33,0x02,0x11,0x01,0x13,0x04,0x33,0x01,0x31,0x01,
    0x21,0x02,0x00,0x02,0x12,0x02,0x21,0x04,0x33,0x01,0x13,0x03,0x33,0x01,0x31,
    0x01,0x13,0x02,0x33,0x01,0x12,0x01,0x10,0x01,0x00,0x01,0x01,0x02,0x22,0x01,
    0x12,0x01,0x13,0x02,0x33,0x07,0x11,0x02,0x33,0x01,0x31,0x01,0x21,0x02,0x00,
    0x02,0x12,0x02,0x21,0x05,0x33,0x01,0x31,0x02,0x33,0x01,0x31,0x01,0x13,0x02,
    0x33,0x01,0x12,0x01,0x10,0x01,0x00,0x01,0x01,0x02,0x22,0x01,0x12,0x01,0x13,
    0x05,0x33,0x01,0x31,0x01,0x11,0x01,0x13,0x03,0x33,0x01,0x31,0x01,0x21,0x02,
    0x00,0x02,0x12,0x02,0x21,0x0C,0x33,0x01,0x12,0x01,0x10,0x01,0x00,0x01,0x01,
    0x02,0x22,0x01,0x12,0x01,0x13,0x0B,0x33,0x01,0x31,0x01,0x21,0x02,0x00,0x03,
    0x11,0x01,0x22,0x0C,0x11,0x01,0x22,0x01,0x10,0x04,0x00,0x01,0x11,0x0D,0x22,
    0x01,0x11,0x05,0x00,0x0E,0x11,0x29,0x00,
};
		
GFXU_ImageAsset NO_USB_DISK_RED =
{
	{
        GFXU_ASSET_TYPE_IMAGE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)NO_USB_DISK_RED_data, // data address pointer
	    218, // data size
    },	
	GFXU_IMAGE_FORMAT_RAW, // image format type
	39, // image width
	19, // image height
	GFX_COLOR_MODE_INDEX_4, // image color mode
	GFXU_IMAGE_COMPRESSION_RLE, // image compression type (raw only)
	GFX_TRUE, // image mask enable
	0x2167, // image mask color
	&NO_USB_DISK_RED_palette // image palette
};
		
/****** USB_DISK_GREEN ******/
// INDEX_4
const uint8_t USB_DISK_GREEN_data[218] =
{
    0x01,0x01,0x2B,0x00,0x0E,0x22,0x05,0x00,0x01,0x22,0x0D,0x33,0x01,0x22,0x02,
    0x00,0x03,0x22,0x01,0x33,0x0C,0x22,0x01,0x33,0x01,0x20,0x01,0x00,0x01,0x02,
    0x02,0x33,0x01,0x23,0x01,0x21,0x0B,0x11,0x01,0x12,0x01,0x32,0x02,0x00,0x02,
    0x23,0x02,0x32,0x0C,0x11,0x01,0x23,0x01,0x20,0x01,0x00,0x01,0x02,0x02,0x33,
    0x01,0x23,0x01,0x21,0x04,0x11,0x02,0x22,0x01,0x21,0x04,0x11,0x01,0x12,0x01,
    0x32,0x02,0x00,0x02,0x23,0x02,0x32,0x04,0x11,0x01,0x21,0x03,0x11,0x01,0x12,
    0x01,0x21,0x02,0x11,0x01,0x23,0x01,0x20,0x01,0x00,0x01,0x02,0x02,0x33,0x01,
    0x23,0x01,0x21,0x02,0x11,0x07,0x22,0x02,0x11,0x01,0x12,0x01,0x32,0x02,0x00,
    0x02,0x23,0x02,0x32,0x05,0x11,0x01,0x12,0x02,0x11,0x01,0x12,0x01,0x21,0x02,
    0x11,0x01,0x23,0x01,0x20,0x01,0x00,0x01,0x02,0x02,0x33,0x01,0x23,0x01,0x21,
    0x05,0x11,0x01,0x12,0x01,0x22,0x01,0x21,0x03,0x11,0x01,0x12,0x01,0x32,0x02,
    0x00,0x02,0x23,0x02,0x32,0x0C,0x11,0x01,0x23,0x01,0x20,0x01,0x00,0x01,0x02,
    0x02,0x33,0x01,0x23,0x01,0x21,0x0B,0x11,0x01,0x12,0x01,0x32,0x02,0x00,0x03,
    0x22,0x01,0x33,0x0C,0x22,0x01,0x33,0x01,0x20,0x04,0x00,0x01,0x22,0x0D,0x33,
    0x01,0x22,0x05,0x00,0x0E,0x22,0x29,0x00,
};
		
GFXU_ImageAsset USB_DISK_GREEN =
{
	{
        GFXU_ASSET_TYPE_IMAGE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)USB_DISK_GREEN_data, // data address pointer
	    218, // data size
    },	
	GFXU_IMAGE_FORMAT_RAW, // image format type
	39, // image width
	19, // image height
	GFX_COLOR_MODE_INDEX_4, // image color mode
	GFXU_IMAGE_COMPRESSION_RLE, // image compression type (raw only)
	GFX_TRUE, // image mask enable
	0x2167, // image mask color
	&USB_DISK_GREEN_palette // image palette
};
		
/*****************************************************************************
 * SECTION:  Palettes
 *****************************************************************************/
/****** MCHP_LOGO_palette ******/
// RGB_565
uint8_t MCHP_LOGO_palette_data[456] =
{
    0x67,0x19,0x25,0x21,0x26,0x21,0x46,0x21,0x47,0x21,0x67,0x21,0x87,0x21,0x88,
    0x21,0xA8,0x21,0x05,0x29,0x26,0x29,0x46,0x29,0x47,0x29,0x87,0x29,0xA7,0x29,
    0xA8,0x29,0xC8,0x29,0xC9,0x29,0xE9,0x29,0x05,0x31,0x25,0x31,0x26,0x31,0x67,
    0x31,0xC8,0x31,0xE9,0x31,0x09,0x32,0x2A,0x32,0x25,0x39,0x67,0x39,0x09,0x3A,
    0x29,0x3A,0x2A,0x3A,0x4A,0x3A,0x25,0x41,0x46,0x41,0x4A,0x42,0x6A,0x42,0x6B,
    0x42,0x8B,0x42,0x05,0x49,0x25,0x49,0x26,0x49,0x8A,0x4A,0x8B,0x4A,0xAB,0x4A,
    0xAC,0x4A,0xCC,0x4A,0x04,0x51,0x05,0x51,0x25,0x51,0xAB,0x52,0xCB,0x52,0xCC,
    0x52,0xEC,0x52,0xED,0x52,0x04,0x59,0x25,0x59,0xEC,0x5A,0x0C,0x5B,0x0D,0x5B,
    0x2D,0x5B,0x2E,0x5B,0xC3,0x60,0xE4,0x60,0x04,0x61,0x2D,0x63,0x4D,0x63,0x4E,
    0x63,0x6E,0x63,0x25,0x69,0x6E,0x6B,0x8E,0x6B,0x8F,0x6B,0xAF,0x6B,0xD0,0x6B,
    0x04,0x71,0xAF,0x73,0xCF,0x73,0xD0,0x73,0xF0,0x73,0x04,0x79,0xAF,0x7B,0xF0,
    0x7B,0x10,0x7C,0x11,0x7C,0x31,0x7C,0xE4,0x80,0x04,0x81,0x31,0x84,0x51,0x84,
    0x72,0x84,0x04,0x89,0x25,0x89,0x71,0x8C,0x72,0x8C,0x92,0x8C,0xB3,0x8C,0x04,
    0x91,0x25,0x91,0xB2,0x94,0xB3,0x94,0xD3,0x94,0x04,0x99,0x25,0x99,0x10,0x9C,
    0xD3,0x9C,0xF3,0x9C,0xF4,0x9C,0x14,0x9D,0x35,0x9D,0x04,0xA1,0x45,0xA1,0x34,
    0xA5,0x35,0xA5,0x55,0xA5,0x76,0xA5,0x75,0xAD,0x96,0xAD,0x04,0xB1,0x24,0xB1,
    0x45,0xB1,0xB6,0xB5,0xD6,0xB5,0xD7,0xB5,0x04,0xB9,0x24,0xB9,0x45,0xB9,0xD7,
    0xBD,0xF7,0xBD,0x18,0xBE,0x04,0xC1,0x24,0xC1,0x18,0xC6,0x38,0xC6,0x58,0xC6,
    0x04,0xC9,0x24,0xC9,0x28,0xCA,0x38,0xCE,0x59,0xCE,0x79,0xCE,0x9A,0xCE,0x24,
    0xD1,0x9A,0xD6,0xBA,0xD6,0xDB,0xD6,0x24,0xD9,0xEB,0xDA,0xDB,0xDE,0xFB,0xDE,
    0xFC,0xDE,0x1B,0xDF,0x1C,0xDF,0x24,0xE1,0x44,0xE1,0x1C,0xE7,0x3C,0xE7,0x5C,
    0xE7,0x44,0xE9,0x45,0xE9,0x65,0xE9,0x85,0xE9,0x86,0xE9,0xA6,0xE9,0xC6,0xE9,
    0xC7,0xE9,0xE7,0xE9,0x07,0xEA,0x28,0xEA,0x49,0xEA,0x69,0xEA,0x89,0xEA,0x8A,
    0xEA,0xAA,0xEA,0xCA,0xEA,0xCB,0xEA,0x0C,0xEB,0x2C,0xEB,0x4C,0xEB,0x5D,0xEF,
    0x7D,0xEF,0x9D,0xEF,0x4D,0xF3,0x6D,0xF3,0xAE,0xF3,0xEF,0xF3,0x30,0xF4,0x50,
    0xF4,0x71,0xF4,0x91,0xF4,0x92,0xF4,0xD2,0xF4,0xF3,0xF4,0x13,0xF5,0x14,0xF5,
    0x34,0xF5,0x55,0xF5,0x75,0xF5,0x95,0xF5,0x96,0xF5,0xB6,0xF5,0xD6,0xF5,0xD7,
    0xF5,0xF7,0xF5,0x18,0xF6,0x38,0xF6,0x9E,0xF7,0xBE,0xF7,0xDE,0xF7,0x58,0xFE,
    0x59,0xFE,0x99,0xFE,0x9A,0xFE,0xBA,0xFE,0xDA,0xFE,0xFB,0xFE,0x1B,0xFF,0x1C,
    0xFF,0x3C,0xFF,0x5C,0xFF,0x5D,0xFF,0x7D,0xFF,0x9D,0xFF,0x9E,0xFF,0xBE,0xFF,
    0xDE,0xFF,0xDF,0xFF,0xFF,0xFF,
};
		
GFXU_PaletteAsset MCHP_LOGO_palette =
{
	{
        GFXU_ASSET_TYPE_PALETTE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)MCHP_LOGO_palette_data, // data address pointer
	    456, // data size
    },	
	228, // color count
	GFX_COLOR_MODE_RGB_565 // image color mode
};

/****** NO_USB_DISK_RED_palette ******/
// RGB_565
uint8_t NO_USB_DISK_RED_palette_data[8] =
{
    0x67,0x21,0xEF,0x7B,0x18,0xC6,0xE4,0xE8,
};
		
GFXU_PaletteAsset NO_USB_DISK_RED_palette =
{
	{
        GFXU_ASSET_TYPE_PALETTE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)NO_USB_DISK_RED_palette_data, // data address pointer
	    8, // data size
    },	
	4, // color count
	GFX_COLOR_MODE_RGB_565 // image color mode
};

/****** USB_DISK_GREEN_palette ******/
// RGB_565
uint8_t USB_DISK_GREEN_palette_data[8] =
{
    0x67,0x21,0x89,0x25,0xEF,0x7B,0x18,0xC6,
};
		
GFXU_PaletteAsset USB_DISK_GREEN_palette =
{
	{
        GFXU_ASSET_TYPE_PALETTE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)USB_DISK_GREEN_palette_data, // data address pointer
	    8, // data size
    },	
	4, // color count
	GFX_COLOR_MODE_RGB_565 // image color mode
};

/*****************************************************************************
 * SECTION:  Fonts
 
 - font lookup table data description -
1 byte - size of the address offset values in this table, 1-4 possible
1 byte - size of the address width values in this table, 1-2 possible
  for each glyph entry in lookup table:
    1-4 bytes - glyph data offset in bytes
    1-2 bytes - glyph raster width in pixels	
 
 *****************************************************************************/
uint8_t Arial12pt_lookup_20_7F[290] =
{
    0x02,0x01,0x00,0x00,0x03,0x0F,0x00,0x03,0x1E,0x00,0x04,0x2D,0x00,0x07,0x3C,
    0x00,0x07,0x4B,0x00,0x0B,0x69,0x00,0x08,0x78,0x00,0x02,0x87,0x00,0x04,0x96,
    0x00,0x04,0xA5,0x00,0x05,0xB4,0x00,0x07,0xC3,0x00,0x03,0xD2,0x00,0x04,0xE1,
    0x00,0x03,0xF0,0x00,0x03,0xFF,0x00,0x07,0x0E,0x01,0x07,0x1D,0x01,0x07,0x2C,
    0x01,0x07,0x3B,0x01,0x07,0x4A,0x01,0x07,0x59,0x01,0x07,0x68,0x01,0x07,0x77,
    0x01,0x07,0x86,0x01,0x07,0x95,0x01,0x03,0xA4,0x01,0x03,0xB3,0x01,0x07,0xC2,
    0x01,0x07,0xD1,0x01,0x07,0xE0,0x01,0x07,0xEF,0x01,0x0C,0x0D,0x02,0x07,0x1C,
    0x02,0x08,0x2B,0x02,0x09,0x49,0x02,0x09,0x67,0x02,0x08,0x76,0x02,0x07,0x85,
    0x02,0x09,0xA3,0x02,0x09,0xC1,0x02,0x03,0xD0,0x02,0x06,0xDF,0x02,0x08,0xEE,
    0x02,0x07,0xFD,0x02,0x09,0x1B,0x03,0x09,0x39,0x03,0x09,0x57,0x03,0x08,0x66,
    0x03,0x09,0x84,0x03,0x09,0xA2,0x03,0x08,0xB1,0x03,0x07,0xC0,0x03,0x09,0xDE,
    0x03,0x07,0xED,0x03,0x0B,0x0B,0x04,0x07,0x1A,0x04,0x07,0x29,0x04,0x07,0x38,
    0x04,0x03,0x47,0x04,0x03,0x56,0x04,0x03,0x65,0x04,0x05,0x74,0x04,0x07,0x83,
    0x04,0x04,0x92,0x04,0x07,0xA1,0x04,0x07,0xB0,0x04,0x06,0xBF,0x04,0x07,0xCE,
    0x04,0x07,0xDD,0x04,0x03,0xEC,0x04,0x07,0xFB,0x04,0x07,0x0A,0x05,0x03,0x19,
    0x05,0x03,0x28,0x05,0x06,0x37,0x05,0x03,0x46,0x05,0x0B,0x64,0x05,0x07,0x73,
    0x05,0x07,0x82,0x05,0x07,0x91,0x05,0x07,0xA0,0x05,0x04,0xAF,0x05,0x07,0xBE,
    0x05,0x03,0xCD,0x05,0x07,0xDC,0x05,0x05,0xEB,0x05,0x09,0x09,0x06,0x05,0x18,
    0x06,0x05,0x27,0x06,0x05,0x36,0x06,0x04,0x45,0x06,0x03,0x54,0x06,0x04,0x63,
    0x06,0x07,0x72,0x06,0x09,
};


GFXU_FontGlyphIndexTable Arial12pt_index_table =
{
	1, // range count
    {
	    /* 0x20-0x7F */
        {
		    96, // glyph count
		    0x20, // starting glyph id
		    0x7F, // ending glyph id
		    Arial12pt_lookup_20_7F // glyph lookup table
        },
    }
};
		
// 96 glyphs @ 1 bpp
const uint8_t Arial12pt_data[1680] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0xA0,0xA0,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x14,0x14,0xFE,0x28,0x28,0xFE,0x50,0x50,0x50,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x54,0x50,0x50,0x38,0x14,0x54,0x54,0x38,0x10,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x31,0x00,0x4A,0x00,0x4A,0x00,0x4C,0x00,0x35,
    0x80,0x06,0x40,0x0A,0x40,0x0A,0x40,0x11,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x18,0x24,0x24,0x28,0x30,0x4A,0x44,0x46,0x39,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x10,0x20,0x20,0x40,0x40,0x40,0x40,0x40,0x20,0x20,0x10,0x00,
    0x00,0x00,0x00,0x80,0x40,0x40,0x20,0x20,0x20,0x20,0x20,0x40,0x40,0x80,0x00,
    0x00,0x00,0x00,0x20,0xF8,0x20,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x7C,0x10,0x10,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x20,0x20,0x40,0x40,0x40,0x40,0x40,0x80,0x80,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x04,0x04,0x08,0x08,0x10,0x20,0x7C,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x04,0x04,0x18,0x04,0x04,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x08,0x18,0x18,0x28,0x28,0x48,0x7C,0x08,0x08,0x00,0x00,0x00,
    0x00,0x00,0x00,0x3C,0x20,0x40,0x78,0x44,0x04,0x04,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x40,0x58,0x64,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7C,0x04,0x08,0x10,0x10,0x10,0x20,0x20,0x20,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x38,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x4C,0x34,0x04,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0x04,0x38,0x40,0x38,0x04,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x40,0x38,0x04,0x38,0x40,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x38,0x44,0x44,0x04,0x08,0x10,0x10,0x00,0x10,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x80,0x18,0x60,0x20,0x20,0x26,0x90,0x49,
    0x90,0x51,0x10,0x51,0x10,0x51,0x20,0x4F,0xC0,0x20,0x10,0x10,0x60,0x0F,0x80,
    0x00,0x00,0x00,0x10,0x28,0x28,0x28,0x44,0x7C,0x44,0x82,0x82,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7C,0x42,0x42,0x42,0x7C,0x42,0x42,0x42,0x7C,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x22,0x00,0x41,0x00,0x40,0x00,0x40,
    0x00,0x40,0x00,0x41,0x00,0x22,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0x00,0x42,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x41,0x00,0x42,0x00,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7E,0x40,0x40,0x40,0x7E,0x40,0x40,0x40,0x7E,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7C,0x40,0x40,0x40,0x78,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x22,0x00,0x41,0x00,0x40,0x00,0x47,
    0x00,0x41,0x00,0x41,0x00,0x22,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x7F,
    0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x88,0x88,0x70,0x00,0x00,0x00,
    0x00,0x00,0x00,0x41,0x42,0x44,0x48,0x50,0x68,0x44,0x42,0x41,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x63,0x00,0x63,0x00,0x55,0x00,0x55,
    0x00,0x55,0x00,0x55,0x00,0x49,0x00,0x49,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x61,0x00,0x51,0x00,0x51,0x00,0x49,
    0x00,0x45,0x00,0x45,0x00,0x43,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x22,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x41,0x00,0x22,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7C,0x42,0x42,0x42,0x7C,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x22,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x4D,0x00,0x22,0x00,0x1D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x7E,
    0x00,0x44,0x00,0x42,0x00,0x42,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x3C,0x42,0x42,0x40,0x3C,0x02,0x42,0x42,0x3C,0x00,0x00,0x00,
    0x00,0x00,0x00,0xFE,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x41,0x00,0x22,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x82,0x82,0x44,0x44,0x44,0x28,0x28,0x10,0x10,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x20,0x8A,0x20,0x8A,0x40,0x4A,0x40,0x51,
    0x40,0x51,0x40,0x51,0x40,0x20,0x80,0x20,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x82,0x44,0x48,0x28,0x10,0x28,0x48,0x44,0x82,0x00,0x00,0x00,
    0x00,0x00,0x00,0x82,0x44,0x44,0x28,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,
    0x00,0x00,0x00,0x7E,0x04,0x08,0x08,0x10,0x20,0x20,0x40,0xFE,0x00,0x00,0x00,
    0x00,0x00,0x00,0x60,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x60,0x00,
    0x00,0x00,0x00,0x80,0x80,0x40,0x40,0x40,0x40,0x40,0x20,0x20,0x00,0x00,0x00,
    0x00,0x00,0x00,0xC0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0xC0,0x00,
    0x00,0x00,0x00,0x20,0x50,0x50,0x50,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,
    0x00,0x00,0x00,0x40,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x04,0x3C,0x44,0x4C,0x34,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x58,0x64,0x44,0x44,0x44,0x64,0x58,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x40,0x40,0x40,0x48,0x30,0x00,0x00,0x00,
    0x00,0x00,0x00,0x04,0x04,0x34,0x4C,0x44,0x44,0x44,0x4C,0x34,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x44,0x7C,0x40,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x20,0x40,0xE0,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x34,0x4C,0x44,0x44,0x44,0x4C,0x34,0x04,0x78,0x00,
    0x00,0x00,0x00,0x40,0x40,0x58,0x64,0x44,0x44,0x44,0x44,0x44,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x80,0x00,
    0x00,0x00,0x00,0x40,0x40,0x44,0x48,0x50,0x70,0x48,0x48,0x44,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x59,0x80,0x66,0x40,0x44,
    0x40,0x44,0x40,0x44,0x40,0x44,0x40,0x44,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x58,0x64,0x44,0x44,0x44,0x44,0x44,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x58,0x64,0x44,0x44,0x44,0x64,0x58,0x40,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0x34,0x4C,0x44,0x44,0x44,0x4C,0x34,0x04,0x04,0x00,
    0x00,0x00,0x00,0x00,0x00,0x50,0x60,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x40,0x38,0x04,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0x40,0x40,0xE0,0x40,0x40,0x40,0x40,0x40,0x60,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x44,0x44,0x44,0x44,0x44,0x4C,0x34,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x88,0x88,0x50,0x50,0x50,0x20,0x20,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x88,0x80,0x88,0x80,0x55,
    0x00,0x55,0x00,0x55,0x00,0x22,0x00,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x88,0x50,0x50,0x20,0x60,0x50,0x88,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x88,0x88,0x50,0x50,0x50,0x20,0x20,0x20,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0xF8,0x10,0x10,0x20,0x40,0x40,0xF8,0x00,0x00,0x00,
    0x00,0x00,0x00,0x20,0x40,0x40,0x40,0x40,0x80,0x40,0x40,0x40,0x40,0x20,0x00,
    0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,
    0x00,0x00,0x00,0x40,0x20,0x20,0x20,0x20,0x10,0x20,0x20,0x20,0x20,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x4C,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

GFXU_FontAsset Arial12pt =
{
	{
        GFXU_ASSET_TYPE_FONT, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)Arial12pt_data, // data address pointer
	    1680, // data size
    },	
	15, // font height
	12, // font max ascent
	3, // font max descent
	3, // font baseline
	GFXU_FONT_BPP_1, // bits per pixel
	&Arial12pt_index_table // glyph index table
};
		
uint8_t Arial14pt_lookup_0[4] =
{
    0x01,0x01,0x00,0x0B,
};


GFXU_FontGlyphIndexTable Arial14pt_index_table =
{
	1, // range count
    {
	    /* 0x0 */
        {
		    1, // glyph count
		    0x0, // starting glyph id
		    0x0, // ending glyph id
		    Arial14pt_lookup_0 // glyph lookup table
        },
    }
};
		
// 1 glyphs @ 1 bpp
const uint8_t Arial14pt_data[34] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x80,0x20,0x80,0x20,
    0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x3F,0x80,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

GFXU_FontAsset Arial14pt =
{
	{
        GFXU_ASSET_TYPE_FONT, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)Arial14pt_data, // data address pointer
	    34, // data size
    },	
	17, // font height
	13, // font max ascent
	3, // font max descent
	4, // font baseline
	GFXU_FONT_BPP_1, // bits per pixel
	&Arial14pt_index_table // glyph index table
};
		


/*****************************************************************************
 * SECTION:  Strings
 
 - String table data format description -

 2 bytes - number of unique string values
 
 for each string value:
   2 bytes - size of the string in bytes
   n bytes - character code point data, 1-4 bytes each character per encoding	 
 *****************************************************************************/
// 1 language, 1 unique string value, ASCII encoding
uint8_t stringTable_data[28] =
{
    0x01,0x00,0x18,0x00,0x55,0x6E,0x69,0x76,0x65,0x72,0x73,0x61,0x6C,0x20,0x41,
    0x75,0x64,0x69,0x6F,0x20,0x44,0x65,0x63,0x6F,0x64,0x65,0x72,0x73,
};
	
/*****************************************************************************
  - String index table data format description -

 2 bytes - number of strings in the string table
 1 byte - number of languages in the string table
 1 byte - size of the string indicies, equals 2 if there are more than 254
          strings
		  
 for each string in table:
    for each language in table:
	    1-2 bytes - string data table entry	 
 *****************************************************************************/
// Lookup table for associating string and language IDs to string data.
uint8_t stringIndexTable_data[5] =
{
    0x01,0x00,0x01,0x01,0x00,
};
	
GFXU_FontAsset* fontList[2] =
{
	&Arial12pt,
	&Arial14pt,
};

/*****************************************************************************
  - Font index table data format description -

 2 bytes - number of strings in the string table
 1 byte - number of languages in the string table
		  
 for each string in table:
    for each language in table:
	    1 byte - the font to use for the string	 
		
 id = 0xFF if no font association
 *****************************************************************************/
// Lookup table for associating strings, languages, and fonts
uint8_t fontIndexTable_data[4] =
{
    0x01,0x00,0x01,0x00,
};
	
GFXU_StringTableAsset stringTable =
{
	{
        GFXU_ASSET_TYPE_STRINGTABLE, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)stringTable_data, // data address pointer
	    28, // data size
    },	
	1, // language count
	1, // string count
    stringIndexTable_data, // font lookup table
    fontList, // font lookup table
    fontIndexTable_data, // font index table
	GFXU_STRING_ENCODING_ASCII // encoding standard
};
		
