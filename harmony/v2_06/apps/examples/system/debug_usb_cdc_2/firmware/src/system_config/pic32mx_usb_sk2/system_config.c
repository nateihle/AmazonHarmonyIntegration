/*******************************************************************************
 System Configuration file.

  Company:
    Microchip Technology Inc.

  File Name:
    system_config.c

  Summary:
    USB Descriptor and data structure declarations required by the USB Stack.

  Description:
    This file contains USB Descriptors and data structure declarations required
    by the USB Stack.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_definitions.h" 
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************


/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
/* Configuration 1 Descriptor */
const uint8_t fullSpeedConfigurationDescriptor1[]=
{
    /* Configuration Descriptor Header */

    0x09,                                                   // Size of this descriptor
    USB_DESCRIPTOR_CONFIGURATION,                           // CONFIGURATION descriptor type
    66,0,                                                   // Total length of data for this configuration
    2,                                                      // Number of interfaces in this configuration
    1,                                                      // Index value of this configuration
    0,                                                      // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED,     // Attributes, see usb_device.h
    50,                                                     // Max power consumption (2X mA)

    /* Interface Descriptor 1 */

    9,                                              // Size of the descriptor
    USB_DESCRIPTOR_INTERFACE,                       // INTERFACE descriptor type
    0,                                              // Interface Number
    0,                                              // Alternate Setting Number
    1,                                              // Number of endpoints in this intf
    USB_CDC_COMMUNICATIONS_INTERFACE_CLASS_CODE,    // Class code
    USB_CDC_SUBCLASS_ABSTRACT_CONTROL_MODEL,        // Subclass code
    USB_CDC_PROTOCOL_AT_V250,                       // Protocol code
    0,                                              // Interface string index

    /* CDC Class-Specific Descriptors */

    sizeof(USB_CDC_HEADER_FUNCTIONAL_DESCRIPTOR),                   // Size of the descriptor
    USB_CDC_DESC_CS_INTERFACE,                                      // CS_INTERFACE
    USB_CDC_FUNCTIONAL_HEADER,                                      // Type of functional descriptor
    0x20,0x01,                                                      // CDC spec version

    sizeof(USB_CDC_ACM_FUNCTIONAL_DESCRIPTOR),                      // Size of the descriptor
    USB_CDC_DESC_CS_INTERFACE,                                      // CS_INTERFACE
    USB_CDC_FUNCTIONAL_ABSTRACT_CONTROL_MANAGEMENT,                 // Type of functional descriptor
    USB_CDC_ACM_SUPPORT_LINE_CODING_LINE_STATE_AND_NOTIFICATION,    // bmCapabilities of ACM

    sizeof(USB_CDC_UNION_FUNCTIONAL_DESCRIPTOR_HEADER),             // Size of the descriptor
    USB_CDC_DESC_CS_INTERFACE,                                      // CS_INTERFACE
    USB_CDC_FUNCTIONAL_UNION,                                       // Type of functional descriptor
    0,                                                              // com interface number

    sizeof(USB_CDC_CALL_MANAGEMENT_DESCRIPTOR),                     // Size of the descriptor
    USB_CDC_DESC_CS_INTERFACE,                                      // CS_INTERFACE
    USB_CDC_FUNCTIONAL_CALL_MANAGEMENT,                             // Type of functional descriptor
    0x00,                                                           // bmCapabilities of CallManagement
    1,                                                              // Data interface number

    /* Interrupt Endpoint (IN) Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    0x81,                           // EndpointAddress ( EP1 IN INTERRUPT)
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes type of EP (INTERRUPT)
    0x0A,0x00,                      // Max packet size of this EP
    0x02,                           // Interval (in ms)

    /* Interface Descriptor */

    9,                                  // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,           // INTERFACE descriptor type
    1,                                  // Interface Number
    0,                                  // Alternate Setting Number
    2,                                  // Number of endpoints in this interface
    USB_CDC_DATA_INTERFACE_CLASS_CODE,  // Class code
    0,                                  // Subclass code
    USB_CDC_PROTOCOL_NO_CLASS_SPECIFIC, // Protocol code
    0,                                  // Interface string index

    /* Bulk Endpoint (OUT) Descriptor */

    0x07,                       // Sizeof of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,    // Endpoint Descriptor
    0x02,                       // Endpoint Address BULK OUT
    USB_TRANSFER_TYPE_BULK,     // Attributes BULK EP
    0x40,0x00,                  // MaxPacket Size of EP (BULK OUT)
    0x00,                       // Interval Can be ignored for BULK EPs.

    /* Bulk Endpoint (IN)Descriptor */

    0x07,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,    // Endpoint Descriptor
    0x82,                       // EndpointAddress BULK IN
    USB_TRANSFER_TYPE_BULK,     // Attributes BULK EP
    0x40,0x00,                  // MaxPacket Size of EP (BULK IN)
    0x00,                       // Interval Can be ignored for BULK EPs.
};

USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor1
};


const USB_DEVICE_CDC_INIT  cdcInit =
{
    .queueSizeRead = 1,
    .queueSizeWrite = 4,
    .queueSizeSerialStateNotification = 1
};

const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    {
         .configurationValue = 1 ,                  // Configuration descriptor index
         .driver = (void *)USB_DEVICE_CDC_FUNCTION_DRIVER,  // CDC APIs exposed to the device layer
         .funcDriverIndex = 0 ,                     // Instance index of CDC function driver
         .funcDriverInit = (void *)&cdcInit,        // CDC init data
         .interfaceNumber = 0 ,                     // Start interface number of this instance
         .numberOfInterfaces = 2 ,                  // Total number of interfaces contained in this instance
         .speed = USB_SPEED_FULL|USB_SPEED_HIGH     // USB Speed
    }
};

/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
//Language code string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[1];}sd000={
sizeof(sd000),USB_DESCRIPTOR_STRING,{0x0409
}};

//Manufacturer string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[25];}sd001={
sizeof(sd001),USB_DESCRIPTOR_STRING,
{'M','i','c','r','o','c','h','i','p',' ',
'T','e','c','h','n','o','l','o','g','y',' ','I','n','c','.'
}};

//Product string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[22];}sd002={
sizeof(sd002),USB_DESCRIPTOR_STRING,
{'U','S','B','-','C','D','C',' ','S','y','s',
't','e','m',' ','C','o','n','s','o','l','e'
}};

USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};

// USB descriptors
/* Device Descriptor */
const USB_DEVICE_DESCRIPTOR fullSpeedDeviceDescriptor =
{
    0x12,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,      // DEVICE descriptor type
    0x0200,                     // USB Spec Release Number in BCD format
    USB_CDC_CLASS_CODE,         // Class Code
    USB_CDC_SUBCLASS_CODE,      // Subclass code
    0x00,                       // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE, // Max packet size for EP0, see system_config.h
    0x04D8,                     // Vendor ID
    0x000A,                     // Product ID: CDC RS-232 Emulation Demo
    0x0100,                     // Device release number in BCD format
    0x01,                       // Manufacturer string index
    0x02,                       // Product string index
    0x00,                       // Device serial number string index
    0x01                        // Number of possible configurations
};

const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &fullSpeedDeviceDescriptor,   // Full Speed Device Descriptor.
    1,                            // Total number of full speed configurations available.
    &fullSpeedConfigDescSet[0],   // Pointer to array of full speed configurations descriptors.

    NULL,                         // High speed device desc is not supported.
    0,                            // Total number of high speed configurations available.
    NULL,                         // Pointer to array of high speed configurations descriptors.

    3,                            // Total number of string descriptors available.
    stringDescriptors,            // Pointer to array of string descriptors

    NULL,                         // Pointer to full speed dev qualifier.
    NULL,                         // Pointer to high speed dev qualifier.
};

