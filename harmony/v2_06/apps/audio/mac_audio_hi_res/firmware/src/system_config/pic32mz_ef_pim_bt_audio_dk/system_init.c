/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_2X
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_8_16_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_33
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_12MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     OFF
#pragma config FETHIO =     OFF
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   OFF

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0xffff
#pragma config CSEQ =       0xffff
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK4384 Initialization Data">
/*** CODEC Driver Initialization Data ***/
const DRV_AK4384_INIT drvak4384Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
    .i2sDriverModuleIndex = DRV_AK4384_I2S_DRIVER_MODULE_INDEX_IDX0,
    .volume = DRV_AK4384_VOLUME,
    .mclkMode = DRV_AK4384_MCLK_MODE_MACRO,
    .delayDriverInitialization = DRV_AK4384_DELAY_INITIALIZATION,
};
// </editor-fold>
 // <editor-fold defaultstate="collapsed" desc="DRV_I2S Initialization Data">
/*** I2S Driver Initialization Data ***/
const DRV_I2S_INIT drvI2S0InitData =
{
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX0,
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX0, 
    .usageMode = DRV_I2S_USAGE_MODE_IDX0,
    .baudClock = SPI_BAUD_RATE_CLK_IDX0,
    .baud = DRV_I2S_BAUD_RATE,
    .clockMode = DRV_I2S_CLK_MODE_IDX0,
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX0,
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX0,
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX0,
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0,
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX0,
    .queueSizeReceive = QUEUE_SIZE_RX_IDX0,
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX0,
    .dmaChaningChannelTransmit = DRV_I2S_TX_DMA_CHAINING_CHANNEL_IDX0,
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX0,    
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX0,    
    .dmaInterruptChainingTransmitSource = DRV_I2S_TX_DMA_CHAINING_SOURCE_IDX0,
    .dmaChannelReceive = DMA_CHANNEL_NONE,
};





// </editor-fold>
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0, 
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_IDX0,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};
const DRV_TMR_INIT drvTmr1InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX1,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX1,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX1, 
    .prescale = DRV_TMR_PRESCALE_IDX1,
    .mode = DRV_TMR_OPERATION_MODE_IDX1,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX1,
    .asyncWriteEnable = false,
};
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
const DRV_USBHS_INIT drvUSBHSInit =
{
    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* Interrupt Source for USB module */
    .interruptSourceUSBDma = INT_SOURCE_USB_1_DMA,

    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Operation Mode */
    .operationMode = DRV_USBHS_OPMODE_DEVICE,

    /* Operation Speed */ 
    .operationSpeed = USB_SPEED_HIGH,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USBHS_ID_0,
    
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="SYS_CLK Initialization Data">
// *****************************************************************************
/* System Clock Initialization Data
*/
const SYS_CLK_INIT sysClkInit =
{
    .moduleInit = {0},
    .systemClockFrequencyHz = SYS_CLK_FREQ,
    .waitTillComplete = true,
    .onWaitInstruction = SYS_CLK_ON_WAIT,
};
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_DMA Initialization Data">
/*** System DMA Initialization Data ***/

const SYS_DMA_INIT sysDmaInit =
{
	.sidl = SYS_DMA_SIDL_DISABLE,

};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_TMR Initialization Data">
/*** TMR Service Initialization Data ***/
const SYS_TMR_INIT sysTmrInitData =
{
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    .drvIndex = DRV_TMR_INDEX_0,
    .tmrFreq = 1000, 
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">


/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
    const USB_DEVICE_AUDIO_V2_INIT audioInit0 =
    {
        .queueSizeRead = 64,
        .queueSizeWrite = 2
    };
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_HIGH,    /* Function Speed */ 
        .numberOfInterfaces = 2,    /* Number of interfaces */
        .funcDriverIndex = 0,  /* Index of Audio Function Driver */
        .driver = (void*)USB_DEVICE_AUDIO_V2_FUNCTION_DRIVER,   /* USB Audio function data exposed to device layer */
        .funcDriverInit = (void*)&audioInit0,   /* Function driver init data*/
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
	0xEF,                           // Class Code
    0x02,                           // Subclass code
    0x01,                           // Protocol code	
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x04D8,                         // Vendor ID
    0xABCD,                         // Product ID
	0x0200,							// Device release number in BCD format				
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x00,                           // Device serial number string index
    0x01                            // Number of possible configurations
};

/*******************************************
 *  USB Device Qualifier Descriptor for this
 *  demo.
 *******************************************/
const USB_DEVICE_QUALIFIER deviceQualifierDescriptor1 =
{
    0x0A,                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE_QUALIFIER,    // Device Qualifier Type
    0x0200,                             // USB Specification Release number
	0xEF,                           // Class Code
    0x02,                           // Subclass code
    0x01,                           // Protocol code	
    USB_DEVICE_EP0_BUFFER_SIZE,         // Maximum packet size for endpoint 0
	0x00,
    0x00                                // Reserved for future use.
};

/*******************************************
 *  USB High Speed Configuration Descriptor
 *******************************************/
 
const uint8_t highSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    160,0,                //(160 Bytes)Size of the Config descriptor.e
    2,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    50,                                                 // Max power consumption (2X mA)
    
	/* Interface Association Descriptor: */
    0x08,									// bLength
    USB_AUDIO_V2_DESCRIPTOR_IA,				// bDescriptorType
    0,
											// bFirstInterface
    2,
											// bInterfaceCount
    AUDIO_V2_FUNCTION,						// bFunctionClass   (Audio Device Class)
    AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED,	// bFunctionSubClass
    AUDIO_V2_AF_VERSION_02_00,				// bFunctionProtocol
    0x00,									// iFunction

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0,	
											// bInterfaceNumber
    0x00,									// bAlternateSetting
    0x00,									// bNumEndPoints
    AUDIO_V2,								// bInterfaceClass   (Audio Device Class)
    AUDIO_V2_AUDIOCONTROL,					// bInterfaceSubClass   (Audio Control Interface)
    AUDIO_V2_IP_VERSION_02_00,				// bInterfaceProtocol
    0x00,									// iInterface

    // AC Interface Header Descriptor:
    0x09,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_HEADER,						// bDescriptorSubtype
    0x00, 0x02,								// bcdADC
    AUDIO_V2_IO_BOX,						// bCategory
    0x48, 0x00,								// wTotalLength
    0x00,									// bmControls

    // AC Clock Source Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_CLOCK_SOURCE,					// bDescriptorSubtype
    0x28,									// bClockID
    0x03,									// bmAttributes: Internal Progamable Clock
    0x07,									// bmControls
											// D[1:0]/11 : Clock Freq Control R/W
											// D[3:2]/01 : Clock Validity Control R
    0x00,									// bAssocTerminal
    0x00,									// iClockSource

    // AC Clock Selector Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_CLOCK_SELECTOR,				// bDescriptorSubtype
    0x11,									// bClockID
    0x01,									// bNrInPins
    0x28,									// baCSourceID(1)
    0x03,									// bmControls
											/* D[1:0] : Clock Selector Control R/W
											   D[7:4] : Reserved (0) */
    0x00,									// iClockSelector

    // AC Input Terminal Descriptor:
    0x11,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_INPUT_TERMINAL,				// bDescriptorSubtype
    0x02,					// bTerminalID
    (uint8_t )USB_AUDIO_V2_TERMTYPE_USB_STREAMING,
    (uint8_t )(USB_AUDIO_V2_TERMTYPE_USB_STREAMING >> 8),	
											// wTerminalType   (USB Streaming)
    0x00,									// bAssocTerminal
    0x11,									// bCSourceID
    0x02,									// bNrChannels
    0x00, 0x00, 0x00, 0x00,					// bmChannelConfig N/A Front Left/Right
    0x00,									// iChannelNames
    0x00, 0x00,								// bmControls
    0x00,									// iTerminal

    // AC Feature Unit Descriptor:
    0x12,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_FEATURE_UNIT,					// bDescriptorSubtype
    0x0A,									// bUnitID
    0x02,									// bSourceID
    0x03, 0x00, 0x00, 0x00,					// bmaControls(0): Master Mute R/W
    0x00, 0x00, 0x00, 0x00,					// bmaControls(1)
    0x00, 0x00, 0x00, 0x00,					// bmaControls(2)
    0x00,									// iFeature

    // AC Output Terminal Descriptor:
    0x0C,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_OUTPUT_TERMINAL,				// bDescriptorSubtype
    0x14,					// bTerminalID
    (uint8_t )AUDIO_V2_SPEAKER,
    (uint8_t )(AUDIO_V2_SPEAKER>>8),		// wTerminalType   (Speaker)
    0,										// bAssocTerminal
    0x0A,									// bSourceID
    0x11,									// bCSourceID
    0x00, 0x00,								// bmControls
    0x00,									// iTerminal

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0x01,									// bInterfaceNumber
    0x00,									// bAlternateSetting
    0x00,									// bNumEndPoints
    AUDIO_V2,              					// bInterfaceClass: AUDIO
    AUDIO_V2_AUDIOSTREAMING,     			// bInterfaceSubClass: AUDIO_STREAMING
    AUDIO_V2_IP_VERSION_02_00,   			// bInterfaceProtocol: IP_VERSION_02_00
    0x00,									// iInterface

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0x01,									// bInterfaceNumber
    0x01,									// bAlternateSetting
    0x02,									// bNumEndPoints
    AUDIO_V2,              					// bInterfaceClass: AUDIO
    AUDIO_V2_AUDIOSTREAMING,     			// bInterfaceSubClass: AUDIO_STREAMING
    AUDIO_V2_IP_VERSION_02_00,   			// bInterfaceProtocol: IP_VERSION_02_00
    0x00,									// iInterface

    // AS Interface Descriptor:
    0x10,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_AS_GENERAL,					// bDescriptorSubtype
    0x02,									// bTerminalLink/Input Terminal
    0x00,									// bmControls
    0x01,									// bFormatType
    (uint8_t )AUDIO_V2_PCM, 0x00, 0x00, 0x00,      
											// bmFormats (note this is a bitmap)
    0x02,									// bNrChannels
    0x00, 0x00, 0x00, 0x00,					// bmChannelConfig: N/A Front Left/Right
    0x00,									// iChannelNames

    // AS Format Type 1 Descriptor:
    0x06,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_FORMAT_TYPE,					// bDescriptorSubtype
    0x01,									// bFormatType
    0x04,									// bSubslotSize
    0x18,									// bBitResolution   (24 Bits/sample)
    // Data Endpoint Descriptor:
    0x07,									// bLength
    USB_DESCRIPTOR_ENDPOINT,				// bDescriptorType
    0x01,									// bEndpointAddress   (OUT Endpoint)
    0x05,									// bmAttributes	(Transfer: Isochronous / Synch: Async / Usage: Data)
    0x00, 0x04,								// Endpoint size
    0x01,									// bInterval

    // AS Isochronous Data Endpoint Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_ENDPOINT,					// bDescriptorType
    AUDIO_V2_EP_GENERAL,					// bDescriptorSubtype
    0x00,									// bmAttributes
    0x35,									// bmControls
    0x00,									// bLockDelayUnits   (Decoded PCM samples)
    0x00, 0x00,								// wLockDelay

    // Feedback Endpoint Descriptor:
    0x07,									// bLength
    USB_DESCRIPTOR_ENDPOINT,				// bDescriptorType
    0x81,									// bEndpointAddress   (IN Endpoint)
    0x11,									// bmAttributes	(Transfer: Isochronous / Synch: None / Usage: Feedback)
    0x04, 0x00,								// Endpoint Size
    0x04,									// bInterval


};

/*******************************************
 * Array of High speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE highSpeedConfigDescSet[1] =
{
    highSpeedConfigurationDescriptor
};

/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    160,0,                //(160 Bytes)Size of the Config descriptor.e
    2,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    50,                                                 // Max power consumption (2X mA)
	/* Interface Association Descriptor: */
    0x08,									// bLength
    USB_AUDIO_V2_DESCRIPTOR_IA,				// bDescriptorType
    0,
											// bFirstInterface
    2,
											// bInterfaceCount
    AUDIO_V2_FUNCTION,						// bFunctionClass   (Audio Device Class)
    AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED,	// bFunctionSubClass
    AUDIO_V2_AF_VERSION_02_00,				// bFunctionProtocol
    0x00,									// iFunction

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0,	
											// bInterfaceNumber
    0x00,									// bAlternateSetting
    0x00,									// bNumEndPoints
    AUDIO_V2,								// bInterfaceClass   (Audio Device Class)
    AUDIO_V2_AUDIOCONTROL,					// bInterfaceSubClass   (Audio Control Interface)
    AUDIO_V2_IP_VERSION_02_00,				// bInterfaceProtocol
    0x00,									// iInterface

    // AC Interface Header Descriptor:
    0x09,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_HEADER,						// bDescriptorSubtype
    0x00, 0x02,								// bcdADC
    AUDIO_V2_IO_BOX,						// bCategory
    0x48, 0x00,								// wTotalLength
    0x00,									// bmControls

    // AC Clock Source Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_CLOCK_SOURCE,					// bDescriptorSubtype
    0x28,									// bClockID
    0x03,									// bmAttributes: Internal Progamable Clock
    0x07,									// bmControls
											// D[1:0]/11 : Clock Freq Control R/W
											// D[3:2]/01 : Clock Validity Control R
    0x00,									// bAssocTerminal
    0x00,									// iClockSource

    // AC Clock Selector Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_CLOCK_SELECTOR,				// bDescriptorSubtype
    0x11,									// bClockID
    0x01,									// bNrInPins
    0x28,									// baCSourceID(1)
    0x03,									// bmControls
											/* D[1:0] : Clock Selector Control R/W
											   D[7:4] : Reserved (0) */
    0x00,									// iClockSelector

    // AC Input Terminal Descriptor:
    0x11,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_INPUT_TERMINAL,				// bDescriptorSubtype
    0x02,					// bTerminalID
    (uint8_t )USB_AUDIO_V2_TERMTYPE_USB_STREAMING,
    (uint8_t )(USB_AUDIO_V2_TERMTYPE_USB_STREAMING >> 8),	
											// wTerminalType   (USB Streaming)
    0x00,									// bAssocTerminal
    0x11,									// bCSourceID
    0x02,									// bNrChannels
    0x00, 0x00, 0x00, 0x00,					// bmChannelConfig N/A Front Left/Right
    0x00,									// iChannelNames
    0x00, 0x00,								// bmControls
    0x00,									// iTerminal

    // AC Feature Unit Descriptor:
    0x12,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_FEATURE_UNIT,					// bDescriptorSubtype
    0x0A,									// bUnitID
    0x02,									// bSourceID
    0x03, 0x00, 0x00, 0x00,					// bmaControls(0): Master Mute R/W
    0x00, 0x00, 0x00, 0x00,					// bmaControls(1)
    0x00, 0x00, 0x00, 0x00,					// bmaControls(2)
    0x00,									// iFeature

    // AC Output Terminal Descriptor:
    0x0C,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_OUTPUT_TERMINAL,				// bDescriptorSubtype
    0x14,					// bTerminalID
    (uint8_t )AUDIO_V2_SPEAKER,
    (uint8_t )(AUDIO_V2_SPEAKER>>8),		// wTerminalType   (Speaker)
    0,										// bAssocTerminal
    0x0A,									// bSourceID
    0x11,									// bCSourceID
    0x00, 0x00,								// bmControls
    0x00,									// iTerminal

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0x01,									// bInterfaceNumber
    0x00,									// bAlternateSetting
    0x00,									// bNumEndPoints
    AUDIO_V2,              					// bInterfaceClass: AUDIO
    AUDIO_V2_AUDIOSTREAMING,     			// bInterfaceSubClass: AUDIO_STREAMING
    AUDIO_V2_IP_VERSION_02_00,   			// bInterfaceProtocol: IP_VERSION_02_00
    0x00,									// iInterface

    // Interface Descriptor:
    0x09,									// bLength
    USB_DESCRIPTOR_INTERFACE,				// bDescriptorType
    0x01,									// bInterfaceNumber
    0x01,									// bAlternateSetting
    0x02,									// bNumEndPoints
    AUDIO_V2,              					// bInterfaceClass: AUDIO
    AUDIO_V2_AUDIOSTREAMING,     			// bInterfaceSubClass: AUDIO_STREAMING
    AUDIO_V2_IP_VERSION_02_00,   			// bInterfaceProtocol: IP_VERSION_02_00
    0x00,									// iInterface

    // AS Interface Descriptor:
    0x10,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_AS_GENERAL,					// bDescriptorSubtype
    0x02,									// bTerminalLink/Input Terminal
    0x00,									// bmControls
    0x01,									// bFormatType
    (uint8_t )AUDIO_V2_PCM, 0x00, 0x00, 0x00,      
											// bmFormats (note this is a bitmap)
    0x02,									// bNrChannels
    0x00, 0x00, 0x00, 0x00,					// bmChannelConfig: N/A Front Left/Right
    0x00,									// iChannelNames

    // AS Format Type 1 Descriptor:
    0x06,									// bLength
    AUDIO_V2_CS_INTERFACE,					// bDescriptorType
    AUDIO_V2_FORMAT_TYPE,					// bDescriptorSubtype
    0x01,									// bFormatType
    0x04,									// bSubslotSize
    0x18,									// bBitResolution   (24 Bits/sample)
    // Data Endpoint Descriptor:
    0x07,									// bLength
    USB_DESCRIPTOR_ENDPOINT,				// bDescriptorType
    0x01,									// bEndpointAddress   (OUT Endpoint)
    0x05,									// bmAttributes	(Transfer: Isochronous / Synch: Async / Usage: Data)
    0x00, 0x04,								// Endpoint size
    0x01,									// bInterval

    // AS Isochronous Data Endpoint Descriptor:
    0x08,									// bLength
    AUDIO_V2_CS_ENDPOINT,					// bDescriptorType
    AUDIO_V2_EP_GENERAL,					// bDescriptorSubtype
    0x00,									// bmAttributes
    0x35,									// bmControls
    0x00,									// bLockDelayUnits   (Decoded PCM samples)
    0x00, 0x00,								// wLockDelay

    // Feedback Endpoint Descriptor:
    0x07,									// bLength
    USB_DESCRIPTOR_ENDPOINT,				// bDescriptorType
    0x81,									// bEndpointAddress   (IN Endpoint)
    0x11,									// bmAttributes	(Transfer: Isochronous / Synch: None / Usage: Feedback)
    0x04, 0x00,								// Endpoint Size
    0x04,									// bInterval



};

/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};


/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;
        uint8_t bDscType;
        uint16_t string[1];
    }
    sd000 =
    {
        sizeof(sd000),          // Size of this descriptor in bytes
        USB_DESCRIPTOR_STRING,  // STRING descriptor type
        {0x0409}                // Language ID
    };
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[25];    // String
    }
    sd001 =
    {
        sizeof(sd001),
        USB_DESCRIPTOR_STRING,
        {'M','i','c','r','o','c','h','i','p',' ','T','e','c','h','n','o','l','o','g','y',' ','I','n','c','.'}
		
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[23];    // String
    }
    sd002 =
    {
        sizeof(sd002),
        USB_DESCRIPTOR_STRING,
		{'H','a','r','m','o','n','y',' ','U','S','B',' ','S','p','e','a','k','e','r',' ','2','.','0'}
    }; 

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};

/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    &deviceDescriptor,          /* High speed device descriptor*/
    1,                          /* Total number of high speed configurations available */
    highSpeedConfigDescSet,     /* Pointer to array of high speed configurations descriptors. */
    3,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    &deviceQualifierDescriptor1,// Pointer to full speed dev qualifier.
    &deviceQualifierDescriptor1 // Pointer to high speed dev qualifier.
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,
    
    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_HIGH,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBHS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBHS_DEVICE_INTERFACE,
    
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());

    /* Board Support Package Initialization */
    BSP_Initialize();

    //KEEP THIS - BSP WORKAROUND
    BSP_USBSwitchSelect(BSP_USB_DEVICE);

    /* Initialize Drivers */
    sysObj.drvI2S0 = DRV_I2S_Initialize(DRV_I2S_INDEX_0, (SYS_MODULE_INIT *)&drvI2S0InitData);




    /*Initialize PMP0 */
    DRV_PMP0_Initialize();
    DRV_PMP0_ModeConfig();

    sysObj.sysDma = SYS_DMA_Initialize((SYS_MODULE_INIT *)&sysDmaInit);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA0, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA0, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA1, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA1, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0);
    SYS_INT_SourceEnable(INT_SOURCE_DMA_1);


    sysObj.drvak4384Codec0 = DRV_AK4384_Initialize(DRV_AK4384_INDEX_0, (SYS_MODULE_INIT *)&drvak4384Codec0InitData);


    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);
    sysObj.drvTmr1 = DRV_TMR_Initialize(DRV_TMR_INDEX_1, (SYS_MODULE_INIT *)&drvTmr1InitData);


    SYS_INT_VectorPrioritySet(INT_VECTOR_T2, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T2, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_T4, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T4, INT_SUBPRIORITY_LEVEL0);
 
 
  
    // initialize the GFX hardware abstraction layer
    GFX_Initialize();

     /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBHS_Initialize(DRV_USBHS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBHSInit);

    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);
    
    /* Set the priority of the USB DMA Interrupt */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1_DMA, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of the USB DMA Interrupt */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1_DMA, INT_SUBPRIORITY_LEVEL0);
    

    /* Initialize System Services */
    SYS_PORTS_Initialize();

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /*** TMR Service Initialization Code ***/
    sysObj.sysTmr  = SYS_TMR_Initialize(SYS_TMR_INDEX_0, (const SYS_MODULE_INIT  * const)&sysTmrInitData);
  
    /* Initialize Middleware */
 
    // initialize UI library
    LibAria_Initialize();

    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

