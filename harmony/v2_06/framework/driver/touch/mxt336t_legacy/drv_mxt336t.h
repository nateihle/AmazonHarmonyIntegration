/*******************************************************************************
 Touch Controller MXT336T Driver Interface File

  File Name:
    drv_MXT336T.c

  Summary:
    Touch controller MXT336T Driver interface header file.

  Description:
    This header file describes the macros, data structure and prototypes of the 
    touch controller MXT336T driver interface.
 ******************************************************************************/

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
 ******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRV_MXT336T_H
#define _DRV_MXT336T_H

#include "driver/touch/drv_touch.h"
#include "driver/i2c/drv_i2c.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* MXT336T Driver Handle

  Summary:
    Touch screen controller MXT336T driver handle.

  Description:
    Touch controller MXT336T driver handle is a handle for the driver
    client object. Each driver with successful open call will return a new handle
    to the client object.

  Remarks:
    None.
*/

typedef uintptr_t DRV_MXT336T_HANDLE;



// *****************************************************************************
/* MXT336T Driver Invalid Handle

  Summary:
    Definition of an invalid handle.

  Description:
    This is the definition of an invalid handle. An invalid handle is 
    is returned by DRV_MXT336T_Open() and DRV_MXT336T_Close()
    functions if the request was not successful.

  Remarks:
    None.
*/

#define DRV_MXT336T_HANDLE_INVALID ((DRV_MXT336T_HANDLE)(-1))


// *****************************************************************************
/* MXT336T Driver Module Index Numbers

  Summary:
    MXT336T driver index definitions.

  Description:
    These constants provide the MXT336T driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_MXT336T_Initialize and
    DRV_MXT336T_Open functions to identify the driver instance in use.
*/

#define DRV_MXT336T_INDEX_0         0
#define DRV_MXT336T_INDEX_1         1

// *****************************************************************************
/* MXT336T Driver Module Index Count

  Summary:
    Number of valid Touch controller MXT336T driver indices.

  Description:
    This constant identifies the number of valid Touch Controller MXT336T
    driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

#define DRV_MXT336T_INDEX_COUNT     2

// *****************************************************************************
/* MXT336T Driver Module I2C Frame Size

  Summary:
    I2C Frame size for reading MXT336T touch input.

  Description:
    This constant identifies the size of I2C frame required to read from
    MXT336T touch controller. MXT336T notifies the availability of input data
    through interrupt pin.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific data sheets.
*/
#define DRV_MXT336T_I2C_FRAME_SIZE                   32
#define DRV_MXT336T_I2C_READ_ID_FRAME_SIZE          8

// *****************************************************************************
/* MXT336T Driver Module Master Command Write I2C Address

  Summary:
    MXT336T command register write, I2C address where master sends the
    commands.

  Description:
    This constant defines the MXT336T command register I2C write address. This
    address is used as I2C address to write commands into MXT336T Touch
    controller register.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific data sheets.
*/
#define DRV_MXT336T_I2C_MASTER_WRITE_ID                0x94

// *****************************************************************************
/* MXT336T Driver Module Master Input Read I2C address

  Summary:
    MXT336T input read, I2C address from where master reads touch input data.

  Description:
    This constant defines the MXT336T touch input read I2C address. This
    address is used as I2C address to read Touch input from MXT336T Touch
    controller.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific data sheets.
*/
#define DRV_MXT336T_I2C_MASTER_READ_ID                0x95

// *****************************************************************************
/* MXT336T Driver Object Register Adresses for the registers being read in the driver

  Summary:
    MXT336T Driver Object Register Adresses for the registers being read in the driver 

  Description:
    MXT336T Objects have different registers that contain certain values regarding display resoltuion etc.
    These register addresses are used to read the values from object tables.
 
  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific protocol guides.
*/
#define DRV_MXT336T_T100_XRANGE                13
#define DRV_MXT336T_T100_YRANGE                24
// *****************************************************************************
/*Enumeration 
	DRV_MXT336T_OBJECT_TYPE

  Summary:
    The enum lists the different objects supported by the maxtouch device.
	
  Description:
	The MAxtouch devices follow a Object protocol for their driver implementation.
	This makes it possible to implement a generic driver for many maxtouch devices.
	The device communicates the different properties or status like touch messages etc with the driver through an Object table.
	The different types of objects associated with the maxtouch device are listed in the enum below.

  Remarks:
    None.

*/
typedef enum {
  DRV_MXT336T_OBJECT_RESERVED_T0 = 0 , 
  DRV_MXT336T_OBJECT_RESERVED_T1 = 1 , 
  DRV_MXT336T_OBJECT_DEBUG_DELTAS_T2 = 2 , 
  DRV_MXT336T_OBJECT_DEBUG_REFERENCES_T3 = 3 , 
  DRV_MXT336T_OBJECT_DEBUG_SIGNALS_T4 = 4 , 
  DRV_MXT336T_OBJECT_GEN_MESSAGEPROCESSOR_T5 = 5 , 
  DRV_MXT336T_OBJECT_GEN_COMMANDPROCESSOR_T6 = 6 , 
  DRV_MXT336T_OBJECT_GEN_POWERCONFIG_T7 = 7 , 
  DRV_MXT336T_OBJECT_GEN_ACQUISITIONCONFIG_T8 = 8 , 
  DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T9 = 9 , 
  DRV_MXT336T_OBJECT_TOUCH_SINGLETOUCHSCREEN_T10 = 10 , 
  DRV_MXT336T_OBJECT_TOUCH_XSLIDER_T11 = 11 , 
  DRV_MXT336T_OBJECT_TOUCH_YSLIDER_T12 = 12 , 
  DRV_MXT336T_OBJECT_TOUCH_XWHEEL_T13 = 13 , 
  DRV_MXT336T_OBJECT_TOUCH_YWHEEL_T14 = 14 , 
  DRV_MXT336T_OBJECT_TOUCH_KEYARRAY_T15 = 15 , 
  DRV_MXT336T_OBJECT_PROCG_SIGNALFILTER_T16 = 16 , 
  DRV_MXT336T_OBJECT_PROCI_LINEARIZATIONTABLE_T17 = 17 , 
  DRV_MXT336T_OBJECT_SPT_COMMSCONFIG_T18 = 18 , 
  DRV_MXT336T_OBJECT_SPT_GPIOPWM_T19 = 19 , 
  DRV_MXT336T_OBJECT_PROCI_GRIPFACESUPPRESSION_T20 = 20 , 
  DRV_MXT336T_OBJECT_RESERVED_T21 = 21 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPPRESSION_T22 = 22 , 
  DRV_MXT336T_OBJECT_TOUCH_PROXIMITY_T23 = 23 , 
  DRV_MXT336T_OBJECT_PROCI_ONETOUCHGESTUREPROCESSOR_T24 = 24 , 
  DRV_MXT336T_OBJECT_SPT_SELFTEST_T25 = 25 , 
  DRV_MXT336T_OBJECT_DEBUG_CTERANGE_T26 = 26 , 
  DRV_MXT336T_OBJECT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27 = 27 , 
  DRV_MXT336T_OBJECT_SPT_CTECONFIG_T28 = 28 , 
  DRV_MXT336T_OBJECT_SPT_GPI_T29 = 29 , 
  DRV_MXT336T_OBJECT_SPT_GATE_T30 = 30 , 
  DRV_MXT336T_OBJECT_TOUCH_KEYSET_T31 = 31 , 
  DRV_MXT336T_OBJECT_TOUCH_XSLIDERSET_T32 = 32 , 
  DRV_MXT336T_OBJECT_RESERVED_T33 = 33 , 
  DRV_MXT336T_OBJECT_GEN_MESSAGEBLOCK_T34 = 34 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T35 = 35 , 
  DRV_MXT336T_OBJECT_RESERVED_T36 = 36 , 
  DRV_MXT336T_OBJECT_DEBUG_DIAGNOSTIC_T37 = 37 , 
  DRV_MXT336T_OBJECT_SPT_USERDATA_T38 = 38 , 
  DRV_MXT336T_OBJECT_SPARE_T39 = 39 , 
  DRV_MXT336T_OBJECT_PROCI_GRIPSUPPRESSION_T40 = 40 , 
  DRV_MXT336T_OBJECT_PROCI_PALMSUPPRESSION_T41 = 41 , 
  DRV_MXT336T_OBJECT_PROCI_TOUCHSUPPRESSION_T42 = 42 , 
  DRV_MXT336T_OBJECT_SPT_DIGITIZER_T43 = 43 , 
  DRV_MXT336T_OBJECT_SPT_MESSAGECOUNT_T44 = 44 , 
  DRV_MXT336T_OBJECT_PROCI_VIRTUALKEY_T45 = 45 , 
  DRV_MXT336T_OBJECT_SPT_CTECONFIG_T46 = 46 , 
  DRV_MXT336T_OBJECT_PROCI_STYLUS_T47 = 47 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPPRESSION_T48 = 48 , 
  DRV_MXT336T_OBJECT_GEN_DUALPULSE_T49 = 49 , 
  DRV_MXT336T_OBJECT_SPARE_T50 = 50 , 
  DRV_MXT336T_OBJECT_SPT_SONY_CUSTOM_T51 = 51 , 
  DRV_MXT336T_OBJECT_TOUCH_PROXKEY_T52 = 52 , 
  DRV_MXT336T_OBJECT_GEN_DATASOURCE_T53 = 53 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPPRESSION_T54 = 54 , 
  DRV_MXT336T_OBJECT_PROCI_ADAPTIVETHRESHOLD_T55 = 55 , 
  DRV_MXT336T_OBJECT_PROCI_SHIELDLESS_T56 = 56 , 
  DRV_MXT336T_OBJECT_PROCI_EXTRATOUCHSCREENDATA_T57 = 57 , 
  DRV_MXT336T_OBJECT_SPT_EXTRANOISESUPCTRLS_T58 = 58 , 
  DRV_MXT336T_OBJECT_SPT_FASTDRIFT_T59 = 59 , 
  DRV_MXT336T_OBJECT_SPT_TIMER_T61 = 61 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPPRESSION_T62 = 62 , 
  DRV_MXT336T_OBJECT_PROCI_ACTIVESTYLUS_T63 = 63 , 
  DRV_MXT336T_OBJECT_SPT_REFERENCERELOAD_T64 = 64 , 
  DRV_MXT336T_OBJECT_PROCI_LENSBENDING_T65 = 65 , 
  DRV_MXT336T_OBJECT_SPT_GOLDENREFERENCES_T66 = 66 , 
  DRV_MXT336T_OBJECT_PROCI_CUSTOMGESTUREPROCESSOR_T67 = 67 , 
  DRV_MXT336T_OBJECT_SERIAL_DATA_COMMAND_T68 = 68 , 
  DRV_MXT336T_OBJECT_PROCI_PALMGESTUREPROCESSOR_T69 = 69 , 
  DRV_MXT336T_OBJECT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70 = 70 , 
  DRV_MXT336T_OBJECT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71 = 71 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPPRESSION_T72 = 72 , 
  DRV_MXT336T_OBJECT_PROCI_ZONEINDICATION_T73 = 73 , 
  DRV_MXT336T_OBJECT_PROCG_SIMPLEGESTUREPROCESSOR_T74 = 74 , 
  DRV_MXT336T_OBJECT_MOTION_SENSING_OBJECT_T75 = 75 , 
  DRV_MXT336T_OBJECT_PROCI_MOTION_GESTURES_T76 = 76 , 
  DRV_MXT336T_OBJECT_SPT_CTESCANCONFIG_T77 = 77 , 
  DRV_MXT336T_OBJECT_PROCI_GLOVEDETECTION_T78 = 78 , 
  DRV_MXT336T_OBJECT_SPT_TOUCHEVENTTRIGGER_T79 = 79 , 
  DRV_MXT336T_OBJECT_PROCI_RETRANSMISSIONCOMPENSATION_T80 = 80 , 
  DRV_MXT336T_OBJECT_PROCI_UNLOCKGESTURE_T81 = 81 , 
  DRV_MXT336T_OBJECT_SPT_NOISESUPEXTENSION_T82 = 82 , 
  DRV_MXT336T_OBJECT_ENVIRO_LIGHTSENSING_T83 = 83 , 
  DRV_MXT336T_OBJECT_PROCI_GESTUREPROCESSOR_T84 = 84 , 
  DRV_MXT336T_OBJECT_PEN_ACTIVESTYLUSPOWER_T85 = 85 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPACTIVESTYLUS_T86 = 86 , 
  DRV_MXT336T_OBJECT_PEN_ACTIVESTYLUSDATA_T87 = 87 , 
  DRV_MXT336T_OBJECT_PEN_ACTIVESTYLUSRECEIVE_T88 = 88 , 
  DRV_MXT336T_OBJECT_PEN_ACTIVESTYLUSTRANSMIT_T89 = 89 , 
  DRV_MXT336T_OBJECT_PEN_ACTIVESTYLUSWINDOW_T90 = 90 , 
  DRV_MXT336T_OBJECT_DEBUG_CUSTOMDATACONFIG_T91 = 91 , 
  DRV_MXT336T_OBJECT_PROCI_SYMBOLGESTUREPROCESSOR_T92 = 92 , 
  DRV_MXT336T_OBJECT_PROCI_TOUCHSEQUENCELOGGER_T93 = 93 , 
  DRV_MXT336T_OBJECT_SPT_PTCCONFIG_T95 = 95 , 
  DRV_MXT336T_OBJECT_SPT_PTCTUNINGPARAMS_T96 = 96 , 
  DRV_MXT336T_OBJECT_TOUCH_PTCKEYS_T97 = 97 , 
  DRV_MXT336T_OBJECT_PROCG_PTCNOISESUPPRESSION_T98 = 98 , 
  DRV_MXT336T_OBJECT_PROCI_KEYGESTUREPROCESSOR_T99 = 99 , 
  DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100 = 100 , 
  DRV_MXT336T_OBJECT_SPT_TOUCHSCREENHOVER_T101 = 101 , 
  DRV_MXT336T_OBJECT_SPT_SELFCAPHOVERCTECONFIG_T102 = 102 , 
  DRV_MXT336T_OBJECT_PROCI_SCHNOISESUPPRESSION_T103 = 103 , 
  DRV_MXT336T_OBJECT_SPT_AUXTOUCHCONFIG_T104 = 104 , 
  DRV_MXT336T_OBJECT_SPT_DRIVENPLATEHOVERCONFIG_T105 = 105 , 
  DRV_MXT336T_OBJECT_SPT_ACTIVESTYLUSMMBCONFIG_T106 = 106 , 
  DRV_MXT336T_OBJECT_PROCI_ACTIVESTYLUS_T107 = 107 , 
  DRV_MXT336T_OBJECT_PROCG_NOISESUPSELFCAP_T108 = 108 , 
  DRV_MXT336T_OBJECT_SPT_SELFCAPGLOBALCONFIG_T109 = 109 , 
  DRV_MXT336T_OBJECT_SPT_SELFCAPTUNINGPARAMS_T110 = 110 , 
  DRV_MXT336T_OBJECT_SPT_SELFCAPCONFIG_T111 = 111 , 
  DRV_MXT336T_OBJECT_PROCI_SELFCAPGRIPSUPPRESSION_T112 = 112 , 
  DRV_MXT336T_OBJECT_SPT_PROXMEASURECONFIG_T113 = 113 , 
  DRV_MXT336T_OBJECT_SPT_ACTIVESTYLUSMEASCONFIG_T114 = 114 , 
  DRV_MXT336T_OBJECT_PROCI_SYMBOLGESTURE_T115 = 115 , 
  DRV_MXT336T_OBJECT_SPT_SYMBOLGESTURECONFIG_T116 = 116 , 
  DRV_MXT336T_OBJECT_GEN_INFOBLOCK16BIT_T254 = 254 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T220 = 220 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T221 = 221 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T222 = 222 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T223 = 223 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T224 = 224 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T225 = 225 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T226 = 226 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T227 = 227 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T228 = 228 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T229 = 229 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T230 = 230 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T231 = 231 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T232 = 232 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T233 = 233 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T234 = 234 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T235 = 235 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T236 = 236 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T237 = 237 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T238 = 238 , 
  DRV_MXT336T_OBJECT_SPT_PROTOTYPE_T239 = 239 , 
  DRV_MXT336T_OBJECT_RESERVED_T255 = 255 ,
} DRV_MXT336T_OBJECT_TYPE;



// *****************************************************************************
/*Structure 
	DRV_MXT336T_OBJECT_CLIENT_EVENT_DATA

  Summary:
   This structure maintains the information associated with each msg received or event that occurs
  
  Description:
   This structure maintains the information associated with each msg received or event that occurs.
   Each msg gets a reportID that identifies the object reporting the change in status. 
   For touch messages the xRange and yRange for the touch device gets reported and the 
   data pointer contains the status msg information which has the touch type, touch event and touch 
   coordinates.
   	

  Remarks:
    None.

*/                            
typedef struct 
{      
    uint8_t         reportID;           /* report ID within the object */
    uint8_t         dataSize;           /* max size of data */
    uint8_t         *pData;             /* data associated with the report */
    uint16_t         xRange;
    uint16_t        yRange;
} DRV_MXT336T_OBJECT_CLIENT_EVENT_DATA;

// *****************************************************************************
/*MXT336T Driver Callback Function Pointer

  Summary:
    Pointer to a MXT336T client  callback function data type.

  Description:
    This data type defines a pointer to a MXT336T client  callback function.

  Remarks:
    
*/
typedef void ( *DRV_MXT336T_CLIENT_CALLBACK ) ( DRV_HANDLE clientObject, 
        DRV_MXT336T_OBJECT_CLIENT_EVENT_DATA *updateObject, uintptr_t context);

// *****************************************************************************
/*Structure
	DRV_MXT336T_INIT

  Summary:
    Defines the data required to initialize or reinitialize the MXT336T driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    MXT336T driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/
typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT         moduleInit;

    /* ID */
    int                     touchId;

    /* initialize function for module (normally called statically */
    SYS_MODULE_OBJ          (*drvInitialize) (const SYS_MODULE_INDEX   index,
                                                const SYS_MODULE_INIT    * const init);

    /* open function for I2C driver */
    DRV_HANDLE              (*drvOpen) ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );
    
    /* interrupt source for driver instance */
    INT_SOURCE              interruptSource;
    
    /* interrupt pin for driver instance */
    PORTS_BIT_POS           interruptPin;
    
    /* port channel for interrupt instance */
    PORTS_CHANNEL           interruptChannel;
       
    /* reset pin for driver instance */
    PORTS_BIT_POS           resetPin;
    
    /* port channel for reset pin */
    PORTS_CHANNEL           resetChannel;
    
    /* */
    uint16_t	           orientation;          // Orientation of the display (given in degrees of 0,90,180,270)

    /* */
    uint16_t               horizontalResolution; // Horizontal Resolution of the displayed orientation in Pixels

    /* */
    uint16_t               verticalResolution;
             
} DRV_MXT336T_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_MXT336T_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the MXT336T instance for the specified driver index

  Description:
    This routine initializes the MXT336T driver instance for the specified
    driver index, making it ready for clients to open and use it. The
    initialization data is specified by the 'init' parameter. The initialization
    may fail if the number of driver objects allocated are insufficient or if
    the specified driver instance is already initialized. The driver instance
    index is independent of the MXT336T module ID. For example, driver instance
    0 can be assigned to MXT336T2.  If the driver is built statically, then
    some of the initialization parameters are overridden by configuration
    macros. Refer to the description of the DRV_MXT336T_INIT data
    structure for more details on which members on this data structure are
    overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the MXT336T ID.  The hardware MXT336T ID is set in the
             initialization structure. This is the index of the driver index to
             use.

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    DRV_MXT336T_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the MXT336T initialization structure
    // Touch Module Id
    init.moduleInit                  = {0},
    init.touchId                     = DRV_TOUCH_INDEX_0,
    init.drvInitialize               = NULL,
    init.drvOpen                     = DRV_I2C_Open,
    init.interruptSource             = INT_SOURCE_EXTERNAL_1,
    init.interruptChannel            = PORT_CHANNEL_D,
    init.interruptPin                = PORTS_BIT_POS_1,
    init.resetChannel                = PORT_CHANNEL_A,
    init.resetPin                    = PORTS_BIT_POS_14,

    objectHandle = DRV_MXT336T_Initialize(DRV_TOUCH_INDEX_0,
                                              (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other MXT336T routine is called.

    This routine should only be called once during system initialization
    unless DRV_MXT336T_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_MXT336T_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );

/*************************************************************************
  Function:
       void DRV_MXT336T_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the MXT336T driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the MXT336T driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    Function DRV_MXT336T_Initialize must have been called before calling 
    this routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameter:
    object -  Driver object handle, returned from DRV_MXT336T_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;    //Returned from DRV_MXT336T_Initialize
    SYS_STATUS          status;

    DRV_MXT336T_Deinitialize ( object );

    status = DRV_MXT336T_Status( object );
    if( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the De-initialize
    operation must be called before the Initialize operation can be called
    again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_MXT336T_Status operation. The system has to use
    DRV_MXT336T_Status to determine when the module is in the ready state.
*/

void DRV_MXT336T_Deinitialize ( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_MXT336T_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the MXT336T driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function provides the current status of the MXT336T driver module.

  Precondition:
    The DRV_MXT336T_Initialize function must have been called before 
    calling this function.

  Parameters:
    object -  Driver object handle, returned from DRV_MXT336T_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system-level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;  // Returned from DRV_MXT336T_Initialize
    SYS_STATUS          status;

    status = DRV_MXT336T_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>

  Remarks:
    Any value greater than SYS_STATUS_READY is also a normal running state
    in which the driver is ready to accept new operations.

    SYS_MODULE_UNINITIALIZED - Indicates that the driver has been
    deinitialized

    This value is less than SYS_STATUS_ERROR.

    This function can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the previous operation
    has not yet completed. Once the status operation returns
    SYS_STATUS_READY, any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1). Any value less than
    that is also an error state.

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be
    cleared by calling the reinitialize operation. If that fails, the
    deinitialize operation will need to be called, followed by the
    initialize operation to return to normal operations.
*/

SYS_STATUS DRV_MXT336T_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_MXT336T_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available in drvI2CReadFrameData.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the 
    specified MXT336T driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_MXT336T_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_MXT336T_Initialize

    void SYS_Tasks( void )
    {
        DRV_MXT336T_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)

*/

void DRV_MXT336T_Tasks ( SYS_MODULE_OBJ object );

// *****************************************************************************
/* Function:
    void DRV_MXT336T_ReadRequest( SYS_MODULE_OBJ object )

  Summary:
    Sends a read request to I2C bus driver and adds the read task to queue.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to send a touch input read request to the I2C bus
        driver and adding the input read decode task to the queue. It is always
        called from MXT336T interrupt ISR routine.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the 
    specified MXT336T driver instance. 

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_MXT336T_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_MXT336T_Initialize

    void __ISR(_EXTERNAL_INT_VECTOR, ipl5) _IntHandlerDrvMXT(void)
    {
        DRV_MXT336T_ReadRequest ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the MXT336T ISR routine.

*/
void DRV_MXT336T_ReadRequest( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_MXT336T_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified MXT336T driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified MXT336T driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The current version of driver does not support the DRV_IO_INTENT feature.
    The driver is by default non-blocking. The driver can perform both read
    and write to the MXT336T device. The driver supports single client only.	
	
  Precondition:
    The DRV_MXT336T_Initialize function must have been called before 
    calling this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_MXT336T_Initialize().
                
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver. The current version of driver does not support
				the selective IO intent feature.
				
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_MXT336T_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
		
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_MXT336T_Open( DRV_MXT336T_INDEX_0,
                                      DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>
	
  Remarks:
    The handle returned is valid until the DRV_MXT336T_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_MXT336T_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_MXT336T_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MXT336T driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes an opened instance of the MXT336T driver, invalidating
    the handle.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the 
    specified MXT336T driver instance.

    DRV_MXT336T_Open must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_Open

    DRV_MXT336T_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be 
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_MXT336T_Open before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the 
          Close operation has completed.
*/

void DRV_MXT336T_Close ( DRV_HANDLE handle );

/**************************************************************************
  Function:
       DRV_HANDLE DRV_MXT336T_OpenObject ( const DRV_HANDLE deviceHandle, const uint8_t objType,
               const uint8_t objInstance )

  Summary:
    Opens the specified MXT336T object driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified MXT336T object driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. 
	
  Precondition:
    The DRV_MXT336T_Initialize function must have been called before 
    calling this function. The driver must have been opened.
	
  Parameters:
    deviceHandle -  Handle of the MXT336T device
    objType      -  Object type being requested
    objInstance  -  Instance of the object of this type
   
				
  Returns:
    If successful, the routine returns a valid object-instance handle (
		
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_MXT336T_OpenObject(drvHandle, GEN_PROCESSOR_T5, 1);
    </code>
	
  Remarks:

*/

DRV_HANDLE DRV_MXT336T_OpenObject ( const DRV_HANDLE deviceHandle, const uint8_t objType,
               const uint8_t objInstance );

// *****************************************************************************
/* Function:
    void DRV_MXT336T_CloseObject ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MXT336T client object

  Description:
    This function closes an opened instance of the MXT336T client object, invalidating
    the handle.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the
    specified MXT336T driver instance.

    DRV_MXT336T_OpenObject must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_Open

    DRV_MXT336T_CloseObject ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_MXT336T_OpenObject before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the
          Close operation has completed.
*/

void DRV_MXT336T_CloseObject ( DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    bool DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(const DRV_HANDLE clientHandle,
        const DRV_MXT336T_CLIENT_CALLBACK callback, uintptr_t context)

  Summary:
    Sets the event handler for a MXT336T client object

  Description:
    This function sets the event handler used to handle report messages
    from a MXT336T object.
  Precondition:
    The DRV_MXT336T_OpenObject routine must have been called for the
    specified MXT336T driver instance.

    DRV_MXT336T_OpenObject must have been called to obtain a valid opened
    device handle.

  Parameters:
    clientHandle        - A valid open-instance handle, returned from the driver's
                            openobject routine
    
    callback            - A callback function to handle report messages
 
    context             - The context for the call
  Returns:
    bool                - true if the handler was successfully set
                        - false if the handler could not be set

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_OpenObject

    DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(handle, objectCallback, NULL);
    </code>

  Remarks:

*/

bool DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(const DRV_HANDLE clientHandle,
        const DRV_MXT336T_CLIENT_CALLBACK callback, uintptr_t context);


#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_MXT336T_H