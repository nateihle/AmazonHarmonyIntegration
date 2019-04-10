/*******************************************************************************
 Touch controller AR1021 driver file

  File Name:
    drv_ar1021.c

  Summary:
    Touch controller AR1021 driver implementation.

  Description:
    This file consist of touch controller AR1021 driver interfaces. It
    implements the driver interfaces which read the touch input data from
    AR1021 through SPI bus.
 ****************************************************************************/

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
 ******************************************************************************/
// DOM-IGNORE-END

#include "system_config.h"
#include "driver/touch/ar1021/drv_ar1021.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: MACROS
// *****************************************************************************
// *****************************************************************************
#ifndef BSP_TouchScreenChipOff()
#define BSP_TouchScreenChipOff Nop
#endif

#ifndef BSP_TouchScreenChipOff()
#define BSP_TouchScreenChipOff Nop
#endif

 
DRV_TOUCH_POSITION_STATUS       touchStatus;

// *****************************************************************************
/* AR1021 Accessible Register Identification.

  Summary:
    List of AR1021 Accessible Register Identification.

  Description:
    This enumeration identifies the different accessible AR1021 Registers. 
    The identifier is passed as source to the register read routine or as 
    destination to the register write routine. The AR1021 driver routine to 
    read the accessible AR1021 registers is DRV_AR1021_RegisterRead. 
    The AR1021 driver routine to write the accessible AR1021 registers 
    is DRV_MTCH6303_RegisterWrite.

  Remarks:
    none
*/
        
typedef enum
{
    /* Threshold register */
    DRV_AR1021_REG_TOUCH_THRESHOLD /* DOM-IGNORE-BEGIN */ = 0x02, /* DOM-IGNORE-END */
            
    /* Sensitivity register */
    DRV_AR1021_REG_SENSITIVITY_FILTER /* DOM-IGNORE-BEGIN */ = 0x03, /* DOM-IGNORE-END */
            
    /* Sampling Fast register */
    DRV_AR1021_REG_SAMPLING_FAST /* DOM-IGNORE-BEGIN */ = 0x04, /* DOM-IGNORE-END */

    /* Sampling Slow register */            
    DRV_AR1021_REG_SAMPLING_SLOW /* DOM-IGNORE-BEGIN */ = 0x05, /* DOM-IGNORE-END */
            
    /* Accuracy Filter Fast register */
    DRV_AR1021_REG_ACCURACY_FILTER_FAST /* DOM-IGNORE-BEGIN */ = 0x06, /* DOM-IGNORE-END */
    
    /* Accuracy Filter Slow register */
    DRV_AR1021_REG_ACCURACY_FILTER_SLOW /* DOM-IGNORE-BEGIN */ = 0x07, /* DOM-IGNORE-END */

    /* Speed Threshold register */            
    DRV_AR1021_REG_SPEED_THRESHOLD /* DOM-IGNORE-BEGIN */ = 0x08, /* DOM-IGNORE-END */

    /* Speed Delay register */
    DRV_AR1021_REG_SLEEP_DELAY /* DOM-IGNORE-BEGIN */ = 0x0A, /* DOM-IGNORE-END */
            
    /* Pen Up Delay register */
    DRV_AR1021_REG_PEN_UP_DELAY /* DOM-IGNORE-BEGIN */ = 0x0B, /* DOM-IGNORE-END */
            
    /* Touch Mode register */
    DRV_AR1021_REG_TOUCH_MODE /* DOM-IGNORE-BEGIN */ = 0x0C, /* DOM-IGNORE-END */
    
    /* Touch Options register */
    DRV_AR1021_REG_TOUCH_OPTIONS /* DOM-IGNORE-BEGIN */ = 0x0D, /* DOM-IGNORE-END */

    /* Calibration Inset register */            
    DRV_AR1021_REG_CALIBRATION_INSET /* DOM-IGNORE-BEGIN */ = 0x0E, /* DOM-IGNORE-END */

    /* Pen State Report Delay register */
    DRV_AR1021_REG_PEN_STATE_REPORT_DELAY      /* DOM-IGNORE-BEGIN */ = 0x0F, /* DOM-IGNORE-END */
            
    /* Touch Report Delay register */
    DRV_AR1021_REG_TOUCH_REPORT_DELAY /* DOM-IGNORE-BEGIN */ = 0x11, /* DOM-IGNORE-END */
              
} DRV_TOUCH_AR1021_REGISTER_MAP;

// not used
//#define TOUCH_AR1021_FREE_EEPROM_START_ADDR     0x0080


// *****************************************************************************
/* AR1021 Driver task states

  Summary
    Lists the different states that AR1021 task routine can have.

  Description
    This enumeration lists the different states that AR1021 task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* */
    DRV_TOUCH_AR1021_RUNNING,

} DRV_TOUCH_AR1021_OBJECT_TASK;

// *****************************************************************************
/* AR1021 Driver client object

  Summary
    AR1021 Driver client object maintaining client data.

  Description
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/
typedef struct _DRV_TOUCH_AR1021_CLIENT_OBJECT
{
    /* The status of the driver */
    SYS_STATUS                        status;
    
    /* Driver Object associated with the client */
    struct DRV_TOUCH_AR1021_OBJECT*   driverObject;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                     intent;

    struct _DRV_SPI_CLIENT_OBJECT *   pNext;
    
    /* Flag to indicate instance in use  */
    bool                              inUse;
    
    DRV_TOUCH_AR1021_OBJECT_TASK      task;
    
} DRV_TOUCH_AR1021_CLIENT_OBJECT;

// *****************************************************************************
/* AR1021 Driver Instance Object.

  Summary:
    Defines the data structure maintaining AR1021 driver instance object.

  Description:
    This data structure maintains the AR1021 driver instance object. The
    object exists once per hardware instance.

  Remarks:
    None.
*/
typedef struct
{
    SYS_STATUS                      status;

    SYS_MODULE_INDEX                touchId;

    SYS_MODULE_INDEX                drvIndex;
    
    /* Flag to indicate instance in use  */
    bool                            inUse;
    
    bool                            isExclusive;

    uint8_t                         numClients;

    uint16_t	                    orientation;

    uint16_t                        horizontalResolution;

    uint16_t                        verticalResolution;
    
    /* Callback for SPI Driver Open call */
    DRV_HANDLE                      (*drvOpen) ( const SYS_MODULE_INDEX index,
                                                const DRV_IO_INTENT intent );
    /* Touch status */
    DRV_TOUCH_POSITION_STATUS       touchStatus;
    
    DRV_TOUCH_AR1021_CLIENT_OBJECT  * client;
    
} DRV_TOUCH_AR1021_OBJECT;

/* AR1021 Driver instance object */
static DRV_TOUCH_AR1021_OBJECT sAR1021DriverInstances[1];

/* AR1021 Driver client object */
static DRV_TOUCH_AR1021_CLIENT_OBJECT sAR1021ClientInstances[1];

#define DRV_TOUCH_AR1021_MAX_REGS_PKTS 12
static uint8_t regPacket[DRV_TOUCH_AR1021_MAX_REGS_PKTS];

// *****************************************************************************
/* AR1021 Command Set Identifiers.

  Summary:
    List of AR1021 Command Identifiers.

  Description:
    This enumeration identifies the different AR1021 commands. 
    The identifier is passed as parameter in the send and receive message 
    protocol.
 * 
  Remarks:
    none
*/

typedef enum
{
    TOUCH_AR1021_CMD_HEADER             /* DOM-IGNORE-BEGIN */ = 0x55, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_ENABLE_TOUCH       /* DOM-IGNORE-BEGIN */ = 0x12, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_DISABLE_TOUCH      /* DOM-IGNORE-BEGIN */ = 0x13, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_CALIBRATE_MODE     /* DOM-IGNORE-BEGIN */ = 0x14, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_REG_READ           /* DOM-IGNORE-BEGIN */ = 0x20, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_REG_WRITE          /* DOM-IGNORE-BEGIN */ = 0x21, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_REG_START_ADDR     /* DOM-IGNORE-BEGIN */ = 0x22, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_REG_WRITE_EEPROM   /* DOM-IGNORE-BEGIN */ = 0x23, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_EEPROM_READ        /* DOM-IGNORE-BEGIN */ = 0x28, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_EEPROM_WRITE       /* DOM-IGNORE-BEGIN */ = 0x29, /* DOM-IGNORE-END */
    TOUCH_AR1021_CMD_EEPROM_WRITE_REG   /* DOM-IGNORE-BEGIN */ = 0x2B, /* DOM-IGNORE-END */

    TOUCH_AR1021_RESP_SUCCESS            /* DOM-IGNORE-BEGIN */ = 0x00, /* DOM-IGNORE-END */
    TOUCH_AR1021_RESP_BAD_CMD            /* DOM-IGNORE-BEGIN */ = 0x01, /* DOM-IGNORE-END */
    TOUCH_AR1021_RESP_BAD_HDR            /* DOM-IGNORE-BEGIN */ = 0x03, /* DOM-IGNORE-END */
    TOUCH_AR1021_RESP_CMD_TO             /* DOM-IGNORE-BEGIN */ = 0x04, /* DOM-IGNORE-END */
    TOUCH_AR1021_RESP_CAL_CANCEL         /* DOM-IGNORE-BEGIN */ = 0xFC, /* DOM-IGNORE-END */
    TOUCH_AR1021_RESP_INVALID            /* DOM-IGNORE-BEGIN */ = 0xFF, /* DOM-IGNORE-END */
            
    TOUCH_AR1021_RESP_NO_DATA            /* DOM-IGNORE-BEGIN */ = 0x4D, /* DOM-IGNORE-END */

} DRV_TOUCH_AR1021_CMD_ID;

/* spi module selected by the user in  MHC */
static SPI_MODULE_ID spiModule = DRV_AR1021_SPI_CHANNEL_INDEX;

////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
/* AR1021 Touch Coordinate report.

  Summary:
    Defines AR1021 Touch Data Packet.

  Description:
     This structure defines AR1021 Touch Data Packet return 
     upon completion of a touch detection.
 
  Remarks:
    none
*/
typedef union
{
    struct
    {
        uint8_t pen:       1;
        uint8_t reserved:  6;
        uint8_t startBit:  1;
        uint8_t lowX;
        uint8_t highX;
        uint8_t lowY;
        uint8_t highY;
    };
    uint8_t packet[5];
} DRV_TOUCH_AR1021_TOUCH_REPORT_PACKET;


// *****************************************************************************
// *****************************************************************************
// Section: Prototypes
// *****************************************************************************
// *****************************************************************************

static void         _AR1021_TMR_DelayMS ( unsigned int delayMs );
static uint8_t      _GetByte(void);
static void         _SendByte(uint8_t data);
static uint16_t     _TouchAR1021GetRegisterStartAddress(void);
static uint8_t      _TouchAR1021GetResponceStatus(uint8_t command, uint8_t *data, uint8_t dataSize);
static uint8_t      _TouchAR1021SendCommandAndGetResponse(uint8_t command, uint8_t *data, uint8_t dataSize);
static void         _TouchAR1021SendCommand(uint8_t command, uint8_t *data, uint8_t dataSize);
static void         _TouchDetectPosition(void);
static DRV_HANDLE   _TouchHardwareInit(void *initValues);

// *****************************************************************************
// *****************************************************************************
// Section: Local Variable Definitions
// *****************************************************************************
// *****************************************************************************

volatile short xcor = -1;
volatile short ycor = -1;
volatile DRV_TOUCH_PEN_STATE xypen = DRV_TOUCH_PEN_UNKNOWN;
static uint8_t detectPosition;

// *****************************************************************************
// *****************************************************************************
// Section: AR1021 Driver Routines
// *****************************************************************************
// *****************************************************************************


//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_AR1021_Initialize(const SYS_MODULE_INDEX index,
                                               const SYS_MODULE_INIT * const init )

   Summary:
    Initializes the AR1021 instance for the specified driver index
*/
SYS_MODULE_OBJ DRV_TOUCH_AR1021_Initialize( const SYS_MODULE_INDEX index,
                                            const SYS_MODULE_INIT * const init )
{
    if ( index >= DRV_TOUCH_AR1021_INDEX_COUNT )
    {
        SYS_ASSERT(false, "AR1021 Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }
           
    DRV_TOUCH_AR1021_OBJECT * pDrvInstance =
                ( DRV_TOUCH_AR1021_OBJECT *)&sAR1021DriverInstances[index];
    
    if ( pDrvInstance->inUse == true )
    {
        SYS_ASSERT(false, "AR1021 Driver: Attempting to reinitialize a driver instance that is already in use");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    pDrvInstance->inUse = true;

    const DRV_TOUCH_INIT * const pInit = (const DRV_TOUCH_INIT * const)init;

    pDrvInstance->touchId              = pInit->touchId;
    pDrvInstance->drvOpen              = pInit->drvOpen;
    pDrvInstance->status               = SYS_STATUS_READY;
    pDrvInstance->verticalResolution   = pInit->verticalResolution;
    pDrvInstance->horizontalResolution = pInit->horizontalResolution;

    return (SYS_MODULE_OBJ)pDrvInstance;

}

/*************************************************************************
  Function:
       void DRV_TOUCH_AR1021_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the AR1021 driver module.
 */

void DRV_TOUCH_AR1021_Deinitialize ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_AR1021_OBJECT * pDrvInstance =
                                        (DRV_TOUCH_AR1021_OBJECT *)object;

    if( pDrvInstance == NULL )
    {
        SYS_ASSERT(false, "AR1021 Driver: Attempting to deinitialize a NULL object");
        return;
    }

    if ( pDrvInstance->inUse == false )
    {
        SYS_ASSERT(false, "AR1021 Driver: Attempting to deinitialize a driver instance that is not in use");
        return;
    }
    
    return;
}

/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_AR1021_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the ADC10BIT driver module.

  Description:
    This function provides the current status of the ADC10BIT driver module.
*/

SYS_STATUS DRV_TOUCH_AR1021_Status ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_AR1021_OBJECT * pDrvInstance = ( DRV_TOUCH_AR1021_OBJECT *)object;
    return pDrvInstance->status;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
    from SYS_Tasks() function. This routine detects a touch press position.
 */
void DRV_TOUCH_AR1021_Tasks ( SYS_MODULE_OBJ object )
{    
    static long lastDebounceTime = 0;
    static long debounceDelay = 50;
    
    /* Host object */
    DRV_TOUCH_AR1021_CLIENT_OBJECT *obj = (DRV_TOUCH_AR1021_CLIENT_OBJECT *)&sAR1021ClientInstances[0];

    if ( ! obj->inUse )
    {
        return;
    }

    switch ( obj->task )
    {

        case DRV_TOUCH_AR1021_RUNNING:
            if ( (SYS_TMR_TickCountGetLong() - lastDebounceTime) > debounceDelay )
            {
                _TouchDetectPosition();
                lastDebounceTime =  SYS_TMR_TickCountGetLong();
            }

            break;
    }
    
}

// *****************************************************************************
// *****************************************************************************
// Section: AR1021 Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_AR1021_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified  AR1021 driver instance and returns a handle to it.
*/

DRV_HANDLE DRV_TOUCH_AR1021_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent )
{
    DRV_TOUCH_AR1021_OBJECT *pClient = &sAR1021DriverInstances[0];
    if (pClient == NULL)
    {
        SYS_ASSERT(false, "AR1021 Driver: Couldn't find a free client to open");
        return DRV_HANDLE_INVALID;
    }

    /* Open the bus driver */
    if(_TouchHardwareInit(0) == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "AR1021 Driver: Bus driver init failed");
        return DRV_HANDLE_INVALID;
    }
    
    return (DRV_HANDLE)pClient;
}

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_AR1021_CalibrationSet (DRV_TOUCH_SAMPLE_POINTS * samplePoints)

  Summary:
     Set the driver to use the given sample points for calibration values.
*/
void DRV_TOUCH_AR1021_CalibrationSet(DRV_TOUCH_SAMPLE_POINTS * samplePoints)
{

}

/*********************************************************************
  Function:
    DRV_TOUCH_PEN_STATE DRV_TOUCH_AR1021_TouchPenGet( uint8 touchNumber )

  Summary:
    Returns the pen status of touch input.

*/
DRV_TOUCH_PEN_STATE DRV_TOUCH_AR1021_TouchPenGet(uint8_t touchNumber)
{
    return xypen;
}

/*********************************************************************
  Function:
    short DRV_TOUCH_AR1021_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.

  Description:
    It returns the x coordinate of the touched point in pixels.

*/
short DRV_TOUCH_AR1021_TouchGetX(uint8_t touchNumber)
{
    return (short)xcor;
}

/*********************************************************************
* Function: short DRV_TOUCH_AR1021_TouchGetY(uint8_t touchNumber)
*
* PreCondition: none
*
* Input: none
*
* Output: y coordinate
*
* Side Effects: none
*
* Overview: returns y coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
short DRV_TOUCH_AR1021_TouchGetY(uint8_t touchNumber)
{
    return (short)ycor;
}

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_AR1021_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.
*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_AR1021_TouchStatus( const SYS_MODULE_INDEX index )
{
    return (touchStatus);
}

/*********************************************************************
  Function:
    void DRV_TOUCH_AR1021_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read.
*/
void DRV_TOUCH_AR1021_TouchDataRead( const SYS_MODULE_INDEX index )
{
    touchStatus = DRV_TOUCH_POSITION_NONE;
}

// *****************************************************************************
// *****************************************************************************
// Section: Local functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void _TouchDetectPosition (void)

  Summary:
    Poll for a touch position.
*/
void _TouchDetectPosition(void)
{
    static long xc,yc;
    static DRV_TOUCH_AR1021_TOUCH_REPORT_PACKET touchReport;
    uint8_t i;

    xcor = -1;
    ycor = -1;
    
    /* touch coordinates are sent from the controller to the PIC
       in 5-byte data packet, containing the x and y axis as well 
       as a pen status. Each axis returns 12bits of information. 
       We shift all bits into one field. */
        
    if( detectPosition )
    {
        BSP_TouchScreenChipOff();

        /* get first byte of 5 packet */
        touchReport.packet[0] = _GetByte();
        
        // ignore invalid first byte if it is not a pen up or pen down
        if ( TOUCH_AR1021_RESP_NO_DATA == touchReport.packet[0] || 0x0 == touchReport.packet[0] || TOUCH_AR1021_RESP_INVALID == touchReport.packet[0]  )
        {
            touchStatus = DRV_TOUCH_POSITION_NONE;
            return ;
        }
        
        /* get remaining 4 bytes - x/y coordinate */
        for(i = 1; i < sizeof(DRV_TOUCH_AR1021_TOUCH_REPORT_PACKET); i++)
        {
            touchReport.packet[i] = _GetByte();
        }                  

        /* check to ensure that there is a startBit */
        if(touchReport.startBit)
        {
            /* check for pen up*/
            if(touchReport.pen)
            {
                /* get x coordinate */
                xc = (long)touchReport.highX;
                xc <<= 7;
                xc |= (long)touchReport.lowX;
                xc *= DISP_HOR_RESOLUTION;
                xc >>= 11;
                xc++;
                xc >>= 1;

                /* get y coordinate */
                yc = (long)touchReport.highY;
                yc <<= 7;
                yc |= (long)touchReport.lowY;
                yc *= DISP_VER_RESOLUTION;
                yc >>= 11;
                yc++;
                yc >>= 1;

                xcor = xc;
                ycor = yc;
                xypen = DRV_TOUCH_PEN_DOWN;
                touchStatus = DRV_TOUCH_POSITION_SINGLE;
            } else 
            {
                /* set pen up state - (note if may be necessary to set 
                   coordinate to a valid position
                 */
                xcor = -1;
                ycor = -1;
                xypen = DRV_TOUCH_PEN_UP;
                touchStatus = DRV_TOUCH_POSITION_SINGLE;
            }
        }

        BSP_TouchScreenChipOn();
    }
                  
    return ;    
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_Calibrate ( const DRV_TOUCH_AR1021_CALIBRATION_PROMPT_CALLBACK * prompt )

  Summary:
    Calibrate the touch screen
*/

void DRV_TOUCH_AR1021_Calibrate ( const DRV_TOUCH_AR1021_CALIBRATION_PROMPT_CALLBACK * prompt )
{
    
    /* disable normal touch reporting during calibration */
    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_DISABLE_TOUCH, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);          //Disable Touch Messages
    _AR1021_TMR_DelayMS(100);     

#if defined (__PIC32MZ__)
    /* AR 1011/1021 controllers recover defaults 0xFF to 0x01 and 0xFF to 0x29 */
    DRV_TOUCH_AR1021_RegisterConfigWrite(0x01, 0xff);
    DRV_TOUCH_AR1021_RegisterConfigWrite(0x29,0xff);
#else
    /* AR 1010/1020 controllers recover defaults 0xFF to 0x00 */
    DRV_TOUCH_AR1021_RegisterConfigWrite(0x00, 0xff);
#endif
    
    uint8_t cid2[4];
    /* set calibration type to four points */
    cid2[0] = 0x04; //Calibration Type
    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_CALIBRATE_MODE, cid2, 1) != TOUCH_AR1021_RESP_SUCCESS);          //Enable Touch Messages
    _AR1021_TMR_DelayMS(100); 
    
    /* allow user to display application defined first prompt information */
    prompt->firstPromptCallback();
    
    /* wait for user to press and release the first point */
    BSP_TouchScreenChipOff();
    while(_TouchAR1021GetResponceStatus(TOUCH_AR1021_CMD_CALIBRATE_MODE, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    BSP_TouchScreenChipOn();
    
    /* allow user to display application defined second prompt information */
    prompt->secondPromptCallback();

    /* wait for user to press and release the second point */
    BSP_TouchScreenChipOff();
    while(_TouchAR1021GetResponceStatus(TOUCH_AR1021_CMD_CALIBRATE_MODE, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    BSP_TouchScreenChipOn();
    
    /* allow user to display application defined third prompt information */
    prompt->thirdPromptCallback();
    
    /* wait for user to press and release the third point */
    BSP_TouchScreenChipOff();
    while(_TouchAR1021GetResponceStatus(TOUCH_AR1021_CMD_CALIBRATE_MODE, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    BSP_TouchScreenChipOn();
    
    /* allow user to display application defined fourth prompt information */
    prompt->fourthPromptCallback();
    
    /* wait for user to press and release the fourth point */
    BSP_TouchScreenChipOff();
    while(_TouchAR1021GetResponceStatus(TOUCH_AR1021_CMD_CALIBRATE_MODE, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    BSP_TouchScreenChipOn();
 
#if defined (__PIC32MZ__)
    /* AR 1011/1021 controllers wait for controller to write calibration data */
    BSP_TouchScreenChipOff();
    while(_TouchAR1021GetResponceStatus(TOUCH_AR1021_CMD_CALIBRATE_MODE, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    BSP_TouchScreenChipOn();
#else
    /* AR 1010/1020 controllers wait for 1s */
    _AR1021_TMR_DelayMS(10000);   
#endif
    
        
    /* allow user to display application defined fourth prompt information */
    prompt->completeCallback();
    
    /* enable touch reporting */
    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_ENABLE_TOUCH, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);          //Enable Touch Messages
    _AR1021_TMR_DelayMS(100);  
    
    detectPosition = 1;

    return ;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_FactoryDefaultSet ( void )

  Summary:
    Set AR1021 controller to factory default configuration.
*/
void DRV_TOUCH_AR1021_FactoryDefaultSet(void)
{
    uint16_t startAddress;

    startAddress = 0x01;

    regPacket[0] = (uint8_t)(startAddress >> 8);
    regPacket[1] = (uint8_t)(startAddress);
    regPacket[2] = 1;
    regPacket[3] = 0xFF;

    if(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_EEPROM_WRITE , regPacket, 4) == TOUCH_AR1021_RESP_SUCCESS)
    {
        startAddress = 0x29;

        regPacket[0] = (uint8_t)(startAddress >> 8);
        regPacket[1] = (uint8_t)(startAddress);
        regPacket[2] = 1;
        regPacket[3] = 0xFF;

        if(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_EEPROM_WRITE , regPacket, 4) == TOUCH_AR1021_RESP_SUCCESS)
        {
           ;
        }
    }

}

void DRV_TOUCH_AR1021_RegisterConfigWrite(uint16_t regOffset, uint8_t Value)
{
    uint8_t regPacket[12];
    uint16_t startAddress;

    startAddress = _TouchAR1021GetRegisterStartAddress();

    startAddress += regOffset;

    regPacket[0] = (uint8_t)(startAddress >> 8);
    regPacket[1] = (uint8_t)(startAddress);
    regPacket[2] = 1;
    regPacket[3] = Value;

    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_REG_WRITE, regPacket, 4) != TOUCH_AR1021_RESP_SUCCESS);
}


/*********************************************************************
* Function: void TouchInit(void)
* PreCondition: none
* Input: none
* Output: none
* Side Effects: none
* Overview: sets ADC 
* Note: none
********************************************************************/
DRV_HANDLE _TouchHardwareInit(void *initValues)
{
    /* Host object */
    DRV_TOUCH_AR1021_CLIENT_OBJECT *obj = (DRV_TOUCH_AR1021_CLIENT_OBJECT *)&sAR1021ClientInstances[0];

    /* open SPI */
    if ( DRV_SPI_Open ( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE ) == DRV_HANDLE_INVALID )
    {
        return DRV_HANDLE_INVALID;
    }
            
    /* Set IOs directions for AR1021 SPI */
    BSP_TouchScreenChipOn();
    
    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_DISABLE_TOUCH, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);
    _AR1021_TMR_DelayMS(50);   
    
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_TOUCH_THRESHOLD , DRV_TOUCH_AR1021_REG_TOUCH_THRESHOLD);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_SENSITIVITY_FILTER, DRV_TOUCH_AR1021_REG_SENSITIVITY_FILTER);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_SAMPLING_FAST, DRV_TOUCH_AR1021_REG_SAMPLING_FAST);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_SAMPLING_SLOW, DRV_TOUCH_AR1021_REG_SAMPLING_SLOW);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_ACCURACY_FILTER_FAST, DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_FAST);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_ACCURACY_FILTER_SLOW, DRV_TOUCH_AR1021_REG_ACCURACY_FILTER_SLOW);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_SPEED_THRESHOLD, DRV_TOUCH_AR1021_REG_SPEED_THRESHOLD);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_SLEEP_DELAY, DRV_TOUCH_AR1021_REG_SLEEP_DELAY);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_PEN_UP_DELAY, DRV_TOUCH_AR1021_REG_PEN_UP_DELAY);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_TOUCH_MODE, DRV_TOUCH_AR1021_REG_TOUCH_MODE);
//    _TouchAR1021_REGConfigWrite(TOUCH_AR1021_REG_TOUCH_OPTIONS, DRV_TOUCH_AR1021_REG_TOUCH_OPTIONS);
//    _TouchAR1021_REGConfigWrite(TOUCH_AR1021_REG_CALIBRATION_INSET, 0x19);
    DRV_TOUCH_AR1021_RegisterConfigWrite(DRV_AR1021_REG_PEN_STATE_REPORT_DELAY, DRV_TOUCH_AR1021_REG_PEN_STATE_REPORT_DELAY);

    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_ENABLE_TOUCH, NULL, 0) != TOUCH_AR1021_RESP_SUCCESS);          //Enable Touch Messages

    obj->inUse = true;
    obj->task = DRV_TOUCH_AR1021_RUNNING;
    obj->status = SYS_STATUS_READY;
    detectPosition = 1;
    
    return (DRV_HANDLE)obj;
}

void _TouchAR1021SendCommand(uint8_t command, uint8_t *data, uint8_t dataSize)
{
    _SendByte(TOUCH_AR1021_CMD_HEADER);
    _SendByte(1 + dataSize);
    _SendByte(command);
    
    //    Send: <0x55><0x04><0x28><EEPROM AddressHigh byte><EEPROM Address Lowbyte><# of EEPROM to Read>
//Register Address High byte: 0x00 # of Registers to Read: 0x01 thru 0x08
    
    while(dataSize)
    {
        _SendByte(*data);
        //*data++;
        data++;
        dataSize--;
    }
}

uint8_t TouchAR1021GetResponceData(uint8_t command, uint8_t cmdDataSize, uint8_t *data)
{
    uint8_t responce;

    /* due to a not interrupt driver - we wait a fudge time for bytes to arrive*/
    _AR1021_TMR_DelayMS(10); 
    responce = _GetByte();
    if(responce != TOUCH_AR1021_CMD_HEADER)
        return TOUCH_AR1021_RESP_INVALID;

    cmdDataSize = _GetByte();

    if(!cmdDataSize)
        return TOUCH_AR1021_RESP_INVALID;

    responce = _GetByte();

    if(_GetByte() != command)
        return TOUCH_AR1021_RESP_INVALID;

    cmdDataSize -= 2;

    while(cmdDataSize)
    {
        if(data != NULL)
        {
            *data = _GetByte();
            data++;
        }else
        {
            _GetByte();
        }

        cmdDataSize--;
    }

    return responce;
}

uint8_t _TouchAR1021GetResponceStatus(uint8_t command, uint8_t * data, uint8_t dataSize)
{
    return TouchAR1021GetResponceData(command, dataSize, data);
}

uint8_t _TouchAR1021SendCommandAndGetResponse(uint8_t command, uint8_t *data, uint8_t dataSize)
{
    uint8_t responce;

    BSP_TouchScreenChipOff();
    
    _TouchAR1021SendCommand(command, data, dataSize);

    responce = _TouchAR1021GetResponceStatus(command, data, dataSize);
    
    BSP_TouchScreenChipOn();

    return responce;

}

void _TouchAR1021_REGConfigWrite(uint16_t regOffset, uint8_t Value)
{
    uint8_t regPacket[12];
    uint16_t startAddress;

    startAddress = _TouchAR1021GetRegisterStartAddress();

    startAddress += regOffset;

    regPacket[0] = (uint8_t)(startAddress >> 8);
    regPacket[1] = (uint8_t)(startAddress);
    regPacket[2] = 1;
    regPacket[3] = Value;

    while(_TouchAR1021SendCommandAndGetResponse(TOUCH_AR1021_CMD_REG_WRITE, regPacket, 4) != TOUCH_AR1021_RESP_SUCCESS);
}

void _SendByte(uint8_t data)
{
    PLIB_SPI_BufferWrite(spiModule, data);
    _AR1021_TMR_DelayMS(1);       //Asked for in AR1021 Spec
    PLIB_SPI_BufferRead(spiModule);
}

uint8_t _GetByte(void)
{
    uint8_t test;
    PLIB_SPI_BufferWrite(spiModule, 0);
    _AR1021_TMR_DelayMS(1);
    test =PLIB_SPI_BufferRead(spiModule);
    return test;
}

void _AR1021_TMR_DelayMS ( unsigned int delayMs )
{
    if(delayMs)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk/20000)*delayMs);
    }
}

uint16_t _TouchAR1021GetRegisterStartAddress(void)
{
    uint8_t responce;
    uint8_t startAddress = 0;
    uint8_t size = 0;

    BSP_TouchScreenChipOff();

    _TouchAR1021SendCommand(TOUCH_AR1021_CMD_REG_START_ADDR, NULL, 0);

    responce = TouchAR1021GetResponceData(TOUCH_AR1021_CMD_REG_START_ADDR, size, &startAddress);

    BSP_TouchScreenChipOn();

    if(responce != TOUCH_AR1021_RESP_SUCCESS)
        return 0xFFFF;

    return (uint16_t)startAddress;
        
}


/*********************************************************************
* Function: void TouchStoreCalibration(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: stores calibration parameters into EEPROM
*
* Note: none
*
********************************************************************/
void TouchStoreCalibration(void)
{
    // not implemented
}

/*********************************************************************
* Function: void TouchLoadCalibration(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: loads calibration parameters from EEPROM
*
* Note: none
*
********************************************************************/
void TouchLoadCalibration(void)
{
    // not implemented
}