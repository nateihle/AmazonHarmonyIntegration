/*******************************************************************************
 Touch ADC Driver Interface File

  File Name:
    drv_touch_adc_static.c

  Summary:
    Touch ADC Driver interface file.

  Description:
    This is a simple 4-wire resistive touch screen driver. The file consist of
    touch controller ADC driver interfaces. It implements the driver interfaces
    which read the touch input data from display overlay through the ADC peripheral.

    Note: This driver is based on the MPLAB Harmony ADC driver.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 released Microchip Technology Inc.  All rights reserved.

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
#include "system_definitions.h"
#include "driver/touch/touch_adc/drv_touch_adc.h"

#ifdef ENABLE_DEBUG_TOUCHSCREEN
void TouchScreenResistiveTestXY(void);
#endif

//////////////////////// GLOBAL VARIABLES ////////////////////////////

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:
    use this scale factor to avoid working in floating point numbers

*/
#define SCALE_FACTOR (1<<8)

#if (DISP_ORIENTATION == 90)
    #define ADC_MaxXGet() (272)
#elif (DISP_ORIENTATION == 270)
    #define ADC_MaxXGet() (272)
#elif (DISP_ORIENTATION == 0)
    #define ADC_MaxXGet() (480)
#elif (DISP_ORIENTATION == 180)
    #define ADC_MaxXGet() (480)
#endif
#if (DISP_ORIENTATION == 90)
    #define ADC_MaxYGet() (480)
#elif (DISP_ORIENTATION == 270)
    #define ADC_MaxYGet() (480)
#elif (DISP_ORIENTATION == 0)
    #define ADC_MaxYGet() (272)
#elif (DISP_ORIENTATION == 180)
    #define ADC_MaxYGet() (272)
#endif

#define DRV_TOUCH_ADC10BIT_MUX_TYPE          ADC_MUX_A

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
#define CAL_X_INSET    (((ADC_MaxXGet())*(20>>1))/100)

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
#define CAL_Y_INSET    (((ADC_MaxYGet())*(20>>1))/100)

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/

#define DRV_TOUCH_ADC_SAMPLE_POINTS   4

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
typedef enum
{
    /* */
    DRV_TOUCH_ADC_STATE_IDLE,

    /* */
    DRV_TOUCH_ADC_STATE_SET_X,

    /* */
    DRV_TOUCH_ADC_STATE_RUN_X,

    /* */
    DRV_TOUCH_ADC_STATE_GET_X,

    /* */
    DRV_TOUCH_ADC_STATE_RUN_CHECK_X,

    /* */
    DRV_TOUCH_ADC_STATE_CHECK_X,

    /* */
    DRV_TOUCH_ADC_STATE_SET_Y,

    /* */
    DRV_TOUCH_ADC_STATE_RUN_Y,

    /* */
    DRV_TOUCH_ADC_STATE_GET_Y,

    /* */
    DRV_TOUCH_ADC_STATE_CHECK_Y,

    /* */
    DRV_TOUCH_ADC_STATE_SET_VALUES,

    /* */
    DRV_TOUCH_ADC_STATE_GET_POT,

    /* */
    DRV_TOUCH_ADC_STATE_RUN_POT

} DRV_TOUCH_ADC_TOUCH_STATES;

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
typedef struct
{
    /* */
    SYS_STATUS       status;

    /* */
    SYS_MODULE_INDEX touchId;

    /* */
    bool             isExclusive;

    /* */
    uint8_t          numClients;

    /* */
    uint16_t	     orientation;

    /* */
    uint16_t         horizontalResolution;

    /* */
    uint16_t         verticalResolution;

    DRV_TOUCH_ADC_TOUCH_STATES state;

    /* Touch status */
    DRV_TOUCH_POSITION_STATUS       touchStatus;

} DRV_TOUCH_ADC_OBJECT;

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/

typedef struct
{
    /* Driver Object associated with the client */
    DRV_TOUCH_ADC_OBJECT * driverObject;

    /* The intent with which the client was opened */
    DRV_IO_INTENT         intent;

} DRV_TOUCH_ADC_CLIENT_OBJECT;

//******************************************************************************
// Local functions
//******************************************************************************
void  _DRV_TOUCH_ADC_HardwareInit       (void *initValues);
void  _DRV_TOUCH_ADC_CalculateCalPoints (void);


//#define CALIBRATION_DELAY   300				                // delay between calibration touch points

//#define RESISTIVETOUCH_MANUAL_SAMPLE_MODE

//#define TOUCHSCREEN_RESISTIVE_FLIP_Y

// Current ADC values for X and Y channels
volatile short adcX = -1;
volatile short adcY = -1;
volatile short adcPot = 0;

// coefficient values
volatile long _trA;
volatile long _trB;
volatile long _trC;
volatile long _trD;

// copy of the stored or sampled raw points (this is the calibration data stored)
/*      xRawTouch[0] - x sample from upper left corner; 
        xRawTouch[1] - x sample from upper right corner
        xRawTouch[2] - x sample from lower right corner
        xRawTouch[3] - x sample from lower left corner
        yRawTouch[0] - y sample from upper left corner; 
        yRawTouch[1] - y sample from upper right corner
        yRawTouch[2] - y sample from lower right corner
        yRawTouch[3] - y sample from lower left corner
 */

/* Driver instance object */
static DRV_TOUCH_ADC_OBJECT        sADCDriverInstances[1];

static DRV_TOUCH_ADC_CLIENT_OBJECT sADCClientInstances[1];

volatile short xRawTouch[DRV_TOUCH_ADC_SAMPLE_POINTS] = { 0x0099,
                                                         0x0369,
                                                         0x0368,
                                                         0x00AC };

volatile short yRawTouch[DRV_TOUCH_ADC_SAMPLE_POINTS] = { 0x00FF,
                                                         0x00F1,
                                                         0x033C,
                                                         0x032A };

// *****************************************************************************
// Section: ADC10Bit Driver Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_ADC_Initialize ( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes and ADC10BIT psuedo controller.

  Description:
    This function initializes the ADC10BIT psuedo controller.
*/

SYS_MODULE_OBJ DRV_TOUCH_ADC_Initialize( const SYS_MODULE_INDEX index,
                                              const SYS_MODULE_INIT * const init )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) NULL;
    DRV_TOUCH_SAMPLE_POINTS samplePoints;

    if (index >= 1)
    {
        SYS_ASSERT(false, "ADC10BIT Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }

    pDrvInstance = ( DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];

    const DRV_TOUCH_INIT * const pInit = (const DRV_TOUCH_INIT * const)init;

    pDrvInstance->status               = SYS_STATUS_READY;
    pDrvInstance->state                = DRV_TOUCH_ADC_STATE_IDLE;
    pDrvInstance->touchId              = pInit->touchId;
    pDrvInstance->verticalResolution   = pInit->verticalResolution;
    pDrvInstance->horizontalResolution = pInit->horizontalResolution;
    pDrvInstance->touchStatus          = DRV_TOUCH_POSITION_NONE;

    _DRV_TOUCH_ADC_HardwareInit(0);
    
    // set the state of the statemachine to start the sampling
    pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
    
    samplePoints.touchCalUlx = 0x0099;
    samplePoints.touchCalUly = 0x00FF;

    samplePoints.touchCalUrx = 0x0369;
    samplePoints.touchCalUry = 0x00F1;

    samplePoints.touchCalLrx = 0x0368;
    samplePoints.touchCalLry = 0x033C;

    samplePoints.touchCalLlx = 0x00AC;
    samplePoints.touchCalLly = 0x032A;

    DRV_TOUCH_ADC_CalibrationSet(&samplePoints);

    return (SYS_MODULE_OBJ)pDrvInstance;
}

void DRV_TOUCH_ADC_Deinitialize ( SYS_MODULE_OBJ object )
{
//    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) NULL;
//    pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) &sADCDriverInstances[0];
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_ADC_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine detects a touch press position.
 */
void DRV_TOUCH_ADC_Tasks ( SYS_MODULE_OBJ object )
{
    static short    prevX = -1;
    static short    prevY = -1;

    short           xpos, ypos;

    xpos = DRV_TOUCH_ADC_TouchGetX(0);
    ypos = DRV_TOUCH_ADC_TouchGetY(0);

    if((xpos == -1) || (ypos == -1))
    {
        ypos = -1;
        xpos = -1;
    }

    if((prevX == xpos) && (prevY == ypos) && (xpos != -1) && (ypos != -1))
    {
        return;
    }

    if((prevX != -1) || (prevY != -1))
    {
        if((xpos != -1) && (ypos != -1))
        {

            // Move
            SYS_INP_InjectTouchMove(0, xpos, ypos);
        }
        else
        {

            // Released
            SYS_INP_InjectTouchUp(0, prevX, prevY);
            prevX = xpos;
            prevY = ypos;
            return;
        }
    }
    else
    {
        if((xpos != -1) && (ypos != -1))
        {

            // Pressed
            SYS_INP_InjectTouchDown(0, xpos, ypos);
        }
        else
        {

            // No message
        }
    }

    prevX = xpos;
    prevY = ypos;
}

/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_ADC_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the ADC10BIT driver module.

  Description:
    This function provides the current status of the ADC10BIT driver module.
*/

SYS_STATUS DRV_TOUCH_ADC_Status ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)object;
    return pDrvInstance->status;
}

// *****************************************************************************
// *****************************************************************************
// Section: ADC10Bit Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_ADC_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified ADC10BIT driver instance and returns a handle to it.

  Description:
    This routine opens the specified ADC10BIT driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.
 */
DRV_HANDLE DRV_TOUCH_ADC_Open ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];
    
    if (index >= 1)
    {
        SYS_ASSERT(false, "ADC10BIT Driver: Attempting to open an instance number greater than the max");
        return DRV_HANDLE_INVALID;
    }

    DRV_TOUCH_ADC_CLIENT_OBJECT * pClient = &sADCClientInstances[index];
    if (pClient == NULL)
    {
        SYS_ASSERT(false, "ADC10BIT Driver: Couldn't find a free client to open");
        return DRV_HANDLE_INVALID;
    }

    pClient->driverObject = pDrvInstance;
    pClient->intent = intent;
    
    if ((intent & DRV_IO_INTENT_EXCLUSIVE) == DRV_IO_INTENT_EXCLUSIVE)
    {
        pDrvInstance->isExclusive = true;
    }
    pDrvInstance->numClients++;

     return (DRV_HANDLE)pClient;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_ADC_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the ADC10BIT driver

  Description:
    This function closes an opened instance of the ADC10BIT driver, invalidating
    the handle.
 */
void DRV_TOUCH_ADC_Close ( DRV_HANDLE handle )
{
    DRV_TOUCH_ADC_CLIENT_OBJECT * pClient = (DRV_TOUCH_ADC_CLIENT_OBJECT *)handle;

    if ( pClient != &sADCClientInstances[0] )
    {
         SYS_ASSERT(false, "ADC10BIT Driver: Trying to close a client to a driver that is outside the range of client handles")
         return;
    }

    return;
}

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

short DRV_TOUCH_ADC_PositionDetect(void)
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) &sADCDriverInstances[0];
    static short tempX;
    static short tempY;
    short temp;

    switch ( pDrvInstance->state ) 
    {
        case DRV_TOUCH_ADC_STATE_IDLE:
        {
            adcX = -1;
            adcY = -1;
            
            break;
        }
        
        case DRV_TOUCH_ADC_STATE_SET_VALUES:
        {            
            if ( !PLIB_ADC_ConversionHasCompleted(ADC_ID_1) )
            {
                break; 
            }
            if ( (uint16_t) 256 <
                (uint16_t) PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0) )
            {
                adcX = -1;
                adcY = -1;
            } 
            else 
            {
                adcX = tempX;
                adcY = tempY;
            }
            
            // If the hardware supports an analog pot, if not skip it
            pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
            pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
            return 1; // touch screen acquisition is done
            
        }
        
        case DRV_TOUCH_ADC_STATE_SET_X:
        {
            //x+
            PLIB_ADC_MuxChannel0InputPositiveSelect( ADC_ID_1,
                                         DRV_TOUCH_ADC10BIT_MUX_TYPE,
                                         BSP_ADC_TOUCH_XPLUS_PIN );

            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_XPLUS_PORT, 
                                       BSP_ADC_TOUCH_XPLUS_PIN_MASK );
            //y+
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_YPLUS_PORT, 
                                       BSP_ADC_TOUCH_YPLUS_PIN_MASK );
            //x-
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_XMINUS_PORT, 
                                       BSP_ADC_TOUCH_XMINUS_PIN_MASK );
            //y-
            SYS_PORTS_PinClear( PORTS_ID_0, BSP_ADC_TOUCH_YMINUS_PORT, 
                                BSP_ADC_TOUCH_YMINUS_PIN );
            
            //y-
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, 
                                       BSP_ADC_TOUCH_YMINUS_PORT, 
                                       BSP_ADC_TOUCH_YMINUS_PIN_MASK);

            PLIB_ADC_SamplingStart( ADC_ID_1 );

            pDrvInstance->state = DRV_TOUCH_ADC_STATE_CHECK_X;
            
            break;
        }

        case DRV_TOUCH_ADC_STATE_CHECK_X:
        case DRV_TOUCH_ADC_STATE_CHECK_Y:
        {
            if ( !PLIB_ADC_ConversionHasCompleted(ADC_ID_1) )
            {
                break;
            }

            if ((uint16_t) 256 >
                (uint16_t) PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0))
            {
                if (pDrvInstance->state == DRV_TOUCH_ADC_STATE_CHECK_X)
                {
                    //y+
                    SYS_PORTS_PinSet( PORTS_ID_0,
                                      BSP_ADC_TOUCH_YPLUS_PORT,
                                      BSP_ADC_TOUCH_YPLUS_PIN );

                    SYS_PORTS_DirectionSelect( PORTS_ID_0,
                                               SYS_PORTS_DIRECTION_OUTPUT,
                                               BSP_ADC_TOUCH_YPLUS_PORT,
                                               BSP_ADC_TOUCH_YPLUS_PIN_MASK);

                    tempX = -1;
                    pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_X;
                }
                else
                {
                    //x+
                    SYS_PORTS_PinSet( PORTS_ID_0,
                                      BSP_ADC_TOUCH_XPLUS_PORT,
                                      BSP_ADC_TOUCH_XPLUS_PIN );

                    SYS_PORTS_DirectionSelect( PORTS_ID_0,
                                               SYS_PORTS_DIRECTION_OUTPUT,
                                               BSP_ADC_TOUCH_XPLUS_PORT,
                                               BSP_ADC_TOUCH_XPLUS_PIN_MASK );

                    tempY = -1;
                    pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_Y;
                }
            }
            else
            {
                adcX = -1;
                adcY = -1;

                pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
                pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
                return 1; // touch screen acquisition is done

                break;
            }
        }

        case DRV_TOUCH_ADC_STATE_RUN_X:
        case DRV_TOUCH_ADC_STATE_RUN_Y:
        {
            PLIB_ADC_SamplingStart( ADC_ID_1 );

            pDrvInstance->state = (pDrvInstance->state == DRV_TOUCH_ADC_STATE_RUN_X) ?
                DRV_TOUCH_ADC_STATE_GET_X : DRV_TOUCH_ADC_STATE_GET_Y;

        }

        case DRV_TOUCH_ADC_STATE_GET_X:
        case DRV_TOUCH_ADC_STATE_GET_Y:
        {
            if (!PLIB_ADC_ConversionHasCompleted(ADC_ID_1))
            {
                break;
            }
            temp = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0);
            if (pDrvInstance->state == DRV_TOUCH_ADC_STATE_GET_X)
            {
                if (temp != tempX)
                {
                    tempX = temp;
                    pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_X;
                    break;
                }
            }
            else
            {
                if (temp != tempY)
                {
                    tempY = temp;
                    pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_Y;
                    break;
                }
            }
            if (pDrvInstance->state == DRV_TOUCH_ADC_STATE_GET_X)
            {
                //y+
                SYS_PORTS_DirectionSelect( PORTS_ID_0,
                                           SYS_PORTS_DIRECTION_INPUT,
                                           BSP_ADC_TOUCH_YPLUS_PORT,
                                           BSP_ADC_TOUCH_YPLUS_PIN_MASK );
            }
            else
            {
                //x+
                SYS_PORTS_DirectionSelect( PORTS_ID_0,
                                           SYS_PORTS_DIRECTION_INPUT,
                                           BSP_ADC_TOUCH_XPLUS_PORT,
                                           BSP_ADC_TOUCH_XPLUS_PIN_MASK );
            }

            PLIB_ADC_SamplingStart(ADC_ID_1);

            pDrvInstance->state = ( pDrvInstance->state == DRV_TOUCH_ADC_STATE_GET_X ) ?
                DRV_TOUCH_ADC_STATE_SET_Y : DRV_TOUCH_ADC_STATE_SET_VALUES;
            break;
        }

        case DRV_TOUCH_ADC_STATE_SET_Y:
        {
            if (!PLIB_ADC_ConversionHasCompleted(ADC_ID_1))
            {
                break;
            }
                   
            if ( (uint16_t) 256 <
                PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0) )
            {
                adcX = -1;
                adcY = -1;
                pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
                pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
                return 1; // touch screen acquisition is done
                break;
            }

            //Y+
            PLIB_ADC_MuxChannel0InputPositiveSelect( ADC_ID_1,
                                         DRV_TOUCH_ADC10BIT_MUX_TYPE,
                                         BSP_ADC_TOUCH_YPLUS_PIN );

            //x+
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_XPLUS_PORT, 
                                       BSP_ADC_TOUCH_XPLUS_PIN_MASK );
            //y+
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_YPLUS_PORT, 
                                       BSP_ADC_TOUCH_YPLUS_PIN_MASK );
            //x-
            SYS_PORTS_PinClear( PORTS_ID_0, BSP_ADC_TOUCH_XMINUS_PORT, 
                                BSP_ADC_TOUCH_XMINUS_PIN );
            
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, 
                                       BSP_ADC_TOUCH_XMINUS_PORT, 
                                       BSP_ADC_TOUCH_XMINUS_PIN_MASK );
            
            //y-
            SYS_PORTS_DirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_YMINUS_PORT, 
                                       BSP_ADC_TOUCH_YMINUS_PIN_MASK );

            PLIB_ADC_SamplingStart(ADC_ID_1);

            pDrvInstance->state = DRV_TOUCH_ADC_STATE_CHECK_Y;
            break;
        }
        
        default:
        {
            pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
            pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
            return 1; // touch screen acquisition is done
        }
    }

    return 0; // touch screen acquisition is not done
}

/*********************************************************************
 * Function: short DRV_TOUCH_ADC_TouchGetX( uint8_t touchNumber )
 *
 * PreCondition: none
 *
 * Input: none
 *
 * Output: x coordinate
 *
 * Side Effects: none
 *
 * Overview: returns x coordinate if touch screen is pressed
 *           and -1 if not
 *
 * Note: none
 *
 ********************************************************************/
short DRV_TOUCH_ADC_TouchGetX( uint8_t touchNumber )
{
    long result;


    result = DRV_TOUCH_ADC_TouchGetRawX();

    if (result >= 0) {
        result = (long) ((((long) _trC * result) + _trD) >> 8);

#ifdef TOUCHSCREEN_RESISTIVE_FLIP_X
        DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) NULL;
        pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) &sADCDriverInstances[0];

        result =  pDrvInstance->horizontalResolution - result;
#endif	
    }
    return ((short) result);
}


/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_ADC_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_ADC_TouchStatus( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];
    return (pDrvInstance->touchStatus);
}


/*********************************************************************
  Function:
    void DRV_TOUCH_ADC_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_ADC_TouchDataRead( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];
    pDrvInstance->touchStatus = DRV_TOUCH_POSITION_NONE;
}


/*********************************************************************
 * Function: short DRV_TOUCH_ADC_TouchGetRawX()
 *
 * PreCondition: none
 *
 * Input: none
 *
 * Output: x coordinate
 *
 * Side Effects: none
 *
 * Overview: returns x coordinate if touch screen is pressed
 *           and -1 if not
 *
 * Note: none
 *
 ********************************************************************/
short DRV_TOUCH_ADC_TouchGetRawX(void)
{
    return adcX;
}

/*********************************************************************
 * Function: short DRV_TOUCH_ADC_TouchGetY( uint8_t touchNumber )
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
short DRV_TOUCH_ADC_TouchGetY( uint8_t touchNumber )
{
    long result;

    result = DRV_TOUCH_ADC_TouchGetRawY();

    if (result >= 0) {
        result = (long) ((((long) _trA * result) + (long) _trB) >> 8);

#ifdef TOUCHSCREEN_RESISTIVE_FLIP_Y
        DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) NULL;
        pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) &sADCDriverInstances[0];
        result =  pDrvInstance->verticalResolution - result;
#endif	
    }
    return ((short) result);
}

/*********************************************************************
 * Function: short DRV_TOUCH_ADC_TouchGetRawY()
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
short DRV_TOUCH_ADC_TouchGetRawY(void)
{
    return adcY;
}

/*********************************************************************
 * Function: void DRV_TOUCH_ADC_TouchStoreCalibration(void)
 *
 * PreCondition: Non-volatile memory initialization function must be called before
 *
 * Input: none
 *
 * Output: none
 *
 * Side Effects: none
 *
 * Overview: stores calibration parameters into non-volatile memory
 *
 * Note: none
 *
 ********************************************************************/
void DRV_TOUCH_ADC_TouchStoreCalibration(void)
{

// Deprecated API
//
//    if (pCalDataWrite != NULL)
//    {
//        // the upper left X sample address is used since it is the first one
//        // and this assumes that all stored values are located in one
//        // sector
//        if (pCalDataSectorErase != NULL)
//        {
//            pCalDataSectorErase(ADDRESS_RESISTIVE_TOUCH_ULX);
//        }
//
//        pCalDataWrite(xRawTouch[0], ADDRESS_RESISTIVE_TOUCH_ULX);
//        pCalDataWrite(yRawTouch[0], ADDRESS_RESISTIVE_TOUCH_ULY);
//
//        pCalDataWrite(xRawTouch[1], ADDRESS_RESISTIVE_TOUCH_URX);
//        pCalDataWrite(yRawTouch[1], ADDRESS_RESISTIVE_TOUCH_URY);
//
//        pCalDataWrite(xRawTouch[3], ADDRESS_RESISTIVE_TOUCH_LLX);
//        pCalDataWrite(yRawTouch[3], ADDRESS_RESISTIVE_TOUCH_LLY);
//
//        pCalDataWrite(xRawTouch[2], ADDRESS_RESISTIVE_TOUCH_LRX);
//        pCalDataWrite(yRawTouch[2], ADDRESS_RESISTIVE_TOUCH_LRY);
//
//        pCalDataWrite(mchpTouchScreenVersion, ADDRESS_RESISTIVE_TOUCH_VERSION);
//
//    }

}

/*********************************************************************
 * Function: void DRV_TOUCH_ADC_CalibrationSet(void)
 *
 * PreCondition: Non-volatile memory initialization function must be called before
 *
 * Input: none
 *
 * Output: none
 *
 * Side Effects: none
 *
 * Overview: loads calibration parameters from non-volatile memory
 *
 * Note: none
 *
 ********************************************************************/
void DRV_TOUCH_ADC_CalibrationSet(DRV_TOUCH_SAMPLE_POINTS * samplePoints)
{

    xRawTouch[0] = samplePoints->touchCalUlx;
    yRawTouch[0] = samplePoints->touchCalUly;

    xRawTouch[1] = samplePoints->touchCalUrx;
    yRawTouch[1] = samplePoints->touchCalUry;

    xRawTouch[3] = samplePoints->touchCalLlx;
    yRawTouch[3] = samplePoints->touchCalLly;

    xRawTouch[2] = samplePoints->touchCalLrx;
    yRawTouch[2] = samplePoints->touchCalLry;

    _DRV_TOUCH_ADC_CalculateCalPoints();

}

// *****************************************************************************
// *****************************************************************************
// Section: Local functinos
// *****************************************************************************
// *****************************************************************************

/*********************************************************************
 * Function: void _TouchHardwareInit(void)
 *
 * PreCondition: none
 *
 * Input: none
 *
 * Output: none
 *
 * Side Effects: none
 *
 * Overview: Initializes touch screen module.
 *
 * Note: none
 *
 ********************************************************************/
void _DRV_TOUCH_ADC_HardwareInit(void *initValues)
{
    PLIB_ADC_Disable(ADC_ID_1);
    PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS);
    PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA);
    PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_1SAMPLE_PER_INTERRUPT);
    PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 31);
    PLIB_ADC_ConversionClockSet(ADC_ID_1, 80000000, 156250 );
    PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
    PLIB_ADC_Enable(ADC_ID_1);

    PLIB_ADC_MuxAInputScanDisable(ADC_ID_1);

}

/*********************************************************************
 * Function: void TouchGetCalPoints(void)
 *
 * PreCondition: InitGraph() must be called before
 *
 * Input: none
 *
 * Output: none
 *
 * Side Effects: none
 *
 * Overview: gets values for 3 touches
 *
 * Note: none
 *
 ********************************************************************/
void _DRV_TOUCH_ADC_CalculateCalPoints(void)
{
    long trA, trB, trC, trD; // variables for the coefficients
    long trAhold, trBhold, trChold, trDhold;
    long test1, test2; // temp variables (must be signed type)

    short xPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];
    short yPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];

    yPoint[0] = yPoint[1] = CAL_Y_INSET;
    yPoint[2] = yPoint[3] = (ADC_MaxYGet() - CAL_Y_INSET);
    xPoint[0] = xPoint[3] = CAL_X_INSET;
    xPoint[1] = xPoint[2] = (ADC_MaxXGet() - CAL_X_INSET);

    // calculate points transfer functions
    // based on two simultaneous equations solve for the
    // constants

    // use sample points 1 and 4
    // Dy1 = aTy1 + b; Dy4 = aTy4 + b
    // Dx1 = cTx1 + d; Dy4 = aTy4 + b

    test1 = (long) yPoint[0] - (long) yPoint[3];
    test2 = (long) yRawTouch[0] - (long) yRawTouch[3];

    trA = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[0] * SCALE_FACTOR) - (trA * (long) yRawTouch[0]));

    test1 = (long) xPoint[0] - (long) xPoint[2];
    test2 = (long) xRawTouch[0] - (long) xRawTouch[2];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[0] * SCALE_FACTOR) - (trC * (long) xRawTouch[0]));

    trAhold = trA;
    trBhold = trB;
    trChold = trC;
    trDhold = trD;

    // use sample points 2 and 3
    // Dy2 = aTy2 + b; Dy3 = aTy3 + b
    // Dx2 = cTx2 + d; Dy3 = aTy3 + b

    test1 = (long) yPoint[1] - (long) yPoint[2];
    test2 = (long) yRawTouch[1] - (long) yRawTouch[2];

    trA = ((long) (test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[1] * SCALE_FACTOR) - (trA * (long) yRawTouch[1]));

    test1 = (long) xPoint[1] - (long) xPoint[3];
    test2 = (long) xRawTouch[1] - (long) xRawTouch[3];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[1] * SCALE_FACTOR) - (trC * (long) xRawTouch[1]));

    // get the average and use the average
    _trA = (trA + trAhold) >> 1;
    _trB = (trB + trBhold) >> 1;
    _trC = (trC + trChold) >> 1;
    _trD = (trD + trDhold) >> 1;

}

void DRV_TOUCH_ADC_CoefficientSet(short xRawTouch[], short yRawTouch[])
{
    long trA, trB, trC, trD; // variables for the coefficients
    long trAhold, trBhold, trChold, trDhold;
    long test1, test2; // temp variables (must be signed type)

    short xPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];
    short yPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];

    yPoint[0] = yPoint[1] = CAL_Y_INSET;
    yPoint[2] = yPoint[3] = (ADC_MaxYGet() - CAL_Y_INSET);
    xPoint[0] = xPoint[3] = CAL_X_INSET;
    xPoint[1] = xPoint[2] = (ADC_MaxXGet() - CAL_X_INSET);

    // calculate points transfer functions
    // based on two simultaneous equations solve for the
    // constants

    // use sample points 1 and 4
    // Dy1 = aTy1 + b; Dy4 = aTy4 + b
    // Dx1 = cTx1 + d; Dy4 = aTy4 + b

    test1 = (long) yPoint[0] - (long) yPoint[3];
    test2 = (long) yRawTouch[0] - (long) yRawTouch[3];

    trA = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[0] * SCALE_FACTOR) - (trA * (long) yRawTouch[0]));

    test1 = (long) xPoint[0] - (long) xPoint[2];
    test2 = (long) xRawTouch[0] - (long) xRawTouch[2];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[0] * SCALE_FACTOR) - (trC * (long) xRawTouch[0]));

    trAhold = trA;
    trBhold = trB;
    trChold = trC;
    trDhold = trD;

    // use sample points 2 and 3
    // Dy2 = aTy2 + b; Dy3 = aTy3 + b
    // Dx2 = cTx2 + d; Dy3 = aTy3 + b

    test1 = (long) yPoint[1] - (long) yPoint[2];
    test2 = (long) yRawTouch[1] - (long) yRawTouch[2];

    trA = ((long) (test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[1] * SCALE_FACTOR) - (trA * (long) yRawTouch[1]));

    test1 = (long) xPoint[1] - (long) xPoint[3];
    test2 = (long) xRawTouch[1] - (long) xRawTouch[3];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[1] * SCALE_FACTOR) - (trC * (long) xRawTouch[1]));

    // get the average and use the average
    _trA = (trA + trAhold) >> 1;
    _trB = (trB + trBhold) >> 1;
    _trC = (trC + trChold) >> 1;
    _trD = (trD + trDhold) >> 1;

}

void __ISR(_TIMER_3_VECTOR, ipl1AUTO) IntHandlerDrvTouchAdc(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_3);
    DRV_TOUCH_ADC_PositionDetect();
}

/*******************************************************************************
 End of File
*/
