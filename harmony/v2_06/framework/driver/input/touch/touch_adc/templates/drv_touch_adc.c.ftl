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
#include "driver/input/touch_adc/drv_touch_adc.h"

//////////////////////// GLOBAL VARIABLES ////////////////////////////

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:
    use this scale factor to avoid working in floating point numbers

*/
#define SCALE_FACTOR (1<<8)

<#if CONFIG_PIC32MX>
#define DRV_TOUCH_ADC_MUX_TYPE          ADC_MUX_A
</#if>

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
#define CAL_X_INSET    (((${CONFIG_DRV_GFX_DISPLAY_WIDTH})*(${20}>>1))/100)

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
#define CAL_Y_INSET    (((${CONFIG_DRV_GFX_DISPLAY_HEIGHT})*(${20}>>1))/100)

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

// Current ADC values for X and Y channels
volatile short adcX = -1;
volatile short adcY = -1;

//   coefficient for screen to display coordinate translation
volatile long coefA;
volatile long coefB;
volatile long coefC;
volatile long coefD;

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

short xRawTouch[DRV_TOUCH_ADC_SAMPLE_POINTS] = { ${CONFIG_DRV_TOUCH_ADC_CAL_ULX_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_URX_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_LRX_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_LLX_SAMPLE} };

short yRawTouch[DRV_TOUCH_ADC_SAMPLE_POINTS] = { ${CONFIG_DRV_TOUCH_ADC_CAL_ULY_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_URY_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_LRY_SAMPLE},
                                                         ${CONFIG_DRV_TOUCH_ADC_CAL_LLY_SAMPLE} };

// *****************************************************************************
// Section: ADC Driver Interface Routines
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
    Initializes and ADC psuedo controller.

  Description:
    This function initializes the ADC psuedo controller.
*/

SYS_MODULE_OBJ DRV_TOUCH_ADC_Initialize( const SYS_MODULE_INDEX index,
                                              const SYS_MODULE_INIT * const init )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *) NULL;
    DRV_TOUCH_SAMPLE_POINTS samplePoints;

    if (index >= 1)
    {
        SYS_ASSERT(false, "ADC Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }

    pDrvInstance = ( DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];

    const DRV_TOUCH_ADC_INIT * const pInit = (const DRV_TOUCH_ADC_INIT * const)init;

    pDrvInstance->status               = SYS_STATUS_READY;
    pDrvInstance->state                = DRV_TOUCH_ADC_STATE_IDLE;
    pDrvInstance->touchId              = pInit->touchId;
    pDrvInstance->verticalResolution   = pInit->verticalResolution;
    pDrvInstance->horizontalResolution = pInit->horizontalResolution;
    pDrvInstance->touchStatus          = DRV_TOUCH_POSITION_NONE;

    _DRV_TOUCH_ADC_HardwareInit(0);
    
    // set the state of the statemachine to start the sampling
    pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
    
    samplePoints.touchCalUlx = ${CONFIG_DRV_TOUCH_ADC_CAL_ULX_SAMPLE};
    samplePoints.touchCalUly = ${CONFIG_DRV_TOUCH_ADC_CAL_ULY_SAMPLE};

    samplePoints.touchCalUrx = ${CONFIG_DRV_TOUCH_ADC_CAL_URX_SAMPLE};
    samplePoints.touchCalUry = ${CONFIG_DRV_TOUCH_ADC_CAL_URY_SAMPLE};

    samplePoints.touchCalLrx = ${CONFIG_DRV_TOUCH_ADC_CAL_LRX_SAMPLE};
    samplePoints.touchCalLry = ${CONFIG_DRV_TOUCH_ADC_CAL_LRY_SAMPLE};

    samplePoints.touchCalLlx = ${CONFIG_DRV_TOUCH_ADC_CAL_LLX_SAMPLE};
    samplePoints.touchCalLly = ${CONFIG_DRV_TOUCH_ADC_CAL_LLY_SAMPLE};

    DRV_TOUCH_ADC_CalibrationSet(&samplePoints);

    <#if CONFIG_DRV_TOUCH_ADC_TMR_INSTANCE_NUMBER = "0">
    DRV_TMR0_Start();
    <#elseif CONFIG_DRV_TOUCH_ADC_TMR_INSTANCE_NUMBER = "1">
    DRV_TMR1_Start();
    <#elseif CONFIG_DRV_TOUCH_ADC_TMR_DRIVER_INDEX = "2">
    DRV_TMR2_Start();
    <#elseif CONFIG_DRV_TOUCH_ADC_TMR_DRIVER_INDEX = "3">
    DRV_TMR3_Start();
    <#elseif CONFIG_DRV_TOUCH_ADC_TMR_DRIVER_INDEX = "4">
    DRV_TMR3_Start();
</#if>

    return (SYS_MODULE_OBJ)pDrvInstance;
}

void DRV_TOUCH_ADC_Deinitialize ( SYS_MODULE_OBJ object )
{
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
    Provides the current status of the ADC driver module.

  Description:
    This function provides the current status of the ADC driver module.
*/

SYS_STATUS DRV_TOUCH_ADC_Status ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)object;
    return pDrvInstance->status;
}

// *****************************************************************************
// *****************************************************************************
// Section: ADC Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_ADC_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified ADC driver instance and returns a handle to it.

  Description:
    This routine opens the specified ADC driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.
 */
DRV_HANDLE DRV_TOUCH_ADC_Open ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent )
{
    DRV_TOUCH_ADC_OBJECT * pDrvInstance = (DRV_TOUCH_ADC_OBJECT *)&sADCDriverInstances[index];
    
    if (index >= 1)
    {
        SYS_ASSERT(false, "ADC Driver: Attempting to open an instance number greater than the max");
        return DRV_HANDLE_INVALID;
    }

    DRV_TOUCH_ADC_CLIENT_OBJECT * pClient = &sADCClientInstances[index];
    if (pClient == NULL)
    {
        SYS_ASSERT(false, "ADC Driver: Couldn't find a free client to open");
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
    Closes an opened instance of the ADC driver

  Description:
    This function closes an opened instance of the ADC driver, invalidating
    the handle.
 */
void DRV_TOUCH_ADC_Close ( DRV_HANDLE handle )
{
    DRV_TOUCH_ADC_CLIENT_OBJECT * pClient = (DRV_TOUCH_ADC_CLIENT_OBJECT *)handle;

    if ( pClient != &sADCClientInstances[0] )
    {
         SYS_ASSERT(false, "ADC Driver: Trying to close a client to a driver that is outside the range of client handles");
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
<#if CONFIG_PIC32MZ>
            if ( !PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1}) )
<#else>
            if ( !PLIB_ADC_ConversionHasCompleted(ADC_ID_1) )
</#if>
            {
                break; 
            }
            if ( (uint16_t) ${CONFIG_DRV_TOUCH_ADC_PRESS_THRESHOLD} <
<#if CONFIG_PIC32MZ>
                (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1})>>2 )
<#else>
                (uint16_t) PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0) )
</#if>
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
<#if CONFIG_PIC32MX>
            PLIB_ADC_MuxChannel0InputPositiveSelect( ADC_ID_1,
                                         DRV_TOUCH_ADC_MUX_TYPE,
                                         BSP_ADC_TOUCH_XPLUS_PIN );
</#if>

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

 <#if CONFIG_PIC32MZ>
            PLIB_ADCHS_GlobalSoftwareTriggerEnable(DRV_ADC_ID_1);
 <#else>
            PLIB_ADC_SamplingStart( ADC_ID_1 );
 </#if>

            pDrvInstance->state = DRV_TOUCH_ADC_STATE_CHECK_X;
            
            break;
        }

<#if CONFIG_PIC32MZ>
        case DRV_TOUCH_ADC_STATE_CHECK_X:
        {
            if ( !PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0}) )
            {
                break;
            }

            if ((uint16_t) ${CONFIG_DRV_TOUCH_ADC_PRESS_THRESHOLD} >
                (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0})>>2)
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
                adcX = -1;
                adcY = -1;

                pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
                pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
                return 1; // touch screen acquisition is done
    
                break;
            }
            break;
        }
<#else>
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
</#if>

<#if CONFIG_PIC32MZ>
        case DRV_TOUCH_ADC_STATE_CHECK_Y:
        {
            if ( !PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1}) )
            {
                break;
            }

            if ((uint16_t) ${CONFIG_DRV_TOUCH_ADC_PRESS_THRESHOLD} >
                (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1})>>2)
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

</#if>
        case DRV_TOUCH_ADC_STATE_RUN_X:
        case DRV_TOUCH_ADC_STATE_RUN_Y:
        {
        <#if CONFIG_PIC32MZ>
            PLIB_ADCHS_GlobalSoftwareTriggerEnable(DRV_ADC_ID_1);
        <#else>
            PLIB_ADC_SamplingStart( ADC_ID_1 );
        </#if>

            pDrvInstance->state = (pDrvInstance->state == DRV_TOUCH_ADC_STATE_RUN_X) ?
                DRV_TOUCH_ADC_STATE_GET_X : DRV_TOUCH_ADC_STATE_GET_Y;

        <#if CONFIG_PIC32MZ>
            break;
        </#if>
        }

<#if CONFIG_PIC32MZ>
        case DRV_TOUCH_ADC_STATE_GET_X:
        {
            if (!PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0}))
            {
                break;
            }
            temp = (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0})>>2;
            if (temp != tempX) 
            {
                tempX = temp;
                pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_X;
                break;
            }

            //y+
            SYS_PORTS_DirectionSelect( PORTS_ID_0, 
                                       SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_YPLUS_PORT, 
                                       BSP_ADC_TOUCH_YPLUS_PIN_MASK );
            
            PLIB_ADCHS_GlobalSoftwareTriggerEnable(DRV_ADC_ID_1);

            pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_Y;
            
            break;
        }
        
        case DRV_TOUCH_ADC_STATE_GET_Y:
        {

            if (!PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1}))
            {
                break;
            }

            temp = (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX1})>>2;

            if (temp != tempY) 
            {
                tempY = temp;
                pDrvInstance->state = DRV_TOUCH_ADC_STATE_RUN_Y;
                break;
            }

            //x+
            SYS_PORTS_DirectionSelect( PORTS_ID_0, 
                                       SYS_PORTS_DIRECTION_INPUT, 
                                       BSP_ADC_TOUCH_XPLUS_PORT, 
                                       BSP_ADC_TOUCH_XPLUS_PIN_MASK );

            PLIB_ADCHS_GlobalSoftwareTriggerEnable(DRV_ADC_ID_1);

            pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_VALUES;
            
            break;
        }

<#else>
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

</#if>
        case DRV_TOUCH_ADC_STATE_SET_Y:
        {
<#if CONFIG_PIC32MZ>
            if (!PLIB_ADCHS_AnalogInputDataIsReady(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0}))
<#else>
            if (!PLIB_ADC_ConversionHasCompleted(ADC_ID_1))
</#if>
            {
                break;
            }
                   
            if ( (uint16_t) ${CONFIG_DRV_TOUCH_ADC_PRESS_THRESHOLD} <
<#if CONFIG_PIC32MZ>
                (uint16_t) PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1, ${CONFIG_DRV_ADCHS_ANALOG_INPUT_ID_IDX0})>>2 )
<#else>
                PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0) )
</#if>
            {
                adcX = -1;
                adcY = -1;
                pDrvInstance->state = DRV_TOUCH_ADC_STATE_SET_X;
                pDrvInstance->touchStatus = DRV_TOUCH_POSITION_SINGLE;
                return 1; // touch screen acquisition is done
                break;
            }

<#if CONFIG_PIC32MX>
            //Y+
            PLIB_ADC_MuxChannel0InputPositiveSelect( ADC_ID_1,
                                         DRV_TOUCH_ADC_MUX_TYPE,
                                         BSP_ADC_TOUCH_YPLUS_PIN );
</#if>

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

<#if CONFIG_PIC32MZ>
            PLIB_ADCHS_GlobalSoftwareTriggerEnable(DRV_ADC_ID_1);
<#else>
            PLIB_ADC_SamplingStart(ADC_ID_1);
</#if>

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
        result = (long) ((((long) coefC * result) + coefD) >> 8);
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
<#if CONFIG_DRV_GFX_DISPLAY_REQUIRE_SWAP_X_Y ||CONFIG_DRV_GFX_DISPLAY_ORIENTATION == "90" || CONFIG_DRV_GFX_DISPLAY_ORIENTATION == "270">
    return adcY;
<#else>
    return adcX;
</#if>
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
        result = (long) ((((long) coefA * result) + (long) coefB) >> 8);
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
<#if CONFIG_DRV_GFX_DISPLAY_REQUIRE_SWAP_X_Y || CONFIG_DRV_GFX_DISPLAY_ORIENTATION == "90" || CONFIG_DRV_GFX_DISPLAY_ORIENTATION == "270">
    return adcX;
<#else>
    return adcY;
</#if>
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

    DRV_TOUCH_ADC_CoefficientSet(xRawTouch, yRawTouch);
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
<#if CONFIG_PIC32MZ>
    PLIB_ADCHS_ChannelDigitalFeatureEnable(DRV_ADC_ID_1, ADCHS_CHANNEL_7);
<#else>
    PLIB_ADC_Disable(ADC_ID_1);
    PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS);
    PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA);
    PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_1SAMPLE_PER_INTERRUPT);
    PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 31);
    PLIB_ADC_ConversionClockSet(ADC_ID_1, 80000000, 156250 );
    PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
    PLIB_ADC_Enable(ADC_ID_1);
</#if>

<#if CONFIG_PIC32MX>
    PLIB_ADC_MuxAInputScanDisable(ADC_ID_1);
</#if>

}

void DRV_TOUCH_ADC_CoefficientSet(short xraw[], short yraw[])
{
    long trA, trB, trC, trD;
    long trAhold, trBhold, trChold, trDhold;
    long test1, test2;

    short xPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];
    short yPoint[DRV_TOUCH_ADC_SAMPLE_POINTS];

    // set expected user input 10% from corners
    yPoint[0] = yPoint[1] = CAL_Y_INSET;
    yPoint[2] = yPoint[3] = (${CONFIG_DRV_GFX_DISPLAY_HEIGHT} - CAL_Y_INSET);
    xPoint[0] = xPoint[3] = CAL_X_INSET;
    xPoint[1] = xPoint[2] = (${CONFIG_DRV_GFX_DISPLAY_WIDTH} - CAL_X_INSET);

    // get slopes for expected and actual  m = (y2-y1/x2-x1)
    // get y-intercepts

    test1 = (long) yPoint[0] - (long) yPoint[3];
    test2 = (long) yraw[0] - (long) yraw[3];

    trA = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[0] * SCALE_FACTOR) - (trA * (long) yraw[0]));

    test1 = (long) xPoint[0] - (long) xPoint[2];
    test2 = (long) xraw[0] - (long) xraw[2];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[0] * SCALE_FACTOR) - (trC * (long) xraw[0]));

    // temp hold slopes and y-intercepts
    trAhold = trA;
    trBhold = trB;
    trChold = trC;
    trDhold = trD;

    // get slopes for expected and actual  m = (y2-y1/x2-x1)
    // get y-intercepts

    test1 = (long) yPoint[1] - (long) yPoint[2];
    test2 = (long) yraw[1] - (long) yraw[2];

    trA = ((long) (test1 * SCALE_FACTOR) / test2);
    trB = ((long) ((long) yPoint[1] * SCALE_FACTOR) - (trA * (long) yraw[1]));

    test1 = (long) xPoint[1] - (long) xPoint[3];
    test2 = (long) xraw[1] - (long) xraw[3];

    trC = ((long) ((long) test1 * SCALE_FACTOR) / test2);
    trD = ((long) ((long) xPoint[1] * SCALE_FACTOR) - (trC * (long) xraw[1]));

    // save the average slopes and y-intercepts and use the average slopes
    coefA = (trA + trAhold) >> 1;
    coefB = (trB + trBhold) >> 1;
    coefC = (trC + trChold) >> 1;
    coefD = (trD + trDhold) >> 1;

}

void __ISR(${CONFIG_DRV_TOUCH_ADC_INTERRUPT_VECTOR}, ipl1AUTO) IntHandlerDrvTouchAdc(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_TOUCH_ADC_INTERRUPT_SOURCE});
    DRV_TOUCH_ADC_PositionDetect();
}

/*******************************************************************************
 End of File
*/
