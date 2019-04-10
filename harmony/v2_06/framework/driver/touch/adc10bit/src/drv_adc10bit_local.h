/*******************************************************************************
 Touch adc 10bit Driver Local Interface File
 
  Company:
    Microchip Technology Inc.

  File Name:
    drv_adc10bit_local.h

  Summary:
    Touch adc 10bit Local Driver interface header file.

  Description:
    This header file describes the macros, data structure and prototypes of the 
    touch adc 10bit local driver interface.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_TOUCH_ADC10BIT_LOCAL_H
#define _DRV_TOUCH_ADC10BIT_LOCAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/touch/drv_touch.h"
#include "driver/touch/adc10bit/drv_adc10bit.h"
#include "osal/osal.h"
#include "system/int/sys_int.h"
#include "system/debug/sys_debug.h"
#include "peripheral/adc/plib_adc.h"
#include "system/ports/sys_ports.h"

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    use this scale factor to avoid working in floating point numbers

*/
#define SCALE_FACTOR (1<<DRV_TOUCH_ADC10BIT_CALIBRATION_SCALE_FACTOR)

#if (DISP_ORIENTATION == 90)
    #define ADC_MaxXGet() (DISP_VER_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 270)
    #define ADC_MaxXGet() (DISP_VER_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 0)
    #define ADC_MaxXGet() (DISP_HOR_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 180)
    #define ADC_MaxXGet() (DISP_HOR_RESOLUTION - 1)
#endif
#if (DISP_ORIENTATION == 90)
    #define ADC_MaxYGet() (DISP_HOR_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 270)
    #define ADC_MaxYGet() (DISP_HOR_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 0)
    #define ADC_MaxYGet() (DISP_VER_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 180)
    #define ADC_MaxYGet() (DISP_VER_RESOLUTION - 1)
#endif

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/
#define CAL_X_INSET    (((ADC_MaxXGet()+1)*(DRV_TOUCH_ADC10BIT_CALIBRATION_INSET>>1))/100)

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/
#define CAL_Y_INSET    (((ADC_MaxYGet()+1)*(DRV_TOUCH_ADC10BIT_CALIBRATION_INSET>>1))/100)

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

#define DRV_ADC10BIT_SAMPLE_POINTS   4

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/
typedef enum 
{
    /* */
    DRV_ADC10BIT_STATE_IDLE,
            
    /* */
    DRV_ADC10BIT_STATE_SET_X,
            
    /* */
    DRV_ADC10BIT_STATE_RUN_X,
    
    /* */
    DRV_ADC10BIT_STATE_GET_X,
            
    /* */
    DRV_ADC10BIT_STATE_RUN_CHECK_X,
            
    /* */
    DRV_ADC10BIT_STATE_CHECK_X,
            
    /* */
    DRV_ADC10BIT_STATE_SET_Y,
            
    /* */
    DRV_ADC10BIT_STATE_RUN_Y,
            
    /* */
    DRV_ADC10BIT_STATE_GET_Y,
            
    /* */
    DRV_ADC10BIT_STATE_CHECK_Y,
            
    /* */
    DRV_ADC10BIT_STATE_SET_VALUES,
            
    /* */
    DRV_ADC10BIT_STATE_GET_POT,
            
    /* */
    DRV_ADC10BIT_STATE_RUN_POT
            
} DRV_ADC10BIT_TOUCH_STATES;

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
    
    DRV_ADC10BIT_TOUCH_STATES state;

    /* Touch status */
    DRV_TOUCH_POSITION_STATUS       touchStatus;

} DRV_ADC10BIT_OBJECT;

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

typedef struct
{
    /* Driver Object associated with the client */
    DRV_ADC10BIT_OBJECT * driverObject;

    /* The intent with which the client was opened */
    DRV_IO_INTENT         intent;

} DRV_ADC10BIT_CLIENT_OBJECT;

//******************************************************************************
// Local functions
//******************************************************************************
void  _DRV_TOUCH_ADC10BIT_HardwareInit       (void *initValues);
void  _DRV_TOUCH_ADC10BIT_CalculateCalPoints (void);

#endif

/*******************************************************************************
 End of File
*/
