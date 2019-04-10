/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_graph.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides prototypes and definitions for the application.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef APP_GRAPH_H
#define	APP_GRAPH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include "gfx/libaria/libaria_init.h"
#include "app_display_task.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
#define PIXELS_PER_Y_DIV                            20    
#define PIXELS_PER_X_DIV                            22   

#define APP_GRAPH_MAX_AMP_ZOOM_LEVELS               7
#define APP_GRAPH_MAX_TIME_PER_DIV_LEVELS           8  

#define APP_GRAPH_GRID_COLOR                        0x634d  
#define APP_GRAPH_AXIS_COLOR                        0xFFE0
#define APP_GRAPH_PLOT_COLOR                        0xFFE0    
    
typedef struct
{
    int16_t*                    pSignal;
    uint32_t                    nSamples;
    uint32_t                    samplingFreq;
            
}APP_GRAPH_SIG_PARAMS;
    
typedef struct
{
    bool                         isGridShow;    
    float                        timePerDiv;
    uint32_t                     amplitudeZoomLevel;
    uint8_t                      timePerDivScaleIndex;
    uint8_t                      amplitudeZoomScaleIndex;
    APP_GRAPH_SIG_PARAMS         sigParams;
    
}APP_GRAPH_PARAMS;

void APP_GRAPH_Init(void);

void APP_GRAPH_GridShow(bool isShow);

void APP_GRAPH_Show(void);

void APP_GRAPH_Update(void);   

void APP_GRAPH_AmpScaleChanged(APP_GRAPH_CHANGE_TYPE change);

void APP_GRAPH_TimePerDivChanged(APP_GRAPH_CHANGE_TYPE change);

#ifdef	__cplusplus
}
#endif

#endif	/* APP_GRAPH_H */

