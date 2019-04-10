/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_graph.c

  Summary:
    Contains the functional implementation of the graphing application.

  Description:
    This file contains the functional implementation of the graphing
    application. It plots the time-domain signal selected by the user.
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_graph.h"
#include "app_tone_generator.h"


static const float TIME_PER_DIV_SCALE_IN_MSEC[APP_GRAPH_MAX_TIME_PER_DIV_LEVELS] = 
{
    0.01,
    0.02,
    0.05,
    0.1,
    0.2,
    0.5,
    1.0,
    2.0,    
};

static const uint32_t AMP_ZOOM_SCALE[APP_GRAPH_MAX_AMP_ZOOM_LEVELS]=
{
    1,      //1X Zoom
    2,      //2X Zoom
    4,      //4X Zoom
    8,      //8X Zoom   
    16,     //16X Zoom
    32,     //32X Zoom
    64,     //64X Zoom
};


static APP_GRAPH_PARAMS     appGraphParams;

static laBool APP_GRAPH_CustomDraw(
    laDrawSurfaceWidget* sfc,
    GFX_Rect* bounds
);

static void APP_GRAPH_UpdateTimePerDivLabel(float timePerDiv)
{
    static char strBuffer[10] = {0};
    static laString gfxString = {0};     
    
    sprintf((char*)strBuffer, "%.2f ms", (double)timePerDiv);
    gfxString = laString_CreateFromCharBuffer((const char*)strBuffer, &Arial_Bold);
    laLabelWidget_SetText(CurrentTimePerDivValueLabel, gfxString);    
}

static void APP_GRAPH_UpdateAmpScaleLabel(uint32_t ampScale)
{
    static char strBuffer[10] = {0};
    static laString gfxString = {0};     
    
    sprintf((char*)strBuffer, "%dX", ampScale);
    gfxString = laString_CreateFromCharBuffer((const char*)strBuffer, &Arial_Bold);
    laLabelWidget_SetText(CurrentAmpPerDivValueLabel, gfxString);    
}


void APP_GRAPH_Init(void)
{    
    appGraphParams.timePerDivScaleIndex = 3;
    appGraphParams.timePerDiv = TIME_PER_DIV_SCALE_IN_MSEC[appGraphParams.timePerDivScaleIndex];
    appGraphParams.sigParams.samplingFreq = APP_TONE_GetSamplingFreq();
    appGraphParams.isGridShow = true;    
    appGraphParams.amplitudeZoomScaleIndex = 0;
    appGraphParams.amplitudeZoomLevel = AMP_ZOOM_SCALE[appGraphParams.amplitudeZoomScaleIndex];
    
    laDrawSurfaceWidget_SetDrawCallback(
        GraphSurfaceWidget, 
        APP_GRAPH_CustomDraw
    ); 
}

void APP_GRAPH_Show(void)
{
    APP_GRAPH_UpdateTimePerDivLabel(appGraphParams.timePerDiv);
    APP_GRAPH_UpdateAmpScaleLabel(appGraphParams.amplitudeZoomLevel);    
}

void APP_GRAPH_Update(void)
{
    appGraphParams.sigParams.pSignal = APP_TONE_GetMixedSignal();
    
    appGraphParams.sigParams.nSamples = APP_TONE_GetNumSamples();
    
    laWidget_Invalidate((laWidget*)GraphSurfaceWidget);
}

void APP_GRAPH_GridShow(bool isShow)
{
    appGraphParams.isGridShow = isShow;
    laWidget_Invalidate((laWidget*)GraphSurfaceWidget);
}

void APP_GRAPH_AmpScaleChanged(APP_GRAPH_CHANGE_TYPE change)
{
    uint8_t index = appGraphParams.amplitudeZoomScaleIndex;
    bool isGraphUpdate = false;
            
    if (APP_GRAPH_CHANGE_TYPE_INC == change)
    {
        if (index < (APP_GRAPH_MAX_AMP_ZOOM_LEVELS-1))
        {
            index++;
            isGraphUpdate = true;
        }        
    }
    else
    {
        if (index > 0)
        {
            index--;
            isGraphUpdate = true;
        }
    }
    
    if (true == isGraphUpdate)
    {
        appGraphParams.amplitudeZoomScaleIndex = index;
        appGraphParams.amplitudeZoomLevel = AMP_ZOOM_SCALE[index];
        APP_GRAPH_UpdateAmpScaleLabel(appGraphParams.amplitudeZoomLevel);
        laWidget_Invalidate((laWidget*)GraphSurfaceWidget);
    }
}

void APP_GRAPH_TimePerDivChanged(APP_GRAPH_CHANGE_TYPE change)
{  
    uint8_t index = appGraphParams.timePerDivScaleIndex;
    bool isGraphUpdate = false;
            
    if (APP_GRAPH_CHANGE_TYPE_INC == change)
    {
        if (index < (APP_GRAPH_MAX_TIME_PER_DIV_LEVELS-1))
        {
            index++;
            isGraphUpdate = true;
        }        
    }
    else
    {
        if (index > 0)
        {
            index--;
            isGraphUpdate = true;
        }
    }        
    
    if (true == isGraphUpdate)
    {
        appGraphParams.timePerDivScaleIndex = index;
        appGraphParams.timePerDiv = TIME_PER_DIV_SCALE_IN_MSEC[index];
        APP_GRAPH_UpdateTimePerDivLabel(appGraphParams.timePerDiv);
        laWidget_Invalidate((laWidget*)GraphSurfaceWidget);
    }
}

static void APP_GRAPH_DrawRef(GFX_Rect gridCoordinates)
{
    //Set zero axis color
    GFX_Set(GFXF_DRAW_COLOR, APP_GRAPH_AXIS_COLOR);    
    
    //draw reference axis
    GFX_DrawLine(
        gridCoordinates.x, 
        gridCoordinates.y + gridCoordinates.height/2, 
        gridCoordinates.x + gridCoordinates.width,
        gridCoordinates.y + gridCoordinates.height/2
    );            
    //draw reference axis    
    GFX_DrawLine(
        gridCoordinates.x, 
        gridCoordinates.y + gridCoordinates.height/2, 
        gridCoordinates.x + gridCoordinates.width,
        gridCoordinates.y + gridCoordinates.height/2
    );
}

static void APP_GRAPH_DrawGrid(
    GFX_Rect gridCoordinates,
    GFX_Size divSize
)
{
    int32_t i;    
    int32_t width;
    int32_t height;
    int32_t numYGridLines = gridCoordinates.width/divSize.width + 1;
    int32_t numXGridLines = gridCoordinates.height/divSize.height + 1;
    
    //Set Color
    GFX_Set(GFXF_DRAW_COLOR, APP_GRAPH_GRID_COLOR);    
    
    //draw Y grid lines
    for (i = 0, width = 0; i < numYGridLines; i++, width += divSize.width)
    {
        GFX_DrawLine(
            gridCoordinates.x + width, 
            gridCoordinates.y, 
            gridCoordinates.x + width, 
            gridCoordinates.y + gridCoordinates.height);
    }
    
    //draw X grid lines
    for (i = 0, height = 0; i < numXGridLines; i++, height += divSize.height)
    {
        GFX_DrawLine(
            gridCoordinates.x, 
            gridCoordinates.y + height, 
            gridCoordinates.x + gridCoordinates.width, 
            gridCoordinates.y + height);
    }        
}

static void APP_GRAPH_PlotGraph(
    GFX_Rect gridRect,
    GFX_Rect refAxis,    
    const int16_t* const pRawData,
    uint32_t nSamples
)
{
    uint32_t samplesDrawn = 0;
    float xInc = ((float)PIXELS_PER_Y_DIV)/((appGraphParams.timePerDiv/1000.0) * appGraphParams.sigParams.samplingFreq);
    float xDistance = 0;
    float yScale = (gridRect.height/2)/(float)((SIGNAL_FS_VALUE)/(float)appGraphParams.amplitudeZoomLevel);
    GFX_Point p1, p2;        
    
    //Set color
    GFX_Set(GFXF_DRAW_COLOR, APP_GRAPH_PLOT_COLOR); 
        
    for (xDistance = 0; 
        ((xDistance + xInc) < refAxis.width) && (samplesDrawn < nSamples); 
        xDistance += xInc, samplesDrawn++
    )
    {        
        p1.x = refAxis.x + xDistance;
        p1.y = refAxis.y - pRawData[samplesDrawn]*yScale;
        p2.x = refAxis.x + xDistance + xInc;
        p2.y = refAxis.y - pRawData[samplesDrawn+1]*yScale;
        
        if ((p1.y > gridRect.y) &&
             (p1.y < (gridRect.y + gridRect.height)) &&
             (p2.y > gridRect.y) &&
             (p2.y < (gridRect.y + gridRect.height))
        )
        {                        
            GFX_DrawLine(p1.x, p1.y, p2.x, p2.y);        
        }                
    }
}

static laBool APP_GRAPH_CustomDraw(
    laDrawSurfaceWidget* sfc,
    GFX_Rect* bounds
)
{    
    GFX_Rect gridRect = {bounds->x, bounds->y, bounds->width-1, bounds->height-1};
    GFX_Rect refAxis = {gridRect.x, gridRect.y + gridRect.height/2, gridRect.width, gridRect.height/2};
    GFX_Size divSize = {PIXELS_PER_Y_DIV, PIXELS_PER_X_DIV};        
    
    //draw grid
    if (true == appGraphParams.isGridShow)
    {
        APP_GRAPH_DrawGrid(gridRect, divSize);    
    }
    
    //Draw Reference
    APP_GRAPH_DrawRef(gridRect);
               
    //Plot Graph
    if (appGraphParams.sigParams.pSignal)
    {
        APP_GRAPH_PlotGraph(
            gridRect,
            refAxis,             
            appGraphParams.sigParams.pSignal,
            appGraphParams.sigParams.nSamples
        );
    }
    
    return LA_TRUE;
}