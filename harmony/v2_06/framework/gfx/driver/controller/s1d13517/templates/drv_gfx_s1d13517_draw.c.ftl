/*******************************************************************************
  Company:
    Microchip Technology Incorporated

  File Name:
    drv_gfx_s1d13517_draw.c

  Summary:
    Contains driver-specific draw function implementations for Epson S1D13517
	display driver

  Description:
    None
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#include "framework/gfx/driver/controller/s1d13517/drv_gfx_s1d13517.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#include "framework/driver/pmp/drv_pmp_static.h"
<#if CONFIG_GFX_HAL_DRAW_PIPELINE_ENABLED>
<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
<#if CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
// 16 bit PMP, 16 bit color mode
static void WritePixel(GFX_Color color)
{
    DRV_PMP0_Write(color);
}
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
// 16 bit PMP, 24 bit color mode
static void WritePixel(GFX_Color color)
{
    DRV_PMP0_Write(GFX_ColorChannelRed(color, GFX_COLOR_MODE_RGB_888));
    DRV_PMP0_Write((GFX_ColorChannelGreen(color, GFX_COLOR_MODE_RGB_888) << 8) |
                   (GFX_ColorChannelBlue(color, GFX_COLOR_MODE_RGB_888)));
}
</#if>
<#elseif CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
<#if CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_565">
// 8 bit PMP, 16 bit color mode
static void WritePixel(GFX_Color color)
{
    DRV_PMP0_Write((uint8_t)(color >> 8));
    DRV_PMP0_Write((uint8_t)color);
}
<#elseif CONFIG_DRV_GFX_S1D13517_COLOR_MODE == "RGB_888">
// 8 bit PMP, 24 bit color mode
static void WritePixel(GFX_Color color)
{
    DRV_PMP0_Write(GFX_ColorChannelRed(color, GFX_COLOR_MODE_RGB_888));
    DRV_PMP0_Write(GFX_ColorChannelGreen(color, GFX_COLOR_MODE_RGB_888));
    DRV_PMP0_Write(GFX_ColorChannelBlue(color, GFX_COLOR_MODE_RGB_888));
}
</#if>
</#if>


static void setupWriteWindow(uint32_t x, uint32_t y, uint16_t width, uint16_t height)
{
    struct S1D13517DriverData* data = (struct S1D13517DriverData*)GFX_ActiveContext()->driver_data;
    
    while(drvS1D13517_SetRegister(S1D13517_REG5A_WRITE_WIN_X_SP,(x >> 2)));

    DRV_PMP0_Write(y >> 2);
    DRV_PMP0_Write(y);
    DRV_PMP0_Write((x + (width - 1)) >> 2);
    DRV_PMP0_Write((y + height - 1) >> 2);
    DRV_PMP0_Write((y + height - 1));

    data->preDrawCount = ((x) & 7);
    data->postDrawCount = (data->preDrawCount + width);
    data->postDrawCount &= 7;
    data->postDrawCount = (8 - data->postDrawCount) & 7;
}

static void fillDrawWindow(GFX_Color color, uint16_t width, uint16_t height)
{
    short temp;
    struct S1D13517DriverData* data = (struct S1D13517DriverData*)GFX_ActiveContext()->driver_data;

    while(height)
    {
		for(temp = 0; temp < data->preDrawCount; temp++)
			WritePixel(S1D13517_MASK_COLOR_1);

		for(temp = 0; temp < width; temp++)
			WritePixel(color);

		for(temp = 0; temp < data->postDrawCount; temp++)
			WritePixel(S1D13517_MASK_COLOR_1);

		height--;
   }
}

GFX_Result drvS1D13517_SetPixel(const GFX_PixelBuffer* buf, const GFX_Point* pnt, GFX_Color color)
{
    setupWriteWindow(pnt->x, pnt->y, 1, 1);

    fillDrawWindow(color, 1, 1);
	
	return GFX_SUCCESS;
}  

void drawLine(int32_t x1,
              int32_t x2,
              int32_t y,
              const GFX_DrawState* state)
{
    GFX_Rect rect, clipRect;
    GFX_Context* context = GFX_ActiveContext();
    
    if(x1 > x2)
    {
        rect.x = x2;
        rect.width = x1 - x2;
    }
    else
    {
        rect.x = x1;
        rect.width = x2 - x1;
    }
    
    rect.y = y;
    rect.height = 1;
    
    // ensure draw rect is inside layer display bounds
    // always doing this because it's unknown how the hardware
    // reacts to drawing outside its physical bounds
    if(GFX_RectIntersects(&rect, &context->layer.active->rect.display) == GFX_FALSE)
        return;
    
    GFX_RectClip(&rect, &context->layer.active->rect.display, &clipRect);
    
    rect = clipRect;
    
#if GFX_BOUNDS_CLIPPING_ENABLED
    // clip against any defined/enabled global clip rectangle
    if(state->clipEnable == GFX_TRUE)
    {
        if(GFX_RectIntersects(&rect, &state->clipRect) == GFX_FALSE)
            return;

        GFX_RectClip(&rect, &state->clipRect, &clipRect);
        
        rect = clipRect;
    }
#endif
    
    setupWriteWindow(rect.x,
                     y,
                     rect.width,
                     1);
    
    fillDrawWindow(state->color, rect.width, 1);

    return;
}

GFX_Result drvS1D13517_DrawLine(const GFX_Point* p1,
                                const GFX_Point* p2,
                                const GFX_DrawState* state)
{
    GFX_Context* context = GFX_ActiveContext();
    
    // mask out the whole operation
    if(state->maskEnable && state->maskValue != state->color)
        return GFX_SUCCESS;
    
    // a basic horizontal line is an optimal case for this driver
    // everything else should go through normal pixel pipeline
    if(context->orientation == GFX_ORIENTATION_0 &&
       context->mirrored == GFX_FALSE &&
       state->thickness == 1 &&
       p1->y == p2->y)
    {
        drawLine(p1->x, p2->x, p1->y, state);
        
        return GFX_SUCCESS;
    }
    
    return cpuDrawLine(p1, p2, state);
}

GFX_Result drvS1D13517_FillRect(const GFX_Rect* rect,
                                const GFX_DrawState* state)
{
    GFX_Context* context = GFX_ActiveContext();
    GFX_Layer* layer = context->layer.active;
    GFX_Rect clipRect;
	
	// a basic fill is an optimal case for this driver
    // everything else should go through software pixel pipeline
    if(context->orientation != GFX_ORIENTATION_0 ||
       context->mirrored != GFX_FALSE)
        return cpuDrawRect_Fill(rect, state);
    
	// clip against the physical layer bounds
    if(GFX_PixelBufferClipRect(&layer->buffers[layer->buffer_write_idx].pb,
                               rect,
                               &clipRect) == GFX_FAILURE)
    {
        return GFX_FAILURE;
    }
	
#if GFX_BOUNDS_CLIPPING_ENABLED
    // clip against the global clipping rectangle
    if(state->clipEnable == GFX_TRUE)
    {
        if(GFX_RectIntersects(rect, &state->clipRect) == GFX_FALSE)
            return GFX_SUCCESS;
            
        GFX_RectClip(rect, &state->clipRect, &clipRect);
    }
    else
        clipRect = *rect;
#else
	clipRect = *rect;
#endif

    /* Todo
    
    custom draw calls aren't optimized for non 0 degree orientations
    rectangles can be reoriented to fill using optimized methods
    */
    
    setupWriteWindow(clipRect.x,
                     clipRect.y,
                     clipRect.width,
                     clipRect.height);
    
    fillDrawWindow(state->color, clipRect.width, clipRect.height);
    
    return GFX_SUCCESS;
}
</#if>
