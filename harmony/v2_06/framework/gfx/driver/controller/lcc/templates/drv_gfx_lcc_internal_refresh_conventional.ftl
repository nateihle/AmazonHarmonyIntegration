void DRV_GFX_LCC_DisplayRefresh(void)
{
	GFX_Point drawPoint;
	GFX_PixelBuffer* buffer;
	
	uint8_t* bufferPtr;
	
<#if CONFIG_DRV_GFX_LCC_PALETTE_MODE == true>
	uint16_t* palette;
    uint32_t i;
</#if>

    typedef enum
	{
        HSYNC_FRONT_PORCH,
        HSYNC_PULSE,
        HSYNC_BACK_PORCH,
        HSYNC_DATA_ENABLE,
        HSYNC_DATA_ENABLE_OVERFLOW        
    } HSYNC_STATES;

    typedef enum
    {
        VSYNC_FRONT_PORCH,
        VSYNC_PULSE,
        VSYNC_BACK_PORCH,
        VSYNC_BLANK        
    } VSYNC_STATES;

    static HSYNC_STATES hsyncState = HSYNC_FRONT_PORCH;
    static VSYNC_STATES vsyncState = VSYNC_BLANK;

    switch(vsyncState)
    {
        case VSYNC_FRONT_PORCH:
		{
            if (hSyncs > vsyncPulseDown)
            {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOn();
<#else>
                BSP_LCD_VSYNCOff();
</#if>
                vsyncPulseUp = hSyncs + DISP_VER_PULSE_WIDTH;
                vsyncState = VSYNC_PULSE;

				if(cntxt->layer.active->vsync == GFX_TRUE
					&& cntxt->layer.active->swap == GFX_TRUE
                        && swapPending == true)
                {
					GFX_LayerSwap(cntxt->layer.active);
                    swapPending = false;
                }

                line = 0;
            }
			
            break;
		}
        case VSYNC_PULSE:
		{
            if (hSyncs >= vsyncPulseUp)
            {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOff();
<#else>
                BSP_LCD_VSYNCOn();
</#if>
                vsyncEnd = hSyncs + DISP_VER_BACK_PORCH;
                vsyncState = VSYNC_BACK_PORCH;
            }
			
            break;
		}
        case VSYNC_BACK_PORCH:
		{
            if (hSyncs >= vsyncEnd)
                vsyncState = VSYNC_BLANK;
            
            break;
		}
        case VSYNC_BLANK:
		{
            break;
		}
    }

    switch (hsyncState)
    {
        case HSYNC_FRONT_PORCH:
		{
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();			
<#else>
            BSP_LCD_DEOn();			
</#if>
</#if>

            hsyncState = HSYNC_PULSE;
			
			if (DISP_HOR_FRONT_PORCH > 0)
			{
	            pixels = DISP_HOR_FRONT_PORCH;
	            break;
			}
		}
        case HSYNC_PULSE:
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>

            if (hSyncs >= vsyncPeriod)
            {
				vsyncPeriod = hSyncs + DISP_VER_PULSE_WIDTH + DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION + DISP_VER_BACK_PORCH;
                vsyncPulseDown = hSyncs + DISP_VER_FRONT_PORCH;
                vsyncState = VSYNC_FRONT_PORCH;
            }
			
            hSyncs++; 
            
			pixels = DISP_HOR_PULSE_WIDTH;
            hsyncState = HSYNC_BACK_PORCH;  
            
			break;
		}
        case HSYNC_BACK_PORCH:
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if>

            hsyncState = HSYNC_DATA_ENABLE; 
			
			if (DISP_HOR_BACK_PORCH > 0)
			{
				pixels = DISP_HOR_BACK_PORCH;
				break;
			}
		}
        case HSYNC_DATA_ENABLE:
		{
            if (vsyncState == VSYNC_BLANK)
            {
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
                BSP_LCD_DEOn();
<#else>
                BSP_LCD_DEOff();
</#if>
</#if>
				drawPoint.x = 0;
				drawPoint.y = line++;
				
				buffer = &cntxt->layer.active->buffers[cntxt->layer.active->buffer_read_idx].pb;

				bufferPtr = GFX_PixelBufferOffsetGet_Unsafe(buffer, &drawPoint);
				
<#if CONFIG_DRV_GFX_LCC_PALETTE_MODE == false>
                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,
				                                       DRV_GFX_LCC_DMA_CHANNEL_INDEX,
													   (uint32_t)bufferPtr);
<#else>
				palette = (uint16_t*)GFX_ActiveContext()->globalPalette;
                
                for(i = 0; i < DISPLAY_WIDTH; i++)
                    frameLine[i] = palette[bufferPtr[i]];
                
				PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,
				                                       DRV_GFX_LCC_DMA_CHANNEL_INDEX,
													   (uint32_t)frameLine);
</#if>													   
			}
            
			pixels = DISP_HOR_RESOLUTION;
            hsyncState = HSYNC_FRONT_PORCH;
			
			break;
		}
        case HSYNC_DATA_ENABLE_OVERFLOW:
		{
            hsyncState = HSYNC_FRONT_PORCH;
			
            break;
		}
    }

    PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX,(uint16_t)(pixels<<1));
    PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(pixels<<1));
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    SYS_DMA_ChannelForceStart(dmaHandle);
}