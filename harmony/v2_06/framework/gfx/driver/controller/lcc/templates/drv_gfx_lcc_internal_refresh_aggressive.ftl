void DRV_GFX_LCC_DisplayRefresh(void)
{
	GFX_Point drawPoint;
	GFX_PixelBuffer* buffer;

	static uint8_t GraphicsState = ACTIVE_PERIOD;
    static uint16_t remaining = 0;
    static short line = 0;
	
	uint8_t* bufferPtr;
	
<#if CONFIG_DRV_GFX_LCC_PALETTE_MODE == true>
	uint16_t* palette;
    uint32_t i;
</#if>

    switch(GraphicsState)
    {
        case ACTIVE_PERIOD:
		{
            remaining = DISP_HOR_RESOLUTION << PMP_DATA_LENGTH;
            GraphicsState = BLANKING_PERIOD;

<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if>

            if(line >= 0)
            {
				drawPoint.x = 0;
				drawPoint.y = line;
				
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

                if(line == (DISP_VER_RESOLUTION))
                {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOn();
<#else>
                    BSP_LCD_VSYNCOff();
</#if>
                    line = -(VER_BLANK);
					
					if(cntxt->layer.active->vsync == GFX_TRUE
						&& cntxt->layer.active->swap == GFX_TRUE
							&& swapPending == true)
					{
						GFX_LayerSwap(cntxt->layer.active);
						swapPending = false;
					}
                }
                else
                {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOff();
<#else>
                    BSP_LCD_VSYNCOn();
</#if>
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
                    BSP_LCD_DEOn();
<#else>
                    BSP_LCD_DEOff();
</#if>
</#if>
                }
           }

           PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX,remaining);
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
           PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, remaining);
</#if>
           break;
        }
        case BLANKING_PERIOD:
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true >
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();
<#else>
            BSP_LCD_DEOn();
</#if>
</#if>
            PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch << PMP_DATA_LENGTH));
            GraphicsState = ACTIVE_PERIOD;
            line++;

<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
            PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch<<1));
</#if>
		}
    }

    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
   
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    SYS_DMA_ChannelForceStart(dmaHandle);
<#else>
    SYS_DMA_ChannelEnable(dmaHandle);
</#if>
}