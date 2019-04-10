void DRV_GFX_LCC_DisplayRefresh(void)
{
    static uint8_t GraphicsState = ACTIVE_PERIOD;
    static uint16_t remaining = 0;
    static short line = 0;
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    DrawCount = 0;
</#if>

    switch(GraphicsState)
    {
        case ACTIVE_PERIOD:
		{
            remaining = DISP_HOR_RESOLUTION;
            GraphicsState = BLANKING_PERIOD;

<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if>
</#if>
            if(line >= 0)
            {
                PMADDR = ((PMP_ADDRESS_LINES) & ((line) * DISP_HOR_RESOLUTION));

                if((line) == (DISP_VER_RESOLUTION))
                {
					if(cntxt->layer.active->vsync == GFX_TRUE
						&& cntxt->layer.active->swap == GFX_TRUE
							&& swapPending == true)
					{
						GFX_LayerSwap(cntxt->layer.active);
						swapPending = false;
					}
				
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOn();
<#else>
                    BSP_LCD_VSYNCOff();
</#if>
                    line = (-VER_BLANK);

<#if CONFIG_DRV_GFX_LCC_ADDRESS_LINE_15 == true >
                    BSP_SRAM_A15StateSet(0);
                    BSP_SRAM_A16StateSet(0);
                    overflowcount = 0;
<#else>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "QVGA or lower"
	|| CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA">
                    BSP_SRAM_A16StateSet(0);
                    overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
                    BSP_SRAM_A16StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
                    BSP_SRAM_A17StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
                    BSP_SRAM_A18StateSet(0);
				
					if ( cntxt->layer.active->buffer_read_idx == 1 )
						overflowcount = 3;
					else
						overflowcount = 0;
						
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
					BSP_SRAM_A16StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
					BSP_SRAM_A17StateSet(0);
					BSP_SRAM_A18StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
					
					if ( cntxt->layer.active->buffer_read_idx == 1 )
						overflowcount = 5;
					else
						overflowcount = 0;
						
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
					BSP_SRAM_A16StateSet(0);
					BSP_SRAM_A17StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
					BSP_SRAM_A18StateSet(cntxt->layer.active->buffer_read_idx & 0x1);
					
					if ( cntxt->layer.active->buffer_read_idx == 1 )
						overflowcount = 6;
					else
						overflowcount = 0;
						
</#if>
</#if>
<#if CONFIG_PIC32MZ == true>
					BSP_SRAM_A19StateSet(0);
</#if>
               }
               else
               {
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
                    BSP_LCD_DEOn();
					
<#else>
                    BSP_LCD_DEOff();
					
</#if>
</#if>
					if((PMADDR_OVERFLOW - PMADDR) < (DISP_HOR_RESOLUTION))       
					{   
						  GraphicsState = OVERFLOW;      //Do Overflow routine
						  
						  PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(((PMADDR_OVERFLOW)- PMADDR)<<1));
						  PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
						  PLIB_DMA_ChannelXEnable(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX);
						  
						  remaining = DISP_HOR_RESOLUTION - ((PMADDR_OVERFLOW)- PMADDR);
						  
						  return;
					}           
                }
            }
            break;
	    }
        case OVERFLOW: //Adjust the address lines that aren't part of PMP
	    {
            GraphicsState = BLANKING_PERIOD;     //goto Front Porch
			
<#if CONFIG_DRV_GFX_LCC_ADDRESS_LINE_15 == true >
            BSP_SRAM_A15StateSet((++overflowcount) & 0x1);
            BSP_SRAM_A16StateSet((overflowcount >> 1) & 0x1);
<#else>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "QVGA or lower"
	|| CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA">
            BSP_SRAM_A16StateSet((++overflowcount) & 0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
            BSP_SRAM_A16StateSet((++overflowcount) & 0x1);
            BSP_SRAM_A17StateSet((overflowcount >> 1) & 0x1);
            BSP_SRAM_A18StateSet((overflowcount >> 2) & 0x1);
			
</#if>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
            BSP_SRAM_A16StateSet((++overflowcount) & 0x1);
            BSP_SRAM_A17StateSet((overflowcount >> 1) & 0x1);
            BSP_SRAM_A18StateSet((overflowcount >> 2) & 0x1);
            BSP_SRAM_A19StateSet((overflowcount >> 3) & 0x1);
			
</#if>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
            BSP_SRAM_A16StateSet((++overflowcount) & 0x1);
            BSP_SRAM_A17StateSet((overflowcount >> 1) & 0x1);
            BSP_SRAM_A18StateSet((overflowcount >> 2) & 0x1);
            BSP_SRAM_A19StateSet((overflowcount >> 3) & 0x1);
			
</#if>
</#if>

			break;
		}
        case BLANKING_PERIOD:   //Front Porch then Back Porch Start
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();
<#else>
            BSP_LCD_DEOn();
</#if>
</#if>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL != "WVGA">
            GraphicsState = PMDIN;
            while(PMMODEbits.BUSY == 1);
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if></#if>
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_VSYNCOff();
<#else>
            BSP_LCD_VSYNCOn();
</#if>
            //Setup DMA Back Porch
            remaining = HBackPorch;
            GraphicsState = ACTIVE_PERIOD;   
            line++;

            break;
		}
        default:
		{
            break;
		}
    }

    //Setup DMA Transfer
    PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t) (remaining << 1));
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    SYS_DMA_ChannelEnable(dmaHandle);
}
  
