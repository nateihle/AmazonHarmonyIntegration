/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Implementation File

  File Name:
    drv_gfx_lcc_generic.c

  Summary:
    Build-time generated implementation for the LCC Driver.

  Description:
    Build-time generated implementation for the LCC Driver.

    Created with MPLAB Harmony Version 2.02
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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
*******************************************************************************/
// DOM-IGNORE-END

#include "framework/gfx/driver/controller/lcc/drv_gfx_lcc_generic.h"

#include <xc.h>
#include <sys/attribs.h>

#include "peripheral/pmp/plib_pmp.h"
#include "peripheral/tmr/plib_tmr.h"


static SYS_DMA_CHANNEL_HANDLE dmaHandle = SYS_DMA_CHANNEL_HANDLE_INVALID;

#define EBI_ADDR_CS0   0xC0000000

#define PMP_DATA_LENGTH PMP_DATA_SIZE_16_BITS 

#define MAX_LAYER_COUNT 1
#define BUFFER_COUNT    1
#define DISPLAY_WIDTH   172
#define DISPLAY_HEIGHT  220


const char* DRIVER_NAME = "LCC";
static uint32_t supported_color_formats = GFX_COLOR_MASK_RGB_565;

uint32_t state;

uint16_t __attribute__((coherent, aligned(16))) frameBuffer[BUFFER_COUNT][DISPLAY_WIDTH * DISPLAY_HEIGHT];

#define DRV_GFX_LCC_DMA_CHANNEL_INDEX DMA_CHANNEL_1
#define DRV_GFX_LCC_DMA_TRIGGER_SOURCE DMA_TRIGGER_TIMER_2


/**** Hardware Abstraction Interfaces ****/
enum
{
    INIT = 0,
    RUN
};

static int DRV_GFX_LCC_Start();

GFX_Context* cntxt;

uint16_t HBackPorch;
uint32_t VER_BLANK;

uint32_t DISP_HOR_FRONT_PORCH;
uint32_t DISP_HOR_RESOLUTION;
uint32_t DISP_HOR_BACK_PORCH;
uint32_t DISP_HOR_PULSE_WIDTH;

uint32_t DISP_VER_FRONT_PORCH;
uint32_t DISP_VER_RESOLUTION;
uint32_t DISP_VER_BACK_PORCH;
uint32_t DISP_VER_PULSE_WIDTH;

int16_t line = 0;
uint32_t offset = 0;
uint16_t pixels = 0;
uint32_t hSyncs = 0;
    
uint32_t vsyncPeriod = 0;
uint32_t vsyncPulseDown = 0;
uint32_t vsyncPulseUp = 0;
uint32_t vsyncEnd = 0;

volatile bool allowFrameUpdate = false;
volatile bool allowLineUpdate = false;

// function that returns the information for this driver
GFX_Result driverLCCInfoGet(GFX_DriverInfo* info)
{
	if(info == NULL)
        return GFX_FAILURE;

	// populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supported_color_formats;
    info->layer_count = MAX_LAYER_COUNT;
    
    return GFX_SUCCESS;
}

static GFX_Result lccUpdate()
{
    GFX_Context* context = GFX_ActiveContext();
   
    if(context == NULL)
        return GFX_FAILURE;
    
    if(state == INIT)
    {
        if(DRV_GFX_LCC_Start() != 0)
            return GFX_FAILURE;
        
        state = RUN;
    }
    
    return GFX_SUCCESS;
}

static void lccDestroy(GFX_Context* context)
{	
	// driver specific shutdown tasks
	if(context->driver_data != GFX_NULL)
	{
		context->memory.free(context->driver_data);
		context->driver_data = GFX_NULL;
	}
	
	// general default shutdown
	defDestroy(context);
}

static GFX_Result layerBufferCountSet(uint32_t count)
{
    count = count;
        
    return GFX_FAILURE;
}

static GFX_Result layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
    idx = 0;
    address = address;
    
    return GFX_FAILURE;
}

static GFX_Result layerBufferAllocate(uint32_t idx)
{
    idx = 0;
    
    return GFX_FAILURE;
}

static GFX_Result lccInitialize(GFX_Context* context)
{
	uint32_t i, j;
	
	cntxt = context;
	
	// general default initialization
	if(defInitialize(context) == GFX_FAILURE)
		return GFX_FAILURE;	
		
	// override default HAL functions with LCC specific implementations
    context->hal.update = &lccUpdate;
    context->hal.destroy = &lccDestroy;
	context->hal.layerBufferCountSet = &layerBufferCountSet;
    context->hal.layerBufferAddressSet = &layerBufferAddressSet;
    context->hal.layerBufferAllocate = &layerBufferAllocate;
	
	// driver specific initialization tasks	
	// initialize all layer color modes
    for(i = 0; i < MAX_LAYER_COUNT; i++)
    {        
        for(j = 0; j < BUFFER_COUNT; j++)
        {
            GFX_PixelBufferCreate(DISPLAY_WIDTH,
                                  DISPLAY_HEIGHT,
                                  GFX_COLOR_MODE_RGB_565,
                                  frameBuffer[j],
                                  &context->layer.layers[i].buffers[j].pb);
            
            context->layer.layers[i].buffers[j].state = GFX_BS_MANAGED;
        }
	}
	
	VER_BLANK = context->display_info->attributes.vert.pulse_width +
	            context->display_info->attributes.vert.back_porch +
				context->display_info->attributes.vert.front_porch - 1;
	
	HBackPorch = context->display_info->attributes.horz.pulse_width +
	             context->display_info->attributes.horz.back_porch;
	
	DISP_HOR_FRONT_PORCH = context->display_info->attributes.horz.front_porch;
	DISP_HOR_RESOLUTION = DISPLAY_WIDTH;
	DISP_HOR_BACK_PORCH = context->display_info->attributes.horz.back_porch;
	DISP_HOR_PULSE_WIDTH = context->display_info->attributes.horz.pulse_width;
	
	DISP_VER_FRONT_PORCH = context->display_info->attributes.vert.front_porch;
	DISP_VER_RESOLUTION = DISPLAY_HEIGHT;
	DISP_VER_BACK_PORCH = context->display_info->attributes.vert.back_porch;
	DISP_VER_PULSE_WIDTH = context->display_info->attributes.vert.pulse_width;



    BSP_LCD_RESETOn();
    BSP_LCD_CSOn();

    /* Disable the PMP module */
    PLIB_PMP_Disable(0);

    PLIB_PMP_OperationModeSelect(0, PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT);

    /* pins polarity setting */
    PLIB_PMP_ReadWriteStrobePolaritySelect(0, 1 - ((cntxt->display_info->attributes.inv_left_shift)));
    PLIB_PMP_WriteEnableStrobePolaritySelect(0, PMP_POLARITY_ACTIVE_LOW);

    PLIB_PMP_ReadWriteStrobePortEnable(0);
    PLIB_PMP_WriteEnableStrobePortEnable(0);

    PLIB_PMP_DataSizeSelect(0, PMP_DATA_LENGTH);

    /* wait states setting */
    PLIB_PMP_WaitStatesDataHoldSelect(0, PMP_DATA_HOLD_1);
    PLIB_PMP_WaitStatesDataSetUpSelect(0, PMP_DATA_WAIT_ONE);
    PLIB_PMP_WaitStatesStrobeSelect(0, PMP_STROBE_WAIT_3);

    /* setting the hardware for the required interrupt mode */
    PLIB_PMP_InterruptModeSelect(0, PMP_INTERRUPT_EVERY_RW_CYCLE);

    PLIB_PMP_AddressIncrementModeSelect(0, PMP_ADDRESS_AUTO_INCREMENT);

    /* Enable the PMP module */
    PLIB_PMP_Enable(0);

    /*Turn Backlight on*/
    BSP_LCD_BACKLIGHTOn();
	
	return GFX_SUCCESS;
}

// function that initialized the driver context
GFX_Result driverLCCContextInitialize(GFX_Context* context)
{
	// set driver-specific data initialization function address
	context->hal.initialize = &lccInitialize; 
	
	// set driver-specific destroy function address
    context->hal.destroy = &lccDestroy;
	
	/* dynamic frame buffer allocation - not working at the moment
	// allocate space for driver-specific data
	//data = context->memory.calloc(1, sizeof(LCCDriverData));
	
	//if(data == GFX_NULL)
	//	return GFX_FAILURE;
	*/
	
	return GFX_SUCCESS;
}

/**** End Hardware Abstraction Interfaces ****/

static int DRV_GFX_LCC_Start()
{
	/*Suspend DMA Module*/
    SYS_DMA_Suspend();
	
    /* Allocate DMA channel */
    dmaHandle = SYS_DMA_ChannelAllocate(DRV_GFX_LCC_DMA_CHANNEL_INDEX);
    
	if (SYS_DMA_CHANNEL_HANDLE_INVALID == dmaHandle)
        return -1;
		
    // set the transfer parameters: source & destination address,
    // source & destination size, number of bytes per event
    SYS_DMA_ChannelTransferAdd(dmaHandle,
							   cntxt->layer.active->buffers[cntxt->layer.active->buffer_read_idx].pb.pixels,
                               (HBackPorch << 1),
                               (uint32_t*) EBI_ADDR_CS0,
                               (1 << PMP_DATA_LENGTH),
                               (HBackPorch << 1));

    // once we configured the DMA channel we can enable it
    SYS_DMA_ChannelEnable( dmaHandle );

    SYS_DMA_ChannelForceStart( dmaHandle );

	SYS_INT_VectorPrioritySet(INT_VECTOR_DMA1, INT_PRIORITY_LEVEL7);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA1, INT_SUBPRIORITY_LEVEL0);

    /* Enable the transfer done interrupt, when all buffer transferred */
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0,
	                                 DRV_GFX_LCC_DMA_CHANNEL_INDEX,
                                     DMA_INT_BLOCK_TRANSFER_COMPLETE);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0 + DRV_GFX_LCC_DMA_CHANNEL_INDEX);
	
    PLIB_DMA_ChannelXAutoEnable(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX);

    // set the transfer event control: what event is to start the DMA transfer
    SYS_DMA_ChannelSetup(dmaHandle,
                         SYS_DMA_CHANNEL_OP_MODE_BASIC,
                         DRV_GFX_LCC_DMA_TRIGGER_SOURCE);

    /*Unsuspend DMA Module*/
    SYS_DMA_Resume();
	
	return 0;
}

void DRV_GFX_LCC_DisplayRefresh(void)
{
	GFX_Point drawPoint;
	GFX_PixelBuffer* buffer;

	static uint8_t GraphicsState = ACTIVE_PERIOD;
    static uint16_t remaining = 0;
    static short line = 0;

    switch(GraphicsState)
    {
        case ACTIVE_PERIOD:
		{
            remaining = DISP_HOR_RESOLUTION << PMP_DATA_LENGTH;
            GraphicsState = BLANKING_PERIOD;

            BSP_LCD_HSYNCOn();

            if(line >= 0)
            {
				drawPoint.x = 0;
				drawPoint.y = line;
				
				buffer = &cntxt->layer.active->buffers[cntxt->layer.active->buffer_read_idx].pb;

                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,
				                                       DRV_GFX_LCC_DMA_CHANNEL_INDEX,
													   (uint32_t)GFX_PixelBufferOffsetGet_Unsafe(buffer, &drawPoint));
                if(line == (DISP_VER_RESOLUTION))
                {
                    BSP_LCD_VSYNCOff();
                    line = -(VER_BLANK);
					
					if(cntxt->layer.active->vsync == GFX_TRUE
						&& cntxt->layer.active->swap == GFX_TRUE)
						GFX_LayerSwap(cntxt->layer.active);
                }
                else
                {
                    BSP_LCD_VSYNCOn();
                    BSP_LCD_DEOn();
                }
           }

           PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX,remaining);
           PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, remaining);
           break;
        }
        case BLANKING_PERIOD:
		{
            BSP_LCD_HSYNCOff();
            BSP_LCD_DEOff();
            PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch << PMP_DATA_LENGTH));
            GraphicsState = ACTIVE_PERIOD;
            line++;

            PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch<<1));
		}
    }

    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
   
    SYS_DMA_ChannelForceStart(dmaHandle);
}



void __ISR(_DMA0_VECTOR + DMA_CHANNEL_1, ipl7AUTO) _LCCRefreshISR(void)
{
    SYS_INT_SourceStatusClear(INT_SOURCE_DMA_0 + DMA_CHANNEL_1);
	
    DRV_GFX_LCC_DisplayRefresh();
}
