/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Implementation File

  File Name:
    drv_gfx_lcc_generic.c

  Summary:
    Build-time generated implementation for the LCC Driver.

  Description:
    Build-time generated implementation for the LCC Driver.

    Created with MPLAB Harmony Version 2.01
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

#include "framework/driver/gfx/controller/lcc/drv_gfx_lcc_generic.h"

#include <xc.h>
#include <sys/attribs.h>

#include "peripheral/pmp/plib_pmp.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/ebi/plib_ebi.h"


#define VER_BLANK                 (DISP_VER_PULSE_WIDTH+DISP_VER_BACK_PORCH+DISP_VER_FRONT_PORCH-1)


static SYS_DMA_CHANNEL_HANDLE dmaHandle = SYS_DMA_CHANNEL_HANDLE_INVALID;

#define EBI_ADDR_CS0   0xC0000000
#define PMP_ADDRESS_LINES 0
#define PAGE_COUNT 1
#define PMP_DATA_LENGTH PMP_DATA_SIZE_16_BITS
uint16_t HBackPorch = (DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH);


volatile GFX_COLOR *activePageAddress;
volatile GFX_COLOR *frameBufferAddress; 

volatile bool allowFrameUpdate = false;
volatile bool allowLineUpdate = false;

// *****************************************************************************
/* GFX LCC Driver Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance.

  Description:
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _DRV_GFX_LCC_OBJ
{
    /* Flag to indicate in use  */
    bool                                        inUse;

    /* Save the index of the driver */
    SYS_MODULE_INDEX                            drvIndex;

    /* LCC machine state */
    DRV_GFX_STATES                              state;

    /* Status of this driver instance */
    SYS_STATUS                                  status;

    /* Number of clients */
    uint32_t                                    nClients;

    /* Client of this driver */
    DRV_GFX_CLIENT_OBJ *                        pDrvLCCClientObj;

    DRV_GFX_INIT *                              initData;

    uint16_t      maxY;
    uint16_t      maxX;
	
	LCC_VSYNC_Callback_FnPtr                    vsync_cb;

} DRV_GFX_LCC_OBJ;

static DRV_GFX_LCC_OBJ        drvLCCObj;
static DRV_GFX_CLIENT_OBJ drvLCCClients;

// *****************************************************************************
/*
  Function: DRV_GFX_LCC_GetBuffer( void )

  Summary:
    returns address to the framebuffer

  Description:
    none

  Input:
    none

  Output:
    address to the framebuffer
*/

unsigned short * DRV_GFX_LCC_GetBuffer(void) {
    return (unsigned short *)frameBufferAddress;
}

// *****************************************************************************
/*
  Function: DRV_GFX_LCC_FrameBufferAddressSet( void * address )

  Summary:
    Sets address of the framebuffer

  Description:
    none

  Input:
    none

  Output:
    Sets address of the framebuffer
*/

uint16_t DRV_GFX_LCC_FrameBufferAddressSet( void * address )
{

    if ( NULL == address )
    {
        return 1;
    }

    frameBufferAddress = (GFX_COLOR *) address;

    return 0;
}

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_LCC_Initialize(uint8_t instance)

  Summary:
    resets LCD, initializes PMP

  Description:
    none

  Input:
        instance - driver instance
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
SYS_MODULE_OBJ DRV_GFX_LCC_Initialize(const SYS_MODULE_INDEX index,
                                      const SYS_MODULE_INIT* const init)
{
    static uint16_t horizontalSize, verticalSize;

    /* Validate the driver index */
    if ( index >= GFX_CONFIG_NUMBER_OF_MODULES )
        return SYS_MODULE_OBJ_INVALID;

    DRV_GFX_LCC_OBJ *dObj = &drvLCCObj;

    /* Object is valid, set it in use */
    dObj->inUse = true;
    dObj->state = SYS_STATUS_BUSY;
    dObj->initData = (DRV_GFX_INIT *) init;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    dObj->drvIndex = index;
    drvLCCObj.initData->TCON_Init = TCON_MODULE;

    if ((drvLCCObj.initData->orientation == 90) || (drvLCCObj.initData->orientation == 270))
	{
        horizontalSize = drvLCCObj.initData->verticalResolution;
        verticalSize = drvLCCObj.initData->horizontalResolution;
        drvLCCObj.maxX = horizontalSize - 1;
        drvLCCObj.maxY = verticalSize - 1;
    }
	else
	{
        horizontalSize = drvLCCObj.initData->horizontalResolution;
        verticalSize = drvLCCObj.initData->verticalResolution;
        drvLCCObj.maxX = horizontalSize - 1;
        drvLCCObj.maxY = verticalSize - 1;
    }

    BSP_LCD_RESETOn();
    BSP_LCD_CSOn();

    if (drvLCCObj.initData->TCON_Init != NULL)
		drvLCCObj.initData->TCON_Init();

    drvLCCObj.initData->activePage = 0;
    drvLCCObj.initData->visualPage = 0;

    activePageAddress  = KVA0_TO_KVA1(frameBufferAddress);
    PLIB_EBI_WriteOutputControlSet(EBI_ID_0, true, false);
    PLIB_EBI_DataEnableSet(EBI_ID_0, true, true);
    
    PLIB_EBI_BaseAddressSet(EBI_ID_0, 0, 0x20000000);
    PLIB_EBI_MemoryCharacteristicsSet(EBI_ID_0, 0, SRAM, MEMORY_SIZE_8MB, CS_TIMING_0);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 0, 0, 0, 4, 3, 3, 0);

    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 0, MEMORY_WIDTH_16BIT);
    PLIB_EBI_FlashPowerDownModeSet(EBI_ID_0, true);

    /*Turn Backlight on*/
    BSP_LCD_BACKLIGHTOn();

    drvLCCObj.initData->horizontalResolution = horizontalSize;
    drvLCCObj.initData->verticalResolution = verticalSize;
    drvLCCObj.state = SYS_STATUS_READY;

    dObj->nClients = 0;
    dObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return (SYS_MODULE_OBJ)dObj;
}

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
    SYS_DMA_ChannelTransferAdd( dmaHandle,
                               (void *)frameBufferAddress,
                               (HBackPorch << 1),
                               (uint32_t*) EBI_ADDR_CS0,
                               (1 << PMP_DATA_LENGTH),
                               (HBackPorch << 1));

    // once we configured the DMA channel we can enable it
    SYS_DMA_ChannelEnable( dmaHandle );

    SYS_DMA_ChannelForceStart( dmaHandle );

    /* Enable the transfer done interrupt, when all buffer transferred */
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX,
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

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr cb)

  Summary:
    sets VSYNC state callback pointer

  Description:
    none

  Input:
        cb - callback pointer
*/
void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr cb)
{
	drvLCCObj.vsync_cb = cb;
}

// *****************************************************************************
/*
  Function: LCC_VSYNC_STATE DRV_GFX_LCC_VSYNC_GetState(void)

  Summary:
    Gets the state of the current VSYNC mode

  Description:
    none

  Output:
        state of VSYNC register
		LCC_VSYNC_TRUE = in VSYNC
		LCC_VSYNC_FALSE = not in VSYNC
*/
LCC_VSYNC_STATE DRV_GFX_LCC_VSYNC_GetState(void)
{
	return BSP_LCD_VSYNCStateGet();
}

// *****************************************************************************
/*
  Function: DRV_GFX_LCC_Open(uint8_t instance)

  Summary:
    opens an instance of the graphics controller

  Description:
    none

  Input:
    instance of the driver

  Output:
    1 - driver not initialied
    2 - instance doesn't exist
    3 - instance already open
    instance to driver when successful
*/
DRV_GFX_HANDLE DRV_GFX_LCC_Open( const SYS_MODULE_INDEX index,
                             const DRV_IO_INTENT intent )
{
    DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)DRV_HANDLE_INVALID;

    /* Check if the specified driver index is in valid range */
    if(index >= GFX_CONFIG_NUMBER_OF_MODULES)
    {
    }
    /* Check if instance object is ready*/
    else if(drvLCCObj.status != SYS_STATUS_READY)
    {
    }
    else if(intent != DRV_IO_INTENT_EXCLUSIVE)
    {
    }
    else if(drvLCCObj.nClients > 0)
    {
    }
    else
    {
		if(DRV_GFX_LCC_Start() != 0)
            return DRV_GFX_HANDLE_INVALID; // currently single context only
			
        client = &drvLCCClients;

        client->inUse = true;
        client->drvObj = &drvLCCObj;

        /* Increment the client number for the specific driver instance*/
        drvLCCObj.nClients++;
    }

    /* Return invalid handle */
    return ((DRV_HANDLE)client);
}

// *****************************************************************************
/* Function:
    void DRV_GFX_LCC_Close( DRV_HANDLE handle )

  Summary:
    closes an instance of the graphics controller

  Description:
    This is closes the instance of the driver specified by handle.
*/
void DRV_GFX_LCC_Close( DRV_HANDLE handle )
{
    /* Start of local variable */
    DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)NULL;
    DRV_GFX_LCC_OBJ * drvObj = (DRV_GFX_LCC_OBJ *)NULL;
    /* End of local variable */

    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
//        SYS_DEBUG(0, "Bad Client Handle");
    }
    else
    {
        client = (DRV_GFX_CLIENT_OBJ *)handle;

        if(client->inUse)
        {
            client->inUse = false;
            drvObj = (DRV_GFX_LCC_OBJ *)client->drvObj;

            /* Remove this client from the driver client table */
            drvObj->nClients--;
        }
        else
        {
//            SYS_DEBUG(0, "Client Handle no inuse");
        }

    }
    return;
}

// *****************************************************************************
/*
  Function:
     void DRV_GFX_LCC_MaxXGet()

  Summary:
     Returns x extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_LCC_MaxXGet()
{
    return drvLCCObj.maxX;
}

// *****************************************************************************
/*
  Function:
     void DRV_GFX_LCC_MaxYGet()

  Summary:
     Returns y extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_LCC_MaxYGet()
{
    return drvLCCObj.maxY;
}

/*********************************************************************
  Function:
     void DRV_GFX_LCC_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )

  Summary:
    Returns the API of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
void DRV_GFX_LCC_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )
{
    interface->BarFill = DRV_GFX_LCC_BarFill;
    interface->PixelArrayPut = DRV_GFX_LCC_PixelArrayPut;
    interface->PixelArrayGet = DRV_GFX_LCC_PixelArrayGet;
    interface->PixelPut = DRV_GFX_LCC_PixelPut;
    interface->ColorSet = DRV_GFX_LCC_SetColor;
    interface->PageSet = DRV_GFX_LCC_SetPage;
    interface->MaxXGet = DRV_GFX_LCC_MaxXGet;
    interface->MaxYGet = DRV_GFX_LCC_MaxYGet;
}

uint16_t DRV_GFX_LCC_SetPage(uint8_t pageType, uint8_t page)
{

        switch(pageType)
        {
            case ACTIVE_PAGE:
                drvLCCObj.initData->activePage = page;
                activePageAddress = KVA0_TO_KVA1(frameBufferAddress);
                break;
            case VISUAL_PAGE:
                drvLCCObj.initData->visualPage = page;
	            BSP_SRAM_A17StateSet(page&0x1);
	            BSP_SRAM_A18StateSet(page>>1&0x1);
                break;
        }

        return(0);
}
// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_SetColor(uint8_t instance, GFX_COLOR color)

  Summary: Sets the color for the driver instance

  Description:
  
  Output: none

*/

void DRV_GFX_LCC_SetColor(GFX_COLOR color)
{
  drvLCCObj.initData->color = color;
}

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_DisplayRefresh(void)

  Summary: State Machine driving H-Sync, V-Sync and DE timing

  Description:
  
  Output: none

*/

void DRV_GFX_LCC_DisplayRefresh(void)
{
    static uint8_t GraphicsState = ACTIVE_PERIOD;
    static uint16_t remaining = 0;
    static short line = 0;
    uint32_t offset = 0;

    switch(GraphicsState)
    {
        case ACTIVE_PERIOD:
            remaining = DISP_HOR_RESOLUTION << PMP_DATA_LENGTH;
            GraphicsState = BLANKING_PERIOD;

            BSP_LCD_HSYNCOn();
            if(line >= 0)
            {
                offset = drvLCCObj.initData->visualPage *
                         DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION +
                         line * DISP_HOR_RESOLUTION;

                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset] );
                if(line == (DISP_VER_RESOLUTION))
                {
					// notify registered callback that the display is in vsync mode
					if(drvLCCObj.vsync_cb != NULL)
						drvLCCObj.vsync_cb();
						
                    BSP_LCD_VSYNCOff();
                    line = -(VER_BLANK);
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
            
        case BLANKING_PERIOD:
            BSP_LCD_HSYNCOff();
            BSP_LCD_DEOff();
            PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch << PMP_DATA_LENGTH));
            GraphicsState = ACTIVE_PERIOD;
            line++;

            PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch<<1));
        }

    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
   
    SYS_DMA_ChannelForceStart(dmaHandle);
}
  
// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_PixelPut(uint16_t x, uint16_t y)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        x,y - pixel coordinates
  Output:
    1 - call not successful (lcc driver busy)
    0 - call successful
*/
void DRV_GFX_LCC_PixelPut(uint16_t x, uint16_t y)
{
    GFX_COLOR *point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);
    *point = drvLCCObj.initData->color;
}  

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        left,top - pixel coordinates
        right, bottom - pixel coordinates

  Output:
          1 - call not successful (lcc driver busy)
          0 - call successful
*/
void  DRV_GFX_LCC_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
    static GFX_COLOR *point;
    uint16_t tempy = top;
    uint16_t tempx = left;

    for(tempy = top; tempy <= bottom; tempy++)
    {
        left = tempx;
        top = tempy;
        point = (GFX_COLOR*)activePageAddress + left + top *(DISP_HOR_RESOLUTION);

        for(left = tempx; left <= right; left++)
        {
            *point = drvLCCObj.initData->color;
            point++;
        }
	}
}  

// *****************************************************************************
/*
  Function: void  DRV_GFX_LCC_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         handle to the number of pixels remaining
*/
void  DRV_GFX_LCC_PixelArrayPut(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount)
{
    uint16_t tempy = y;
    uint16_t tempx = x;
    uint16_t tempCount = count;
    GFX_COLOR *point;

    while(lineCount--)
    {
		x = tempx;
		y = tempy;
		point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);

		while(tempCount--)
		{

			*point = *color++;
            point++;
        }

		tempCount = count;
		tempy++;
	}
} 

// *****************************************************************************
/*
  Function: uint16_t*  DRV_GFX_LCC_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         handle to the number of pixels remaining
*/
uint16_t*  DRV_GFX_LCC_PixelArrayGet(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count)
{
	GFX_COLOR *point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);

	while(count--)
	{
        *color = *point;
        color++;
        point++;
    }
    return 0;
}



// *****************************************************************************
/* Function:
    SYS_STATUS DRV_GFX_LCC_Status ( SYS_MODULE_OBJ object )

  Summary:
    Returns status of the specific module instance of the Driver module.

  Description:
    This function returns the status of the specific module instance disabling its
    operation (and any hardware for driver modules).

  PreCondition:
    The DRV_GFX_LCC_Initialize function should have been called before calling
    this function.

  Parameters:
    object          - DRV_GFX_LCC object handle, returned from DRV_GFX_LCC_Initialize

  Returns:
    SYS_STATUS_READY    Indicates that any previous module operation for the
                        specified module has completed

    SYS_STATUS_BUSY     Indicates that a previous module operation for the
                        specified module has not yet completed

    SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/

SYS_STATUS DRV_GFX_LCC_Status ( SYS_MODULE_OBJ object )
{
    DRV_GFX_LCC_OBJ *dObj = (DRV_GFX_LCC_OBJ*)object;
    return ( dObj->state );

} /* SYS_TMR_Status */

