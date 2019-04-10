/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Header File

  File Name:
    drv_gfx_lcc_generic.h

  Summary:
    Build-time generated header file
	Interface for the graphics library where the primitives are rendered and 
	sent to the graphics controller either external or internal

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Low-Cost
    Controllerless (LCC) Graphics Controller.

    Created with MPLAB Harmony Version 2.01
*******************************************************************************/
//DOM-IGNORE-BEGIN
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

#ifndef _DRV_GFX_LCC_H
#define _DRV_GFX_LCC_H

#include "driver/gfx/controller/drv_gfx_controller.h"
#include "system/dma/sys_dma.h"
#include "system/int/sys_int.h"

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef DRV_GFX_USE_LCC_PIP        
#define PIP_BUFFER  (3)
#endif

// function pointer definition for VSYNC callback
typedef void (*LCC_VSYNC_Callback_FnPtr)(void);

typedef enum
{
	LCC_VSYNC_TRUE = 0,
	LCC_VSYNC_FALSE
} LCC_VSYNC_STATE;
 
typedef enum
{
    /* */
    ACTIVE_PERIOD = 0,

    /* */
    BLANKING_PERIOD,
    
    /* */
    FINISH_LINE,
    
    /* */
    OVERFLOW,

    /* */
    PIP,

    /* */
    SCROLL,

} DMA_ISR_TASK;

//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:
    Number of valid LCC driver indices.

  Description:
    This constant identifies LCC driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is device-specific.
*/

#define DRV_GFX_LCC_INDEX_COUNT     DRV_GFX_LCC_NUMBER_OF_MODULES

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:

  Description:

  Remarks:

 */
typedef enum
{
    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_NONE = 0,

    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_PMP,

    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_EBI,

} DRV_GFX_LCC_FB_WRITE_BUS_TYPE;

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:

  Description:

  Remarks:

*/
typedef enum
{
    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_NONE = 0,

    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_PMP,

    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_EBI,

} DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE;

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function: 

  Summary:
    returns address to the framebuffer.

  Description:
    none.

  Input:
    none.

  Output:
    
*/
#ifdef CONFIG_DRV_GFX_USE_LCC_PIP
void GFX_PRIM_SetPIPWindow( uint16_t left, uint16_t top, uint16_t hlength, uint16_t vlength, uint16_t pipx, uint16_t pipy);
#endif
// *****************************************************************************
/*
  Function: DRV_GFX_LCC_GetBuffer( void )

  Summary:
    returns address to the framebuffer.

  Description:
    none.

  Input:
    none.

  Output:
    address to the framebuffer.
*/
//DOM-IGNORE-START
#ifdef DRV_GFX_LCC_INTERNAL_MEMORY
//DOM-IGNORE-END

unsigned short * DRV_GFX_LCC_GetBuffer(void);

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

uint16_t DRV_GFX_LCC_FrameBufferAddressSet( void * address );
//DOM-IGNORE-START
#endif
//DOM-IGNORE-END

// *****************************************************************************
/*
  Function: SYS_MODULE_OBJ DRV_GFX_LCC_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit)
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
SYS_MODULE_OBJ DRV_GFX_LCC_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit);

/*********************************************************************
  Function:
     DRV_GFX_LCC_Open(uint8_t instance)

  Summary:
    opens an instance of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
DRV_HANDLE DRV_GFX_LCC_Open( const SYS_MODULE_INDEX index,
                                 const DRV_IO_INTENT intent );

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_Close( DRV_HANDLE handle )

  Summary:
    closes an instance of the graphics controller

  Description:
    none

  Input:
    instance of the driver

*/
void DRV_GFX_LCC_Close( DRV_HANDLE handle );

/*********************************************************************
  Function:
     DRV_GFX_INTEFACE DRV_GFX_LCC_InterfaceGet( DRV_HANDLE handle )

  Summary:
    Returns the API of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
void DRV_GFX_LCC_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface );

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
uint16_t DRV_GFX_LCC_MaxXGet();

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
uint16_t DRV_GFX_LCC_MaxYGet();

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_SetColor(uint8_t instance, GFX_COLOR color)

  Summary: Sets the color for the driver instance

  Description:
  
  Output: none

*/
void DRV_GFX_LCC_SetColor(GFX_COLOR color);

//DOM-IGNORE-START
uint16_t DRV_GFX_LCC_SetPage(uint8_t pageType,uint8_t page);
//DOM-IGNORE-END

uint16_t* DRV_GFX_LCC_AlphaBlendWindow(GFX_ALPHA_PARAMS* alphaParams, uint16_t width, uint16_t height, uint8_t alpha);

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_DisplayRefresh(void)

  Summary:
    LCD refresh handler

  Description:
    This routine is called  from the timer interrupt, resulting in a complete LCD
    update.

  Input:
    none

  Output:
    none
*/
void DRV_GFX_LCC_DisplayRefresh(void);

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
          1 - call not successful (LCC driver busy)
          0 - call successful
*/
void DRV_GFX_LCC_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);

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
          1 - call not successful (LCC driver busy)
          0 - call successful
*/
void DRV_GFX_LCC_PixelPut(uint16_t x, uint16_t y);

// *****************************************************************************
/*
  Function: void  DRV_GFX_LCC_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          *color - start of the array
	  x - x coordinate of the start point.
          y - y coordinate of the end point.
	  count - number of pixels
          lineCount - number of lines
  Output:
          NULL - call not successful (PMP driver busy)
          !NULL - address of the display driver queue command
*/
void DRV_GFX_LCC_PixelArrayPut(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount);

// *****************************************************************************
/*
  Function: uint16_t*  DRV_GFX_LCC_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    gets an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
          x - x coordinate of the start point.
          y - y coordinate of the end point.
          count - number of pixels
  Output:
         ignore
*/ 
uint16_t* DRV_GFX_LCC_PixelArrayGet(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count);

//DOM-IGNORE-BEGIN
#ifdef DRV_GFX_USE_LCC_PIP
#ifdef DRV_GFX_USE_LCC_LAYERS
uint16_t* DRV_GFX_LCC_Layer(uint8_t type, GFX_LAYER_PARAMS* layer);
#endif
#endif
//DOM-IGNORE-END

/*************************************************************************
  Function:
      void DRV_GFX_LCC_Tasks(void)

  Summary:
    Task machine that renders the driver calls for the graphics library it
    must be called periodically to output the contents of its circular
    buffer
  *************************************************************************/
void DRV_GFX_LCC_Tasks(SYS_MODULE_OBJ object);

/*************************************************************************
  Function:
      void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr)

  Summary:
    Sets a callback function pointer for notification of VSYNC state
  *************************************************************************/
void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr cb);

/*************************************************************************
  Function:
      int32_t DRV_GFX_LCC_VSYNC_GetState(void)

  Summary:
    Gets the state of the current VSYNC mode
  *************************************************************************/
LCC_VSYNC_STATE DRV_GFX_LCC_VSYNC_GetState(void);

#ifdef __cplusplus
    }
#endif
    
#endif
