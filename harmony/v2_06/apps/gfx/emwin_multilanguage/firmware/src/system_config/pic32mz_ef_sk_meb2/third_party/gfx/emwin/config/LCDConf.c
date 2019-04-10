/*********************************************************************
*                 SEGGER Microcontroller Systems LLC                 *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2017  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.38 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to Microchip Technology Inc. for the
purposes  of  creating  libraries  for  16 -bit  PIC microcontrollers,
32-bit  PIC  microntrollers,  dsPIC  digital  signal  controllers  and
microcontrollers   with   part   name   prefix   "PIC16"  and  "PIC18"
commercialized and distributed by Microchip Technology Inc. as part of
the  MPLAB  Integrated  Development  Environment  under  the terms and
conditions  of  an  End  User  License  Agreement  supplied  with  the
libraries. Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include "GUI.h"
#include "system_config.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include "GUIDRV_Lin.h"

#define DRIVER_TYPE			GUIDRV_LIN_16
#define COLOR_CONVERSION	GUICC_M565



// *****************************************************************************
// *****************************************************************************
// Section: Layer Configuration 
// *****************************************************************************
// *****************************************************************************
/* TODO: Add any necessary Layer Configuration. 
*/

	static GFX_ColorMode colorModeConvertToGFX(void)
	{
		return  GFX_COLOR_MODE_RGB_565;
	}

/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*   
*/

void LCD_X_Config(void) 
{


	

		GFX_Buffer buffer = NULL;
		GFX_ColorMode LCDColorMode;	
		GUI_DEVICE_CreateAndLink( DRIVER_TYPE, COLOR_CONVERSION, 0, 0);
		
		
		
		if (LCD_GetSwapXY()) 
		{
			LCD_SetSizeEx (0, 272, 480);
			LCD_SetVSizeEx(0, 272, 480);
		} 
		else 
		{
			LCD_SetSizeEx (0, 480, 272);
			LCD_SetVSizeEx(0, 480, 272);
		}
		

		//Set the Active Layer to be Layer 0
		GFX_Set(GFXF_LAYER_ACTIVE, 0);
		//Enable the active layer
		GFX_Set(GFXF_LAYER_ENABLED, GFX_TRUE);
		//Set the default layer as one for DA GLCD
		GFX_Set(GFXF_LAYER_BUFFER_COUNT, 1);

		//Translate the color mode again segger -> GFX    
		LCDColorMode = colorModeConvertToGFX();
				
		//Overwrite with the segger relevant color mode
		GFX_Set(GFXF_COLOR_MODE, LCDColorMode);
		
		//Get the buffer pointer from the gfx layer (HAL)
		//And pass the frame buffer pointer to the segger library to write to.
		GFX_Get(GFXF_LAYER_BUFFER_ADDRESS, 0, &buffer);
		LCD_SetVRAMAddrEx( 0, ( void * )buffer );
		
    
    return;
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if 
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/

int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) 
{


   int retVal = -1;
    
    switch( Cmd )
    {
        case LCD_X_INITCONTROLLER:
        {
            retVal = 0;
            
            break;
        }
        
        default:
        {
            retVal = -1;
            
            break;
        }
    }
    
    return retVal;

}

/*******************************************************************************
 End of File
 */

