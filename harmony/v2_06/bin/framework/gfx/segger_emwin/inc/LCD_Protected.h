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

** emWin V5.44 - Graphical user interface for embedded applications **
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
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Microchip Technology Inc., 2355 W Chandler Blvd., Chandler, AZ 85224, US
Licensed SEGGER software: emWin
License number:           GUI-00614
License model:            CPU Object Code License, dated Sept. 8, 2015
Licensed product:         Any
Licensed platform:        PIC24, PIC32, dsPIC, PIC16, PIC18 / MPLAB X Integrated Development Evironment, XC16 C Compiler, XC32 C/C++ Compiler
Licensed number of seats: -
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2015-09-28 - 2019-09-29
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCD_Protected.h
Purpose     : LCD level - To be used only internally by the GUI
----------------------------------------------------------------------
*/

#ifndef LCD_PROTECTED_H
#define LCD_PROTECTED_H

#include "LCD.h"

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/*********************************************************************
*
*       Data types
*/
typedef struct {
  LCD_COLOR * paColor;
  I16         NumEntries;
} LCD_LUT_INFO;

typedef struct {
  tLCDDEV_DrawPixel  * pfDrawPixel;
  tLCDDEV_DrawHLine  * pfDrawHLine;
  tLCDDEV_DrawVLine  * pfDrawVLine;
  tLCDDEV_FillRect   * pfFillRect;
  tLCDDEV_DrawBitmap * pfDrawBitmap;
} LCD_API_LIST;

/*********************************************************************
*
*       External data
*/
extern GUI_CONST_STORAGE U8 LCD_aMirror[256];
extern LCD_PIXELINDEX * LCD__aConvTable;

/*********************************************************************
*
*       Misc functions
*/
void LCD_UpdateColorIndices   (void);
int  LCD_PassingBitmapsAllowed(void);
void LCD_EnableCursor         (int OnOff);
void LCD_SelectLCD            (void);

void LCD_DrawBitmap(int x0,    int y0,
                    int xsize, int ysize,
                    int xMul,  int yMul,
                    int BitsPerPixel,
                    int BytesPerLine,
                    const U8 * pPixel,
                    const LCD_PIXELINDEX * pTrans);

void LCD__DrawBitmap_1bpp(int x0,    int y0,
                          int xsize, int ysize,
                          int xMul,  int yMul,
                          int BitsPerPixel,
                          int BytesPerLine,
                          const U8 * pPixel,
                          const LCD_PIXELINDEX * pTrans,
                          int OffData);

/*********************************************************************
*
*       Internal used color conversion routines
*/
tLCDDEV_Index2Color LCD_Index2Color_444_12;
tLCDDEV_Index2Color LCD_Index2Color_M444_12;
tLCDDEV_Index2Color LCD_Index2Color_444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_M444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_444_16;
tLCDDEV_Index2Color LCD_Index2Color_M444_16;
tLCDDEV_Index2Color LCD_Index2Color_555;
tLCDDEV_Index2Color LCD_Index2Color_565;
tLCDDEV_Index2Color LCD_Index2Color_8666;
tLCDDEV_Index2Color LCD_Index2Color_888;
tLCDDEV_Index2Color LCD_Index2Color_8888;
tLCDDEV_Index2Color LCD_Index2Color_M8888I;
tLCDDEV_Index2Color LCD_Index2Color_M555;
tLCDDEV_Index2Color LCD_Index2Color_M565;
tLCDDEV_Index2Color LCD_Index2Color_M888;

tLCDDEV_Color2Index LCD_Color2Index_8666;

#if defined(__cplusplus)
}
#endif

#endif /* LCD_PROTECTED_H */

/*************************** End of file ****************************/

