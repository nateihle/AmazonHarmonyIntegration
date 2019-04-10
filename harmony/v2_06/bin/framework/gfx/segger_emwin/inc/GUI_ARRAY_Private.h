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
File        : GUI_ARRAY_Private.h
Purpose     : Private array handling routines, should be used only
              from within GUI_ARRAY... routines!
---------------------------END-OF-HEADER------------------------------
*/

#ifndef GUI_ARRAY_PRIVATE_H
#define GUI_ARRAY_PRIVATE_H

#include "GUI_ARRAY.h"

#if GUI_WINSUPPORT

/*********************************************************************
*
*       Private types
*
**********************************************************************
*/
typedef struct {
  U16 NumItems;
  WM_HMEM haHandle;   /* Handle to buffer holding handles */
} GUI_ARRAY_OBJ;

/*********************************************************************
*
*       Private functions
*
**********************************************************************
*/
WM_HMEM GUI_ARRAY__GethItem      (const GUI_ARRAY_OBJ * pThis, unsigned int Index);
void  * GUI_ARRAY__GetpItem      (const GUI_ARRAY_OBJ * pThis, unsigned int Index);
void  * GUI_ARRAY__GetpItemLocked(const GUI_ARRAY_OBJ * pThis, unsigned int Index);
int     GUI_ARRAY__SethItem      (      GUI_ARRAY_OBJ * pThis, unsigned int Index, WM_HMEM hItem);

#endif /* GUI_WINSUPPORT */

#endif /* GUI_ARRAY_PRIVATE_H */

/*************************** End of file ****************************/
