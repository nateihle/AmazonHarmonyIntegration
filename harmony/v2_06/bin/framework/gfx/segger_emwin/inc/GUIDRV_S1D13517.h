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
File        : GUIDRV_S1D13517.h
Purpose     : Interface definition for GUIDRV_S1D13517 driver
---------------------------END-OF-HEADER------------------------------
*/

#ifndef GUIDRV_S1D13517_H
#define GUIDRV_S1D13517_H

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/*********************************************************************
*
*       Configuration structure
*/
typedef struct {
  //
  // Driver specific configuration items
  //
  U32 TransColorIndex;
} CONFIG_S1D13517;

/*********************************************************************
*
*       Display drivers
*/
//
// Addresses
//
extern const GUI_DEVICE_API GUIDRV_S1D13517_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OY_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OX_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OXY_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OS_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OSY_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OSX_16C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13517_OSXY_16C0_API;

//
// Macros to be used in configuration files
//
#if defined(WIN32) && !defined(LCD_SIMCONTROLLER)

  #define GUIDRV_S1D13517_16C0       &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OY_16C0    &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OX_16C0    &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OXY_16C0   &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OS_16C0    &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OSY_16C0   &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OSX_16C0   &GUIDRV_Win_API
  #define GUIDRV_S1D13517_OSXY_16C0  &GUIDRV_Win_API

#else

  #define GUIDRV_S1D13517_16C0       &GUIDRV_S1D13517_16C0_API
  #define GUIDRV_S1D13517_OY_16C0    &GUIDRV_S1D13517_OY_16C0_API
  #define GUIDRV_S1D13517_OX_16C0    &GUIDRV_S1D13517_OX_16C0_API
  #define GUIDRV_S1D13517_OXY_16C0   &GUIDRV_S1D13517_OXY_16C0_API
  #define GUIDRV_S1D13517_OS_16C0    &GUIDRV_S1D13517_OS_16C0_API
  #define GUIDRV_S1D13517_OSY_16C0   &GUIDRV_S1D13517_OSY_16C0_API
  #define GUIDRV_S1D13517_OSX_16C0   &GUIDRV_S1D13517_OSX_16C0_API
  #define GUIDRV_S1D13517_OSXY_16C0  &GUIDRV_S1D13517_OSXY_16C0_API

#endif

/*********************************************************************
*
*       Public routines
*/
#if defined(WIN32) && !defined(LCD_SIMCONTROLLER)
  #define GUIDRV_S1D13517_SetBus16(pDevice, pHW_API)
  #define GUIDRV_S1D13517_Config(pDevice, pConfig)
#else
  void GUIDRV_S1D13517_SetBus16(GUI_DEVICE * pDevice, GUI_PORT_API * pHW_API);
  void GUIDRV_S1D13517_Config  (GUI_DEVICE * pDevice, CONFIG_S1D13517 * pConfig);
#endif

#if defined(__cplusplus)
}
#endif

#endif

/*************************** End of file ****************************/
