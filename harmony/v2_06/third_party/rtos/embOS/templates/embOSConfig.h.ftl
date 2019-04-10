/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2015 SEGGER Microcontroller GmbH & Co. KG         *
*                                                                    *
*     Internet: segger.com   Support: support_embos@segger.com       *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system for microcontrollers      *
*                                                                    *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product or a real-time            *
*       operating system for in-house use.                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       OS version: 4.12b                                            *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : RTOS.h
Purpose : Include file for the OS,
          to be included in every C-module accessing OS-routines

NOTE    : NONE OF THE DEFINITIONS IN THIS FILE MAY BE MODIFIED
          as long as embOS libraries are used.
          Any modification, direct or indirect, may cause malfunction.
          Modifications can only be done when the libraries are
          recompiled or the embOS sources are used in the project.
          
--------  END-OF-HEADER  ---------------------------------------------
*/

#ifndef EMBOS_CONFIG_H_INCLUDED        /* Avoid multiple inclusion          */
#define EMBOS_CONFIG_H_INCLUDED


/*********************************************************************
*
*     Includes
*/

/********************************************************************/

#define OS_FSYS  ( ${CONFIG_EMBOS_OS_FSYS?number?c}UL )
#define OS_TICK_FREQ ( ${CONFIG_EMBOS_OS_TICK_FREQ}UL ) 

#endif /* RTOS_H_INCLUDED */

/****** End Of File *************************************************/
