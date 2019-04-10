/*******************************************************************************
  PTG Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ptg_static.h

  Summary:
    PTG driver interface declarations for the static single instance driver.

  Description:
    The PTG device driver provides a simple interface to manage the PTG
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the PTG driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _DRV_PTG_STATIC_H
#define _DRV_PTG_STATIC_H

#include <stdbool.h>
#include "system_config.h"
#include "peripheral/ptg/plib_ptg.h"

// *****************************************************************************
// *****************************************************************************
// Section: PTG Driver Data Types
// *****************************************************************************
// *****************************************************************************

#define	_STEP0	0x00
#define	_STEP1	0x01
#define	_STEP2	0x02
#define	_STEP3	0x03
#define	_STEP4	0x04
#define	_STEP5	0x05
#define	_STEP6	0x06
#define	_STEP7	0x07
#define	_STEP8	0x08
#define	_STEP9	0x09
#define	_STEP10	0x0A
#define	_STEP11	0x0B
#define	_STEP12	0x0C
#define	_STEP13	0x0D
#define	_STEP14	0x0E
#define	_STEP15	0x0F
#define	_STEP16	0x10
#define	_STEP17	0x11
#define	_STEP18	0x12
#define	_STEP19	0x13
#define	_STEP20	0x14
#define	_STEP21	0x15
#define	_STEP22	0x16
#define	_STEP23	0x17
#define	_STEP24	0x18
#define	_STEP25	0x19
#define	_STEP26	0x1A
#define	_STEP27	0x1B
#define	_STEP28	0x1C
#define	_STEP29	0x1D
#define	_STEP30	0x1E
#define	_STEP31	0x1F

#define	PTGCTRL	    ((0x00) << 4)
#define	PTGADD_COPY	((0x01) << 4)
#define	PTGSTRB	    ((0x02) << 4)
#define	PTGWHI	    ((0x04) << 4)
#define	PTGWLO	    ((0x05) << 4)
#define	PTGIRQ	    ((0x07) << 4)
#define	PTGTRIG	    ((0x08) << 4)
#define	PTGJMP	    ((0x0A) << 4)
#define	PTGJMPC0	((0x0C) << 4)
#define	PTGJMPC1	((0x0E) << 4)

#define	ADD_ADJ_C1	(0x01)
#define	ADD_ADJ_T0	(0x02)
#define	ADD_ADJ_T1	(0x03)
#define	ADD_ADJ_SD	(0x04)
#define	ADD_ADJ_L0	(0x05)

#define	COPY_HLD_C0	(0x08)
#define	COPY_HLD_C1	(0x09)
#define	COPY_HLD_T0	(0x0A)
#define	COPY_HLD_T1	(0x0B)
#define	COPY_HLD_SD	(0x0C)
#define	COPY_HLD_L0	(0x0D)

#define WAIT_OCMP1         (0x00)
#define WAIT_OCMP2         (0x01)
#define WAIT_OCMP3         (0x02)
#define WAIT_OCMP4         (0x03)
#define WAIT_ICAP1         (0x04)
#define WAIT_ICAP2         (0x05)
#define WAIT_ICAP3         (0x06)
#define WAIT_ICAP4         (0x07)
#define WAIT_ADC_EVENT_99  (0x08)
#define WAIT_ADC_EVENT_100 (0x09)
#define WAIT_ADC_EVENT_101 (0x0A)

#define PTG_IRQ0 (0x00)
#define PTG_IRQ1 (0x01)
#define PTG_IRQ2 (0x02)
#define PTG_IRQ3 (0x03)

#define NOP    (0x00)
#define SDOFF  (0x02)
#define SDON   (0x06)
#define PTGT0  (0x08)
#define PTGT1  (0x09)
#define SWTRGL (0x0A)
#define SWTRGE (0x0B)
#define STRBC0 (0x0C)
#define STRBC1 (0x0D)
#define STRBL0 (0x0E)
#define BTRIG  (0x0F)

#define PTG_ADC_TRIG12 (0x0C)
#define PTG_ADC_TRIG13 (0x0D)
#define PTG_ADC_TRIG14 (0x0E)
#define PTG_ADC_TRIG15 (0x0F)
#define PTG_PPS_TRIG28 (0x1C)
#define PTG_PPS_TRIG29 (0x1D)
#define PTG_PPS_TRIG30 (0x1E)
#define PTG_PPS_TRIG31 (0x1F)

typedef enum
{
	/* Edge Trigger SWT bit in PTGCON Register (i.e., set SWT 0 -> 1) */
	DRV_PTG_SWT_EDGE_TRIG  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Level Trigger SWT bit in PTGCON Register (i.e., set SWT  to 1) */
	DRV_PTG_SWT_LEVEL_TRIG /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_SWT_TRIGGER_TYPE ;

typedef enum
{
	/* Configures output to pulse mode */
	DRV_PTG_OUTPUT_PULSE_MODE  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Configures output to toggle mode */
	DRV_PTG_OUTPUT_TOGGLE_MODE /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_OUTPUT_MODE;

typedef enum
{
	/* Disable Visibility of PTG timer/counter registers */
	DRV_PTG_IVIS_DISABLE  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Enable Visibility of PTG timer/counter registers */
	DRV_PTG_IVIS_ENABLE   /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_IVIS_MODE;

typedef enum
{
	/* Broadcast Mask Register */
	DRV_PTG_REG_BTE          /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Hold Register */
	DRV_PTG_REG_HOLD         /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,
	
	/* Timer 0 Register */
	DRV_PTG_REG_TIMER0       /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,
	
	/* Timer 1 Register */
	DRV_PTG_REG_TIMER1       /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/,
	
	/* Counter 0 Register */
	DRV_PTG_REG_COUNTER0     /*DOM-IGNORE-BEGIN*/ = 4 /*DOM-IGNORE-END*/,
	
	/* Counter 1 Register */
	DRV_PTG_REG_COUNTER1     /*DOM-IGNORE-BEGIN*/ = 5 /*DOM-IGNORE-END*/,
	
	/* Step Delay Register */
	DRV_PTG_REG_STEP_DELAY   /*DOM-IGNORE-BEGIN*/ = 6 /*DOM-IGNORE-END*/,
	
	/* Adjust Register */
	DRV_PTG_REG_ADJUST       /*DOM-IGNORE-BEGIN*/ = 7 /*DOM-IGNORE-END*/,
	
	/* Literal Strobe register */
	DRV_PTG_REG_LITERAL      /*DOM-IGNORE-BEGIN*/ = 8 /*DOM-IGNORE-END*/,
	
	/* Step Pointer Register */
	DRV_PTG_REG_STEP_POINTER /*DOM-IGNORE-BEGIN*/ = 9 /*DOM-IGNORE-END*/
	
}DRV_PTG_REG_SUBSET;

// *****************************************************************************
// *****************************************************************************
// Section: PTG Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

void DRV_PTG_Initialize( void );

void DRV_PTG_Deinitialize( void );

void DRV_PTG_InputModeSet ( PTG_INPUT_MODE_SEL inputMode );

PTG_INPUT_MODE_SEL DRV_PTG_InputModeGet ( void );

void DRV_PTG_PTGExecutionStart ( void ) ;

void DRV_PTG_PTGDisable ( void );

void DRV_PTG_PTGHalt ( void );

bool DRV_PTG_IsPTGBusy ( void );

void DRV_PTG_WDTConfigure ( PTG_WDT_TIMEOUT_SEL wdtTimeOutSel );

PTG_WDT_TIMEOUT_SEL DRV_PTG_WDTCurrentConfigCheck ( void );

void DRV_PTG_WDTDisable ( void );

void DRV_PTG_SWTTRIGGER ( DRV_PTG_SWT_TRIGGER_TYPE swtTriggerType );

bool DRV_PTG_SWTGet ( void );

void DRV_PTG_SWTClear ( void );

void DRV_PTG_OutputModeConfigure ( DRV_PTG_OUTPUT_MODE outputMode ) ;

void DRV_PTG_VisibilityConfigure ( DRV_PTG_IVIS_MODE ivisMode );

bool DRV_PTG_WDTStatusCheck ( void );

void DRV_PTG_WDTStatusClear ( void );

void DRV_PTG_OutputPulseWidthConfigure ( uint8_t outputPulseWidth );

uint8_t DRV_PTG_OutputPulseWidthGet ( void );

void DRV_PTG_RegisterConfigure ( DRV_PTG_REG_SUBSET ptgReg, uint32_t value );

void DRV_PTG_RegisterRead ( DRV_PTG_REG_SUBSET ptgReg, uint32 *value );

void DRV_PTG_StepsProgram ( uint8_t *stepCommands, uint8_t numSteps );

void DRV_PTG_IndividualStepProgram ( uint8_t stepCommand, uint8_t step );

uint8_t DRV_PTG_StepCommandGet( uint8_t step );

bool DRV_PTG_Open(void);

void DRV_PTG_Close(void);

#endif // #ifndef _DRV_PTG_STATIC_H

/*******************************************************************************
 End of File
*/
