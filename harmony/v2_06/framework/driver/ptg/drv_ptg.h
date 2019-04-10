/*******************************************************************************
  PTG Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ptg.h

  Summary:
    PTG Driver interface definition.

  Description:
    The PTG device driver provides a simple interface to manage the PTG
    modules on Microchip microcontrollers.  This file defines the
    interface definition for the PTG driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_PTG_H
#define _DRV_PTG_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/
#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"
#include "system/debug/sys_debug.h"
#include "system/int/sys_int.h"  
#include "driver/driver_common.h"
#include "osal/osal.h"        
#include "peripheral/ptg/plib_ptg.h"

// *****************************************************************************
// *****************************************************************************
// Section: PTG Driver Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* PTG Step Numbers

  Summary:
    PTG Step numbers.

  Description:
    These constants provide PTG step number definitions.

  Remarks:
	None.
*/

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

// *****************************************************************************
/* PTG command definitions

  Summary:
    PTG command definitions.

  Description:
    These constants can be used as PTG commands.

  Remarks:
	None.
*/

#define	PTGCTRL	((0x00) << 4)
#define	PTGADD	((0x01) << 4)
#define	PTGSTRB	((0x02) << 4)
#define	PTGWHI	((0x04) << 4)
#define	PTGWLO	((0x05) << 4)
#define	PTGIRQ	((0x07) << 4)
#define	PTGTRIG	((0x08) << 4)
#define	PTGJMP	((0x0A) << 4)
#define	PTGJMPC0	((0x0C) << 4)
#define	PTGJMPC1	((0x0E) << 4)

// *****************************************************************************
/* PTGADD parameter definitions

  Summary:
    PTGADD parameter definitions.

  Description:
    These constants can be used as PTGADD parameters.

  Remarks:
	None.
*/

#define	ADD_ADJ_C1	(0x01)
#define	ADD_ADJ_T0	(0x02)
#define	ADD_ADJ_T1	(0x03)
#define	ADD_ADJ_SD	(0x04)
#define	ADD_ADJ_L0	(0x05)

// *****************************************************************************
/* PTGCOPY parameter definitions

  Summary:
    PTGCOPY parameter definitions.

  Description:
    These constants can be used as PTGCOPY parameters.

  Remarks:
	None.
*/

#define	COPY_HLD_C0	(0x08)
#define	COPY_HLD_C1	(0x09)
#define	COPY_HLD_T0	(0x0A)
#define	COPY_HLD_T1	(0x0B)
#define	COPY_HLD_SD	(0x0C)
#define	COPY_HLD_L0	(0x0D)

// *****************************************************************************
/* PTGHI/LO parameter definitions

  Summary:
    PTGHI/LO parameter definitions.

  Description:
    These constants can be used as PTGHI/LO parameters.

  Remarks:
	None.
*/
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

// *****************************************************************************
/* PTGIRQ parameter definitions

  Summary:
    PTGIRQ parameter definitions.

  Description:
    These constants can be used as PTGIRQ parameters.

  Remarks:
	None.
*/

#define PTG_IRQ0 (0x00)
#define PTG_IRQ1 (0x01)
#define PTG_IRQ2 (0x02)
#define PTG_IRQ3 (0x03)

// *****************************************************************************
/* PTGCTRL parameter definitions

  Summary:
    PTGCTRL parameter definitions.

  Description:
    These constants can be used as PTGCTRL parameters.

  Remarks:
	None.
*/

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

// *****************************************************************************
/* PTGTRIG parameter definitions

  Summary:
    PTGTRIG parameter definitions.

  Description:
    These constants can be used as PTGTRIG parameters.

  Remarks:
	None.
*/

#define PTG_ADC_TRIG12 (0x0C)
#define PTG_ADC_TRIG13 (0x0D)
#define PTG_ADC_TRIG14 (0x0E)
#define PTG_ADC_TRIG15 (0x0F)
#define PTG_PPS_TRIG28 (0x1C)
#define PTG_PPS_TRIG29 (0x1D)
#define PTG_PPS_TRIG30 (0x1E)
#define PTG_PPS_TRIG31 (0x1F)

// *****************************************************************************
/* PTG SWT trigger types

  Summary:
    defines the Options for user to select the type of SWT triggering.

  Description:
    This enumeration defines the type of SWT triggering. Level triggering and 
	edge triggering are supported.

  Remarks:
    Not all modes are available on all devices.  Refer to the specific data
    sheet to determine availability.
*/


typedef enum
{
	/* Edge Trigger SWT bit in PTGCON Register (i.e., set SWT 0 -> 1) */
	DRV_PTG_SWT_EDGE_TRIG  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Level Trigger SWT bit in PTGCON Register (i.e., set SWT  to 1) */
	DRV_PTG_SWT_LEVEL_TRIG /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_SWT_TRIGGER_TYPE ;


// *****************************************************************************
/* PTG output toggle mode

  Summary:
    Defines the Options for user to select the output mode for PTG.

  Description:
    This enumeration defines the options for configuring the PTG output mode. 
	PTG supports pulse mode and the toggle mode for output modes.
    
  Remarks:
    Not all modes are available on all devices.  Refer to the specific data
    sheet to determine availability.
*/

typedef enum
{
	/* Configures output to pulse mode */
	DRV_PTG_OUTPUT_PULSE_MODE  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Configures output to toggle mode */
	DRV_PTG_OUTPUT_TOGGLE_MODE /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_OUTPUT_MODE;

// ***********************************************************************
/* PTG Registers visibility configuration

  Summary:
    Used to configure the PTG timer/counter register visibility.
	
  Description:
	This enumeration is used to configure the PTG timer and counter register's 
	visibility. 
  
  Remarks:
    Not all modes are available on all devices.  Refer to the specific data
    sheet to determine availability.
*/  

typedef enum
{
	/* Disable Visibility of PTG timer/counter registers */
	DRV_PTG_IVIS_DISABLE  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
	
	/* Enable Visibility of PTG timer/counter registers */
	DRV_PTG_IVIS_ENABLE   /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/
	
}DRV_PTG_IVIS_MODE;

// *****************************************************************************
/* PTG Register subset

   Summary:
    Enables to select subset of PTG registers to be configured and read using a 
	common interface.

   Description:
    This enumeration contains the subset of PTG registers which can use a common 
	interface function to read write from the register.

   Remarks:
	This enumeration is used along with DRV_PTG_RegisterConfigure and 
	DRV_PTG_RegisterRead to select the register to be read/written. In this 
	enumeration PTGCON register is not used. 
*/

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
// Section: ADC Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

//***********************************************************************************
/*  Function:
       void DRV_PTG_Initialize(void)
    
  Summary:
    Initializes the PTG module.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function initializes the PTG module with the user settings.
	
  Precondition:
    None.
	
  Parameters:
	None.
	
  Returns:
	None.
	
  Example:
    <code>
	DRV_PTG_Initialize();
    </code>
	
  Remarks:
	Once user configures PTG in harmony configurator, and generates the code,
	all the user settings will be implemented in initialize function.
  ***********************************************************************************/

void DRV_PTG_Initialize( void );

// *****************************************************************************
/* Function:
   void DRV_PTG_Deinitialize( void )

  Summary:
    Disables the PTG module.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function disables the PTG module.

  Precondition:
	None.
	
  Parameters:
	None.
	
  Returns:
    None.

  Example:
    <code>
	DRV_PTG_Deinitialize();
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by 
    the DRV_ADC_Status operation.  The system has to use DRV_ADC_Status to find 
    out when the module is in the ready state.
*/

void DRV_PTG_Deinitialize( void );


// *****************************************************************************
/* Function:
   void DRV_PTG_InputModeSet(PTG_INPUT_MODE inputMode)

  Summary:
    Sets the input mode for PTG.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function configures the input mode for PTG by setting the value to 
	PTGCON.PTGITM bit field.  
 
  Precondition:
	None.

  Parameters:
    inputMode    - value corresponding to input mode to be set.

  Returns:
    None.

  Example:
    <code>
    PTG_INPUT_MODE inputMode = PTG_INPUT_MODE_0;  // Input mode to be set       
    
    DRV_PTG_InputModeSet(inputMode);
    </code>

  Remarks:
	None.
 */

void DRV_PTG_InputModeSet ( PTG_INPUT_MODE inputMode );


// *****************************************************************************
/* Function:
   PTG_INPUT_MODE DRV_PTG_InputModeGet()

  Summary:
    Gets the current input mode set for the PTG module.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function gets the current input mode set for PTG module by reading
	PTGCON.PTGITM bit field.  
 
  Precondition:
	None.
	
  Parameters:
	None.
	
  Returns:
    Value corresponding to currently set input mode.

  Example:
    <code>    
	PTG_INPUT_MODE inputMode;
    inputMode = DRV_PTG_InputModeGet();
    </code>

  Remarks:
	None.
*/

PTG_INPUT_MODE DRV_PTG_InputModeGet ();

//****************************************************************************
/* Function:
      void DRV_PTG_PTGExecutionStart()
    
  Summary:
    Start the execution of the commands stored in PTGSTEP register.
	<p><b>Implementation:</b> Static</p>
  Description:
    This function starts the execution of the commands stored in PTGSTEP 
	register by setting PTGCON.PTGSRT bit.
	
  Precondition:
	All the steps should be programmed to the desired opcodes using 
	DRV_PTG_StepsProgram or DRV_PTG_IndividualStepProgram
  
  Parameters:
	None.
	
  Returns:
    None.
	
  Example:
    <code>   
    DRV_PTG_PTGExecutionStart ();
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_PTGExecutionStart () ;


// *****************************************************************************
/* Function:
   void DRV_PTG_PTGDisable ()

  Summary:
    Disables the PTG module.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function stops the execution of PTG step commands by clearing 
	PTGCON.PTGSTRT bit and disables complete PTG module, so that no clock is 
	supplied to PTG module by clearing PTGCON.PTGON bit.

  Precondition:
	None.
	
  Parameters:
	None.
	
  Returns:
	None.
	
  Example:
    <code>
	DRV_PTG_PTGDisable ();
    </code>

  Remarks:
    None.
*/

void DRV_PTG_PTGDisable ();

// *****************************************************************************
/* Function:
   void DRV_PTG_PTGHalt ()

  Summary:
    Stops the execution of step commands.
	<p><b>Implementation:</b> Static</p>

  Description:
	This function stops the execution of STEP commands by clearing PTGCON.PTGSTRT 
	bit. However, the PTG module will be clocked and PTG module will be active.
	
  Precondition:
	None.
	
  Parameters:
	None.
	
  Returns:
	None.
	
  Example:
    <code>
	DRV_PTG_PTGHalt ();
    </code>

  Remarks:
	None.
*/

void DRV_PTG_PTGHalt ();

//*******************************************************************************
/*  Function:
	bool DRV_PTG_IsPTGBusy ()

  Summary:
    Gets the current status of PTG module.
	<p><b>Implementation:</b> Static</p>
  Description:
    This function gets the current status of PTG module. If PTG module is enabled, 
	and PTG state machine is running, the bit PTGBUSY will be set and this 
	function returns TRUE.
	
  Precondition:
	None.
  
  Parameters:
	None.
	
  Returns:
    Status of PTG Module.
	
  Example:
    <code>
	bool status;
	status = DRV_PTG_IsPTGBusy ();
    </code>
	
  Remarks:
	None.
  *******************************************************************************/

bool DRV_PTG_IsPTGBusy ();

// *****************************************************************************
/* Function:
    void DRV_PTG_WDTConfigure(PTG_WDT_TIMEOUT_SEL wdtTimeOutSel)

  Summary:
    Configures the PTG Watch dog timer's time out value.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function configures the PTG's WDT timeout value by setting the value to 
	be configured to PTGCON.PTGWDT bit.	If the wdtTimeOutSel value is passed as 
	PTG_WDT_DISABLE, then WDT will be disabled.

  Precondition:
    None.

  Parameters:
    wdtTimeOutSel - time-out value of WDT to be configured.

  Returns:
    None.

  Example:
    <code>
    PTG_WDT_TIMEOUT_SEL wdtTimeOutSel = PTG_WDT_TIMEOUT_COUNT_CYC_8;

    DRV_PTG_WDTConfigure( wdtTimeOutSel );
    </code>

  Remarks:
    None.
*/

void DRV_PTG_WDTConfigure ( PTG_WDT_TIMEOUT_SEL wdtTimeOutSel );


// *****************************************************************************
/* Function:
    PTG_WDT_TIMEOUT_SEL DRV_PTG_WDTCurrentConfigCheck()

  Summary:
    Gets the currently configured timeout value for PTG's WDT.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function gets the currently configured WDT timeout value by reading 
	PTGCON.PTGWDT bit.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    currently configured timeout value for PTG's WDT.

  Example:
    <code>
	PTG_WDT_TIMEOUT_SEL wdtTimeOut;

    wdtTimeOut = DRV_PTG_WDTCurrentConfigCheck ();
    </code>

  Remarks:
    None.
*/

PTG_WDT_TIMEOUT_SEL DRV_PTG_WDTCurrentConfigCheck ();

//****************************************************************************
/* Function:
   void DRV_PTG_WDTDisable()    
   
  Summary:
    disables the PTG's WDT.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function disables the PTG's WDT by setting 0 in PTGCON.PTGWDT bit-field.
	
  Precondition:
    None.
	
  Parameters:
	None.
	
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_WDTDisable();
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_WDTDisable ();

//****************************************************************************
/* Function:
   void DRV_PTG_SWTTRIGGER(DRV_PTG_SWT_TRIGGER_TYPE swtTriggerType)    
   
  Summary:
    Triggers the PTGSWT bit of PTGCON register.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function triggers the PTGSWT bit of PTGCON register. There are 2 types 
	of triggers supported : Edge and Level trigger. For Edge trigger, the SWT bit 
	is made low and then high. In case of level trigger, SWT bit is made high.
	Basically SWT bit is used to sync up CPU core and PTG. PTG will be waiting 
	for an event in SWT bit to execute its next command. Once, the expected event
	(Level trigger or edge trigger) is triggered in SWT bit, the PTG will continue 
	its execution. 
	
  Precondition:
	None.
	
  Parameters:
	swtTriggerType - To Specify the trigger type for SWT bit.
	
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_SWTTRIGGER(DRV_PTG_SWT_EDGE_TRIG);
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_SWTTRIGGER ( DRV_PTG_SWT_TRIGGER_TYPE swtTriggerType );

//****************************************************************************
/* Function:
    bool DRV_PTG_SWTGet()
    
  Summary:
    Gets the current status of SWT bit.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function gets the current status of PTGCON.PTGSWT bit.
	
  Precondition:
    None.
	
  Parameters:
    None.
	
  Returns:
    Status of PTGCON.PTGSWT bit.
	
  Example:
    <code>    
    if( DRV_PTG_SWTGet() )
	{
		// Do Something
	}
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

bool DRV_PTG_SWTGet ();

//****************************************************************************
/* Function:
    void DRV_PTG_SWTClear()
    
  Summary:
    clears the SWT bit.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function clears the PTGCON.PTGSWT bit.
	
  Precondition:
    None.
	
  Parameters:
    None.
	
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_SWTClear();
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_SWTClear ();

//****************************************************************************
/* Function:
   void DRV_PTG_OutputModeConfigure(PTG_OUTPUT_MODE outputMode)   
   
  Summary:
    Configures the output mode of PTG.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function configures the output mode of PTG by configuring the 
	PTGCON.PTGOGL bit field.
	
  Precondition:
    None.
	
  Parameters:
    outputMode - output mode required to be configured. This supports 
				 PTG_OUTPUT_PULSE_MODE and PTG_OUTPUT_TOGGLE_MODE.
	
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_OutputModeConfigure(PTG_OUTPUT_PULSE_MODE);
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_OutputModeConfigure ( PTG_OUTPUT_MODE outputMode ) ;

//****************************************************************************
/* Function:
   void DRV_PTG_VisibilityConfigure(PTG_IVIS_MODE ivisMode)
   
  Summary:
    Configures IVIS bit of PTGCON register.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function configures PTGCON.PTGIVIS bit to determine the access for 
	timer/counter registers.
	
  Precondition:
    None.
	
  Parameters:
    ivisMode - option to enable or disable visibility
	
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_VisibilityConfigure(PTG_IVIS_ENABLE);
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_VisibilityConfigure ( PTG_IVIS_MODE ivisMode );

//****************************************************************************
/* Function:
   void DRV_PTG_WDTStatusCheck ()
   
  Summary:
    Gets the current status of PTGWDTO bit of PTGCON register.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function gets the time-out status of PTG's WDT by reading PTGCON.PTGWDTO
	bit.
	
  Precondition:
    None.
	
  Parameters:
	None.
  
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_WDTStatusCheck();
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

bool DRV_PTG_WDTStatusCheck ();

//****************************************************************************
/* Function:
   void DRV_PTG_WDTStatusClear ()
   
  Summary:
    Clears the PTGWDTO bit of PTGCON register.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function clears the time-out status of PTG's WDT by clearing 
	PTGCON.PTGWDTO bit.
	
  Precondition:
    None.
	
  Parameters:
	None.
  
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_WDTStatusClear();
    </code>
	
  Remarks:
    None.                                                                    
  ****************************************************************************/

void DRV_PTG_WDTStatusClear ();

//****************************************************************************
/* Function:
   void DRV_PTG_OutputPulseWidthConfigure ( uint8_t outputPulseWidth )
   
  Summary:
    sets the output pulse width of PTG trigger output.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function sets the PTG trigger output pulse width by setting the 
	PTGCON.PTGPWD bit field to desired value.
	
  Precondition:
    None.
	
  Parameters:
	None.
  
  Returns:
    None.
	
  Example:
    <code>    
    DRV_PTG_OutputPulseWidthConfigure( 15 );
    </code>
	
  Remarks:
    This function if effective only if the PTG is configured with output mode as
	pulse mode.                                                                    
  ****************************************************************************/

void DRV_PTG_OutputPulseWidthConfigure ( uint8_t outputPulseWidth );

//****************************************************************************
/* Function:
   uint8_t DRV_PTG_OutputPulseWidthGet()
   
  Summary:
    gets the currently configured PTG trigger output pulse width.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function gets the PTG trigger output pulse width by reading the 
	PTGCON.PTGPWD bit field.
	
  Precondition:
    None.
	
  Parameters:
	None.
  
  Returns:
    None.
	
  Example:
    <code>  
	uint8_t pulseWidth;
	
    pulseWidth = DRV_PTG_OutputPulseWidthGet();
    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

uint8_t DRV_PTG_OutputPulseWidthGet ();

//****************************************************************************
/* Function:
   void DRV_PTG_RegisterConfigure ( DRV_PTG_REG_SUBSET ptgReg, uint32_t value )
   
  Summary:
    Configures the specified register with the desired value.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function configures the specified register with the desired value. Except
	PTGCON and STEP register, all other registers are supported by this function.
	
  Precondition:
    None.
	
  Parameters:
	ptgReg - Register to be configured.
	value  - Value to be configured to the register.
  
  Returns:
    None.
	
  Example:
    <code>  
	DRV_PTG_RegisterConfigure(DRV_PTG_REG_BTE, 0xFFFF);
    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

void DRV_PTG_RegisterConfigure ( PTG_REG_SUBSET ptgReg, uint32_t value );

//****************************************************************************
/* Function:
   void DRV_PTG_RegisterRead(PTG_REG_SUBSET ptgReg, uint32 *value)
   
  Summary:
    Reads the specified register.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function reads the specified register and stores the register value in 
	a pointer specified by the caller.
	
  Precondition:
    None.
	
  Parameters:
	ptgReg - Register to be read.
	*value  - pointer location, where the register value have to be stored.
  
  Returns:
    None.
	
  Example:
    <code>  
	uint32_t regBTE;
	DRV_PTG_RegisterConfigure(DRV_PTG_REG_BTE, &regBTE);
    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

void DRV_PTG_RegisterRead ( PTG_REG_SUBSET ptgReg, uint32 *value );

//****************************************************************************
/* Function:
   void DRV_PTG_StepsProgram(uint8_t *stepCommands, uint8_t numSteps)
   
  Summary:
    Programs the step registers with the desired commands.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function is used to program PTG step register with the desired commands.
	The commands are expected to be stored in an array and a pointer to that 
	array needs to be passed to this function, along with the number of steps to
	be programmed.
	
  Precondition:
    PTG should be halted (not executing the step commands).
	
  Parameters:
	*stepCommands - Pointer to the array of commands.
	numSteps      - Number of steps to be programmed.
  
  Returns:
    None.
	
  Example:
    <code>  
	uint8_t cmdArray[10] = {0,};
	uint8_t numSteps = 0;
	
	cmdArray[numSteps++] = PTGHI | 1;
	cmdArray[numSteps++] = PTGTRIG | 12;
	cmdArray[numSteps++] = PTGJMP  | 0;
	
	DRV_PTG_StepsProgram(cmdArray, numSteps);

    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

void DRV_PTG_StepsProgram ( uint8_t *stepCommands, uint8_t numSteps );

//****************************************************************************
/* Function:
   void DRV_PTG_IndividualStepProgram(uint8_t stepCommand, uint8_t step)
   
  Summary:
    Programs the individual step specified.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function is used to program individual step in the PTG step register. 
	The step command to be programmed and the step which we need to program are
	passed as arguments to the function.
	
  Precondition:
    PTG should be halted (not executing the step commands).
	
  Parameters:
	stepCommand   - Step command to be programmed.
	step          - Step at which the command needs to be programmed.
  
  Returns:
    None.
	
  Example:
    <code>  
	uint8_t cmd = PTGJMP  | 0;
		
	DRV_PTG_IndividualStepProgram(cmd, _STEP2);

    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

void DRV_PTG_IndividualStepProgram ( uint8_t stepCommand, uint8_t step );

//****************************************************************************
/* Function:
   uint8_t DRV_PTG_StepCommandGet(uint8_t step)
   
  Summary:
    Reads the content of the step specified.
	<p><b>Implementation:</b> Static</p>
	
  Description:
    This function is used to read the content of the step specified in the 
	parameter.
	
  Precondition:
    None.
	
  Parameters:
	step          - Step, whose content needs to be read.
  
  Returns:
    The content of the specified step.
	
  Example:
    <code>  
	uint8_t cmd = DRV_PTG_StepCommandGet(_STEP2);
    </code>
	
  Remarks:
	None.                                                                    
  ****************************************************************************/

uint8_t DRV_PTG_StepCommandGet( uint8_t step );

#endif //_DRV_PTG_H
/*******************************************************************************
 End of File
*/

