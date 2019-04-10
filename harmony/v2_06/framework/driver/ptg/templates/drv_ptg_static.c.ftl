/*******************************************************************************
  PTG Driver Initialization File

  File Name:
    drv_ptg_static.c

  Summary:
    This file contains source code necessary to initialize the IC driver.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "DRV_PTG_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/ptg/drv_ptg_static.h"

<#assign steps_inst = [ CONFIG_DRV_STEP_INST_0 ,
						CONFIG_DRV_STEP_INST_1 ,
						CONFIG_DRV_STEP_INST_2 ,
						CONFIG_DRV_STEP_INST_3 ,
						CONFIG_DRV_STEP_INST_4 ,
						CONFIG_DRV_STEP_INST_5 ,
						CONFIG_DRV_STEP_INST_6 ,
						CONFIG_DRV_STEP_INST_7 ,
						CONFIG_DRV_STEP_INST_8 ,
						CONFIG_DRV_STEP_INST_9 ,
						CONFIG_DRV_STEP_INST_10,
						CONFIG_DRV_STEP_INST_11,
						CONFIG_DRV_STEP_INST_12,
						CONFIG_DRV_STEP_INST_13,
						CONFIG_DRV_STEP_INST_14,
						CONFIG_DRV_STEP_INST_15,
						CONFIG_DRV_STEP_INST_16,
						CONFIG_DRV_STEP_INST_17,
						CONFIG_DRV_STEP_INST_18,
						CONFIG_DRV_STEP_INST_19,
						CONFIG_DRV_STEP_INST_20,
						CONFIG_DRV_STEP_INST_21,
						CONFIG_DRV_STEP_INST_22,
						CONFIG_DRV_STEP_INST_23,
						CONFIG_DRV_STEP_INST_24,
						CONFIG_DRV_STEP_INST_25,
						CONFIG_DRV_STEP_INST_26,
						CONFIG_DRV_STEP_INST_27,
						CONFIG_DRV_STEP_INST_28,
						CONFIG_DRV_STEP_INST_29,
						CONFIG_DRV_STEP_INST_30,
						CONFIG_DRV_STEP_INST_31 ]>

<#assign steps_cmd = [ CONFIG_DRV_PTG_STEP_CMD_0 ,
					   CONFIG_DRV_PTG_STEP_CMD_1 ,
					   CONFIG_DRV_PTG_STEP_CMD_2 ,
					   CONFIG_DRV_PTG_STEP_CMD_3 ,
					   CONFIG_DRV_PTG_STEP_CMD_4 ,
					   CONFIG_DRV_PTG_STEP_CMD_5 ,
					   CONFIG_DRV_PTG_STEP_CMD_6 ,
					   CONFIG_DRV_PTG_STEP_CMD_7 ,
					   CONFIG_DRV_PTG_STEP_CMD_8 ,
					   CONFIG_DRV_PTG_STEP_CMD_9 ,
					   CONFIG_DRV_PTG_STEP_CMD_10,
					   CONFIG_DRV_PTG_STEP_CMD_11,
					   CONFIG_DRV_PTG_STEP_CMD_12,
					   CONFIG_DRV_PTG_STEP_CMD_13,
					   CONFIG_DRV_PTG_STEP_CMD_14,
					   CONFIG_DRV_PTG_STEP_CMD_15,
					   CONFIG_DRV_PTG_STEP_CMD_16,
					   CONFIG_DRV_PTG_STEP_CMD_17,
					   CONFIG_DRV_PTG_STEP_CMD_18,
					   CONFIG_DRV_PTG_STEP_CMD_19,
					   CONFIG_DRV_PTG_STEP_CMD_20,
					   CONFIG_DRV_PTG_STEP_CMD_21,
					   CONFIG_DRV_PTG_STEP_CMD_22,
					   CONFIG_DRV_PTG_STEP_CMD_23,
					   CONFIG_DRV_PTG_STEP_CMD_24,
					   CONFIG_DRV_PTG_STEP_CMD_25,
					   CONFIG_DRV_PTG_STEP_CMD_26,
					   CONFIG_DRV_PTG_STEP_CMD_27,
					   CONFIG_DRV_PTG_STEP_CMD_28,
					   CONFIG_DRV_PTG_STEP_CMD_29,
					   CONFIG_DRV_PTG_STEP_CMD_30,
					   CONFIG_DRV_PTG_STEP_CMD_31 ]>

<#assign ptgctrl_param = [ CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_0 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_1 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_2 , 
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_3 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_4 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_5 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_6 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_7 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_8 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_9 ,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_10,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_11,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_12,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_13,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_14,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_15,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_16,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_17,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_18,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_19,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_20,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_21,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_22,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_23,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_24,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_25,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_26,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_27,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_28,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_29,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_30,
						   CONFIG_DRV_PTG_STEP_PTGCTRL_PRM_31 ]>

<#assign ptgadd_param = [ CONFIG_DRV_PTG_STEP_PTGADD_PRM_0 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_1 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_2 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_3 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_4 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_5 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_6 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_7 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_8 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_9 ,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_10,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_11,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_12,
		 				  CONFIG_DRV_PTG_STEP_PTGADD_PRM_13,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_14,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_15,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_16,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_17,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_18,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_19,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_20,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_21,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_22,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_23,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_24,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_25,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_26,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_27,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_28,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_29,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_30,
						  CONFIG_DRV_PTG_STEP_PTGADD_PRM_31 ]>
						  
<#assign ptgstrb_param = [ CONFIG_DRV_PTG_STEP_STRB_PRM_0 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_1 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_2 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_3 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_4 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_5 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_6 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_7 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_8 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_9 ,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_10,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_11,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_12,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_13,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_14,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_15,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_16,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_17,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_18,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_19,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_20,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_21,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_22,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_23,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_24,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_25,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_26,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_27,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_28,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_29,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_30,
						   CONFIG_DRV_PTG_STEP_STRB_PRM_31 ]>
						   
<#assign ptgjmp_param = [ CONFIG_DRV_PTG_STEP_JMP_PRM_0 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_1 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_2 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_3 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_4 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_5 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_6 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_7 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_8 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_9 ,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_10,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_11,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_12,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_13,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_14,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_15,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_16,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_17,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_18,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_19,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_20,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_21,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_22,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_23,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_24,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_25,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_26,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_27,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_28,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_29,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_30,
						  CONFIG_DRV_PTG_STEP_JMP_PRM_31 ]>
						  
<#assign ptgjmpc0_param = [ CONFIG_DRV_PTG_STEP_JMPC0_PRM_0 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_1 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_2 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_3 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_4 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_5 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_6 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_7 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_8 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_9 ,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_10,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_11,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_12,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_13,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_14,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_15,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_16,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_17,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_18,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_19,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_20,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_21,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_22,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_23,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_24,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_25,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_26,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_27,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_28,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_29,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_30,
							CONFIG_DRV_PTG_STEP_JMPC0_PRM_31 ]>

<#assign ptgjmpc1_param = [ CONFIG_DRV_PTG_STEP_JMPC1_PRM_0 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_1 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_2 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_3 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_4 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_5 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_6 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_7 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_8 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_9 ,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_11,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_11,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_12,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_13,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_14,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_15,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_16,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_17,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_18,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_19,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_21,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_21,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_22,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_23,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_24,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_25,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_26,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_27,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_28,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_29,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_31,
							CONFIG_DRV_PTG_STEP_JMPC1_PRM_31 ]>	

<#assign ptghilo_param = [ CONFIG_DRV_PTG_STEP_PTGHILO_PRM_0 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_1 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_2 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_3 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_4 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_5 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_6 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_7 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_8 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_9 ,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_10,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_11,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_12,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_13,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_14,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_15,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_16,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_17,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_18,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_19,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_20,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_21,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_22,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_23,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_24,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_25,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_26,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_27,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_28,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_29,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_30,
						   CONFIG_DRV_PTG_STEP_PTGHILO_PRM_31 ]>
						   
<#assign ptgirq_param = [ CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_0 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_1 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_2 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_3 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_4 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_5 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_6 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_7 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_8 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_9 ,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_10,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_11,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_12,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_13,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_14,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_15,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_16,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_17,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_18,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_19,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_20,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_21,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_22,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_23,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_24,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_25,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_26,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_27,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_28,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_29,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_30,
						  CONFIG_DRV_PTG_STEP_PTGIRQ_PRM_31 ]>
						  
<#assign ptgtrig_param = [ CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_0 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_1 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_2 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_3 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_4 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_5 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_6 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_7 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_8 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_9 ,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_10,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_11,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_12,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_13,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_14,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_15,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_16,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_17,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_18,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_19,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_20,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_21,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_22,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_23,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_24,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_25,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_26,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_27,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_28,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_29,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_30,
						   CONFIG_DRV_PTG_STEP_PTGTRIG_PRM_31 ]>
						   
<#if CONFIG_USE_DRV_PTG == true>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
static bool ptgLock = 0;
<#if CONFIG_DRV_STEP_INST_0 == true>
#define STEPS_TO_PROGRAM (${CONFIG_DRV_PTG_STEP_INSTANCES_NUMBER})
</#if>
// *****************************************************************************
// *****************************************************************************
// Section: PTG Static Driver Functions
// *****************************************************************************
// *****************************************************************************
void DRV_PTG_Initialize(void)
{
	<#if CONFIG_DRV_STEP_INST_0 == true>
	uint8_t Idx = 0;
	uint8_t steps[STEPS_TO_PROGRAM] = {
	<#list 0..31 as stepIdx>
	<#if steps_inst[stepIdx]?has_content  && steps_inst[stepIdx] == true>
	/* Step ${stepIdx} Command */
	<#if steps_cmd[stepIdx] == "PTGCTRL">
	<#if stepIdx == 0>
	PTGCTRL | ${ptgctrl_param[stepIdx]}
	<#else>
	,PTGCTRL | ${ptgctrl_param[stepIdx]}
	</#if>
	<#elseif steps_cmd[stepIdx] == "PTGADD_COPY">
	<#if stepIdx == 0>
	PTPTGADD_COPY | ${ptgadd_param[stepIdx]}
	<#else>
	,PPTGADD_COPY | ${ptgadd_param[stepIdx]}
	</#if>
	<#elseif steps_cmd[stepIdx] == "PTGSTRB">
	<#if stepIdx == 0>
	PTGSTRB | ${ptgstrb_param[stepIdx]}
	<#else>
	,PTGSTRB | ${ptgstrb_param[stepIdx]}
	</#if>	
	<#elseif steps_cmd[stepIdx] == "PTGJMP">
	<#if stepIdx == 0>
	PTGJMP | ${ptgjmp_param[stepIdx]}
	<#else>
	,PTGJMP | ${ptgjmp_param[stepIdx]}
	</#if>
	<#elseif steps_cmd[stepIdx] == "PTGJMPC0">
	<#if stepIdx == 0>
	PTGJMPC0 | ${ptgjmpc0_param[stepIdx]}
	<#else>
	,PTGJMPC0 | ${ptgjmpc0_param[stepIdx]}
	</#if>	
	<#elseif steps_cmd[stepIdx] == "PTGJMPC1">
	<#if stepIdx == 0>
	PTGJMPC1 | ${ptgjmpc1_param[stepIdx]}
	<#else>
	,PTGJMPC1 | ${ptgjmpc1_param[stepIdx]}
	</#if>		
	<#elseif steps_cmd[stepIdx] == "PTGWHI">
	<#if stepIdx == 0>
	PTGWHI | ${ptghilo_param[stepIdx]}
	<#else>
	,PTGWHI | ${ptghilo_param[stepIdx]}
	</#if>		
	<#elseif steps_cmd[stepIdx] == "PTGWLO">
	<#if stepIdx == 0>
	PTGWLO | ${ptghilo_param[stepIdx]}
	<#else>
	,PTGWLO | ${ptghilo_param[stepIdx]}
	</#if>	
	<#elseif steps_cmd[stepIdx] == "PTGIRQ">
	<#if stepIdx == 0>
	PTGIRQ | ${ptgirq_param[stepIdx]}
	<#else>
	,PTGIRQ | ${ptgirq_param[stepIdx]}
	</#if>	
	<#elseif steps_cmd[stepIdx] == "PTGTRIG">
	<#if stepIdx == 0>
	PTGTRIG | ${ptgtrig_param[stepIdx]}
	<#else>
	,PTGTRIG | ${ptgtrig_param[stepIdx]}
	</#if>		
	</#if>
	<#else>
	<#break>
	</#if>
	</#list>	
	};
	</#if>
	
    /* Select Clock Source */
    PLIB_PTG_ClockSourceSelect(PTG_ID_0, ${CONFIG_DRV_PTG_CLOCK_SRC});
	
	/* Select Prescale Value */
	PLIB_PTG_PrescaleSelect(PTG_ID_0, ${CONFIG_DRV_PTG_PRESCALE});
	
	/* PTG Input Mode configuration */ 
	PLIB_PTG_InputTriggerModeSelect(PTG_ID_0, ${CONFIG_DRV_PTG_INPUT_MODE});	
	
	<#if CONFIG_DRV_PTG_OUTPUT_MODE == "PULSE">
	/* PTG Output Mode Configuration : PULSE Mode*/
	PLIB_PTG_OutputTriggerPulse(PTG_ID_0);
	<#elseif CONFIG_DRV_PTG_OUTPUT_MODE == "TOGGLE">
	/* PTG Output Mode Configuration : TOGGLE Mode*/	
	PLIB_PTG_OutputTriggerToggle(PTG_ID_0);
	</#if>
	
	/* PTG WDT Configuration*/
	PLIB_PTG_WDTCountValueSet(PTG_ID_0, ${CONFIG_DRV_PTG_WDT});
	
	<#if CONFIG_DRV_PTG_IVIS == true>
	/* IVIS Bit configuration  : Enable */
	PLIB_PTG_VisiblityEnable(PTG_ID_0);
	<#else>
	/* IVIS Bit configuration  : Disable */
	PLIB_PTG_VisiblityDisable(PTG_ID_0);
	</#if>
	<#if CONFIG_DRV_PTG_TIMER == true>
	<#if CONFIG_DRV_PTG_TIMER0 != "0">
	
	/* Timer0 Configuration */
	PLIB_PTG_TimerLimitSet(PTG_ID_0, PTG_TIMER_0, ${CONFIG_DRV_PTG_TIMER0});
	</#if>
	<#if CONFIG_DRV_PTG_TIMER1 != "0">
	
	/* Timer1 Configuration */
	PLIB_PTG_TimerLimitSet(PTG_ID_0, PTG_TIMER_1, ${CONFIG_DRV_PTG_TIMER1});
	</#if>
	</#if>
	<#if CONFIG_DRV_PTG_COUNTER == true>
	<#if CONFIG_DRV_PTG_COUNTER0 != "0">
	
	/* Counter0 Configuration */	
	PLIB_PTG_CounterLimitSet(PTG_ID_0, PTG_COUNTER_0, ${CONFIG_DRV_PTG_COUNTER0});
	</#if>
	<#if CONFIG_DRV_PTG_COUNTER1 != "0">
	
	/* Counter1 Configuration */	
	PLIB_PTG_CounterLimitSet(PTG_ID_0, PTG_COUNTER_1, ${CONFIG_DRV_PTG_COUNTER1});
	</#if>
	</#if>
	
	/* Broadcast Trigger Mask Configuration */
	PLIB_PTG_TriggerBroadcastMaskSet(PTG_ID_0, ${CONFIG_DRV_PTG_BTE});
	
	/* Hold Register Configuration */
	PLIB_PTG_HoldValueSet(PTG_ID_0, ${CONFIG_DRV_PTG_HOLD});
	
	/* Adjust Register Configuration */
	PLIB_PTG_AdjustValueSet(PTG_ID_0, ${CONFIG_DRV_PTG_ADJ});
	
	/* Literal Strobe Register Configuration */
	PLIB_PTG_LiteralStrobeValueSet(PTG_ID_0, ${CONFIG_DRV_PTG_LIT});
	
	/* Step Delay Register Configuration */
	PLIB_PTG_StepDelaySet(PTG_ID_0, ${CONFIG_DRV_PTG_STEP_DELAY});
	<#if CONFIG_DRV_PTG_SINGLE_STEP_INT == true>
	
	/* Setup Single Step Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_SSTEP_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_SSTEP_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_SS_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_SSTEP_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_SS_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_SSTEP_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_PTG_WDT_INT == true>
	
	/* Setup WDT Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_WDT_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_WDT_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_WDT_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_WDT_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_WDT_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_WDT_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_PTG_IRQ0_INT == true>
	
	/* Setup IRQ0 Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_PTG_IRQ1_INT == true>
	
	/* Setup IRQ1 Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_PTG_IRQ2_INT == true>
	
	/* Setup IRQ2 Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_PTG_IRQ3_INT == true>
	
	/* Setup IRQ3 Interrupt */
	PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_VECTOR}, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_SUB_PRIORITY});          
	PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_SOURCE});
	</#if>
	<#if CONFIG_DRV_STEP_INST_0 == true>
	/* Program the step Register */
	DRV_PTG_StepsProgram(steps, STEPS_TO_PROGRAM);
	</#if>
}

uintptr_t DRV_PTG_Open(void)
{
	if(ptgLock == 0)
	{
		ptgLock = 1;
		return (uintptr_t) &ptgLock;
	}
	else
		return (uintptr_t) NULL;
}

void DRV_PTG_Close(uintptr_t lockPtr)
{
	if(lockPtr == (uintptr_t)&ptgLock)
	{
		inusePTG = 0;
	}
	return;
}

void DRV_PTG_Deinitialize(void)
{
	PLIB_PTG_Disable(PTG_ID_0);
}

void DRV_PTG_InputModeSet(PTG_INPUT_MODE inputMode)
{
	PLIB_PTG_InputTriggerModeSelect(PTG_ID_0, inputMode);
}

PTG_INPUT_MODE DRV_PTG_InputModeGet(void)
{
	return PLIB_PTG_InputTriggerModeGet(PTG_ID_0);
}

void DRV_PTG_PTGExecutionStart(void)
{
	PLIB_PTG_Enable(PTG_ID_0);
	PLIB_PTG_ExecutionStart(PTG_ID_0);
}

void DRV_PTG_PTGDisable(void)
{
	PLIB_PTG_ExecutionHalt(PTG_ID_0);
	PLIB_PTG_Disable(PTG_ID_0);
}

void DRV_PTG_PTGHalt(void)
{
	PLIB_PTG_ExecutionHalt(PTG_ID_0);	
}

bool DRV_PTG_IsPTGBusy(void)
{
	return PLIB_PTG_IsBusy(PTG_ID_0);
}

void DRV_PTG_WDTConfigure(PTG_WDT_TIMEOUT_SEL wdtTimeOutSel)
{
	PLIB_PTG_WDTCountValueSet(PTG_ID_0,wdtTimeOutSel);
}

PTG_WDT_TIMEOUT_SEL DRV_PTG_WDTCurrentConfigCheck(void)
{
	return PLIB_PTG_WDTCountValueGet(PTG_ID_0);
}

void DRV_PTG_WDTDisable(void)
{
	PLIB_PTG_DisableWDT(PTG_ID_0);
}

void DRV_PTG_SWTTRIGGER(DRV_PTG_SWT_TRIGGER_TYPE swtTriggerType)
{
	switch(swtTriggerType)
	{
		case DRV_PTG_SWT_EDGE_TRIG:
			PLIB_PTG_SWTEdgeTrigger(PTG_ID_0);
			break;
		case DRV_PTG_SWT_LEVEL_TRIG:
			PLIB_PTG_SWTLevelTrigger(PTG_ID_0);
			break;
		default:
			return;
	}
}

bool DRV_PTG_SWTGet(void)
{
	return PLIB_PTG_SWTGet(PTG_ID_0);
}

void DRV_PTG_SWTClear(void)
{
	PLIB_PTG_SWTClear(PTG_ID_0);
}

void DRV_PTG_OutputModeConfigure(DRV_PTG_OUTPUT_MODE outputMode)
{
	switch(outputMode)
	{
		case PTG_OUTPUT_PULSE_MODE:
			PLIB_PTG_OutputTriggerPulse(PTG_ID_0);
			break;
		case PTG_OUTPUT_TOGGLE_MODE:
			PLIB_PTG_OutputTriggerToggle(PTG_ID_0);
			break;
		default:
			return;
	}
}

void DRV_PTG_VisibilityConfigure(DRV_PTG_IVIS_MODE ivisMode)
{
	switch(ivisMode)
	{
		case PTG_IVIS_DISABLE:
			PLIB_PTG_VisiblityDisable(PTG_ID_0);
			break;
		case PTG_IVIS_ENABLE:
			PLIB_PTG_VisiblityEnable(PTG_ID_0);
			break;
		default:
			return;
	}
}

bool DRV_PTG_WDTStatusCheck(void)
{
	return PLIB_PTG_WDTStatusGet(PTG_ID_0);
}

void DRV_PTG_WDTStatusClear(void)
{
	PLIB_PTG_WDTStatusClear(PTG_ID_0);
}

void DRV_PTG_OutputPulseWidthConfigure(uint8_t outputPulseWidth)
{
	if((outputPulseWidth < 0) || (outputPulseWidth > 15))
		return;
		
	PLIB_PTG_TriggerPulseWidthSet(PTG_ID_0,outputPulseWidth);
	PLIB_PTG_OutputTriggerPulse(PTG_ID_0);
}

uint8_t DRV_PTG_OutputPulseWidthGet(void)
{
	return PLIB_PTG_TriggerPulseWidthGet(PTG_ID_0);
}

void DRV_PTG_RegisterConfigure(DRV_PTG_REG_SUBSET ptgReg, uint32_t value)
{
	uint16_t value_16bit;
	uint8_t value_8bit;
	
	switch(ptgReg)
	{
		case PTG_REG_BTE:
			PLIB_PTG_TriggerBroadcastMaskSet(PTG_ID_0, value);
			break;
		case PTG_REG_HOLD:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_HoldValueSet(PTG_ID_0, value_16bit);
			break;
		case PTG_REG_TIMER0:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_TimerLimitSet(PTG_ID_0, PTG_TIMER_0, value_16bit);
			break;
		case PTG_REG_TIMER1:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_TimerLimitSet(PTG_ID_0, PTG_TIMER_1, value_16bit);
			break;
		case PTG_REG_COUNTER0:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_CounterLimitSet(PTG_ID_0, PTG_COUNTER_0, value_16bit);
			break;
		case PTG_REG_COUNTER1:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_CounterLimitSet(PTG_ID_0, PTG_COUNTER_1, value_16bit);
			break;
		case PTG_REG_STEP_DELAY:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_StepDelaySet(PTG_ID_0, value_16bit);
			break;
		case PTG_REG_ADJUST:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_AdjustValueSet(PTG_ID_0, value_16bit);
			break;
		case PTG_REG_LITERAL:
			value_16bit = value & 0xFFFF;
			PLIB_PTG_LiteralStrobeValueSet(PTG_ID_0, value_16bit);
			break;
		case PTG_REG_STEP_POINTER:
			value_8bit = value & 0x1F;
			PLIB_PTG_QueuePointerSet(PTG_ID_0, value_8bit);
			break;
		default:
			return;
	}
}

void DRV_PTG_RegisterRead(DRV_PTG_REG_SUBSET ptgReg, uint32 *value)
{
	uint16_t value_16bit;
	uint8_t value_8bit;
	
	switch(ptgReg)
	{
		case PTG_REG_BTE:
			*value = PLIB_PTG_TriggerBroadcastMaskGet(PTG_ID_0);
			break;
		case PTG_REG_HOLD:
			value_16bit = PLIB_PTG_HoldValueGet(PTG_ID_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_TIMER0:
			value_16bit = PLIB_PTG_TimerLimitGet(PTG_ID_0, PTG_TIMER_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_TIMER1:
			value_16bit = PLIB_PTG_TimerLimitGet(PTG_ID_0, PTG_TIMER_1);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_COUNTER0:
			value_16bit = PLIB_PTG_CounterLimitGet(PTG_ID_0, PTG_COUNTER_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_COUNTER1:
			value_16bit = PLIB_PTG_CounterLimitGet(PTG_ID_0, PTG_COUNTER_1);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_STEP_DELAY:
			value_16bit = PLIB_PTG_StepDelayGet(PTG_ID_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_ADJUST:
			value_16bit = PLIB_PTG_AdjustValueGet(PTG_ID_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_LITERAL:
			value_16bit = PLIB_PTG_LiteralStrobeValueGet(PTG_ID_0);
			*value = (uint32_t) value_16bit;
			break;
		case PTG_REG_STEP_POINTER:
			value_8bit = PLIB_PTG_QueuePointerGet(PTG_ID_0);
			*value = (uint32_t) value_8bit;
			break;
		default:
			return;
	}
}


void DRV_PTG_StepsProgram(uint8_t *stepCommands, uint8_t numSteps)
{
	uint8_t step = 0;
	
	if((numSteps < 1) || (numSteps > 32) || (stepCommands == NULL))
		return;
	
	if(DRV_PTG_IsPTGBusy())
	{
		DRV_PTG_PTGDisable();
	}
	
	for(step = 0; step < numSteps; step++)
	{
		PLIB_PTG_StepCommandSet(PTG_ID_0, step, stepCommands[step]);
	}
}

void DRV_PTG_IndividualStepProgram(uint8_t stepCommand, uint8_t step)
{
	if(step < 0 || step > 31)
		return;
	
	if(DRV_PTG_IsPTGBusy())
	{
		DRV_PTG_PTGDisable();
	}

	PLIB_PTG_StepCommandSet(PTG_ID_0, step, stepCommand);
}

uint8_t DRV_PTG_StepCommandGet(uint8_t step)
{
	if(step < 0 || step > 31)
		return;
	
	return PTG_StepCommandGet_Default(PTG_ID_0, step);
}

</#if>
</#if>