/*
	OpenRTOS V8.0.0  Copyright (C)2006 onward WITTENSTEIN aerospace & simulation limited. All rights reserved.

	This file is part of the OpenRTOS product, version 8.0.0.

	OpenRTOS is distributed exclusively by WITTENSTEIN high integrity systems,
	and is subject to the terms of the License granted to your organization,
	including its warranties and limitations on use and distribution. It cannot be
	copied or reproduced in any way except as permitted by the License.

	Licenses authorize use by processor, compiler, business unit, and product.
	
	WITTENSTEIN high integrity systems is a trading name of WITTENSTEIN
	aerospace & simulation ltd, Registered Office: Brown's Court, Long Ashton
	Business Park, Yanley Lane, Long Ashton, Bristol, BS41 9LB, UK.
	Tel: +44 (0) 1275 395 600, fax: +44 (0) 1275 393 630.
	E-mail: info@HighIntegritySystems.com

	http://www.HighIntegritySystems.com
*/

#include "FreeRTOSConfig.h"

#define portCONTEXT_SIZE 160
#define portEPC_STACK_LOCATION 152
#define portSTATUS_STACK_LOCATION 156

/******************************************************************/
.macro	portSAVE_CONTEXT

	/* Make room for the context. First save the current status so it can be
	manipulated, and the cause and EPC registers so their original values are
	captured. */
	mfc0		k0, _CP0_CAUSE
	addiu		sp, sp, -portCONTEXT_SIZE
	mfc0		k1, _CP0_STATUS

	/* Also save s6 and s5 so they can be used.  Any nesting interrupts should
	maintain the values of these registers across the ISR. */
	sw			s6, 44(sp)
	sw			s5, 40(sp)
	sw			k1, portSTATUS_STACK_LOCATION(sp)

	/* Prepare to enable interrupts above the current priority. */
	srl			k0, k0, 0xa
	ins 		k1, k0, 10, 7
	srl			k0, k0, 0x7 /* This copies the MSB of the IPL, but it would be an error if it was set anyway. */
	ins 		k1, k0, 18, 1
	ins			k1, zero, 1, 4

	/* s5 is used as the frame pointer. */
	add			s5, zero, sp

	/* Check the nesting count value. */
	la			k0, uxInterruptNesting
	lw			s6, (k0)

	/* If the nesting count is 0 then swap to the the system stack, otherwise
	the system stack is already being used. */
	bne			s6, zero, 1f
	nop

	/* Swap to the system stack. */
	la			sp, xISRStackTop
	lw			sp, (sp)

	/* Increment and save the nesting count. */
1:	addiu		s6, s6, 1
	sw			s6, 0(k0)

	/* s6 holds the EPC value, this is saved after interrupts are re-enabled. */
	mfc0 		s6, _CP0_EPC

	/* Re-enable interrupts. */
	mtc0		k1, _CP0_STATUS

	/* Save the context into the space just created.  s6 is saved again
	here as it now contains the EPC value.  No other s registers need be
	saved. */
	sw			ra, 120(s5)
	sw			s8, 116(s5)
	sw			t9, 112(s5)
	sw			t8, 108(s5)
	sw			t7, 104(s5)
	sw			t6, 100(s5)
	sw			t5, 96(s5)
	sw			t4, 92(s5)
	sw			t3, 88(s5)
	sw			t2, 84(s5)
	sw			t1, 80(s5)
	sw			t0, 76(s5)
	sw			a3, 72(s5)
	sw			a2, 68(s5)
	sw			a1, 64(s5)
	sw			a0, 60(s5)
	sw			v1, 56(s5)
	sw			v0, 52(s5)
	sw			s6, portEPC_STACK_LOCATION(s5)
	sw			$1, 16(s5)

	/* Save the AC0, AC1, AC2, AC3 registers from the DSP.  s6 is used as a
	scratch register. */
	mfhi		s6, $ac1
	sw			s6, 128(s5)
	mflo		s6, $ac1
	sw			s6, 124(s5)

	mfhi		s6, $ac2
	sw			s6, 136(s5)
	mflo		s6, $ac2
	sw			s6, 132(s5)

	mfhi		s6, $ac3
	sw			s6, 144(s5)
	mflo		s6, $ac3
	sw			s6, 140(s5)

	/* Save the DSP Control register */
	rddsp		s6
	sw			s6, 148(s5)

	/* ac0 is done separately to match the MX port. */
	mfhi		s6, $ac0
	sw			s6, 12(s5)
	mflo		s6, $ac0
	sw			s6, 8(s5)

	/* Update the task stack pointer value if nesting is zero. */
	la			s6, uxInterruptNesting
	lw			s6, (s6)
	addiu		s6, s6, -1
	bne			s6, zero, 1f
	nop

	/* Save the stack pointer. */
	la			s6, uxSavedTaskStackPointer
	sw			s5, (s6)
1:
	.endm

/******************************************************************/
.macro	portRESTORE_CONTEXT

	/* Restore the stack pointer from the TCB.  This is only done if the
	nesting count is 1. */
	la			s6, uxInterruptNesting
	lw			s6, (s6)
	addiu		s6, s6, -1
	bne			s6, zero, 1f
	nop
	la			s6, uxSavedTaskStackPointer
	lw			s5, (s6)

	/* Restore the context. */
1:	lw			s6, 128(s5)
	mthi		s6, $ac1
	lw			s6, 124(s5)
	mtlo		s6, $ac1

	lw			s6, 136(s5)
	mthi		s6, $ac2
	lw			s6, 132(s5)
	mtlo		s6, $ac2

	lw			s6, 144(s5)
	mthi		s6, $ac3
	lw			s6, 140(s5)
	mtlo			s6, $ac3

	/* Restore DSPControl. */
	lw			s6, 148(s5)
	wrdsp		s6

	lw			s6, 8(s5)
	mtlo		s6, $ac0
	lw			s6, 12(s5)
	mthi		s6, $ac0
	lw			$1, 16(s5)

	/* s6 is loaded as it was used as a scratch register and therefore saved
	as part of the interrupt context. */
	lw			s6, 44(s5)
	lw			v0, 52(s5)
	lw			v1, 56(s5)
	lw			a0, 60(s5)
	lw			a1, 64(s5)
	lw			a2, 68(s5)
	lw			a3, 72(s5)
	lw			t0, 76(s5)
	lw			t1, 80(s5)
	lw			t2, 84(s5)
	lw			t3, 88(s5)
	lw			t4, 92(s5)
	lw			t5, 96(s5)
	lw			t6, 100(s5)
	lw			t7, 104(s5)
	lw			t8, 108(s5)
	lw			t9, 112(s5)
	lw			s8, 116(s5)
	lw			ra, 120(s5)

	/* Protect access to the k registers, and others. */
	di
	ehb

	/* Decrement the nesting count. */
	la			k0, uxInterruptNesting
	lw			k1, (k0)
	addiu		k1, k1, -1
	sw			k1, 0(k0)

	lw			k0, portSTATUS_STACK_LOCATION(s5)
	lw			k1, portEPC_STACK_LOCATION(s5)

	/* Leave the stack in its original state.  First load sp from s5, then
	restore s5 from the stack. */
	add			sp, zero, s5
	lw			s5, 40(sp)
	addiu		sp, sp,	portCONTEXT_SIZE

	mtc0		k0, _CP0_STATUS
	mtc0 		k1, _CP0_EPC
	ehb
	eret
	nop

	.endm

