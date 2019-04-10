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

#ifndef PORTMACRO_H
#define PORTMACRO_H

/* System include files */
#include <xc.h>

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portBYTE_ALIGNMENT			8
#define portSTACK_GROWTH			-1
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
/*-----------------------------------------------------------*/

/* Critical section management. */
#define portIPL_SHIFT				( 10UL )
/* Don't straddle the CEE bit.  Interrupts calling FreeRTOS functions should
never have higher IPL bits set anyway. */
#define portALL_IPL_BITS			( 0x7FUL << portIPL_SHIFT )
#define portSW0_BIT					( 0x01 << 8 )

/* This clears the IPL bits, then sets them to
configMAX_SYSCALL_INTERRUPT_PRIORITY.  An extra check is performed if
configASSERT() is defined to ensure an assertion handler does not inadvertently
attempt to lower the IPL when the call to assert was triggered because the IPL
value was found to be above configMAX_SYSCALL_INTERRUPT_PRIORITY when an ISR
safe FreeRTOS API function was executed.  ISR safe FreeRTOS API functions are
those that end in FromISR.  FreeRTOS maintains a separate interrupt API to
ensure API function and interrupt entry is as fast and as simple as possible. */
#ifdef configASSERT
	#define portDISABLE_INTERRUPTS()											\
	{																			\
	uint32_t ulStatus;														\
																				\
		/* Mask interrupts at and below the kernel interrupt priority. */		\
		ulStatus = _CP0_GET_STATUS();											\
																				\
		/* Is the current IPL below configMAX_SYSCALL_INTERRUPT_PRIORITY? */	\
		if( ( ( ulStatus & portALL_IPL_BITS ) >> portIPL_SHIFT ) < configMAX_SYSCALL_INTERRUPT_PRIORITY ) \
		{																		\
			ulStatus &= ~portALL_IPL_BITS;										\
			_CP0_SET_STATUS( ( ulStatus | ( configMAX_SYSCALL_INTERRUPT_PRIORITY << portIPL_SHIFT ) ) ); \
		}																		\
	}
#else /* configASSERT */
	#define portDISABLE_INTERRUPTS()										\
	{																		\
	uint32_t ulStatus;													\
																			\
		/* Mask interrupts at and below the kernel interrupt priority. */	\
		ulStatus = _CP0_GET_STATUS();										\
		ulStatus &= ~portALL_IPL_BITS;										\
		_CP0_SET_STATUS( ( ulStatus | ( configMAX_SYSCALL_INTERRUPT_PRIORITY << portIPL_SHIFT ) ) ); \
	}
#endif /* configASSERT */

#define portENABLE_INTERRUPTS()											\
{																		\
uint32_t ulStatus;													\
																		\
	/* Unmask all interrupts. */										\
	ulStatus = _CP0_GET_STATUS();										\
	ulStatus &= ~portALL_IPL_BITS;										\
	_CP0_SET_STATUS( ulStatus );										\
}


extern void vTaskEnterCritical( void );
extern void vTaskExitCritical( void );
#define portCRITICAL_NESTING_IN_TCB	1
#define portENTER_CRITICAL()		vTaskEnterCritical()
#define portEXIT_CRITICAL()			vTaskExitCritical()

extern UBaseType_t uxPortSetInterruptMaskFromISR();
extern void vPortClearInterruptMaskFromISR( UBaseType_t );
#define portSET_INTERRUPT_MASK_FROM_ISR() uxPortSetInterruptMaskFromISR()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedStatusRegister ) vPortClearInterruptMaskFromISR( uxSavedStatusRegister )

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31 - _clz( ( uxReadyPriorities ) ) )

#endif /* taskRECORD_READY_PRIORITY */

/*-----------------------------------------------------------*/

/* Task utilities. */

#define portYIELD()								\
{												\
uint32_t ulCause;							\
												\
	/* Trigger software interrupt. */			\
	ulCause = _CP0_GET_CAUSE();					\
	ulCause |= portSW0_BIT;						\
	_CP0_SET_CAUSE( ulCause );					\
}

#define portCURRENT_INTERRUPT_PRIORITY ( ( _CP0_GET_STATUS() & portALL_IPL_BITS ) >> portIPL_SHIFT )
#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() configASSERT( portCURRENT_INTERRUPT_PRIORITY <= configMAX_SYSCALL_INTERRUPT_PRIORITY )

#define portNOP()	__asm volatile ( "nop" )

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters ) __attribute__((noreturn))
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )
/*-----------------------------------------------------------*/

#define portEND_SWITCHING_ISR( xSwitchRequired )	if( xSwitchRequired )	\
													{						\
														portYIELD();		\
													}

/* Required by the kernel aware debugger. */
#ifdef __DEBUG
	#define portREMOVE_STATIC_QUALIFIER
#endif

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

