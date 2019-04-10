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
File    : RTOSInit.c

Purpose : Initializes and handles the hardware for embOS as far
          as required by embOS
          Feel free to modify this file acc. to your target system.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"
#include "system_config.h"

/*********************************************************************
*
*       Configuration
*
**********************************************************************
*/

/*********************************************************************
*
*       Clock frequency settings
*/
#ifndef   OS_FSYS
    #define OS_FSYS       ( SYS_CLK_FREQ )        /* CPU Main clock frequency   */
#endif

#ifndef OS_PCLK_TIMER
    #define OS_PCLK_TIMER ( OS_FSYS / 2 )  /* Peripheral clock frequency */
#endif

#ifndef OS_PCLK_UART
    #define OS_PCLK_UART  (OS_FSYS)        /* Peripheral clock for UART  */
#endif
    
#ifndef OS_TICK_FREQ
    #define OS_TICK_FREQ  ( 1000 )              /* Tick frequency             */
#endif

/*********************************************************************
*
*       Configuration of communication to OSView
*/
#ifndef   OS_VIEW_ENABLE
    #define OS_VIEW_ENABLE    (0)     // Global enable of communication
#endif

#ifndef   OS_VIEW_USE_UART          // If set, UART will be used for communication
    #define OS_VIEW_USE_UART  (1)     // Default: 0 => No Uart
#endif

/*********************************************************************
*
*       UART settings for embOSView
*       If you do not want (or can not due to hardware limitations)
*       to dedicate a UART to embOSView, please define it to be -1
*       Currently no uart is supported
*/
#ifndef OS_UART
    #define OS_UART (-1)   // No Uart support so far
#endif

#ifndef OS_BAUDRATE
  #define OS_BAUDRATE (38400)
#endif

/****** End of configurable options *********************************/
#define OS_UART_USED  ((OS_VIEW_ENABLE && (OS_VIEW_USE_UART != 0)) && (OS_UART != -1))

#if OS_UART_USED
  #define OS_COM_INIT() OS_COM_Init()
#else
  #define OS_COM_INIT()
#endif

#define OS_TIMER_RELOAD (OS_PCLK_TIMER / OS_TICK_FREQ)

/*********************************************************************
*
*       Check configuration
*
**********************************************************************
*/

#ifndef   DEBUG     /* Should normally be defined as project option */
  #define DEBUG  (0)
#endif

/*********************************************************************
*
*       Local defines (sfrs used in RTOSInit.c)
*
**********************************************************************
*/
/* System and interrupt defines */
#define CTIE_BIT                0u
#define SYSTICK_PRIORITY        1u
#define SYSTICK_SUBPRIORITY     0u

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

/*********************************************************************
*
*       _InitVIC()
*
*       Set the CP0 registers for multi-vector interrupt Place 
*       Vector Spacing to 32 bytes
*/
static void __attribute__((nomips16)) _InitVIC(void) {
  unsigned int temp;
  OS_MIPS_SetIntCtl(0x00000020u); // Set the Vector Spacing to non-zero value
  temp  = OS_MIPS_GetCause();
  temp |= 0x00800000u;            // Set IV 
  OS_MIPS_SetCause(temp);         // Update Cause
  temp  = OS_MIPS_GetStatus(); 
  temp &= 0xFFBFFFFDu;            // Clear BEV and EXL 
  OS_MIPS_SetStatus(temp);
  INTCON |= 0x1000u;              // Set MVEC bit
}

/********************************************************************* 
* 
*       _OS_GetHWTimerCycles()
* 
* Function description 
*   Returns the current hardware timer count value
* 
* Return value 
*   Current timer count value
*/ 
static unsigned int _OS_GetHWTimerCycles(void) {
  OS_I32 Cnt;
  Cnt  = (OS_MIPS_GetCompare() - OS_MIPS_GetCount());
  if (Cnt < 0) {  // Missed a counter interrupt, adjust time 
    Cnt = 0 - Cnt;
  } else {           
    Cnt = OS_TIMER_RELOAD - Cnt;
  }
  return (unsigned int)Cnt;
}

/********************************************************************* 
* 
*       _OS_GetHWTimer_IntPending()
* 
* Function description 
*   Returns if the hardware timer interrupt pending flag is set
* 
* Return value 
*   == 0; Interrupt pending flag not set
*   != 0: Interrupt pending flag set
*/ 
static unsigned int _OS_GetHWTimer_IntPending(void) {
  OS_I32 Cnt;
  Cnt  = OS_MIPS_GetCompare() - OS_MIPS_GetCount();
  if (Cnt < 0) {
    return 1;
  } else {
    return 0;
  }
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

void __attribute__( (interrupt(IPL1SOFT), vector(0))) OS_SysTick_ISR( void );

/*********************************************************************
*
*       OS_Systick()
*
*       Interrupt handler function for core timer
*
*/
void OS_Systick(void) {
  OS_I32 t, t1;

  IFS0 &= ~0x00000001;      // reset core timer interrupt pending flag

  OS_EnterNestableInterrupt();

  t  = OS_MIPS_GetCompare();
  do {
    t += OS_TIMER_RELOAD;
    OS_TICK_Handle();
    t1 = OS_MIPS_GetCount() + 1000;
  } while ((t - t1) < 0);
  OS_MIPS_SetCompare(t);

  OS_LeaveNestableInterrupt();
}

/*********************************************************************
*
*       OS_InitHW()
*
*       Initialize the hardware (timer) required for the OS to run.
*       May be modified, if an other timer should be used
*/
void OS_InitHW(void) {
  OS_SYSTIMER_CONFIG SysTimerConfig = {OS_PCLK_TIMER, OS_TICK_FREQ, 1, _OS_GetHWTimerCycles, _OS_GetHWTimer_IntPending};

  OS_IncDI();

  //
  // Set wait states and enable prefetch buffer
  //
  PRECON = 0u
         | (2u << 0u)  // 2 wait states
         | (3u << 4u); // Enable prefetch for instructions + data
  //

  // Initialize timer for embOS, assuming PLL is already initialized
  //
  OS_MIPS_SetCompare(OS_MIPS_GetCount() + OS_TIMER_RELOAD);        // Set compare register
  _InitVIC();                                                      // Setup vectored interrupt controller  
  IPC0 |= (SYSTICK_PRIORITY << 2u) | SYSTICK_SUBPRIORITY;          // Set priority
  IEC0 |= (1u << CTIE_BIT);                                        // Enable core time interrupt in interrupt vector controller
  //
  // Setup values for usec precise system time functions
  //
  OS_Config_SysTimer(&SysTimerConfig);
  //
  // Initialize the optional UART for OS viewer
  //
  OS_COM_INIT();
  OS_DecRI();
} 

/*********************************************************************
*
*       OS_Idle()
*
*       Please note:
*       This is basically the "core" of the idle loop.
*       This core loop can be changed, but:
*       The idle loop does not have a stack of its own, therefore no
*       functionality should be implemented that relies on the stack
*       to be preserved. However, a simple program loop can be programmed
*       (like toggling an output or incrementing a counter)
*/
void OS_Idle(void) {    // Idle loop: No task is ready to execute
  while (1) {           // Nothing to do ... wait for interrupt
    #if (DEBUG == 0)
                        // Switch CPU into sleep mode
    #endif
  }
}

/*********************************************************************
*
*       OS_GetTime_Cycles()
*
*       This routine is required for task-info via OSView or high
*       resolution time measurement functions.
*       It returns the system time in timer clock cycles.
*/
OS_U32 OS_GetTime_Cycles(void) {
  OS_U32 Time;
  OS_I32 Cnt;

  Time = OS_GetTime32();
  Cnt  = (OS_MIPS_GetCompare() - OS_MIPS_GetCount());
  if (Cnt < 0) {  // Missed a counter interrupt, adjust time 
    Time++;
    Cnt = 0 - Cnt;
  } else {           
    Cnt = OS_TIMER_RELOAD - Cnt;
  }
  return (OS_TIMER_RELOAD * Time) + Cnt;
}

/*********************************************************************
*
*       OS_ConvertCycles2us()
*
*       Convert Cycles into micro seconds.
*
*       If your clock frequency is not a multiple of 1 MHz,
*       you may have to modify this routine in order to get proper
*       diagnostics.
*
*       This routine is required for profiling or high resolution time
*       measurement only. It does not affect operation of the OS.
*/
OS_U32 OS_ConvertCycles2us(OS_U32 Cycles) {
  return Cycles/(OS_PCLK_TIMER/1000000);
}

/*********************************************************************
*
*       Optional communication with embOSView
*
**********************************************************************
*/

#if OS_UART_USED

#else  /* UART for communication not used, define dummy functions */
void OS_Uart(void) {
}

void OS_COM_Send1(OS_U8 c) {
  OS_USEPARA(c);           /* Avoid compiler warning */
  OS_COM_ClearTxActive();  /* Let the OS know that Tx is not busy */
}

#endif /*  OS_UART_USED  */


/****** End Of File *************************************************/

