/*
************************************************************************************************************************
*                                                     uC/OS-III
*                                                The Real-Time Kernel
*
*                                       (c) Copyright 2009, Micrium, Weston, FL
*                                                 All Rights Reserved
*                                                    www.Micrium.com
*
*                                       OS CONFIGURATION (APPLICATION SPECIFICS)
*
* File    : OS_CFG_APP.H
* By      : JJL
* Version : V3.01.0
*
* LICENSING TERMS:
* ---------------
*       uC/OS-III  is provided in source form to registered licensees.  It is illegal to distribute this source
*       code to any third party unless you receive written permission by an authorized Micrium officer.
*
*       Knowledge of the source code may NOT be used to develop a similar product.
*
*       Please help us continue to provide the  Embedded  community with the  finest software  available.   Your
*       honesty is greatly appreciated.
*
*       You can contact us at www.micrium.com.
************************************************************************************************************************
*/

#ifndef OS_CFG_APP_H
#define OS_CFG_APP_H

/*
************************************************************************************************************************
*                                                      CONSTANTS
************************************************************************************************************************
*/

                                                            /* --------------------- MISCELLANEOUS ------------------ */
#define  OS_CFG_MSG_POOL_SIZE            ${CONFIG_UCOSIII_CFG_MSG_POOL_SIZE}u               /* Maximum number of messages                             */
#define  OS_CFG_ISR_STK_SIZE             ${CONFIG_UCOSIII_CFG_ISR_STK_SIZE}u               /* Stack size of ISR stack (number of CPU_STK elements)   */
#define  OS_CFG_TASK_STK_LIMIT_PCT_EMPTY  ${CONFIG_UCOSIII_CFG_TASK_STK_LIMIT_PCT_EMPTY}u               /* Stack limit position in percentage to empty            */


                                                            /* ---------------------- IDLE TASK --------------------- */
#define  OS_CFG_IDLE_TASK_STK_SIZE       ${CONFIG_UCOSIII_CFG_IDLE_TASK_STK_SIZE}u               /* Stack size (number of CPU_STK elements)                */


                                                            /* ------------------ ISR HANDLER TASK ------------------ */
#define  OS_CFG_INT_Q_SIZE                ${CONFIG_UCOSIII_CFG_INT_Q_SIZE}u               /* Size of ISR handler task queue                         */
#define  OS_CFG_INT_Q_TASK_STK_SIZE      ${CONFIG_UCOSIII_CFG_INT_Q_TASK_STK_SIZE}u               /* Stack size (number of CPU_STK elements)                */


                                                            /* ------------------- STATISTIC TASK ------------------- */
#define  OS_CFG_STAT_TASK_PRIO  ${CONFIG_UCOSIII_CFG_PRIO_MAX}-2u          /* Priority                                               */
#define  OS_CFG_STAT_TASK_RATE_HZ         ${CONFIG_UCOSIII_CFG_STAT_TASK_RATE_HZ}u               /* Rate of execution (10 Hz Typ.)                         */
#define  OS_CFG_STAT_TASK_STK_SIZE       ${CONFIG_UCOSIII_CFG_STAT_TASK_STK_SIZE}u               /* Stack size (number of CPU_STK elements)                */


                                                            /* ------------------------ TICKS ----------------------- */
#define  OS_CFG_TICK_RATE_HZ            ${CONFIG_UCOSIII_CFG_TICK_RATE_HZ}u               /* Tick rate in Hertz (10 to 1000 Hz)                     */
#define  OS_CFG_TICK_TASK_PRIO  ${CONFIG_UCOSIII_CFG_PRIO_MAX}-4u          /* Priority                                               */
#define  OS_CFG_TICK_TASK_STK_SIZE       ${CONFIG_UCOSIII_CFG_TICK_TASK_STK_SIZE}u               /* Stack size (number of CPU_STK elements)                */
#define  OS_CFG_TICK_WHEEL_SIZE           ${CONFIG_UCOSIII_CFG_TICK_WHEEL_SIZE}u               /* Number of 'spokes' in tick  wheel; SHOULD be prime     */


                                                            /* ----------------------- TIMERS ----------------------- */
#define  OS_CFG_TMR_TASK_PRIO   ${CONFIG_UCOSIII_CFG_PRIO_MAX}-2u          /* Priority of 'Timer Task'                               */
#define  OS_CFG_TMR_TASK_RATE_HZ          ${CONFIG_UCOSIII_CFG_TMR_TASK_RATE_HZ}u               /* Rate for timers (10 Hz Typ.)                           */
#define  OS_CFG_TMR_TASK_STK_SIZE        ${CONFIG_UCOSIII_CFG_TMR_TASK_STK_SIZE}u               /* Stack size (number of CPU_STK elements)                */
#define  OS_CFG_TMR_WHEEL_SIZE            ${CONFIG_UCOSIII_CFG_TMR_WHEEL_SIZE}u               /* Number of 'spokes' in timer wheel; SHOULD be prime     */

#endif

