/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "app1.h"
#include "app2.h"
#include "app3.h"
#include "app4.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
void _SYS_FS_Tasks(void);
void _SYS_CONSOLE_IDX0_Tasks(void);

void _SYS_COMMAND_Tasks(void);

 
 
void _TCPIP_Tasks(void);
static void _APP_Tasks(void);
static void _APP1_Tasks(void);
static void _APP2_Tasks(void);
static void _APP3_Tasks(void);
static void _APP4_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                1024, NULL, 4, NULL);


    /* Create task for file system state machine*/
    /* Create OS Thread for SYS_FS Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_FS_Tasks,
                "SYS_FS Tasks",
                2048, NULL, 1, NULL);

    /* Create OS Thread for SYS_CONSOLE Instance 0 Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_CONSOLE_IDX0_Tasks,
                "SYS_CONSOLE Instance 0 Tasks",
                1024, NULL, 8, NULL);


    /* Create OS Thread for Sys Command Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_COMMAND_Tasks,
                "Sys Command Tasks",
                1024, NULL, 7, NULL);


 
 

    /* Create task for TCPIP state machine*/
    /* Create OS Thread for TCPIP Tasks. */
    xTaskCreate((TaskFunction_t) _TCPIP_Tasks,
                "TCPIP Tasks",
                1024, NULL, 9, NULL);

    /* Create OS Thread for APP Tasks. */
    xTaskCreate((TaskFunction_t) _APP_Tasks,
                "APP Tasks",
                512, NULL, 1, NULL);

    /* Create OS Thread for APP1 Tasks. */
    xTaskCreate((TaskFunction_t) _APP1_Tasks,
                "APP1 Tasks",
                512, NULL, 3, NULL);

    /* Create OS Thread for APP2 Tasks. */
    xTaskCreate((TaskFunction_t) _APP2_Tasks,
                "APP2 Tasks",
                512, NULL, 5, NULL);

    /* Create OS Thread for APP3 Tasks. */
    xTaskCreate((TaskFunction_t) _APP3_Tasks,
                "APP3 Tasks",
                512, NULL, 6, NULL);

    /* Create OS Thread for APP4 Tasks. */
    xTaskCreate((TaskFunction_t) _APP4_Tasks,
                "APP4 Tasks",
                512, NULL, 1, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
    /* SYS_TMR Device layer tasks routine */ 
    SYS_TMR_Tasks(sysObj.sysTmr);

        /* Maintain Device Drivers */
    DRV_MIIM_Tasks (sysObj.drvMiim);
 
 

        /* Maintain Middleware */

        /* Task Delay */
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

 void _SYS_FS_Tasks(void)
 {
    while(1)
    {
        SYS_FS_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
 }
 void _SYS_CONSOLE_IDX0_Tasks(void)

 {
    while(1)
    {
        SYS_CONSOLE_Tasks(sysObj.sysConsole0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
 }
void _SYS_COMMAND_Tasks(void)
{

    while(1)
    {
        SYS_CMD_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
 
 
void _TCPIP_Tasks(void)
{
    while(1)
    {
        /* Maintain the TCP/IP Stack*/
        TCPIP_STACK_Task(sysObj.tcpip);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
  Function:
    void _APP_Tasks ( void )

  Summary:
    Maintains state machine of APP.
*/

static void _APP_Tasks(void)
{
    while(1)
    {
        APP_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP1_Tasks ( void )

  Summary:
    Maintains state machine of APP1.
*/

static void _APP1_Tasks(void)
{
    while(1)
    {
        APP1_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP2_Tasks ( void )

  Summary:
    Maintains state machine of APP2.
*/

static void _APP2_Tasks(void)
{
    while(1)
    {
        APP2_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP3_Tasks ( void )

  Summary:
    Maintains state machine of APP3.
*/

static void _APP3_Tasks(void)
{
    while(1)
    {
        APP3_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP4_Tasks ( void )

  Summary:
    Maintains state machine of APP4.
*/

static void _APP4_Tasks(void)
{
    while(1)
    {
        APP4_Tasks();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
 End of File
 */
