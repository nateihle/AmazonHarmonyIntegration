<#include "/utilities/mhc/templates/freemarker_functions.ftl">
<#assign APP_NAME_STR = "CONFIG_APP_NAME_" + "${HCONFIG_APP_INSTANCE?number}">
<#assign APP_NAME = APP_NAME_STR?eval>
<#assign INSTANCE = HCONFIG_APP_INSTANCE>
<@mhc_expand_list_named name="LIST_APP_FREEMARKER_MACROS"/>
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ${APP_NAME?lower_case}.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "${APP_NAME?lower_case}.h"
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_INCLUDES"/>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

${APP_NAME?upper_case}_DATA ${APP_NAME?lower_case}Data;
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_GLOBAL_DATA"/>

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_CALLBACK_FUNCTIONS"/>

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_LOCAL_FUNCTIONS"/>

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;

<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_INITIALIZE"/>
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_TASKS_DATA"/>

    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
       
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_TASKS_STATE_INIT"/>
        
            if (appInitialized)
            {
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_TASKS_CALLS_AFTER_INIT"/>
            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_TASKS_STATE_SERVICE_TASKS"/>
        
            break;
        }

        /* TODO: implement your application state machine.*/
        
<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_TASKS_STATES"/>

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

<@mhc_expand_list_named name="LIST_APP" + HCONFIG_APP_INSTANCE?number + "_C_APP_TASKS_APP_FUNCTIONS"/>
 

/*******************************************************************************
 End of File
 */
