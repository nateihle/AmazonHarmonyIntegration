/*******************************************************************************
 Test Harness Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    test_config_template.h

  Summary:
    Test harness configuration template file.

  Description:
    This file provides the list of all the configurations that can be used with
    the test harness. This file should not be included by any source files.  It
    is strictly for documentation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _TEST_CONFIG_TEMPLATE_H
#define _TEST_CONFIG_TEMPLATE_H


//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Core Functionality Configuration Options
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Maximum Number of Instances of a Library Under Test

  Summary:
    Defines the maximum number of "Tasks" routines allowed for a single test.

  Description:
    This configuration parameter defines the maximum number of  "Tasks" routines 
    are allowed for a test.
    
  Remarks:
    This parameter has a default value defined in test_harness.h that will 
    generate a warning when used.  To eliminate the warning, define a value in 
    system_config.h.
 */

#define TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY      5


// *****************************************************************************
/* Maximum Number of Instances of a Library Under Test

  Summary:
    Defines the maximum number of instances of a single library under test.

  Description:
    This configuration parameter defines the maximum number of instances of a 
    single library under test.
    
  Remarks:
    This parameter has a default value defined in test_harness.h that will 
    generate a warning when used.  To eliminate the warning, define a value in 
    system_config.h.
 */

#define TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY  3


// *****************************************************************************
/* Maximum Number of Tasks for a Library Under Test

  Summary:
    Defines the maximum number of "Tasks" routines implemented by a single 
    library under test.

  Description:
    This configuration parameter defines the maximum number of "Tasks" routines 
    are allowed for a single library under test.
    
  Remarks:
    This parameter has a default value defined in test_harness.h that will 
    generate a warning when used.  To eliminate the warning, define a value in 
    system_config.h.
 */

#define TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY      5


// *****************************************************************************
// *****************************************************************************
// Section: Timeout Timer Configuration Options
// *****************************************************************************
// *****************************************************************************
/* The test harness directly utilizes the selected timer peripheral.  The 
   selected timer must not be used by any other module in the system.
*/

// *****************************************************************************
/* Timeout Timer

  Summary:
    Defines the timer peripheral used for the test timeout timer.

  Description:
    This configuration option defines the timer peripheral used for the test 
    timeout timer.  It must be defined as the timer peripheral library instance 
    ID of the desired timer instance.
    
  Remarks:
    This parameter has a default value defined in test_harness.c that will 
    generate a warning when used.  To eliminate the warning, define a value in 
    system_config.h.
    
    The test harness takes direct ownership of this timer, utilizing the timer
    peripheral library to control it.  The selected timer ID must be valid, as
    defined by the timer peripheral library.
*/

#define TEST_TIMER_ID                       TMR_ID_1


// *****************************************************************************
/* Timeout Timer Interrupt Source

  Summary:
    Defines the interrupt source ID of the timer used for the timeout timer.

  Description:
    This configuration option defines the interrupt source ID of the timer used 
    for the timeout timer.
    
  Remarks:
    The test harness utilizes the interrupt system service to control this 
    interrupt.  The selected interrupt source must be valid for the selected
    timer, as defined by the interrupt system service.
*/

#define TEST_TIMER_INTERRUPT_SOURCE         INT_SOURCE_TIMER_1


// *****************************************************************************
/* Timeout Timer Clock Source

  Summary:
    Defines the selected clock source for the timer used for the timeout timer.

  Description:
    This configuration option defines the selected clock source for the timer 
    used for the timeout timer.
    
  Remarks:    
    The test harness takes direct ownership of this timer, utilizing the timer
    peripheral library to control it.  The selected clock source must be valid
    for the selected timer, as defined by the timer peripheral library.
*/

#define TEST_TIMER_CLOCK_SOURCE             TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK


// *****************************************************************************
/* Timeout Timer Clock Prescaler

  Summary:
    Defines the selected clock prescaler value for the timeout timer.

  Description:
    This configuration option defines the selected clock prescaler value for 
    the timeout timer.
    
  Remarks:
    The frequency of the clock source selected by the TEST_TIMER_CLOCK_SOURCE 
    is divided by the value identified by this option.
    
    The test harness takes direct ownership of this timer, utilizing the timer
    peripheral library to control it.  The selected prescaler value must be 
    valid for the selected timer, as defined by the timer peripheral library.
*/

#define TEST_TIMER_CLOCK_PRESCALER          TMR_PRESCALE_VALUE_256


// *****************************************************************************
/* Test Timer Increment Period

  Summary:
    Defines the number of timer counts that make up a single timer period.

  Description:
    This configuration option defines the number of timer counts of the 
    selected timer clock source that will be counted before a single timer 
    period expires.
    
  Remarks:
    The test harness takes direct ownership of this timer, operating it in 
    16-bit mode.  The selected period value must be valid for the selected 
    timer, as defined by the timer peripheral library.
*/

#define TEST_TIMER_INCREMENT_PERIOD         31250


// *****************************************************************************
/* Test Timer Milliseconds Per Increment

  Summary:
    Defines the number of milliseconds per timer increment period.

  Description:
    This configuration option defines the number of milliseconds that expire 
    per timer period.  The internal timeout timer is incremented by this many
    milliseconds every time the timer period defined by the 
    TEST_TIMER_INCREMENT_PERIOD configuration option expires.
    
  Remarks:
    This configuration option effectively translates the timer period from 
    scaled clock cycles to milliseconds.
*/

#define TEST_TIMER_MS_PER_INCREMENT         100


// *****************************************************************************
/* Test Timer Timeout Value

  Summary:
    Defines the number of milliseconds that must expire before a test will time
    out.
    
  Description:
    This configuration option defines the number of milliseconds that must 
    expire before a test will time out and be considered as a failure.
    
  Remarks:
    None.
*/

#define TEST_TIMER_MS_TIMEOUT               2000


// *****************************************************************************
/* Test Timer Interrupt Vector

  Summary:
    Defines the interrupt vector for the selected test timer.

  Description:
    This configuration option defines the interrupt vector for the selected 
    test timer.

  Remarks:
    None.
*/

#define TEST_TIMER_INTERRUPT_VECTOR         INT_VECTOR_T1


// *****************************************************************************
/* Test Timer Interrupt Priority

  Summary:
    Defines the interrupt priority for the selected test timer.

  Description:
    This configuration option defines the interrupt priority for the selected 
    test timer.
    
  Remarks:
    None.
*/

#define TEST_TIMER_INTERRUPT_PRIORITY       INT_PRIORITY_LEVEL1


// *****************************************************************************
/* Test Timer Interrupt Subpriority

  Summary:
    Defines the interrupt subpriority for the selected test timer.

  Description:
    This configuration option defines the interrupt subpriority for the 
    selected test timer.
    
  Remarks:
    None.
*/

#define TEST_TIMER_INTERRUPT_SUBPRIORITY    INT_SUBPRIORITY_LEVEL0


// *****************************************************************************
// *****************************************************************************
// Section: Test OSAL Idle Sleep Times
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Library under Test Idle Sleep Time

  Summary:
    Defines the number of milliseconds the library tasks function will sleep 
    when the library is idle.

  Description:
    This configuration option defines the number of milliseconds that the test 
    harness will sleep before calling the tasks function of the library under 
    test when the library status reports it is idle.
        
  Remarks:
    This capability is only supported in an RTOS configuration, but this value
    must be defined in all configurations.
*/

#define TEST_IDLE_SLEEP_MS_LIBRARY          5


// *****************************************************************************
/* Test Idle Sleep Time

  Summary:
    Defines the number of milliseconds the test harness will sleep when the 
    test reports that it is idle.

  Description:
    This configuration option defines the number of milliseconds that the test 
    harness will sleep before calling the tasks function of the test when the 
    it reports it is idle.

  Remarks:
    This capability is only supported in an RTOS configuration, but this value
    must be defined in all configurations.
*/

#define TEST_IDLE_SLEEP_MS                  50


#endif // #ifndef _TEST_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/