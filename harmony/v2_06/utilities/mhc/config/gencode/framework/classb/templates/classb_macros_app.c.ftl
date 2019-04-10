<#-- classb_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_classb_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_classb_app_c_global_data>

// Variables used for RAM Tests
extern int  			_stack[];                   // Stack address (placed by the linker)
extern int  			_min_stack_size[];          // Stack size
extern char 			_sdata_begin[];             // Data segment address (placed by the linker)

unsigned int     		*ramTestStartAddress;       // Test area start address 
unsigned int     		ramTestSize;                // Test area size in bytes
unsigned int     		stackTestSize;              // Stack test size in bytes

int         			ramTotSize;        			// Total RAM size (without stack)
unsigned int*       	ramStartAddress;   			// Test RAM start address
unsigned int*       	ramEndAddress;     			// Test RAM end address

int                 	stackTotSize;        		// Total Stack size
unsigned int*       	stackStartAddress;   		// Test stack start address
unsigned int*       	stackEndAddress;     		// Test stack end address
unsigned int 			flashCrcRef, flashCrc; 		// Current and reference CRC values

unsigned int 			crcSeed=0xffff;        		// Initial CRC register value
                                            
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_classb_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_classb_app_c_local_functions>
void ${APP_NAME?upper_case}_CLASSB_Tasks (void )
{
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.classBState )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_CLASSB_STATE_INIT:
        {
			// Disable global interrupts
			SYS_INT_Disable();
			
			// The stack is filled from the high memory to the lower memory 
			// Stack pointer will begin at stackStartAddress
			// stackStartAddress will be a larger value than stackEndAddress
			stackStartAddress= (unsigned int*)_stack;
			stackEndAddress= (unsigned int*)(_stack-((unsigned int)_min_stack_size));
			stackTotSize=(unsigned int)stackStartAddress - (unsigned int)stackEndAddress;
    
			// The available memory is from the start of RAM to the end of the stack.
			ramStartAddress= (unsigned int*)_sdata_begin;
			ramEndAddress= stackEndAddress;
			ramTotSize=(unsigned int)ramEndAddress - (unsigned int)ramStartAddress;

			// FOR THE RAM
			ramTestSize = (ramTotSize>0x400)?0x400:ramTotSize;
			ramTestSize&=0xffffffc0;
			ramTestStartAddress =(unsigned int *)(ramStartAddress+(((ramTotSize-ramTestSize)/2)/sizeof(unsigned int)));

			// FOR THE STACK
			// The ram test area is used to save the stack, so its size has to be larger.
			stackTestSize = (stackTotSize>0x400)?0x400:stackTotSize;
			stackTestSize = (ramTestSize>stackTestSize)?stackTestSize:(ramTestSize-sizeof(unsigned int));
			stackTestSize&=0xffffffc0;
            
			${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_FLASH_CRC_TEST;
            
            break;
        }

        case ${APP_NAME?upper_case}_CLASSB_STATE_FLASH_CRC_TEST:
        {
			// Calculate the CRC16 of the whole program flash      
			char* flashStartAddress = (char*)0x9d000000;       	
			char* flashEndAddress =  flashStartAddress+0x1000;  

			// calculate the reference Flash CRC value
			flashCrcRef = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);
			
			// RE-calculate the CRC of the Flash 
			flashCrc = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

			// Compare reference and calculated Flash CRC values
			if ( flashCrc==flashCrcRef)
            {
				${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_FLASH_CRC_TEST;
            }
            break;
        }
        
        case ${APP_NAME?upper_case}_CLASSB_STATE_CHECKER_BOARD_RAM_TEST :
        {
			if(CLASSB_RAMCheckerBoardTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHB_TEST;
			}
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
            break;
        }
                   
        case ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHB_TEST:
        {
			if(CLASSB_RAMMarchBTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
				${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			else	
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHB_TEST;
            }
            break;
        }
        
        case ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_TEST:
        {
			if(CLASSB_RAMMarchCTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
			}
			else
			{ 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			break;
        }
           
        case ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_STACK_TEST:
        {
			if(CLASSB_RAMMarchCStackTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CLOCK_TEST;
			}
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
            }
            break;
        }
        
        case ${APP_NAME?upper_case}_CLASSB_STATE_CLOCK_TEST:
        {
			if(CLASSB_ClockTest(SYS_CLK_FREQ , CLOCK_TEST_REFERENCE_FREQ, 1, 10) ==CLASSB_TEST_PASS)
			{
				${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CPU_PC_TEST;
			}
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CLOCK_TEST;
            }
            break;
        }
        
        case ${APP_NAME?upper_case}_CLASSB_STATE_CPU_PC_TEST:
        {
			if(CLASSB_CPUPCTest() == CLASSB_TEST_PASS)
			{
                ${APP_NAME?lower_case}Data.classBState =  ${APP_NAME?upper_case}_CLASSB_STATE_CPU_REGISTERS_TEST;
			}
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_CPU_PC_TEST;
            }
            break;
        }
		
        case ${APP_NAME?upper_case}_CLASSB_STATE_CPU_REGISTERS_TEST:
        {
			if(CLASSB_CPURegistersTest() == CLASSB_TEST_PASS)
			{
                ${APP_NAME?lower_case}Data.classBState =  ${APP_NAME?upper_case}_CLASSB_STATE_TEST_PASS;
			}
			else
            { 
                ${APP_NAME?lower_case}Data.classBState = ${APP_NAME?upper_case}_CLASSB_STATE_TEST_FAIL;
            }
            break;
        }
        
        case ${APP_NAME?upper_case}_CLASSB_STATE_TEST_PASS:
        {
            // Enable global interrupts
			SYS_INT_Enable();
			
            break;
        }
              
        case ${APP_NAME?upper_case}_CLASSB_STATE_TEST_FAIL:
        {
            
            break;
        }
      
        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}
</#macro>

<#--
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
-->
<#macro macro_classb_app_c_initialize>
	${APP_NAME?lower_case}Data.classBState =${APP_NAME?upper_case}_CLASSB_STATE_INIT;
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_classb_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_classb_app_c_tasks_state_init>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_classb_app_c_tasks_calls_after_init>
			
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_classb_app_c_tasks_state_service_tasks>
			${APP_NAME?upper_case}_CLASSB_Tasks();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_classb_app_c_tasks_states>
</#macro>
