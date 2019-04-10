
// Implementation in app.c
//
//
// #if defined(USE_CORE_TIMER) && defined(ENABLE_SYS_LOG)
//            unsigned int  core_timer_value; 
//            unsigned int __attribute__((unused)) microseconds; 
//
//            core_timer_value = CoreTimerRead(); 
//#endif
//            //appData.tickSysTmrHandle = SYS_TMR_DelayMS(500);
//            bluetoothTask();
//#if defined(USE_CORE_TIMER) && defined(ENABLE_SYS_LOG)
//            core_timer_value = CoreTimerRead() - core_timer_value ; 
//            // Assume the system clock running at 80 MHz.  
//            microseconds = core_timer_value / 40;
//            SYS_LOG2("Bluetooth Task:  %d Cycles(%df us )", core_timer_value, microseconds);
//#endif


//******************************************************************************
// CoreTimerRead()
//  
// Application Read Core Timer
// --Used for BT MAC address randomization
// --core timer current value could be used to calculate
//   cycles used for processing
//******************************************************************************
uint32_t __attribute__((nomips16)) CoreTimerRead();

//******************************************************************************
//  CoreTimerSave(void)
//  Save a measurement (core timer register value)
//******************************************************************************
void CoreTimerSave(void);

//******************************************************************************
// CoreTimerStart(void)
//  
// Enable the core timer
//******************************************************************************
void CoreTimerStart(void);

//******************************************************************************
// CoreTimerStart()
//  
// Disable the core timer
void CoreTimerStop(void);
