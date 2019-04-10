
//******************************************************************************
// CoreTimerRead()
//  
// Application Read Core Timer
// --Used for BT MAC address randomization
// --core timer current value could be used to calculate
//   cycles used for processing
//******************************************************************************
uint32_t __attribute__((nomips16,inline)) CoreTimerRead()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return(timer);
}

//******************************************************************************
//  CoreTimerSave()
//  Save a measurement (core timer register value)
//******************************************************************************
void __attribute__((inline)) CoreTimerSave()
{
    // to take measurement
    if (coretime_f && (coretime_i < (MAX_CORETIME_I-1)))
    {
        coretime [coretime_i++] = CoreTimerRead();
    }
}

//******************************************************************************
// CoreTimerStart()
//  
// Enable the core timer
//******************************************************************************
void __attribute__((inline)) CoreTimerStart()
{
    coretime_f = 1;  //Coretime flag
}

//******************************************************************************
// CoreTimerStart()
//  
// Disable the core timer
void __attribute__((inline)) CoreTimerStop()
{
    coretime_f = 0;  //Coretime flag
}

