     unsigned int  core_timer_value,  cycle_count; 
     ... 
    
    
     core_timer_value = _CP0_GET_COUNT(); 
     ... 
     // code to be timed 
     ... 
     core_timer_value = _CP0_GET_COUNT() - core_timer_value; 
     cycle_count = core_timer_value * 2; 



     GetSystemClock()


    unsigned int  core_timer_value, microseconds; 
     ... 
     core_timer_value = ReadCoreTimer(); 
    ... 
    // code to be timed 
    ... 
     core_timer_value = ReadCoreTimer() - core_timer_value ; 
     microseconds = core_timer_value / 40;    // This assume the system clock running at 80 MHz.  


     #define READ_CORE_TIMER() _CP0_GET_COUNT() // Read the MIPS Core Timer
     #define TCPIP_TIMEOUT_PERIOD ((SYS_CLK_FREQ / 2) * 9) // in seconds ( MAX 42))
 
     #define TCPIP_TIMEOUT_PERIOD ((SYS_CLK_FREQ / 2/1000) * 90000) // in milliseconds ( MAX 42000))
 
 
     uint32_t startTimeout = READ_CORE_TIMER(); // start timeout
 
     if((READ_CORE_TIMER() - startTimeout) >= TCPIP_TIMEOUT_PERIOD) // check for timeout
     {
     // Next case,  timeout, what ever
     }
