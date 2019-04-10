/***********************************************************
 *
 *	Timing routines for PIC32MX 
 *	using Timer1 and MIPS Interval Counter
 *
 *  File:   Mysil_Time.c
 *
 *  License: GPL  
 *			General Public License by Open Software Foundation.
 * 
 *  Author: Mysil
 * 
 *  Created     18 July    2012, 16:40 
 *  Modified on 31 July    2012, 01:48  Mysil
 *				 5 December2013, 20:12	Mysil
 **********************************************************/
  
//	#include <p32xxxx.h> 
#include "Compiler.h"
#include "HardwareProfile.h" // Only when Oscillator and Timer not available.
 
#include "Mysil_Time.h"

#ifndef	__PIC32MX__
  #include "TimeDelay.h"
#endif

#ifndef		Adjust
  #ifdef	__PIC32MX__
	#define Adjust 1
  #else
//	#define Adjust 0
	#define Adjust 1
  #endif
#endif


volatile unsigned long int Total = 0;	// Millisecond Total counter
static volatile unsigned long int Previous = 0;	// Timer1 millisecond value.
static int Init1 = 0;
static long int PB_Clock = 0;		// Peripheral Bus Clock frequency.
static long int SYS_Clock = 0;		// System Clock frequency.
static short int uScount = 0;		// Number of ticks in one microsecond.
void Init_Timer1(void);				// Internal function.

/***********************************************************
 *	Instead of making assumptions about what frequency the processor 
 *	is Clocked, we may measure the frequency 
 *	using the MIPS Interval counter and the secondary crystal oscillator. 
 *
 *	This routine will count number of Interval counter ticks 
 *	in one millisecond and estimate system clock frequency, 
 *  Peripheral Bus frequency and 
 *	Number of ticks in a microsecond.
 *********************************************************/
int		//	__longramfunc__	
#ifdef	USE_RAMFUNC
    __longramfunc__  
#endif
Init_Mysil_Time(void)
{
  #if defined __PIC32MX__
	if (OSCCONbits.SOSCEN)		//(OSCCONbits.SOSCRDY)
	{	unsigned int count;
		if (Init1 == 0)				// Init Timer1 if not done already.
			Init_Timer1();

		do
		{	Delay_mS(1);			// Wait until the timer increments.
			count = _CP0_GET_COUNT();
			Delay_mS(1);
			count = _CP0_GET_COUNT() - count;
		}	while (count > 1000000000);	// Just in case Count roll around.
		SYS_Clock = count * 2048;
		PB_Clock = SYS_Clock >> OSCCONbits.PBDIV;	
		uScount = (count + 488) / 976;		// Calculate this on Init, and keep. 
	}
	else							// Oscillator not available,
	{	SYS_Clock = GetSystemClock();
		PB_Clock = GetPeripheralClock();
		uScount = SYS_Clock / 2000000;
	}
	return uScount;
  #elif defined __C30__
	if (OSCCONbits.SOSCEN == 0)		//(OSCCONbits.SOSCRDY)
	{								// enable secondary oscillator.
		__builtin_write_OSCCONL((OSCCON & 0x00FF) | 0x0002); 
	}
	{	long int count;
	 //	long int time;
	
		if (Init1 == 0)				// Init Timer1 if not done already.
			Init_Timer1();
	  #if USE_MYSIL_TIME == 23
									//	Init Timer 2 and 3 as 32 bit timer
		T2CONbits.TON	= 0;	 
		PR2				= 0xFFFF;
		PR3				= 0xFFFF;
		TMR2            = 0;
		TMR3            = 0;
		T2CONbits.TCS	= 0;		// Use System clock.
		T2CONbits.T32	= 1;		// 32 bit timer 2 & 3.
		T2CONbits.TGATE	= 0; 
		T2CONbits.TCKPS	= 0;		// System clock with 1:1 Prescaler 
		T2CONbits.TON	= 1;		// Start Timer
	 
		do							// Measure System Clock Frequency.
		{//	time = TMR1;
		 //	while(time == TMR1);	// Wait until the timer increments.
			Delay_mS(1);
			count = TMR2;			// Start count.
			count = count + ((long)TMR3HLD << 16);
			Delay_mS(1);
			count = TMR2 - count;	// End count.
			count = count + ((long)TMR3HLD << 16);
		}	while (count > 1000000000);	// Just in case Count roll around.
		SYS_Clock = count * 1024;		// fosc/2 instruction clock
		PB_Clock = SYS_Clock >> 1;	
		uScount = (count + 488) / 976;		// Calculate this on Init, and keep. 
	  #elif (USE_MYSIL_TIME == 45)
									//	Init Timer 4 and 5 as 32 bit timer
		T4CONbits.TON	= 0;	 
		PR4				= 0xFFFF;
		PR5				= 0xFFFF;
		TMR4            = 0;
		TMR5            = 0;
		T4CONbits.TCS	= 0;		// Use System clock.
		T4CONbits.T32	= 1;		// 32 bit timer 2 & 3.
		T4CONbits.TGATE	= 0; 
		T4CONbits.TCKPS	= 0;		// System clock with 1:1 Prescaler 
		T4CONbits.TON	= 1;		// Start Timer
	 
		do							// Measure System Clock Frequency.
		{	Delay_mS(1);
			count = TMR4;			// Start count.
			count = count + ((long)TMR5HLD << 16);
			Delay_mS(1);
			count = TMR4 - count;	// End count.
			count = count + ((long)TMR4HLD << 16);
		}	while (count > 1000000000);	// Just in case Count roll around.
		SYS_Clock = count * 1024;		// fosc/2 instruction clock
		PB_Clock = SYS_Clock >> 1;	
		uScount = (count + 488) / 976;		// Calculate this on Init, and keep. 
	  #else
		SYS_Clock = GetSystemClock();
		PB_Clock = GetPeripheralClock();
		uScount = 0;
	  #endif
	}
		return uScount;
  #else
	#warning "MIPS system Interval Counter Register (CP0_COUNT) Not available."
	SYS_Clock = GetSystemClock();
	PB_Clock = GetPeripheralClock();
	uScount = SYS_Clock / 1000000;
	return uScount;
  #endif
}

void	//	__ramfunc__ 
#ifdef	USE_RAMFUNC
    __ramfunc__  
#endif
/**************************************************
 *	Init_Timer1(void)
 *
 *	To keep the Software part of the timer updated, 
 *	a Interrupt is established when Timer period is reached.
 *	This routine set interrupt for Timer1 with priority 2.1

 *	Mysil_Tick.c use Interrupt for Timer4 with priority 2.3
 *	              or interrupt for Core timer  priority 2.0
 */
Init_Timer1(void)
{
	if (Init1 == 0)
	{	Init1 =  1;
	#if defined __PIC32MX__
		T1CONbits.ON	= 0;
	#else
		T1CONbits.TON	= 0;
	#endif 
		PR1				= 0xFFFF;
		TMR1            = 0;
		T1CONbits.TCS	= 1;		// Use 32.768 kHz crystal oscillator.
		T1CONbits.TGATE	= 0; 
		T1CONbits.TCKPS	= 1;		// Try with 1:8 Prescaler Should try other prescalers now
	#if defined __PIC32MX__
		T1CONbits.TWDIS	= 1;		
		T1CONbits.ON	= 1;		// Start Timer
									// Ensure Interrupt system is configured
		unsigned int temp;
		temp = _CP0_GET_STATUS();	// Get Status register CP0 Register 12
		if (temp & 0x00400000) 		// Check that BEV bit is not Set
			while (1);
		if (INTCON & 0x01000)		// Check that MVEC bit is Set
									// Timer1 interrupt setup.
		{	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2 | T1_INT_SUB_PRIOR_1);
			IPC1CLR = _IPC1_T1IP_MASK;	// Interrupt Priority
			IPC1SET = 2 << _IPC1_T1IP_POSITION;
			IPC1CLR = _IPC1_T1IS_MASK;	// Interrupt Subpriority
			IPC1SET = 1 << _IPC1_T1IS_POSITION;
			IFS0CLR = _IFS0_T1IF_MASK;	// 0x010;
			IEC0SET = _IEC0_T1IE_MASK;	// 0x010;
		}
	#else
		IFS0bits.T1IF = 0;  //Clear interrupt flag
		IEC0bits.T1IE = 1;  //Enable interrupt
		T1CONbits.TON = 1;		// Start Timer
	#endif 
	}
	return;
}
/***************************************
//	Interrupt Service Routine for Timer1
 ********************************************************************/
#ifdef __PIC32MX__
//	#define __T1_ISR	__ISR(_TIMER_1_VECTOR, ipl2)
	#define __T1_ISR	__attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL2SOFT), nomips16))
#else
	#define __T1_ISR	__attribute__((interrupt, shadow, auto_psv))
#endif

/* Global Tick counter */
//unsigned long int Tick;

void __T1_ISR _T1Interrupt(void)
{
//	Clear flag
  #ifdef __PIC32MX__
	IFS0CLR = _IFS0_T1IF_MASK;	// 0x010;
  #else
	IFS0bits.T1IF = 0;
  #endif

	// Update Millisecond counter
  #if Adjust
	switch (T1CONbits.TCKPS)			// Adjust for Prescaler
	{ case 0:
		Total = Total + 2000ul - Previous;		// Interval 2 seconds  
		break;
	  case 1:
		Total = Total + 16000ul - Previous; 	// Interval 16 seconds
		break;
	  case 2:
		Total = Total + 128000ul - Previous;	// Interval 128 seconds 
		break;
	  case 3:
		Total = Total + 512000ul - Previous;	// Interval 512 seconds
		break;
	}
  #else
		Total = Total + 16384ul - Previous; 
  #endif
	Previous = 0;
}

long int Mysil_SYS_Clock(void)
{
	return SYS_Clock;
}
long int Mysil_PB_Clock(void)
{
  #ifdef  __PIC32MX__
	PB_Clock = SYS_Clock >> OSCCONbits.PBDIV;	
  #endif
	return PB_Clock;
}
short int Mysil_Count_uS(void)
{
	return uScount;
}
/**********************************************************
 *	Delay_uS( Delay)
 *
 *	microsecond delay routine
 *	Delay a number of microseconds, 
 *	but do Not reset the counter.!
 *	On PIC32 use the Interval Counter Register 
 *	in Coprocessor 0 of the MIPS core.
 * 
 *	This require a little logic to handle the case when the
 *	counter wrap around to zero, while allowing the counter
 *	to be used for other purposes, such as time measurement
 *	at the same time.
 *
 *	For PIC32 one or two of the counters 2 thru 5 are used
 *  only if specified by value of USE_MYSIL_TIME  23 macro.
 *  The same timers are used as used for Time measurement 
 *	by Time_uS() function.
 *	
 *	Limits:
 *  Max 2^32 counter ticks is 107.4 seconds when running at
 *	80 MHz.
 *********************************************************/
void	//	__longramfunc__ 
#ifdef	USE_RAMFUNC
    __longramfunc__  
#endif
Delay_uS(unsigned int Delay)
{
  #if defined __PIC32MX__
	unsigned long int Start;	// With unsigned arithmetic this should work
	unsigned long int Wait;		// also when interval timer roll around.

	Start = _CP0_GET_COUNT();	// MIPS Interval Counter
	Wait  =  Delay * uScount;	// Interval Counter ticks.
	
	while ((_CP0_GET_COUNT() - Start) < Wait);  // 
	return;

/*	if (End > Start)
 *	{
 *		while (_CP0_GET_COUNT() < End);
 *		return;
 *	}
 *	else								// Interval Counter register must be made to roll around to zero.
 *	{	while ( _CP0_GET_COUNT() > Start);
 *
 *		while ( _CP0_GET_COUNT() < End);
 *		return;
 *	}
 */
  #elif defined __C30__  && defined USE_MYSIL_TIME
	unsigned long int Start;	// With unsigned arithmetic this should work
	unsigned long int Time;
	unsigned long int Wait;		// also when interval timer roll around.
	unsigned int      Scale;

	#if (USE_MYSIL_TIME == 23)
		Start = TMR2;				// Read Timer 2 and 3.
		if (T2CONbits.T32 == 1)
			Start = Start + ((long)TMR3HLD << 16);
		Scale = T2CONbits.TCKPS;
	#elif (USE_MYSIL_TIME == 3)
		Start = TMR3;				// Read Timer 3.
		Scale = T3CONbits.TCKPS;
	#elif (USE_MYSIL_TIME == 45)
		Start = TMR4;				// Read Timer 4 and 5.
		if (T4CONbits.T32 == 1)
			Start = Start + ((long)TMR5HLD << 16);
		Scale = T4CONbits.TCKPS;
	#elif (USE_MYSIL_TIME == 5)
		Start = TMR5;				// Read Timer 5.
		Scale = T5CONbits.TCKPS;
	#endif

	Wait  =  Delay * uScount;	// Interval Counter ticks.

	if (Scale == 1)					// Adjust for the prescaler.
		Wait = Wait >> 3;	// Shift 3 bits to divide by 8
	else if (Scale == 2)
		Wait = Wait >> 6;	// Shift 6 bits Wait / 64
	else if (Scale == 3)
		Wait = Wait >> 8;	// Shift 8 bits for Prescaler 256

	Time = Start;					// Start condition.
	while (	(Time - Start) < Wait)	// (_CP0_GET_COUNT() - Start) < Wait);
	{
	  #if (USE_MYSIL_TIME == 23)
		Time = TMR2;				// Read Timer 2 and 3.
		if (T2CONbits.T32 == 1)		// Check the 32 bit flag
			Time = Time + ((long)TMR3HLD << 16);
		else if (Time < Start)		// If 16 bit Timer has rolled around
			Time += 0x10000; 		// 16 bit increment

	  #elif (USE_MYSIL_TIME == 3)	// Timer 3, 16 bit timer alone.
		Time = TMR3;				// Read Timer 3.
		if (Time < Start)			// If 16 bit Timer has rolled around
			Time += 0x10000; 		// 16 bit increment

	  #elif (USE_MYSIL_TIME == 45)
		Time = TMR4;				// Read Timer 4 and 5.
		if (T4CONbits.T32 == 1)		// Check the 32 bit timer mark.
			Time = Time + ((long)TMR5HLD << 16);
		else if (Time < Start)
			Time += 0x10000; 		// 16 bit increment

	  #elif (USE_MYSIL_TIME == 5)	// Timer 5, 16 bit timer alone.
		Time = TMR5;				// Read Timer 5.
		if (Time < Start)			// If 16 bit Timer has rolled around
			Time += 0x10000; 		// 16 bit increment
	  #endif
	}
	return;
  #else
	#warning "Core Timer not available, use other timing or delay routines."
	Delay10us(Delay / 10); 
	return;
  #endif
}

/**********************************************************
 *
 *	mcrosecond Timing using MIPS System Interval Counter.
 *	May also be used on PIC24 using one or two of the 
 *	peripheral timers.
 *
 *	Timing:
 *		Calling the Timing routine before and after an activity 
 *		add approx 600 ns to the measured time because of
 *		the ugly division. 
 *		When system frequency is 80 MHz the overhead is 200 ns.
 *	
 *	Limitation:
 *  Max 2^32 counter ticks is 107.4 seconds when running at
 *	80 MHz.
 *********************************************************/
unsigned long int 	//	__longramfunc__
#ifdef	USE_RAMFUNC
    __longramfunc__  
#endif
Time_uS(unsigned long int TimeIn)
{
  #if defined __PIC32MX__
	unsigned int Time;
	Time = _CP0_GET_COUNT();	// ReadCoreTimer();
	if ( uScount == 40)			// Special case when running
		Time = Time / 40;		// at 80 MHz
	else if( uScount == 48)			// Special case when running
		Time = Time / 48;		// at 96 MHz
	else
		Time = Time / uScount;
	if (TimeIn == 0)
	{	return Time;
	}
	else if (Time > TimeIn)
	{	Time = Time - TimeIn;
		return Time;
	}
	else if (Time < TimeIn)
	{
		if ( uScount == 40)			// Special case when running
		{	Time = Time + 0x80000000 / 20;	// at 80 MHz
			Time = Time - TimeIn;
		}
		else if( uScount == 48)			// Special case when running
		{	Time = Time + 0x80000000 / 24;	// at 96 MHz
			Time = Time - TimeIn;
		}
		else
			Time = Time - TimeIn + (0xFFFFFFFF / uScount);
		return Time;
	}
	else
		return 0;
  #elif defined __PIC24F__  && defined USE_MYSIL_TIME
	unsigned long int Time;
	unsigned int      Scale;
	#if (USE_MYSIL_TIME == 23)
		Time = TMR2;				// Read Timer 2 and 3.
		Time = Time + ((long)TMR3HLD << 16);
		Scale = T2CONbits.TCKPS;
	#else
		Time = TMR4;				// Read Timer 4 and 5.
		Time = Time + ((long)TMR5HLD << 16);
		Scale = T4CONbits.TCKPS;
	#endif
	if ( uScount == 16)				// Special case when running
		Time = Time >> 4;	// / 16;// at 16 MipS
	else
		Time = Time / uScount;

	if (Scale == 1)					// Adjust for the prescaler.
		Time  *= 8;
	else if (Scale == 2)
		Time  *= 64;
	else if (Scale == 3)
		Time  *= 256;

	if (TimeIn == 0)
	{	return Time;
	}
	else if (Time > TimeIn)
	{	Time = Time - TimeIn;
		return Time;
	}
	else if (Time < TimeIn)
	{	
		if (uScount == 16)
		{	Time = Time + 0x10000000;
			Time = Time - TimeIn;
		}
		else
			Time = Time - TimeIn + 0xFFFFFFFF / uScount;
		return Time;
	}
	else
		return 0;
  #else
	#warning "Core Timer not available, use other timing or delay routines."
	unsigned int Time;
	return Time;
  #endif
}
/**********************************************************
 *
 *	millisecond Delay using Timer1 with 32.768 kHz Crystal.
 *
 *	Limitation:
 *		Maximum Delay 2^32 ticks is 131 072 seconds is 36 hours.
 *		With Prescaler actually even longer, 
 *		if you have nothing better to do.
 *********************************************************/
void 	//	__longramfunc__ 
#ifdef	USE_RAMFUNC
    __longramfunc__  
#endif
Delay_mS(unsigned int Delay)
{
	unsigned int Start;
	unsigned long int End;
  #if defined __PIC32MX__
	unsigned int count, set;
  #endif

	if (Init1 == 0)
		Init_Timer1();

	Start = TMR1;				//	Start with current value of Timer1.
	switch (T1CONbits.TCKPS)		// Adjust for Prescaler
	{ case 0:						// Timer ticking at 32.768 kHz
		End = Delay << 5;	break;	// No prescaler, multiply by 32
	  case 1:						// Prescaler is 8 ticking at 4.096 kHz
		End = Delay << 2;	break;	// Multiply by 4 for number of ticks.
	  case 2:						// Prescaler 64	 clock is 512 Hz
		End = Delay >> 1;	break;	// Divide by 2
	  case 3:						// Prescaler 256 clock is 128 Hz
		End = Delay >> 3;	break;	// Divide by 8
	}
  #if Adjust
	if (End > 42)				//	Adjust to clock ticks.
	{
/*	  #if defined __PIC32MX__
		if (Delay > 2000)		//	If long delay, do someting fun in the meantime.
		{						//	Get value of core interval counter
			count = _CP0_GET_COUNT();
			set   = 1;
								//	count = ReadCoreTimer();    		
		}
	  #endif
 */

		End = End * 128 / 125;	// Naughty division, but we are here to spend time anyway
		End = End + Start;
	}
	else
	{	if (End == 0)
			End = 1;			// Wait for the next Tick
		End += Start;
	}	
  #else
	if (End == 0)
		End = 1;				// Wait for the next Tick
	End = End + Start;
  #endif
  #if defined __PIC32MX__
	count = 0;
	set   = 1;
  #endif

	do
	{	if (End < 0x0FFFF)
		{	while( TMR1 < End)
			{						// There is a long wait ahead
									// May have up to 80000 instruction cycles
									// to waste for each millisecond delay.
			  #if defined __PIC32MX__	
				if (T1CONbits.TCKPS == 1)
				{	if ((TMR1 - Start) ==    1 && set == 1)
					{							//	Get value of core interval counter
						count = _CP0_GET_COUNT();
						set = 2;
					}
					if ((TMR1 - Start) == 8193 && set == 2)	
					{							// 8192 ticks since start is 2 seconds.
												// Update the System clock measurement
						count = _CP0_GET_COUNT() - count;
						if (count < 1000000000);
						{	SYS_Clock = count;
							PB_Clock = SYS_Clock >> OSCCONbits.PBDIV;	
							uScount = (SYS_Clock + 1000000) / 2000000;
						}
						set = 3;				// Do this only one time.
				}	}
			  #endif
			};
			return;					//	Completed, return immediately.
		}
		if (End > 0x0FFFF)			//	The delay is so long that the 
									//	Timer must be made to roll around.
		{	Time_mS (0);
			while( TMR1 >= Start);	//	Keep going until Timer start from zero.
			End = End - 0xFFFF;
		}
		if (End < Start) 
		{	if (End > 100)
				Time_mS (0);		//	Just to keep up.

			while( TMR1 < End);		//	Wait for the End.
			return;					//	Completed, return immediately.
		}
		else 
		{	if (Start > 100)		//
				Time_mS (0);		//	Just to keep up.

			while(TMR1 < Start);
			End = End - Start;
		}
	}	while( End > 0);
	return;
}

/**********************************************************
 *	This routine will return watch crystal time in milliseconds
 *	minus the value of the input argument. 
 *	Time measurement using Timer1 with 32 KHz crystal oscillator
 *  with a resolution of 1 millisecond. 
 *  The Crystal runs at 32 768 Hz for use with a 1 second clock,
 *	so if dividing by 32, 
 *	there is 1024 ticks in a second, that is 2.4 % too fast.
 *
 *	The "#define Adjust" control the possibility to scale the timing 
 *	to real milliseconds. 
 *	This will use 4 instructions when the code is optimized.
 *
 *	The routine keep a 32 bit Total time in Software.
 *	It is kept updated every time the function is called. 
 *	This makes it possible to use the Timing routine to measure
 *	several activities, possibly intertvined, at the same time, 
 *	and to accumulate time used by an activity that is repeated. 
 *	As long as the function is called at least once every 16 seconds, 
 *	the software timer will stay synchronized.
 *
 *	An Interrupt handler keep the value updated.
 *
 *	Timing:
 *	After the function has been initialized, the first time it is called,
 *	Function seems to use about 22 instructions including call and return.
 *	That should be is about 275 nS when running at 80 MHz. 
 *	By removing the time adjustment, 4 instructions may be saved.
 *	Then it should use about 225 nS each time it is called.
 *
 *	Timer1:
 *	Timer1 may be used for other activities at the same time as 
 *	this timing routine, as long as: 
 *		Timer1 must Not be reset! 
 *		Timer1 Prescaler must Not be changed! 
 *		Period Register PR1 must Not be changed!
 *
 *	Using a 1/8 prescaler gives a roll around time of 16 seconds.
 *	If more than 16 seconds pass between calls, time may be lost.
 *	With Timer1 Interrupt the timer may be kept updated all time, 
 *	then time cycle is 4 294 967 seconds is 50 days.
 *	For longer times use the Real-time clock and calendar (RTCC).
 *
 *	Secondary Oscillator must be running: SOSCEN / FSOSCEN must be Enabled:
 *	#pragma config FSOSCEN  = ON		// Secondary Oscillator 32 kHz or 
 *  OSCCONSET = _OSCCON_SOSCEN_MASK;	// Require unlock sequence.
 **********************************************************/
unsigned long int  	//	__longramfunc__  
#ifdef	USE_RAMFUNC
    __longramfunc__  
#endif
Time_mS (unsigned long int TimeIn)
{
	unsigned long int Time;

	if (Init1 == 0)
		Init_Timer1();
  #if Adjust							// Adjust for frequency.
	Time = TMR1 * 1000ul;
	switch (T1CONbits.TCKPS)			// Adjust for Prescaler
	{	case 0:
			Time = Time >> 15;	break;	// Time = (Time * 1000 / 1024) / 32
		case 1:
			Time = Time >> 12;	break;	// Time = (Time * 1000 / 1024) / 32 * 8
		case 2:
			Time = Time >> 9;	break;	// Time = (Time * 1000 / 1024) / 32 * 64
		case 3:
			Time = Time >> 7;	break;	// Time = (Time * 1000 / 1024) / 32 * 256
	}
  #else
	switch (T1CONbits.TCKPS)			// Adjust for Prescaler
	{	case 0:
			Time = TMR1 >> 5;	break;	// Divide by 32 to obtain milliseconds.
		case 1:
			Time = TMR1 >> 2;	break;	// Divide by 4 to obtain milliseconds.
		case 2:
			Time = TMR1 << 1;	break;	// Multiply by 2 to obtain milliseconds.
		case 3:
			Time = TMR1 << 3;	break;	// Multiply by 8 to obtain milliseconds.
	}
  #endif
	if (Time > Previous)
		Total = Total + Time - Previous;
	else if (Time < Previous)		// Check if timer has rolled around.
  #if Adjust
		Total = Total + Time + 16000ul - Previous; 
  #else
		Total = Total + Time + 16384ul - Previous; 
  #endif
	Previous = Time;
 
	Time = Total - TimeIn;
	return Time;
}
