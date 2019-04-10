/******************************************************************************
 *
 *                  IO PORT Library definitions
 *
 ******************************************************************************
 * FileName:        ports.h
 * Dependencies:
 * Processor:       PIC32 family
 *
 * Complier:        MPLAB Cxx
 *                  MPLAB IDE xx
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PIC32 Microcontroller is intended
 * and supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PIC32 Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * $Id: ports.h,v 1.8 2006/10/10 16:51:48 C11842 Exp $
 *
 * $Name:  $
 *
 *****************************************************************************/

#ifndef _PORTS_H_
#define _PORTS_H_

#if defined(DAYTONA)
	#include <p32xxxx.h>
	#include "Int.h"
#endif


/******************************************************************************
 * Parameter values to be used with ConfigINTx() & SetPriorityINTx()
 *****************************************************************************/
// Interrupt enable/disable values
#define EXT_INT_ENABLE			(1 << 15)
#define EXT_INT_DISABLE			(0)

// Interrupt edge modes - must be any one value only
#define RISING_EDGE_INT			(1 << 3)
#define FALLING_EDGE_INT		(0)

// Priority values - must be any one value only
#define EXT_INT_PRI_7         (7)
#define EXT_INT_PRI_6         (6)
#define EXT_INT_PRI_5         (5)
#define EXT_INT_PRI_4         (4)
#define EXT_INT_PRI_3         (3)
#define EXT_INT_PRI_2         (2)
#define EXT_INT_PRI_1         (1)
#define EXT_INT_PRI_0         (0)

/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void ConfigINTx(int config)
 *
 * Description:		Configures an external interrupt
 *
 * PreCondition:    None
 *
 * Inputs:			config: Bit-wise ORed of Priority, Interrupt Edge
 *							Mode and Interrupt enable/disable value.
 *
 *					Note: An absent symbol assumes corresponding bit(s)
 *					are disabled, or default value, and will be set = 0.
 *
 * Output:          None
 *
 * Example:			ConfigINT0(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE)
 *
 *****************************************************************************/
#define ConfigINT0(config)  (mINT0ClearIntFlag(), mINT0SetIntPriority(config & 7),\
        mINT0SetEdgeMode((config >> 3) & 1), mINT0IntEnable(config >> 15))

#define ConfigINT1(config)  (mINT1ClearIntFlag(), mINT1SetIntPriority(config & 7),\
	    mINT1SetEdgeMode((config >> 3) & 1), mINT1IntEnable(config >> 15))

#define ConfigINT2(config)  (mINT2ClearIntFlag(), mINT2SetIntPriority(config & 7),\
	    mINT2SetEdgeMode((config >> 3) & 1), mINT2IntEnable(config >> 15))

#define ConfigINT3(config)  (mINT3ClearIntFlag(), mINT3SetIntPriority(config & 7),\
	    mINT3SetEdgeMode((config >> 3) & 1), mINT3IntEnable(config >> 15))

#define ConfigINT4(config)  (mINT4ClearIntFlag(), mINT4SetIntPriority(config & 7),\
	    mINT4SetEdgeMode((config >> 3) & 1), mINT4IntEnable(config >> 15))


/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void SetPriorityIntx(int priority)
 *
 * Description:		Sets the priority for an External interrupt
 *
 * PreCondition:    None
 *
 * Inputs:			A Priority value
 *
 * Output:          None
 *
 * Example:			SetPriorityInt0(EXT_INT_PRI_5)
 *
 *****************************************************************************/
#define SetPriorityInt0(priority)	mINT0SetIntPriority(priority)
#define SetPriorityInt1(priority)	mINT1SetIntPriority(priority)
#define SetPriorityInt2(priority)	mINT2SetIntPriority(priority)
#define SetPriorityInt3(priority)	mINT3SetIntPriority(priority)
#define SetPriorityInt4(priority)	mINT4SetIntPriority(priority)


/******************************************************************************
 * Sub-priority alues to be used with SetSubPriorityIntx
 *****************************************************************************/
#define EXT_INT_SUB_PRI_3			(3)
#define EXT_INT_SUB_PRI_2			(2)
#define EXT_INT_SUB_PRI_1			(1)
#define EXT_INT_SUB_PRI_0			(0)

/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void SetSubPriorityIntx(int subPriority)
 *
 * Description:		Sets the sub-priority for an External interrupt
 *
 * PreCondition:    None
 *
 * Inputs:			A Sub-priority value
 *
 * Output:          None
 *
 * Example:			SetSubPriorityInt0(EXT_INT_SUB_PRI_2)
 *
 *****************************************************************************/
#define SetSubPriorityInt0(subpriority) mINT0SetIntSubPriority(subpriority)
#define SetSubPriorityInt1(subpriority) mINT1SetIntSubPriority(subpriority)
#define SetSubPriorityInt2(subpriority) mINT2SetIntSubPriority(subpriority)
#define SetSubPriorityInt3(subpriority) mINT3SetIntSubPriority(subpriority)
#define SetSubPriorityInt4(subpriority) mINT4SetIntSubPriority(subpriority)


/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void CloseINTx(void)
 *
 * Description:		Disables an external interrupt and clears corresponding flag.
 *
 * PreCondition:    None
 *
 * Inputs:			None
 *
 * Output:          None
 *
 * Example:			CloseINT0()
 *
 *****************************************************************************/
#define CloseINT0()                 (mINT0IntEnable(0), mINT0ClearIntFlag())
#define CloseINT1()					(mINT1IntEnable(0), mINT1ClearIntFlag())
#define CloseINT2()					(mINT2IntEnable(0), mINT2ClearIntFlag())
#define CloseINT3()					(mINT3IntEnable(0), mINT3ClearIntFlag())
#define CloseINT4()					(mINT4IntEnable(0), mINT4ClearIntFlag())

/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void EnableINTx(void)
 *
 * Description:		Enables an external interrupt.
 *
 * PreCondition:    None
 *
 * Inputs:			None
 *
 * Output:          None
 *
 * Example:			EnableINT4()
 *
 *****************************************************************************/
#define EnableINT0              mINT0IntEnable(1)
#define EnableINT1              mINT1IntEnable(1)
#define EnableINT2              mINT2IntEnable(1)
#define EnableINT3              mINT3IntEnable(1)
#define EnableINT4              mINT4IntEnable(1)

/******************************************************************************
 * External INTx Control Function
 *
 * Function:        void DisableINTx(void)
 *
 * Description:		Disables an external interrupt.
 *
 * PreCondition:    None
 *
 * Inputs:			None
 *
 * Output:          None
 *
 * Example:			DisableINT0()
 *
 *****************************************************************************/
#define DisableINT0             mINT0IntEnable(0)
#define DisableINT1             mINT0IntEnable(0)
#define DisableINT2             mINT0IntEnable(0)
#define DisableINT3             mINT0IntEnable(0)
#define DisableINT4             mINT0IntEnable(0)


/******************************************************************************
 * Parameter values to be used with ConfigINTCN()
 *****************************************************************************/
// Change Interrupt ON/OFF values.
#define CHANGE_INT_ON           	(1 << 15)
#define CHANGE_INT_OFF          	(0)

// Change Interrupt Priority Values
#define CHANGE_INT_PRI_7        	(7)
#define CHANGE_INT_PRI_6        	(6)
#define CHANGE_INT_PRI_5        	(5)
#define CHANGE_INT_PRI_4        	(4)
#define CHANGE_INT_PRI_3        	(3)
#define CHANGE_INT_PRI_2        	(2)
#define CHANGE_INT_PRI_1        	(1)
#define CHANGE_INT_PRI_0        	(0)


/******************************************************************************
 * Change Notice Interrupt Control Function
 *
 * Function:        void ConfigIntCN(int config)
 *
 * Description:		Configures Change Notice interrupts
 *
 * PreCondition:    None
 *
 * Inputs:			Bit-wise OR value of CHANGE_INT_PRIx and
 *					CHANGE_INT_XXX sets
 *
 * Output:          None
 *
 * Example:			ConfigIntCN(CHANGE_INT_PRI_3 |
 *								  CHANGE_INT_ON)
 *
 *****************************************************************************/
#define ConfigIntCN(config) (mCNClearIntFlag(), mCNSetIntPriority((config & 0x7)),\
	mCNIntEnable(config >> 15))


/******************************************************************************
 * Parameter values to be used with ConfigCNPullups()
 *****************************************************************************/
#define CN21_PULLUP_ENABLE			(1 << 21)
#define CN20_PULLUP_ENABLE			(1 << 20)
#define CN19_PULLUP_ENABLE			(1 << 19)
#define CN18_PULLUP_ENABLE			(1 << 18)
#define CN17_PULLUP_ENABLE			(1 << 17)
#define CN16_PULLUP_ENABLE			(1 << 16)
#define CN15_PULLUP_ENABLE			(1 << 15)
#define CN14_PULLUP_ENABLE			(1 << 14)
#define CN13_PULLUP_ENABLE			(1 << 13)
#define CN12_PULLUP_ENABLE			(1 << 12)
#define CN11_PULLUP_ENABLE			(1 << 11)
#define CN10_PULLUP_ENABLE			(1 << 10)
#define CN9_PULLUP_ENABLE			(1 << 9)
#define CN8_PULLUP_ENABLE			(1 << 8)
#define CN7_PULLUP_ENABLE			(1 << 7)
#define CN6_PULLUP_ENABLE			(1 << 6)
#define CN5_PULLUP_ENABLE			(1 << 5)
#define CN4_PULLUP_ENABLE			(1 << 4)
#define CN3_PULLUP_ENABLE			(1 << 3)
#define CN2_PULLUP_ENABLE			(1 << 2)
#define CN1_PULLUP_ENABLE			(1 << 1)
#define CN0_PULLUP_ENABLE			(1 << 0)

/******************************************************************************
 * Change Notice Interrupt Control Function
 *
 * Function:        void ConfigCNPullups(int config)
 *
 * Description:		Configures Change Notice pull-ups
 *
 * PreCondition:    None
 *
 * Inputs:			Bit-wise OR value of CNx_PULLUP_ENABLE set
 *
 *					Note: An absent symbol assumes corresponding bit(s)
 *					are disabled, or default value, and will be set = 0.
 *
 * Output:          None
 *
 * Example:			To enable CN0 & CN1 pullups and disable all others
 *					ConfigCNPullups(CN0_PULLUP_ENABLE | CN1_PULLUP_ENABLE)
 *
 *****************************************************************************/
#define ConfigCNPullups(config) 	CNPUE = config


/******************************************************************************
 * Change Notice Interrupt Control Function
 *
 * Function:        void EnableCNx(void)
 *
 * Description:		Enables individual Change Notice interrupt
 *
 * PreCondition:    None
 *
 * Inputs:			None
 *
 * Output:          None
 *
 * Example:			EnableCN0()
 *
 *****************************************************************************/
#define EnableCN21()             	CNENSET = (1 << 21)
#define EnableCN20()              	CNENSET = (1 << 20)
#define EnableCN19()              	CNENSET = (1 << 19)
#define EnableCN18()              	CNENSET = (1 << 18)
#define EnableCN17()              	CNENSET = (1 << 17)
#define EnableCN16()              	CNENSET = (1 << 16)
#define EnableCN15()              	CNENSET = (1 << 15)
#define EnableCN14()              	CNENSET = (1 << 14)
#define EnableCN13()              	CNENSET = (1 << 13)
#define EnableCN12()              	CNENSET = (1 << 12)
#define EnableCN11()              	CNENSET = (1 << 11)
#define EnableCN10()              	CNENSET = (1 << 10)
#define EnableCN9()               	CNENSET = (1 << 9)
#define EnableCN8()               	CNENSET = (1 << 8)
#define EnableCN7()               	CNENSET = (1 << 7)
#define EnableCN6()               	CNENSET = (1 << 6)
#define EnableCN5()               	CNENSET = (1 << 5)
#define EnableCN4()               	CNENSET = (1 << 4)
#define EnableCN3()               	CNENSET = (1 << 3)
#define EnableCN2()               	CNENSET = (1 << 2)
#define EnableCN1()               	CNENSET = (1 << 1)
#define EnableCN0()               	CNENSET = (1 << 0)

/******************************************************************************
 * Change Notice Interrupt Control Function
 *
 * Function:        void DisableCNx(void)
 *
 * Description:		Disables individual Change Notice interrupt
 *
 * PreCondition:    None
 *
 * Inputs:			None
 *
 * Output:          None
 *
 * Example:			DisableCN0()
 *
 *****************************************************************************/
#define DisableCN21()              	CNENCLR = (1 << 21)
#define DisableCN20()              	CNENCLR = (1 << 20)
#define DisableCN19()              	CNENCLR = (1 << 19)
#define DisableCN18()              	CNENCLR = (1 << 18)
#define DisableCN17()              	CNENCLR = (1 << 17)
#define DisableCN16()             	CNENCLR = (1 << 16)
#define DisableCN15()              	CNENCLR = (1 << 15)
#define DisableCN14()              	CNENCLR = (1 << 14)
#define DisableCN13()              	CNENCLR = (1 << 13)
#define DisableCN12()              	CNENCLR = (1 << 12)
#define DisableCN11()              	CNENCLR = (1 << 11)
#define DisableCN10()              	CNENCLR = (1 << 10)
#define DisableCN9()               	CNENCLR = (1 << 9)
#define DisableCN8()               	CNENCLR = (1 << 8)
#define DisableCN7()               	CNENCLR = (1 << 7)
#define DisableCN6()               	CNENCLR = (1 << 6)
#define DisableCN5()               	CNENCLR = (1 << 5)
#define DisableCN4()               	CNENCLR = (1 << 4)
#define DisableCN3()               	CNENCLR = (1 << 3)
#define DisableCN2()               	CNENCLR = (1 << 2)
#define DisableCN1()               	CNENCLR = (1 << 1)
#define DisableCN0()               	CNENCLR = (1 << 0)

/******************************************************************************
 * Parameter values to be used with mPORTxxx macros
 *****************************************************************************/
#define IOPORT_BIT_31                       (1 << 31)
#define IOPORT_BIT_30                       (1 << 30)
#define IOPORT_BIT_29                       (1 << 29)
#define IOPORT_BIT_28                       (1 << 28)
#define IOPORT_BIT_27                       (1 << 27)
#define IOPORT_BIT_26                       (1 << 26)
#define IOPORT_BIT_25                       (1 << 25)
#define IOPORT_BIT_24                       (1 << 24)
#define IOPORT_BIT_23                       (1 << 23)
#define IOPORT_BIT_22                       (1 << 22)
#define IOPORT_BIT_21                       (1 << 21)
#define IOPORT_BIT_20                       (1 << 20)
#define IOPORT_BIT_19                       (1 << 19)
#define IOPORT_BIT_18                       (1 << 18)
#define IOPORT_BIT_17                       (1 << 17)
#define IOPORT_BIT_16                       (1 << 16)
#define IOPORT_BIT_15                       (1 << 15)
#define IOPORT_BIT_14                       (1 << 14)
#define IOPORT_BIT_13                       (1 << 13)
#define IOPORT_BIT_12                       (1 << 12)
#define IOPORT_BIT_11                       (1 << 11)
#define IOPORT_BIT_10                       (1 << 10)
#define IOPORT_BIT_9                        (1 << 9)
#define IOPORT_BIT_8                        (1 << 8)
#define IOPORT_BIT_7                        (1 << 7)
#define IOPORT_BIT_6                        (1 << 6)
#define IOPORT_BIT_5                        (1 << 5)
#define IOPORT_BIT_4                        (1 << 4)
#define IOPORT_BIT_3                        (1 << 3)
#define IOPORT_BIT_2                        (1 << 2)
#define IOPORT_BIT_1                        (1 << 1)
#define IOPORT_BIT_0                        (1 << 0)

/******************************************************************************
 * GPIO Control Macros
 *
 * Function:
 *			To set I/O pin directions for a PORTx
 *					void mPORTxConfig(int _direction)
 *					int mPORTxGetConfig(void)
 *					int mPORTxReadConfigBit(int _bits)
 *
 *			To read an input pin
 *					int mPORTxRead(void)
 *					int mPORTxReadBit(int _bits)
 *
 *			To write to an output pin
 *					void mPORTxWrite(int _value)
 *					int mPORTxReadLatch(void)
 *					int mPORTxReadLatchBit(int _bits)
 *
 *			To configure select pins in a PORT without affecting other pins
 *                  void mPORTxSetPinsDigitalInput(int _inputs)
 *                  void mPORTxSetPinsDigitalOutput(int _outputs) 
 *                  void mPORTxSetPinsAnalogInput(int _inputs)
 *                  void mPORTxSetPinsAnalgoOutput(int _outputs)
 *					void mPORTxOutputConfig(int _outputs)
 *					void mPORTxInputConfig(int _inputs)
 *
 *			To manipulate select pins in a PORT without affecting other pins
 *					void mPORTxSetbits(int _bits)
 *					void mPORTxClearBits(int _bits)
 *					void mPORTxToggleBits(int _bits)
 *
 *			To make a PORT all input and clear latch values.
 *					void mPORTxCloseAll(void)
 *					void mPORTxCloseBits(int _bits)
 *
 *			To configure open drain I/Os in a PORT.
 *					void mPORTxOpenDrainOpen(int _bits)
 *					void mPORTxOpenDrainClose(int _bits)
 *
 * Description:		None
 *
 * PreCondition:    None
 *
 * Inputs:			Depends
 *
 * Output:          None
 *
 * Example:			mPORTCSetPinsDigitalOutput(IO_PORT_BIT_4 | IO_PORT_BIT_3)
 *
 *****************************************************************************/

#if defined _PORTA
/******************************************************************************
 * PORTA macros
 *****************************************************************************/
#define mPORTAConfig(_tris)                 TRISA = (unsigned int)_tris
#define mPORTARead()                        PORTA
#define mPORTAWrite(_lat)                   LATA = (unsigned int)_lat
#define mPORTAReadLatch()                   LATA
#define mPORTAGetConfig()                   TRISA
#define mPORTAReadBit(_bits)                (PORTA & (unsigned int)_bits)
#define mPORTAReadLatchBit(_bits)           (LATA & (unsigned int)_bits)
#define mPORTAReadConfigBit(_bits)          (TRISA & (unsigned int)_bits)

#define mPORTASetPinsDigitalIn(_inputs)     TRISASET = (unsigned int)_inputs
#define mPORTASetPinsDigitalOut(_outputs)   TRISACLR = (unsigned int)_outputs
#define mPORTAOutputConfig(_outputs)         TRISACLR = (unsigned int)_outputs
#define mPORTAInputConfig(_inputs)           TRISASET = (unsigned int)_inputs
#define mPORTASetBits(_bits)                LATASET = (unsigned int)_bits
#define mPORTAClearBits(_bits)              LATACLR = (unsigned int)_bits
#define mPORTAToggleBits(_bits)             LATAINV = (unsigned int)_bits
#define mPORTACloseAll()                    (TRISASET = 0xFFFFFFFF,\
                                             LATACLR = 0xFFFFFFFF)
#define mPORTACloseBits(_bits)              (TRISASET = (unsigned int)_bits,\
                                             LATACLR = (unsigned int)_bits)
#define mPORTAOpenDrainOpen(_bits)          (ODCFGASET = (unsigned int)_bits,\
                                             TRISACLR = (unsigned int)_bits)
#define mPORTAOpenDrainClose(_bits)         (ODCFGACLR = (unsigned int)_bits,\
                                             TRISASET = (unsigned int)_bits)
#endif  //end PORTA


#if defined _PORTB
/******************************************************************************
 * PORTB macros
 *****************************************************************************/
#define mPORTBConfig(_tris)                 TRISB = (unsigned int)_tris
#define mPORTBRead()                        PORTB
#define mPORTBWrite(_lat)                   LATB = (unsigned int)_lat
#define mPORTBReadLatch()                   LATB
#define mPORTBGetConfig()                   TRISB
#define mPORTBReadBit(_bits)                (PORTB & (unsigned int)_bits)
#define mPORTBReadLatchBit(_bits)           (LATB & (unsigned int)_bits)
#define mPORTBReadConfigBit(_bits)          (TRISB & (unsigned int)_bits)

#define mPORTBSetPinsAnalogOut(_outputs)    (TRISBSET = (unsigned int)_outputs,\
                                            AD1PCFGCLR = (unsigned int)_outputs)
#define mPORTBSetPinsAnalogIn(_inputs)      (TRISBSET = (unsigned int)_inputs,\
                                            AD1PCFGCLR = (unsigned int)_inputs)
#define mPORTBSetPinsDigitalOut(_bits)      (TRISBCLR = (unsigned int)_inputs,\
                                            AD1PCFGSET = (unsigned int)_outputs)
#define mPORTBSetPinsDigitalIn(_bits)       (TRISBSET = (unsigned int)_bits,\
                                            AD1PCFGSET = (unsigned int)_bits)
#define mPORTBOutputConfig(_outputs)        (AD1PCFGSET = (unsigned int) _outputs,\
                                            TRISBCLR = (unsigned int)_outputs)
#define mPORTBInputConfig(_inputs)          (AD1PCFGSET = (unsigned int) _inputs,\
                                            TRISBSET = (unsigned int)_inputs)
#define mPORTBSetBits(_bits)                LATBSET = (unsigned int)_bits
#define mPORTBClearBits(_bits)              LATBCLR = (unsigned int)_bits
#define mPORTBToggleBits(_bits)             LATBINV = (unsigned int)_bits
#define mPORTBCloseAll()                    (TRISBSET = 0xFFFFFFFF,\
                                             LATBCLR = 0xFFFFFFFF)
#define mPORTBCloseBits(_bits)              (TRISBSET = (unsigned int)_bits,\
                                             LATBCLR = (unsigned int)_bits)
#define mPORTBOpenDrainOpen(_bits)          (ODCFGBSET = (unsigned int)_bits,\
                                             TRISBCLR = (unsigned int)_bits)
#define mPORTBOpenDrainClose(_bits)         (ODCFGBCLR = (unsigned int)_bits,\
                                             TRISBSET = (unsigned int)_bits)
#endif  //end PORTB

#if defined _PORTC
/******************************************************************************
 * PORTC macros
 *****************************************************************************/
#define mPORTCConfig(_tris)                 TRISC = (unsigned int)_tris
#define mPORTCRead()                        PORTC
#define mPORTCWrite(_lat)                   LATC = (unsigned int)_lat
#define mPORTCReadLatch()                   LATC
#define mPORTCGetConfig()                   TRISC
#define mPORTCReadBit(_bits)                (PORTC & (unsigned int)_bits)
#define mPORTCReadLatchBit(_bits)           (LATC & (unsigned int)_bits)
#define mPORTCReadConfigBit(_bits)          (TRISC & (unsigned int)_bits)

#define mPORTCSetPinsDigitalIn(_inputs)     TRISCSET = (unsigned int)_inputs
#define mPORTCSetPinsDigitalOut(_outputs)   TRISCCLR = (unsigned int)_outpust
#define mPORTCOutputConfig(_outputs)        TRISCCLR = (unsigned int)_outputs
#define mPORTCInputConfig(_inputs)          TRISCSET = (unsigned int)_inputs
#define mPORTCSetBits(_bits)                LATCSET = (unsigned int)_bits
#define mPORTCClearBits(_bits)              LATCCLR = (unsigned int)_bits
#define mPORTCToggleBits(_bits)             LATCINV = (unsigned int)_bits
#define mPORTCCloseAll()                    (TRISCSET = 0xFFFFFFFF,\
                                             LATCCLR = 0xFFFFFFFF)
#define mPORTCCloseBits(_bits)              (TRISCSET = (unsigned int)_bits,\
                                             LATCCLR = (unsigned int)_bits)
#define mPORTCOpenDrainOpen(_bits)          (ODCFGCSET = (unsigned int)_bits,\
                                             TRISCCLR = (unsigned int)_bits)
#define mPORTCOpenDrainClose(_bits)         (ODCFGCCLR = (unsigned int)_bits,\
                                             TRISCSET = (unsigned int)_bits)
#endif  // end PORTC

#if defined _PORTD
/******************************************************************************
 * PORTD macros
 *****************************************************************************/
#define mPORTDConfig(_tris)                 TRISD = (unsigned int)_tris
#define mPORTDRead()                        PORTD
#define mPORTDWrite(_lat)                   LATD = (unsigned int)_lat
#define mPORTDReadLatch()                   LATD
#define mPORTDGetConfig()                   TRISD
#define mPORTDReadBit(_bits)                (PORTD & (unsigned int)_bits)
#define mPORTDReadLatchBit(_bits)           (LATD & (unsigned int)_bits)
#define mPORTDReadConfigBit(_bits)          (TRISD & (unsigned int)_bits)

#define mPORTDSetPinsDigitalIn(_inputs)     TRISDSET = (unsigned int)_inputs
#define mPORTDSetPinsDigitalOut(_outputs)   TRISDCLR = (unsigned int)_outputs
#define mPORTDOutputConfig(_outputs)        TRISDCLR = (unsigned int)_outputs
#define mPORTDInputConfig(_inputs)          TRISDSET = (unsigned int)_inputs
#define mPORTDSetBits(_bits)                LATDSET = (unsigned int)_bits
#define mPORTDClearBits(_bits)              LATDCLR = (unsigned int)_bits
#define mPORTDToggleBits(_bits)             LATDINV = (unsigned int)_bits
#define mPORTDCloseAll()                    (TRISDSET = 0xFFFFFFFF,\
                                             LATDCLR = 0xFFFFFFFF)
#define mPORTDCloseBits(_bits)              (TRISDSET = (unsigned int)_bits,\
                                             LATDCLR = (unsigned int)_bits)
#define mPORTDOpenDrainOpen(_bits)          (TRISDCLR = (unsigned int)_bits,\
                                             ODCFGDSET = (unsigned int)_bits)
#define mPORTDOpenDrainClose(_bits)         (TRISDSET = (unsigned int)_bits,\
                                             ODCFGDCLR = (unsigned int)_bits)
#endif  // end PORTD

#if defined _PORTE
/******************************************************************************
 * PORTE macros
 *****************************************************************************/
#define mPORTEConfig(_tris)                 TRISE = (unsigned int)_tris
#define mPORTERead()                        PORTE
#define mPORTEWrite(_lat)                   LATE = (unsigned int)_lat
#define mPORTEReadLatch()                   LATE
#define mPORTEGetConfig()                   TRISE
#define mPORTEReadBit(_bits)                (PORTE & (unsigned int)_bits)
#define mPORTEReadLatchBit(_bits)           (LATE & (unsigned int)_bits)
#define mPORTEReadConfigBit(_bits)          (TRISE & (unsigned int)_bits)

#define mPORTESetPinsDigitalIn(_inputs)     TRISESET = (unsigned int)_inputs
#define mPORTESetPinsDigitalOut(_outputs)   TRISECLR = (unsigned int)_outputs
#define mPORTEOutputConfig(_outputs)        TRISECLR = (unsigned int)_outputs
#define mPORTEInputConfig(_inputs)          TRISESET = (unsigned int)_inputs
#define mPORTESetBits(_bits)                LATESET = (unsigned int)_bits
#define mPORTEClearBits(_bits)              LATECLR = (unsigned int)_bits
#define mPORTEToggleBits(_bits)             LATEINV = (unsigned int)_bits
#define mPORTECloseAll()                    (TRISESET = 0xFFFFFFFF,\
                                             LATECLR = 0xFFFFFFFF)
#define mPORTECloseBits(_bits)              (TRISESET = (unsigned int)_bits,\
                                             LATECLR = (unsigned int)_bits)
#define mPORTEOpenDrainOpen(_bits)          (ODCFGESET = (unsigned int)_bits,\
                                             TRISECLR = (unsigned int)_bits)
#define mPORTEOpenDrainClose(_bits)         (ODCFGECLR = (unsigned int)_bits,\
                                             TRISESET = (unsigned int)_bits)
#endif  // end PORTE

#if defined _PORTF
/******************************************************************************
 * PORTF macros
 *****************************************************************************/
#define mPORTFConfig(_tris)                 TRISF = (unsigned int)_tris
#define mPORTFRead()                        PORTF
#define mPORTFWrite(_lat)                   LATF = (unsigned int)_lat
#define mPORTFReadLatch()                   LATF
#define mPORTFGetConfig()                   TRISF
#define mPORTFReadBit(_bits)                (PORTF & (unsigned int)_bits)
#define mPORTFReadLatchBit(_bits)           (LATF & (unsigned int)_bits)
#define mPORTFReadConfigBit(_bits)          (TRISF & (unsigned int)_bits)

#define mPORTFSetPinsDigitalIn(_inputs)     TRISFSET = (unsigned int)_inputs
#define mPORTFSetPinsDigitalOut(_outputs)   TRISFCLR = (unsigned int)_outputs
#define mPORTFOutputConfig(_outputs)        TRISFCLR = (unsigned int)_outputs
#define mPORTFInputConfig(_inputs)          TRISFSET = (unsigned int)_inputs
#define mPORTFSetBits(_bits)                LATFSET = (unsigned int)_bits
#define mPORTFClearBits(_bits)              LATFCLR = (unsigned int)_bits
#define mPORTFToggleBits(_bits)             LATFINV = (unsigned int)_bits
#define mPORTFCloseAll()                    (TRISFSET = 0xFFFFFFFF,\
                                             LATFCLR = 0xFFFFFFFF)
#define mPORTFCloseBits(_bits)              (TRISFSET = (unsigned int)_bits,\
                                             LATFCLR = (unsigned int)_bits)
#define mPORTFOpenDrainOpen(_bits)          (ODCFGFSET = (unsigned int)_bits,\
                                             TRISFCLR = (unsigned int)_bits)
#define mPORTFOpenDrainClose(_bits)         (ODCFGFCLR = (unsigned int)_bits,\
                                             TRISFSET = (unsigned int)_bits)
#endif  // end PORTF

#if defined _PORTG
/******************************************************************************
 * PORTG macros
 *****************************************************************************/
#define mPORTGConfig(_tris)                 TRISG = (unsigned int)_tris
#define mPORTGSetPinsDigitalIn(_bits)       TRISGSET = (unsigned int)_bits
#define mPORTGSetPinsDigitalOut(_bits)      TRISGCLR = (unsigned int)_bits
#define mPORTGRead()                        PORTG
#define mPORTGWrite(_lat)                   LATG = (unsigned int)_lat
#define mPORTGReadLatch()                   LATG
#define mPORTGGetConfig()                   TRISG
#define mPORTGReadBit(_bits)                (PORTG & (unsigned int)_bits)
#define mPORTGReadLatchBit(_bits)           (LATG & (unsigned int)_bits)
#define mPORTGReadConfigBit(_bits)          (TRISG & (unsigned int)_bits)

#define mPORTGOutputConfig(_outputs)        TRISGCLR = (unsigned int)_outputs
#define mPORTGInputConfig(_inputs)          TRISGSET = (unsigned int)_inputs
#define mPORTGSetBits(_bits)                LATGSET = (unsigned int)_bits
#define mPORTGClearBits(_bits)              LATGCLR = (unsigned int)_bits
#define mPORTGToggleBits(_bits)             LATGINV = (unsigned int)_bits
#define mPORTGCloseAll()                    (TRISGSET = 0xFFFFFFFF,\
                                             LATGCLR = 0xFFFFFFFF)
#define mPORTGCloseBits(_bits)              (TRISGSET = (unsigned int)_bits,\
                                             LATGCLR = (unsigned int)_bits)
#define mPORTGOpenDrainOpen(_bits)          (ODCFGGSET = (unsigned int)_bits,\
                                             TRISGCLR = (unsigned int)_bits)
#define mPORTGOpenDrainClose(_bits)         (ODCFGGCLR = (unsigned int)_bits,\
                                             TRISGSET = (unsigned int)_bits)
#endif  // end PORTG


/******************************************************************************
 * Parameter values to be used with mJTAGEnable()
 *****************************************************************************/
#define DEBUG_JTAG_ON                       (1)
#define DEBUG_JTAG_OFF                      (0)

/******************************************************************************
 * JTAG macro       void mJTAGEnable(unsigned int enable)
 *
 * Function:
 *			        To enable/disable JTAG module
 *					mJTAGEnable(int _enable)
 *
 * Description:		Enables or disables JTAG port.
 *
 * PreCondition:    None
 *
 * Inputs:			0 = disable, 1 = enable
 *
 * Output:          None
 *
 * Example:			mJTAGEnable(1)
 *
 *****************************************************************************/
// disabled until we have an address for ORBCFG0
//#define mJTAGEnable(_enable)                   (ORBCFG0 &= ~(1 << 4),\
                                                ORBCFG0 |= _enable)
#endif  // _PORTS_H_
