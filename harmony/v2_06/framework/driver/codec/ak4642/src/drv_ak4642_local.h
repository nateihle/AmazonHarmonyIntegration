/*******************************************************************************
  AK4642 CODEC Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_AK4642_local.h

  Summary:
    AK4642 CODEC driver local declarations and definitions

  Description:
    This file contains the AK4642 CODEC driver's local declarations and definitions.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRV_AK4642_LOCAL_H
#define _DRV_AK4642_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "osal/osal.h"
#include "driver/codec/ak4642/drv_ak4642.h"
#include "driver/tmr/drv_tmr.h"
#include "driver/i2s/drv_i2s.h"
#include "peripheral/i2c/plib_i2c.h"
#include "driver/i2c/drv_i2c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants and Macros
// *****************************************************************************
// *****************************************************************************
/* AK4642 Driver I2C communication macros

  Summary:
    AK4642 driver I2C communication.

  Description:
    These constants provide AK4642 driver I2C communication information. 
    This includes the I2C address for the codec and other codec specific I2C macros if any.
    

  Remarks:
    AK4642 needs a dummy addr and byte to be sent to I2C module to start the communications.
 */

#define AK4642_I2C_ADDR (0x26)


// *****************************************************************************
/* AK4642 command queue size

  Summary
    Specifies AK4642 command queue size.

  Description
    This type specifies the AK4642 command queue size.

  Remarks:
  	None.
*/

#define AK4642_COMMAND_QUEUE_BUFFER 5


/* AK4642 Driver Version Macros

  Summary:
    AK4642 driver version.

  Description:
    These constants provide AK4642 driver version information. The driver
    version is:
    DRV_AK4642_VERSION_MAJOR.DRV_AK4642_VERSION_MINOR[.<DRV_AK4642_VERSION_PATCH>][DRV_AK4642_VERSION_TYPE]
    It is represented in DRV_I2S_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_AK4642_VERSION_STR.
    DRV_AK4642_VERSION_TYPE provides the type of the release when the
    release is a(alpha) or b(beta). The interfaces DRV_AK4642_VersionGet
    and DRV_AK4642_VersionStrGet provide interfaces to the access the
    version and the version string.

  Remarks:
    Modify the return value of DRV_I2S_VersionStrGet and the
    DRV_AK4642_VERSION_MAJOR, DRV_AK4642_VERSION_MINOR,
    DRV_AK4642_VERSION_PATCH, and DRV_AK4642_VERSION_TYPE.
 */

#define _DRV_AK4642_VERSION_MAJOR         0
#define _DRV_AK4642_VERSION_MINOR         1
#define _DRV_AK4642_VERSION_PATCH         0
#define _DRV_AK4642_VERSION_TYPE          a
#define _DRV_AK4642_VERSION_STR           "0.1.0a"


// *****************************************************************************
/* AK4642 DMA transfer abort

  Summary:
    AK4642 DMA transfer abort

  Description:
    This constant indicates that the AK4642 DMA transfer is aborted.

  Remarks:
    None.
*/
#define DRV_AK4642_DMA_TRANSFER_ABORT /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/


// *****************************************************************************
/* AK4642 DMA transfer error

  Summary:
    AK4642 DMA transfer error

  Description:
    This constant indicates that the AK4642 DMA transfer has an address error.

  Remarks:
    None.
*/
#define DRV_AK4642_DMA_TRANSFER_ERROR /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/


// *****************************************************************************
/* AK4642 Master Clock from REFCLOCK

  Summary:
    AK4642 Master Clock from REFCLOCK

  Description:
    The following constants gives computation of divisor (RODIV) and
    trim value (ROTRIM) constants for generating master clock value to
    AK4642. The source t0 generate master clock is REFCLOCK.

  Remarks:
    None.
*/
#define DRV_AK4642_MCLK_RODIV(source, rate, ratio)  ((source / rate / (ratio << 1)) )
#define DRV_AK4642_MCLK_ROTRIM(source, rate, ratio) (((source / rate) & ((ratio << 1)-1)) )


// *****************************************************************************
/* AK4642 Control register bit fields

  Summary:
    AK4642 Control register bit fields

  Description:
    The following constants gives computation for writing to a particular bit or
    field in the AK4642 control registers

  Remarks:
    None.
*/
#define DRV_AK4642_CONTROL_REG_BIT_WRITE(regValue, pos, newvale)    \
                            ((regValue & ~(1<<(pos)) ) | ((0x1&(newvale))<<(pos)))

#define DRV_AK4642_CONTROL_REG_FIELD_WRITE(reg,mask,pos,val)        \
                            ((reg) & ~(mask)) | ((mask)&((val)<<(pos)))


// *****************************************************************************
// *****************************************************************************
// Section: CODEC 4642 Register Map
// *****************************************************************************
// *****************************************************************************

/*  Register Map for 4642

  Summary:
    Register Map for 4642, Registers listed have address 00H, 01H, 02H, 03H
	and so on, respectively

  Description:
    Registers in AK 4642 for programming and controlling various functions
	and features of the codec.
    Abbreviations for the Registers for 4642 are be used in individual bit names for
	identification purposes. e.g. Register AK4642A_REG_PWR_MGMT1, will have
        individual bits named as PWRMGMT1_PMADL_UP or PWRMGMT1_LSV_UP to indicate
        that the bits belong to the AK4642A_REG_PWR_MGMT1 register.


  Remarks: None
*/
typedef enum{
								
	AK4642A_REG_PWR_MGMT1=0,	//0x0		<--
	AK4642A_REG_PWR_MGMT2,		//1
	AK4642A_REG_SIG_SLCT1,		//2
	AK4642A_REG_SIG_SLCT2,		//3
        AK4642A_REG_MODE_CTRL1,		//4
	AK4642A_REG_MODE_CTRL2,		//5
        AK4642A_REG_TMR_SLCT,		//6
        AK4642A_REG_ALC_MODE_CTRL1,     //7
	AK4642A_REG_ALC_MODE_CTRL2,	//8
        AK4642A_REG_LIN_VOL_CTRL,       //9
        AK4642A_REG_LDIG_VOL_CTRL,      //A
        AK4642A_REG_ALC_MODE_CTRL3,     //B
        AK4642A_REG_RIN_VOL_CTRL,       //C
        AK4642A_REG_RDIG_VOL_CTRL,      //D
        AK4642A_REG_MODE_CTRL3,         //E
        AK4642A_REG_MODE_CTRL4,         //F
        AK4642A_REG_PWR_MGMT3,          //10
        AK4642A_REG_DFLTR_SLCT,         //11
	AK4642A_REG_FIL3_0,             //12
        AK4642A_REG_FIL3_1,             //13
        AK4642A_REG_FIL3_2,             //14
        AK4642A_REG_FIL3_3,             //15
        AK4642A_REG_EQ_0,               //16
        AK4642A_REG_EQ_1,               //17
        AK4642A_REG_EQ_2,               //18
        AK4642A_REG_EQ_3,               //19
        AK4642A_REG_EQ_4,               //1A
        AK4642A_REG_EQ_5,               //1B
       	AK4642A_REG_FIL1_0,             //1C
        AK4642A_REG_FIL1_1,             //1D
        AK4642A_REG_FIL1_2,             //1E
        AK4642A_REG_FIL1_3,             //1F

}DRV_AK4642_CONTROL_REGISTER;

/* Register bit fields. */
/*  Individual Register expansions

  Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.

    //Individual register bits definitions
    //e.g Reg PowerManagement1 has 8 bits described as below :
    Reg                 D7        D6     D5	  D4	 D3	  D2      D1     D0
    PowerManagement1	0	PMVCM	PMBP	PMPSK	PMLO	PMDAC	  0	PMADL

    These control bits are reflected as enum
                                            {
                                                PwrMgmt1_PMADL = 0x1,
                                                PwrMgmt1_PMDAC = 0x4,
                                                PwrMgmt1_PMLO  = 0x8,
                                                PwrMgmt1_PMSPK = 0x10,
                                                PwrMgmt1_PMBP  = 0x20,
                                                PwrMgmt1_PMVCM = 0x40
                                            } PowerManagement1;

  Remarks:
    None.
*/
typedef enum
{
    PwrMgmt1_PMADL = 0x1,   //Mic Amp Lch and ADC Lch Power
    PwrMgmt1_PMDAC = 0x4,   //DAC Power
    PwrMgmt1_PMLO  = 0x8,   //Stereo Line out Power
    PwrMgmt1_PMSPK = 0x10,  //Speaker Amp Power
    PwrMgmt1_PMBP  = 0x20,  //Mono Input (MIN) Power
    PwrMgmt1_PMVCM = 0x40   //VCOM Power
} PowerManagement1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    PwrMgmt2_PMPLL = 0x1,   //PLL Power
    PwrMgmt2_MCKO = 0x2,    //Master CLK Output Enable
    PwrMgmt2_MS = 0x8,      //Master / Slave select
    PwrMgmt2_PMHPR = 0x10,  //Headphone Amp Rch Power
    PwrMgmt2_PMHPL = 0x20,  //Headphone Amp Lch power
    PwrMgmt2_HPMTN = 0x40,  //Headphone Amp Mute
} PowerManagement2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    SigSel1_MGAIN0 = 0x1,   //Mic gain
    SigSel1_PMMP = 0x4,     //Microphone power supply pin power management
    SigSel1_DACL = 0x10,    //Switch DAC to Stereo line out
    SigSel1_DACS = 0x20,    //Switch DAC to Speaker- Amp
    SigSel1_BEEPS = 0x40,   //Mono Input switched to Speaker- Amp
    SigSel1_SPPSN = 0x80,   //Speaker Amp Power save mode
} SignalSelect1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    SigSel2_BEEPL = 0x04,   //Switch from MIN to Stereo Line Output
    SigSel2_SPKG0 = 0x08,   //Speaker Amp Output gains select
    SigSel2_SPKG1 = 0x10,
    SigSel2_MGAIN1 = 0x20,  //MIC Amp gain
    SigSel2_LOPS = 0x40,    //Stereo Line Output PowerSave
    SigSel2_LOVL = 0x80     //Stereo Line Output Gain Select
} SignalSelect2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ModeCtrl1_DIF0 = 0x01,  //DIF 1-0 Audio Interface Mode select
    ModeCtrl1_DIF1 = 0x02,
    ModeCtrl1_BCKO = 0x08,  //Bit CLK Output freq select in Master mode
    ModeCtrl1_PLL0 = 0x10,  //PLL 3-0 Ref clock select, Default "0000" - LRCK pin
    ModeCtrl1_PLL1 = 0x20,
    ModeCtrl1_PLL2 = 0x40,
    ModeCtrl1_PLL3 = 0x80
} ModeControl1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ModeCtrl2_FS0 = 0x1,    //FS 3-0 Sampling freq select
    ModeCtrl2_FS1 = 0x2,
    ModeCtrl2_FS2 = 0x4,
    ModeCtrl2_FS3 = 0x20,
    ModeCtrl2_PS0 = 0x40,   //PS 1-0 MCKO Freq select, Default "00" - 256 times fs
    ModeCtrl2_PS1 = 0x80
} ModeControl2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    TimeSel_WTM0 = 0x4,     //WTM 1-0 ALC Recovery waiting period
    TimeSel_WTM1 = 0x8,
    TimeSel_ZTM0 = 0x10,    //ZTM 1-0 ALC Limiter / Recovery Zero crossing Timeout period
    TimeSel_ZTM1 = 0x20,
    TimeSel_DVTM = 0x80     //Digital volume transition time setting
} TimerSelect;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ALCCtrl1_LMTH0 = 0x1,   //LMTH 1-0 ALC Limiter detection level / recovery counter reset level
    ALCCtrl1_RGAIN0 = 0x2,  //Rgain 1-0 ALC recovery gain step
    ALCCtrl1_LMAT0 = 0x4,   //LMAT 1-0 ALC limited ATT step
    ALCCtrl1_LMAT1 = 0x8,
    ALCCtrl1_ZELMN = 0x10,  //Zero crossing detection enable
    ALCCtrl1_ALC = 0x20     //ALC enable
} ALCModeControl1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ALCCtrl2_REF0 = 0x1,    //Reference value for ALC operation
    ALCCtrl2_REF1 = 0x2,
    ALCCtrl2_REF2 = 0x4,
    ALCCtrl2_REF3 = 0x8,
    ALCCtrl2_REF4 = 0x10,
    ALCCtrl2_REF5 = 0x20,
    ALCCtrl2_REF6 = 0x40,
    ALCCtrl2_REF7 = 0x80
} ALCModeControl2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    LInpVol_IVL0 = 0x1,     //Input Digital Volume, default "E1" +30dB
    LInpVol_IVL1 = 0x2,
    LInpVol_IVL2 = 0x4,
    LInpVol_IVL3 = 0x8,
    LInpVol_IVL4 = 0x10,
    LInpVol_IVL5 = 0x20,
    LInpVol_IVL6 = 0x40,
    LInpVol_IVL7 = 0x80
} LchInputVolumeControl;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    LDigVol_DVL0 = 0x1,     //Output Digital Volume, default "18" 0dB
    LDigVol_DVL1 = 0x2,
    LDigVol_DVL2 = 0x4,
    LDigVol_DVL3 = 0x8,
    LDigVol_DVL4 = 0x10,
    LDigVol_DVL5 = 0x20,
    LDigVol_DVL6 = 0x40,
    LDigVol_DVL7 = 0x80
} LchDigitalVolumeControl;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ALCCtrl3_LMTH1 = 0x40,  //LMTH 1-0 ALC Limiter detection level / recovery counter reset level
    ALCCtrl3_RGAIN1 = 0x80  //Rgain 1-0 ALC recovery gain step
} ALCModeControl3;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    RInpVol_RVL0 = 0x1,     //Input Digital Volume, default "E1"H ~ +30dB
    RInpVol_RVL1 = 0x2,
    RInpVol_RVL2 = 0x4,
    RInpVol_RVL3 = 0x8,
    RInpVol_RVL4 = 0x10,
    RInpVol_RVL5 = 0x20,
    RInpVol_RVL6 = 0x40,
    RInpVol_RVL7 = 0x80
} RchInputVolumeControl;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    RDigVol_DVL0 = 0x1,     //Output Digital Volume, default "18"H ~ 0dB
    RDigVol_DVL1 = 0x2,
    RDigVol_DVL2 = 0x4,
    RDigVol_DVL3 = 0x8,
    RDigVol_DVL4 = 0x10,
    RDigVol_DVL5 = 0x20,
    RDigVol_DVL6 = 0x40,
    RDigVol_DVL7 = 0x80
} RchDigitalVolumeControl;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ModeCtrl3_DEM0 = 0x1,   //De-emphasis freq, default "01 (OFF)
    ModeCtrl3_DEM1 = 0x2,
    ModeCtrl3_BST0 = 0x4,   //Bass boost function select, default "00" (OFF)
    ModeCtrl3_BST1 = 0x8,
    ModeCtrl3_DVOLC = 0x10, //Output digital volume control mode - Independent or Dependant Lch and Rch channels
    ModeCtrl3_SMUTE = 0x20, //Soft mute
    ModeCtrl3_LOOP = 0x40   //Digital loop back from Mic to Headphone internally within CODEC
} ModeControl3;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    ModeCtrl4_DACH = 0x1,   //Switch control from DAC to Headphone Amp
    ModeCtrl4_BEEPH = 0x2,  //Switch control from MIN pin to headphone amp
    ModeCtrl4_HPM = 0x4,    //Headphone Amp Mono output select
    ModeCtrl4_IVOLC = 0x8   //Input digital volume control mode - Independent or Dependant Lch and Rch channels
} ModeControl4;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    PwrMgmt3_PMADR = 0x1,   //Mic Amp Lch and Rch Power
    PwrMgmt3_INL   = 0x2,   //ADC Lch Input source - INT / EXT Mic
    PwrMgmt3_INR   = 0x4,   //ADC Rch Input source - INT / EXT Mic
    PwrMgmt3_MDIF1 = 0x8,   //ADC Lch Single Ended / Differential
    PwrMgmt3_MDIF2 = 0x10,  //ADC Rch Single Ended / Differential
    PwrMgmt3_HPG = 0x20,    //Headphone Amp Gain select default "0" 0dB
} PowerManagement3;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    DigFilt_FIL3 = 0x4,     //Stereo separation emphasis filter Coefficient setting
    DigFilt_EQ = 0x8,       //Gain compensation filter coefficient setting
    DigFilt_FIL1 = 0x10,    //Wind noise reduction filter coefficient setting
    DigFilt_GN0 = 0x40,     //GN 1-0 Gain select, default "00" 0dB
    DigFilt_GN1 = 0x80
} DigitalFilterSelect;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F3Coeff0_F3A0 = 0x1,    //F3 A13-0, B13-0 - Stereo separation emphasis filter coefficient
    F3Coeff0_F3A1 = 0x2,
    F3Coeff0_F3A2 = 0x4,
    F3Coeff0_F3A3 = 0x8,
    F3Coeff0_F3A4 = 0x10,
    F3Coeff0_F3A5 = 0x20,
    F3Coeff0_F3A6 = 0x40,
    F3Coeff0_F3A7 = 0x80,
} FIL3Coefficient0;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F3Coeff1_F3A8 = 0x1,    //F3 A13-0, B13-0 - Stereo separation emphasis filter coefficient
    F3Coeff1_F3A9 = 0x2,
    F3Coeff1_F3A10 = 0x4,
    F3Coeff1_F3A11 = 0x8,
    F3Coeff1_F3A12 = 0x10,
    F3Coeff1_F3A13 = 0x20,
    F3Coeff1_F3AS = 0x80,
} FIL3Coefficient1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F3Coeff2_F3B0 = 0x1,    //F3 A13-0, B13-0 - Stereo separation emphasis filter coefficient
    F3Coeff2_F3B1 = 0x2,
    F3Coeff2_F3B2 = 0x4,
    F3Coeff2_F3B3 = 0x8,
    F3Coeff2_F3B4 = 0x10,
    F3Coeff2_F3B5 = 0x20,
    F3Coeff2_F3B6 = 0x40,
    F3Coeff2_F3B7 = 0x80,
} FIL3Coefficient2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F3Coeff3_F3B8 = 0x1,    //F3 A13-0, B13-0 - Stereo separation emphasis filter coefficient
    F3Coeff3_F3B9 = 0x2,
    F3Coeff3_F3B10 = 0x4,
    F3Coeff3_F3B11 = 0x8,
    F3Coeff3_F3B12 = 0x10,
    F3Coeff3_F3B13 = 0x20,
} FIL3Coefficient3;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff0_EQA0 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff0_EQA1 = 0x2,
    EQCoeff0_EQA2 = 0x4,
    EQCoeff0_EQA3 = 0x8,
    EQCoeff0_EQA4 = 0x10,
    EQCoeff0_EQA5 = 0x20,
    EQCoeff0_EQA6 = 0x40,
    EQCoeff0_EQA7 = 0x80,
} EQCoefficient0;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff1_EQA8 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff1_EQA9 = 0x2,
    EQCoeff1_EQA10 = 0x4,
    EQCoeff1_EQA11 = 0x8,
    EQCoeff1_EQA12 = 0x10,
    EQCoeff1_EQA13 = 0x20,
    EQCoeff1_EQA14 = 0x40,
    EQCoeff1_EQA15 = 0x80,
} EQCoefficient1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff2_EQB0 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff2_EQB1 = 0x2,
    EQCoeff2_EQB2 = 0x4,
    EQCoeff2_EQB3 = 0x8,
    EQCoeff2_EQB4 = 0x10,
    EQCoeff2_EQB5 = 0x20,
    EQCoeff2_EQB6 = 0x40,
    EQCoeff2_EQB7 = 0x80,
} EQCoefficient2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff3_EQB8 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff3_EQB9 = 0x2,
    EQCoeff3_EQB10 = 0x4,
    EQCoeff3_EQB11 = 0x8,
    EQCoeff3_EQB12 = 0x10,
    EQCoeff3_EQB13 = 0x20,
} EQCoefficient3;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff4_EQC0 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff4_EQC1 = 0x2,
    EQCoeff4_EQC2 = 0x4,
    EQCoeff4_EQC3 = 0x8,
    EQCoeff4_EQC4 = 0x10,
    EQCoeff4_EQC5 = 0x20,
    EQCoeff4_EQC6 = 0x40,
    EQCoeff4_EQC7 = 0x80,
} EQCoefficient4;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    EQCoeff5_EQC8 = 0x1,    //EQ A15-0, B13-0, C15-0 - Gain compensation filter coefficient
    EQCoeff5_EQC9 = 0x2,
    EQCoeff5_EQC10 = 0x4,
    EQCoeff5_EQC11 = 0x8,
    EQCoeff5_EQC12 = 0x10,
    EQCoeff5_EQC13 = 0x20,
    EQCoeff5_EQC14 = 0x40,
    EQCoeff5_EQC15 = 0x80,
} EQCoefficient5;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F1Coeff0_F1A0 = 0x1,    //Fil1 A13-0, B13-0 - Wind noise reduction filter coefficient
    F1Coeff0_F1A1 = 0x2,
    F1Coeff0_F1A2 = 0x4,
    F1Coeff0_F1A3 = 0x8,
    F1Coeff0_F1A4 = 0x10,
    F1Coeff0_F1A5 = 0x20,
    F1Coeff0_F1A6 = 0x40,
    F1Coeff0_F1A7 = 0x80,
} FIL1Coefficient0;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F1Coeff1_F1A8 = 0x1,    //Fil1 A13-0, B13-0 - Wind noise reduction filter coefficient
    F1Coeff1_F1A9 = 0x2,
    F1Coeff1_F1A10 = 0x4,
    F1Coeff1_F1A11 = 0x8,
    F1Coeff1_F1A12 = 0x10,
    F1Coeff1_F1A13 = 0x20,
    F1Coeff1_F1AS = 0x80,
} FIL1Coefficient1;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F1Coeff2_F1B0 = 0x1,    //Fil1 A13-0, B13-0 - Wind noise reduction filter coefficient
    F1Coeff2_F1B1 = 0x2,
    F1Coeff2_F1B2 = 0x4,
    F1Coeff2_F1B3 = 0x8,
    F1Coeff2_F1B4 = 0x10,
    F1Coeff2_F1B5 = 0x20,
    F1Coeff2_F1B6 = 0x40,
    F1Coeff2_F1B7 = 0x80,
} FIL1Coefficient2;

/*  Individual Register expansions

  Summary:
    Summary:
    Each register is expanded as an enum with the control bit enumerated

  Description:
    Each register is expanded as an enum with the control bit enumerated.
    The value of each enumerated control bit is the location of the bit as in
	an 8-bit register.
    The prefix before the underscore in every bit name is the reg name
	abbreviation specified above.
    For detalied e.g refer to PowerManagement1
  Remarks:
*/
typedef enum
{
    F1Coeff3_F1B8 = 0x1,    //Fil1 A13-0, B13-0 - Wind noise reduction filter coefficient
    F1Coeff3_F1B9 = 0x2,
    F1Coeff3_F1B10 = 0x4,
    F1Coeff3_F1B11 = 0x8,
    F1Coeff3_F1B12 = 0x10,
    F1Coeff3_F1B13 = 0x20,
} FIL1Coefficient3;


// *****************************************************************************
/* AK4642 Channel Attenuation

  Summary:
    AK4642 Channel Attenuation

  Description:
    The following constants left and right channel attenuation

  Remarks:
    None.
*/
#define DRV_AK4642_LATT_ATT(n)                      (n)
#define DRV_AK4642_RATT_ATT(n)                      (n)
#define DRV_AK4642_ATT_MASK                         0xFF


// *****************************************************************************
/* AK4642 Volume Mapping 

  Summary:
    AK4642 Volume Mapping

  Description:
    Volume gain for Audio applications using Codec / DAC needs to be scaled as the dB range supported by the Codec is too large for human hearing.
    The allowed range of the codec stretches from +12dB to -115 dB.
    However for most applications, the entire dB range is not audible.
    So we first define the modified dB range for the driver and then normalize the volume value for that range.
    by converting max gain in dB to max gain in counts, 0x00 to 0xFF
    The MAximum and minimum values specified can be a floating point number.  It can be negative as well.
   (It is converted to codec gain counts before being used in the code.)
  Remarks:
    None.
*/

#define MAX_AUDIO_OUTPUT_GAIN_IN_dB   12.0
#define MIN_AUDIO_OUTPUT_GAIN_IN_dB  -60.0
#define MAX_AUDIO_GAIN  ( MAX_AUDIO_OUTPUT_GAIN_IN_dB >   12 ?   12 : MAX_AUDIO_OUTPUT_GAIN_IN_dB )
#define MIN_AUDIO_GAIN  ( MIN_AUDIO_OUTPUT_GAIN_IN_dB < -115 ? -115 : MIN_AUDIO_OUTPUT_GAIN_IN_dB )

#define MAX_VOLUME_CODE_FLOAT ( 256.0 - 2.0*(12.0-MAX_AUDIO_GAIN) )
#define MAX_VOLUME_CODE_ROUND (uint8_t) MAX_VOLUME_CODE_FLOAT 
#define MAX_VOLUME_CODE       ( MAX_VOLUME_CODE_FLOAT > 255 ? 255 : MAX_VOLUME_CODE_ROUND )

#define MIN_VOLUME_CODE_FLOAT ( 256.0 - 2.0*(12.0-MIN_AUDIO_GAIN) )
#define MIN_VOLUME_CODE_ROUND (int8_t)MIN_VOLUME_CODE_FLOAT 
#define MIN_VOLUME_CODE       ( MIN_VOLUME_CODE_ROUND )
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* AK4642 Supported control commands

  Summary:
    AK4642 Supported control commands

  Description:
    This enumeration identifies AK4642 supported control commands

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4642_COMMAND_NONE,
    DRV_AK4642_COMMAND_INIT_CLK_PDN_SET,
    DRV_AK4642_COMMAND_INIT_START,
    DRV_AK4642_COMMAND_INIT_AUDIO_FORMAT,
    DRV_AK4642_COMMAND_INIT_END,
    DRV_AK4642_COMMAND_SAMPLING_RATE_SET,
    DRV_AK4642_COMMAND_VOLUME_SET_CHANNEL_LEFT,
    DRV_AK4642_COMMAND_VOLUME_SET_CHANNEL_RIGHT,
    DRV_AK4642_COMMAND_VOLUME_SET_CHANNEL_LEFT_ONLY,
    DRV_AK4642_COMMAND_VOLUME_SET_CHANNEL_RIGHT_ONLY,
    DRV_AK4642_COMMAND_VOLUME_SET_CHANNELS_INIT,
    DRV_AK4642_COMMAND_MUTE_ON,
    DRV_AK4642_COMMAND_MUTE_OFF,
    DRV_AK4642_COMMAND_DIGITAL_BLOCK_CONTROL_SET,
    DRV_AK4642_COMMAND_INT_EXT_MIC_SET,
    DRV_AK4642_COMMAND_MIC_SET,
    DRV_AK4642_COMMAND_MONO_STEREO_MIC_SET,
    DRV_AK4642_COMMAND_SEND,
    DRV_AK4642_COMMAND_COMPLETE
}DRV_AK4642_COMMAND;


// *****************************************************************************
/* AK4642 supported sampling rates

  Summary:
    AK4642 supported sampling rates

  Description:
    This enumeration identifies AK4642 supported sampling rates

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4642_SAMPLERATE_192000HZ = 192000,
    DRV_AK4642_SAMPLERATE_176400HZ = 176400,
    DRV_AK4642_SAMPLERATE_120000HZ = 120000,
    DRV_AK4642_SAMPLERATE_96000HZ =  96000,
    DRV_AK4642_SAMPLERATE_88200HZ =  88200,
    DRV_AK4642_SAMPLERATE_60000HZ =  60000,
    DRV_AK4642_SAMPLERATE_48000HZ =  48000,
    DRV_AK4642_SAMPLERATE_44100HZ =  44100,
    DRV_AK4642_SAMPLERATE_32000HZ =  32000,
    DRV_AK4642_SAMPLERATE_24000HZ =  24000,
    DRV_AK4642_SAMPLERATE_16000HZ =  16000,
    DRV_AK4642_SAMPLERATE_8000HZ =   8000,
    DRV_AK4642_SAMPLERATE_DEFAULT =  44100
} DRV_AK4642_SAMPLERATE;


// *****************************************************************************
/* AK4642 Driver task states

  Summary
    Lists the different states that AK4642 Driver task routine can have.

  Description
    This enumeration lists the different states that AK4642 Driver task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AK4642_TASK_STATE_CONTROL,
    DRV_AK4642_TASK_STATE_DATA_IN,
    DRV_AK4642_TASK_STATE_DATA_OUT,
} DRV_AK4642_TASK;


/**********************************************
 * Driver Client Obj
 **********************************************/
typedef struct _DRV_AK4642_CLIENT_OBJ_STRUCT
{
    /* Indicates that this object is in use */
    bool inUse;

    /* Indicate whether the client is open in
     * read,write or read/write mode */
    DRV_IO_INTENT ioIntent;

    /* Indicate the possible events that can 
     * result from a buffer add request */

    /* Call back function for this client */
    DRV_AK4642_BUFFER_EVENT_HANDLER  pEventCallBack;

    /* Client data(Event Context) that will be
     * returned at callback */
    uintptr_t hClientArg;

    /* pointer to the driver that own this object */
    void* hDriver;

} DRV_AK4642_CLIENT_OBJ;




/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_AK4642_OBJ_STRUCT
{

    /* Status of this driver instance */
    SYS_STATUS status;

    /* Indicates this object is in use */
    bool inUse;

    /* Flag to indicate that the hardware instance is used
     *  in exclusive access mode */
    bool isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t numClients;

    /* Identifies data module(I2S) driver ID for
     * data interface of CODEC */
    SYS_MODULE_INDEX i2sDriverModuleIndex;

    /* Identifies data module(I2C) driver ID for
     * control interface of CODEC */
    SYS_MODULE_INDEX i2cDriverModuleIndex;
    /* Identifies data module(I2S) driver open handle */
    DRV_HANDLE i2sDriverHandle;

    DRV_HANDLE i2sDriverClientHandleRead;

    DRV_HANDLE i2sDriverClientHandleWrite;

    /* Identifies control module timer ID for
     * control interface of CODEC */
    SYS_MODULE_INDEX tmrDriverModuleIndex;

    /* Identifies control module(Timer) driver open handle */    
    DRV_HANDLE tmrDriverHandle;

    /* Sampling rate */
    uint32_t samplingRate;
    
    /* Identifies the Audio data format */
    DRV_AK4642_AUDIO_DATA_FORMAT audioDataFormat;

     /* Keeps track if the driver is in interrupt
     * context */
    bool isInInterruptContext;

    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE(mutexDriverInstance);


    /* AK4642 hardware object task state */
    DRV_AK4642_TASK task;

    /* ----------------------------------------------------*/
    /* Control interface specific Implementation variables */
    /* ----------------------------------------------------*/

    /* Volume for volume command */
    uint8_t volume[DRV_AK4642_NUMBER_OF_CHANNELS];

    /* Volume set under process */
    bool isVolumeSetUnderProcess;

    /* Command under execution */
    DRV_AK4642_COMMAND command;

    /* Command value being transmitted */
    uint32_t controlCommand;

    /* Status of command under execution */
    bool controlCommandStatus;

    /* Number of bits transfered for a control command */
    uint8_t countBit;

    /* Array holding the last programmed command value */
    uint8_t lastRegValue[30];

    /* Command complete callback function */
    DRV_AK4642_COMMAND_EVENT_HANDLER commandCompleteCallback;

    /* command complete event context */
    uintptr_t commandContextData;

    uint16_t mclk_multiplier;
    
    uintptr_t drvI2CMasterHandle;
    uintptr_t drvI2CBuffHandle;
    
} DRV_AK4642_OBJ;


// *****************************************************************************
/* AK4642 Driver Global Instances Object

  Summary:
    Object used to keep track of data that is common to all instances of the
    AK4642 driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the AK4642 driver.

  Remarks:
    None.
*/

typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool membersAreInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexClientObjects);

} DRV_AK4642_COMMON_DATA_OBJ;

// *****************************************************************************
/* AK4642 Driver Control Command Object

  Summary:
    Object used to keep track of control data.

  Description:
    Object used to keep track of control data.

  Remarks:
    None.
*/
typedef struct{
    uint8_t command;
    uint8_t control_data[3];
    uint8_t array_size;
    DRV_I2C_BUFFER_HANDLE drvI2CBufHandle;
}AK4642_COMMAND;

// *****************************************************************************
/* AK4642 Driver Control Command Queue

  Summary:
    Object used to keep track of AK4642 Command Queue.

  Description:
    Object used to keep track of AK4642 Command Queue.

  Remarks:
    None.
*/
typedef struct{
    AK4642_COMMAND commandBuffer[AK4642_COMMAND_QUEUE_BUFFER];
    int8_t queueIn;
    int8_t queueOut;
    int8_t status; // Empty:0, Full:1, Normal:2
}AK4642_COMMAND_QUEUE;


/**************************************
 * Local functions.
 *************************************/

// *****************************************************************************
/* Function:
    void DRV_AK4642_VolumeReMapping( DRV_AK4642_OBJ* drvObj, DRV_AK4642_CHANNEL channel,uint8_t volume)

  Summary:
    Volume remapping to reverse the codec volume value to dB mapping which currently works reverse

  Description:
    Volume remapping to reverse the codec volume value to dB mapping which currently works reverse

  Precondition:
    DRV_AK4642_OBJ - driver object should be available

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK4642_Initialize routine
    volume          - 0 - 255 value supported by the codec for volume adjustment

  Returns:
    None.

  Remarks:

*/
static void DRV_AK4642_VolumeReMapping( DRV_AK4642_OBJ* drvObj, DRV_AK4642_CHANNEL channel, uint8_t volume);


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4642_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier)

  Summary:
    Generates the master clock(to AK4642) from REFCLOCK  for the given
    sampling rate.

  Description:
    Generates the master clock(to AK4642) from REFCLOCK  for the given
    sampling rate.

  Remarks:
    None
*/
static void _DRV_AK4642_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier);


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4642_ConrolRegisterSet (DRV_AK4642_OBJ *drvObj,
                DRV_AK4642_CONTROL_REGISTER contRegister, uint8_t value )

  Summary:
    Prepares the control command to be sent to AK4642. Also starts the
    timer to initiate the control command transfer.

  Description:
    Prepares the control command to be sent to AK4642. Also starts the
    timer to initiate the control command transfer.

  Remarks:
    None
*/
//static void _DRV_AK4642_ConrolRegisterSet (DRV_AK4642_OBJ *drvObj,
//                DRV_AK4642_CONTROL_REGISTER contRegister, uint32_t value );

// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4642_ConrolRegisterSet (DRV_AK4642_OBJ *drvObj,
                uint8_t *controlData, uint32_t size )

  Summary:
    Send control commands bytes to AK4953

  Description:
    Send control commands bytes to AK4953

  Remarks:
    None
*/
static uintptr_t _DRV_AK4642_ConrolRegisterSet
(
    DRV_AK4642_OBJ *drvObj,
    uint8_t *controlData,
    uint32_t size
);

// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4642_ControlTasks(DRV_AK4642_OBJ *drvObj)

  Summary:
    Implements the state maching for the Audio control interface of AK4642

  Description:
    Implements the state maching for the Audio control interface of AK4642

  Remarks:
    None
*/
static void _DRV_AK4642_ControlTasks(DRV_AK4642_OBJ *drvObj);








// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4642_I2SBufferEventHandler(DRV_I2S_BUFFER_EVENT event,
        DRV_I2S_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)

  Summary:
    Implements the handler for i2s buffer request completion.

  Description:
    Implements the handler for i2s buffer request completion.

  Remarks:
    None
*/
static void _DRV_AK4642_I2SBufferEventHandler
(
    DRV_I2S_BUFFER_EVENT event,
    DRV_I2S_BUFFER_HANDLE bufferHandle,
    uintptr_t contextHandle
);

 /*
  Function:
        static void _DRV_AK4642_CommandQueueGetSlot
        (
        )

  Summary:
    Get a free slot from AK4642 command queue.

  Description:
    Get a free slot from AK4642 command queue.

  Remarks:
    None
*/
static AK4642_COMMAND* _DRV_AK4642_CommandQueueGetSlot();
 /*
  Function:
        static void _DRV_AK4642_CommandQueuePop
        (
        )

  Summary:
    Pop up the top slot of AK4642 command queue.

  Description:
    Pop up the top slot of AK4642 command queue.

  Remarks:
    None
*/
static AK4642_COMMAND* _DRV_AK4642_CommandQueuePop();
 /*
  Function:
        static void _DRV_AK4642_CommandQueueTop
        (
        )

  Summary:
    Return the top slot of AK4642 command queue.

  Description:
    Return the top slot of AK4642 command queue.

  Remarks:
    None
*/
static AK4642_COMMAND* _DRV_AK4642_CommandQueueTop();
 /*
  Function:
        static uint8_t _DRV_AK4642_CONTROL_REG_FIELD_WRITE_Wrapper
        (
            DRV_AK4642_OBJ *drvObj, 
            uint8_t reg_addr, 
            uint8_t mask, 
            uint8_t pos, 
            uint8_t val
        )

  Summary:
    A wrapper function of DRV_AK4642_CONTROL_REG_FIELD_WRITE macro.

  Description:
    A wrapper function of DRV_AK4642_CONTROL_REG_FIELD_WRITE macro.

  Remarks:
    None
*/
static uint8_t _DRV_AK4642_CONTROL_REG_FIELD_WRITE_Wrapper(DRV_AK4642_OBJ *drvObj, uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t val);

 /*
  Function:
        static uint8_t _DRV_AK4642_CONTROL_REG_BIT_WRITE_Wrapper
        (
            DRV_AK4642_OBJ *drvObj, 
            uint8_t reg_addr, 
            uint8_t pos, 
            uint8_t val
        )

  Summary:
    A wrapper function of DRV_AK4642_CONTROL_REG_BIT_WRITE macro.

  Description:
    A wrapper function of DRV_AK4642_CONTROL_REG_BIT_WRITE macro.

  Remarks:
    None
*/
static uint8_t _DRV_AK4642_CONTROL_REG_BIT_WRITE_Wrapper(DRV_AK4642_OBJ *drvObj, uint8_t reg_addr, uint8_t pos, uint8_t val);

#endif // #ifndef _DRV_AK4642_LOCAL
/*******************************************************************************
 End of File
*/