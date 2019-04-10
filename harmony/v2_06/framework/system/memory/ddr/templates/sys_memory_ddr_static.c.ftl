/*******************************************************************************
  Memory System Service Functions for DDR Initialization

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_ddr_static.c

  Summary:
    Memory System Service implementation for the DDR controller.

  Description:
    The Memory System Service initializes the DDR Controller and PHY to 
    provide access to external DDR2 SDRAM.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "peripheral/ddr/plib_ddr.h"
#include "peripheral/devcon/plib_devcon.h"
#include "peripheral/power/plib_power.h"
#include "system/memory/ddr/sys_memory_ddr_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Memory System Service DDR static functions
// *****************************************************************************
// *****************************************************************************

#define sys_mem_ddr_max(a,b) (((a)>(b))?(a):(b))
#define sys_mem_ddr_round_up(x,y) (((x) + (y) - 1) / (y))
#define sys_mem_ddr_hc_clk_dly(dly) (sys_mem_ddr_max((sys_mem_ddr_round_up((dly),${CONFIG_SYS_MEMORY_DDR_CLK_PER})),2)-2)

static void DDR_PMD_Init(void)
{
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PMD_REGISTERS);
	PLIB_POWER_PeripheralModuleEnable(POWER_ID_0, POWER_MODULE_DDR2);
	PLIB_DEVCON_SystemLock(DEVCON_ID_0);
}

static void DDR_Init(void)
{
    uint32_t tmp;
    uint32_t ba_field, ma_field;

    /* Target Arbitration */
    PLIB_DDR_MinLimit(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINLIM_TGT0}, DDR_TARGET_0);
    PLIB_DDR_ReqPeriod(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_REQPER_TGT0}, DDR_TARGET_0);
    PLIB_DDR_MinCommand(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINCMD_TGT0}, DDR_TARGET_0);

    PLIB_DDR_MinLimit(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINLIM_TGT1}, DDR_TARGET_1);
    PLIB_DDR_ReqPeriod(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_REQPER_TGT1}, DDR_TARGET_1);
    PLIB_DDR_MinCommand(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINCMD_TGT1}, DDR_TARGET_1);

    PLIB_DDR_MinLimit(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINLIM_TGT2}, DDR_TARGET_2);
    PLIB_DDR_ReqPeriod(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_REQPER_TGT2}, DDR_TARGET_2);
    PLIB_DDR_MinCommand(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINCMD_TGT2}, DDR_TARGET_2);

    PLIB_DDR_MinLimit(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINLIM_TGT3}, DDR_TARGET_3);
    PLIB_DDR_ReqPeriod(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_REQPER_TGT3}, DDR_TARGET_3);
    PLIB_DDR_MinCommand(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINCMD_TGT3}, DDR_TARGET_3);

    PLIB_DDR_MinLimit(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINLIM_TGT4}, DDR_TARGET_4);
    PLIB_DDR_ReqPeriod(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_REQPER_TGT4}, DDR_TARGET_4);
    PLIB_DDR_MinCommand(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MINCMD_TGT4}, DDR_TARGET_4);

    /* Addressing */
    PLIB_DDR_RowAddressSet(DDR_ID_0, ROW_ADDR_RSHIFT, ROW_ADDR_MASK);
    PLIB_DDR_ColumnAddressSet(DDR_ID_0, COL_HI_RSHFT, COL_LO_MASK, COL_HI_MASK);
    PLIB_DDR_BankAddressSet(DDR_ID_0, BA_RSHFT, BANK_ADDR_MASK);
    PLIB_DDR_ChipSelectAddressSet(DDR_ID_0, CS_ADDR_RSHIFT, CS_ADDR_MASK);

    /* Refresh */
    PLIB_DDR_RefreshTimingSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRFC}, ${CONFIG_SYS_MEMORY_DDR_TRFI}, CTRL_CLK_PERIOD);
    PLIB_DDR_MaxPendingRefSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_MAX_PEND_REFS});
    PLIB_DDR_AutoSelfRefreshDisable(DDR_ID_0);

    /* Power */
<#if CONFIG_SYS_MEMORY_DDR_AUTO_PWR_DOWN = true>
    PLIB_DDR_AutoPowerDownEnable(DDR_ID_0);
<#else>
    PLIB_DDR_AutoPowerDownDisable(DDR_ID_0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_AUTO_PCHRG = true>
    PLIB_DDR_AutoPchrgEnable(DDR_ID_0);
<#else>
    PLIB_DDR_AutoPchrgDisable(DDR_ID_0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_AUTO_PCHRG = true>
    PLIB_DDR_AutoPchrgPowerDownEnable(DDR_ID_0);
<#else>
    PLIB_DDR_AutoPchrgPowerDownDisable(DDR_ID_0);
</#if>

    /* Timing */
    PLIB_DDR_ReadWriteDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN}, ${CONFIG_SYS_MEMORY_DDR_WR_LAT}, ${CONFIG_SYS_MEMORY_DDR_CAS_LAT});
    PLIB_DDR_WriteReadDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TWTR}, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN}, ${CONFIG_SYS_MEMORY_DDR_WR_LAT}, CTRL_CLK_PERIOD);
    PLIB_DDR_ReadReadDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN});
    PLIB_DDR_WriteWriteDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN});
    PLIB_DDR_SelfRefreshDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_SELF_REFRESH_DELAY}, ${CONFIG_SYS_MEMORY_DDR_TCKE}, ${CONFIG_SYS_MEMORY_DDR_TDLLK});
    PLIB_DDR_PowerDownDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PWR_DOWN_DELAY}, ${CONFIG_SYS_MEMORY_DDR_TCKE}, ${CONFIG_SYS_MEMORY_DDR_TXP});
    PLIB_DDR_PrechargAllBanksSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRP}, CTRL_CLK_PERIOD);
    PLIB_DDR_ReadToPrechargeDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRTP}, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN}, CTRL_CLK_PERIOD);
    PLIB_DDR_WriteToPrechargeDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TWR}, ${CONFIG_SYS_MEMORY_DDR_BRST_LEN}, ${CONFIG_SYS_MEMORY_DDR_WR_LAT}, CTRL_CLK_PERIOD);
    PLIB_DDR_PrechargeToRASDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRP}, CTRL_CLK_PERIOD);
    PLIB_DDR_RASToPrechargeDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRAS}, CTRL_CLK_PERIOD);
    PLIB_DDR_RASToRASBankDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRC}, CTRL_CLK_PERIOD);
    PLIB_DDR_RASToRASDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRRD}, CTRL_CLK_PERIOD);
    PLIB_DDR_RASToCASDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TRCD}, CTRL_CLK_PERIOD);
    PLIB_DDR_DataDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_CAS_LAT}, ${CONFIG_SYS_MEMORY_DDR_WR_LAT});
    PLIB_DDR_TfawDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_TFAW}, CTRL_CLK_PERIOD);

    /* On-Die Termination */
<#if CONFIG_SYS_MEMORY_DDR_ODT_READ_ENABLE == true>
    PLIB_DDR_OdtReadEnable(DDR_ID_0, 0);
    PLIB_DDR_OdtReadParamSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_ODT_READ_CLOCKS}, ${CONFIG_SYS_MEMORY_DDR_ODT_READ_DLY});
<#else>
    PLIB_DDR_OdtReadDisable(DDR_ID_0, 0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_ODT_WRITE_ENABLE == true>
    PLIB_DDR_OdtWriteEnable(DDR_ID_0, 0);
    PLIB_DDR_OdtWriteParamSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_ODT_WRITE_CLOCKS}, ${CONFIG_SYS_MEMORY_DDR_ODT_WRITE_DLY});
<#else>
    PLIB_DDR_OdtWriteDisable(DDR_ID_0, 0);
</#if>

    /* Controller Settings */
    PLIB_DDR_LittleEndianSet(DDR_ID_0);
    PLIB_DDR_HalfRateSet(DDR_ID_0);
    PLIB_DDR_MaxCmdBrstCntSet(DDR_ID_0, 3);
    PLIB_DDR_NumHostCmdsSet(DDR_ID_0, 12);

    /* DRAM Initialization */

    /* bring CKE high after reset and wait 400 nsec */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_10, DRV_DDR_IDLE_NOP);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_20, (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(400000) << 11)));

    /* issue precharge all command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_11, DRV_DDR_PRECH_ALL_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_21, (0x04 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TRP} + ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* initialize EMR2 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_12, DRV_DDR_LOAD_MODE_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_22, (0x00 | (0x02 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* initialize EMR3 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_13, DRV_DDR_LOAD_MODE_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_23, (0x00 | (0x03 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* RDQS disable, DQSB enable, OCD exit, 150 ohm termination, AL=0, DLL enable */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_14, (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_24, (0x00 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    tmp = ((sys_mem_ddr_round_up(${CONFIG_SYS_MEMORY_DDR_TWR}, ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) -1 ) << 1) | 1;
    ma_field = tmp & 0xFF;
    ba_field = (tmp >> 8) & 0x03;

    /* PD fast exit, WR REC = tWR in clocks -1, DLL reset, CAS = RL, burst = 4 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_15, (DRV_DDR_LOAD_MODE_CMD | (((${CONFIG_SYS_MEMORY_DDR_CAS_LAT} << 4) | 2) << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_25, (ma_field | (ba_field << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* issue precharge all command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_16, DRV_DDR_PRECH_ALL_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_26, (0x04 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TRP} + ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* issue refresh command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_17, DRV_DDR_REF_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_27, (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TRFC}) << 11)));

    /* issue refresh command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_18, DRV_DDR_REF_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_28, (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TRFC}) << 11)));
	
    tmp = ((sys_mem_ddr_round_up(15000, 2500) -1 ) << 1);
    ma_field = tmp & 0xFF;
    ba_field = (tmp >> 8) & 0x03;

    /* Mode register programming as before without DLL reset */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_19, (DRV_DDR_LOAD_MODE_CMD | (((${CONFIG_SYS_MEMORY_DDR_CAS_LAT} << 4) | 3) << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_29, (ma_field | (ba_field << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* extended mode register same as before with OCD default */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_110, (DRV_DDR_LOAD_MODE_CMD | (0xC0 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_210, (0x03 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(${CONFIG_SYS_MEMORY_DDR_TMRD} * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

    /* extended mode register same as before with OCD exit */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_111, (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_211, (0x00 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(140 * ${CONFIG_SYS_MEMORY_DDR_CLK_PER}) << 11)));

	/* Set number of host commands */
	PLIB_DDR_NumHostCmdsSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_NUM_CMDS});
	
    PLIB_DDR_CmdDataValid(DDR_ID_0);
    PLIB_DDR_CmdDataSend(DDR_ID_0);
    while (PLIB_DDR_CmdDataIsComplete(DDR_ID_0));
    PLIB_DDR_ControllerEnable(DDR_ID_0);
}

static void DDR_PHY_Init(void)
{
<#if CONFIG_SYS_MEMORY_DDR_PHY_ODT_ENABLE == true>
    PLIB_DDR_PHY_OdtEnable(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_ODT_VALUE});
<#else>
    PLIB_DDR_PHY_OdtDisable(DDR_ID_0);
</#if>
    PLIB_DDR_PHY_DataDriveStrengthSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SYS_MEMORY_STRENGTH_DATA});
    PLIB_DDR_PHY_AddCtlDriveStrengthSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SYS_MEMORY_STRENGTH_ADDC});
    PLIB_DDR_PHY_OdtCal(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_ODT_PU_CAL}, ${CONFIG_SYS_MEMORY_DDR_PHY_ODT_PD_CAL});
    PLIB_DDR_PHY_DrvStrgthCal(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SYS_MEMORY_STR_NFET_CAL}, ${CONFIG_SYS_MEMORY_DDR_PHY_SYS_MEMORY_STR_PFET_CAL});
<#if CONFIG_SYS_MEMORY_DDR_PHY_EXTRA_CLK == true>
    PLIB_DDR_PHY_ExtraClockEnable(DDR_ID_0); 
<#else>
    PLIB_DDR_PHY_ExtraClockDisable(DDR_ID_0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_PHY_DLL == "Internal">
    PLIB_DDR_PHY_InternalDllEnable(DDR_ID_0);
<#else>
    PLIB_DDR_PHY_ExternalDllEnable(DDR_ID_0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_PHY_RCVREN == true>
    PLIB_DDR_PHY_PadReceiveEnable(DDR_ID_0);
<#else>
    PLIB_DDR_PHY_PadReceiveDisable(DDR_ID_0);
</#if>
    PLIB_DDR_PHY_PreambleDlySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_PRE_DLY});
	PLIB_DDR_PHY_HalfRateSet(DDR_ID_0);
<#if CONFIG_SYS_MEMORY_DDR_PHY_WR_CMD_DLY == true>
	PLIB_DDR_PHY_WriteCmdDelayEnable(DDR_ID_0);
<#else>
	PLIB_DDR_PHY_WriteCmdDelayDisable(DDR_ID_0);
</#if>
<#if CONFIG_SYS_MEMORY_DDR_PHY_RECALIB_EN == true>
    PLIB_DDR_PHY_DllRecalibEnable(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_RECALIB_COUNT});
<#else>
    PLIB_DDR_PHY_DllRecalibDisable(DDR_ID_0);
</#if>
    PLIB_DDR_PHY_DllMasterDelayStartSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_DLL_MASTER_DELAY_START});
    PLIB_DDR_PHY_SCLTestBurstModeSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SCL_BURST_MODE});
    PLIB_DDR_PHY_DDRTypeSet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_DDR_TYPE});
    PLIB_DDR_PHY_ReadCASLatencySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_CAS_LAT});
    PLIB_DDR_PHY_WriteCASLatencySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_WR_LAT});
<#if CONFIG_SYS_MEMORY_DDR_PHY_ODT_CS_EN == true>
    PLIB_DDR_PHY_OdtCSEnable(DDR_ID_0);
<#else>
    PLIB_DDR_PHY_OdtCSDisable(DDR_ID_0);
</#if>
    PLIB_DDR_PHY_SCLDelay(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SCL_DLY});
	PLIB_DDR_PHY_SCLDDRClkDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SCL_MAIN_CLK_DELAY});
	PLIB_DDR_PHY_SCLCapClkDelaySet(DDR_ID_0, ${CONFIG_SYS_MEMORY_DDR_PHY_SCL_CAP_CLK_DELAY});
<#if CONFIG_SYS_MEMORY_DDR_SCL_CS_EN>
    PLIB_DDR_PHY_SCLEnable(DDR_ID_0, 0);
</#if>
}

static void DDR_PHY_Calib(void)
{
	PLIB_DDR_PHY_SCLStart(DDR_ID_0);
	while (!PLIB_DDR_PHY_SCLStatus(DDR_ID_0));
}

void SYS_MEMORY_DDR_Initialize(void)
{
	DDR_PMD_Init();
    DDR_PHY_Init();
    DDR_Init();
	DDR_PHY_Calib();
}

/*******************************************************************************
 End of File
*/
