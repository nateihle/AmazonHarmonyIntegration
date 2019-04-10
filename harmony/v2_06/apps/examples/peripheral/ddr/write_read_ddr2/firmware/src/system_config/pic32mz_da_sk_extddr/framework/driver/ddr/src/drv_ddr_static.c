/*******************************************************************************
  CMP Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ddr_static.c

  Summary:
    CMP driver implementation for the static single instance driver.

  Description:
    The CMP device driver provides a simple interface to manage the CMP
    modules on Microchip microcontrollers.
    
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

// *****************************************************************************
// *****************************************************************************
// Section: DDR static driver functions
// *****************************************************************************
// *****************************************************************************

#define drv_ddr_max(a,b) (((a)>(b))?(a):(b))
#define drv_ddr_round_up(x,y) ((x)%(y))?(((x)/(y))+1):((x)/(y))
#define drv_ddr_hc_clk_dly(dly) (drv_ddr_max((drv_ddr_round_up((dly),)),2)-2)

static void DDR_PMD_Init(void)
{
	*(volatile uint32_t*)(0xBF8000A0) = 0;
}

static void DDR_MPLL_Init(void)
{
	*(volatile uint32_t*)(0xBF800100) = 0x0A003203;

	while ((*(volatile uint32_t*)(0xBF800100) & 0x80800000) != 0x80800000);
}

static void DDR_Init(void)
{
#if 0
    uint32_t tmp;
    uint32_t ba_field, ma_field;

    /* Target Arbitration */
    PLIB_DDR_MinLimit(DDR_ID_0, , DDR_TARGET_CPU);
    PLIB_DDR_ReqPeriod(DDR_ID_0 index, , DDR_TARGET_CPU);
    PLIB_DDR_MinCommand(DDR_ID_0 index, , DDR_TARGET_CPU);

    PLIB_DDR_MinLimit(DDR_ID_0, , DDR_TARGET_GC_IN);
    PLIB_DDR_ReqPeriod(DDR_ID_0 index, , DDR_TARGET_GC_IN);
    PLIB_DDR_MinCommand(DDR_ID_0 index, , DDR_TARGET_GC_IN);

    PLIB_DDR_MinLimit(DDR_ID_0, , DDR_TARGET_GC_OUT);
    PLIB_DDR_ReqPeriod(DDR_ID_0 index, , DDR_TARGET_GC_OUT);
    PLIB_DDR_MinCommand(DDR_ID_0 index, , DDR_TARGET_GC_OUT);

    PLIB_DDR_MinLimit(DDR_ID_0, , DDR_TARGET_GPU_IN);
    PLIB_DDR_ReqPeriod(DDR_ID_0 index, , DDR_TARGET_GPU_IN);
    PLIB_DDR_MinCommand(DDR_ID_0 index, , DDR_TARGET_GPU_IN);

    PLIB_DDR_MinLimit(DDR_ID_0, , DDR_TARGET_GPU_OUT);
    PLIB_DDR_ReqPeriod(DDR_ID_0 index, , DDR_TARGET_GPU_OUT);
    PLIB_DDR_MinCommand(DDR_ID_0 index, , DDR_TARGET_GPU_OUT);

    /* Addressing */
    PLIB_DDR_RowAddressSet(DDR_ID_0, ROW_ADDR_RSHIFT, ROW_ADDR_MASK);
    PLIB_DDR_ColumnAddressSet(DDR_ID_0, COL_HI_RSHFT, COL_LO_MASK, COL_HI_MASK);
    PLIB_DDR_BankAddressSet(DDR_ID_0, BA_RSHFT, BANK_ADDR_MASK);
    PLIB_DDR_ChipSelectAddressSet(DDR_ID_0, CS_ADDR_RSHIFT, CS_ADDR_MASK);

    /* Refresh */
    PLIB_DDR_RefreshTimingSet(DDR_ID_0, , , CTRL_CLK_PERIOD);
    PLIB_DDR_MaxPendingRefSet(DDR_ID_0, );
    PLIB_DDR_AutoSelfRefreshDisable(DDR_ID_0);

    /* Power */
    PLIB_DDR_AutoPowerDownDisable(DDR_ID_0);
    PLIB_DDR_AutoPchrgDisable(DDR_ID_0);
    PLIB_DDR_AutoPchrgPowerDownDisable(DDR_ID_0);

    /* Timing */
    PLIB_DDR_ReadWriteDelaySet(DDR_ID_0, , , );
    PLIB_DDR_WriteReadDelaySet(DDR_ID_0, , , , CTRL_CLK_PERIOD);
    PLIB_DDR_ReadReadDelaySet(DDR_ID_0, );
    PLIB_DDR_WriteWriteDelaySet(DDR_ID_0, );
    PLIB_DDR_SelfRefreshDelaySet(DDR_ID_0, , , );
    PLIB_DDR_PowerDownDelaySet(DDR_ID_0, , , );
    PLIB_DDR_PrechargAllBanksSet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_ReadToPrechargeDelaySet(DDR_ID_0, , , CTRL_CLK_PERIOD);
    PLIB_DDR_WriteToPrechargeDelaySet(DDR_ID_0, , , , CTRL_CLK_PERIOD);
    PLIB_DDR_PrechargeToRASDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_RASToPrechargeDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_RASToRASBankDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_RASToRASDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_RASToCASDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);
    PLIB_DDR_DataDelaySet(DDR_ID_0, , );
    PLIB_DDR_TfawDelaySet(DDR_ID_0, , CTRL_CLK_PERIOD);


    /* On-Die Termination */
    PLIB_DDR_OdtReadDisable(DDR_ID_0, 0);
    PLIB_DDR_OdtWriteDisable(DDR_ID_0, 0);

    /* Controller Settings */
    PLIB_DDR_LittleEndianSet(DDR_ID_0);
    PLIB_DDR_HalfRateSet(DDR_ID_0);
    PLIB_DDR_MaxCmdBrstCntSet(DDR_ID_0, 3);
    PLIB_DDR_NumHostCmdsSet(DDR_ID_0, 12);

    /* DRAM Initialization */

    /* bring CKE high after reset and wait 400 nsec */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_100, DRV_DDR_IDLE_NOP);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_200, (0x00 | (0x00 << 8) | (drv_ddr_hc_clk_dly(400000) << 11)));

    /* issue precharge all command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_101, DRV_DDR_PRECH_ALL_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_201, (0x04 | (0x00 << 8) | (drv_ddr_hc_clk_dly( + ) << 11)));

    /* initialize EMR2 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_102, DRV_DDR_LOAD_MODE_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_202, (0x00 | (0x02 << 8) | (drv_ddr_hc_clk_dly( * ) << 11)));

    /* initialize EMR3 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_103, DRV_DDR_LOAD_MODE_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_203, (0x00 | (0x03 << 8) | (drv_ddr_hc_clk_dly( * ) << 11)));

    /* RDQS disable, DQSB enable, OCD exit, 150 ohm termination, AL=0, DLL enable */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_104, (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_204, (0x00 | (0x01 << 8) | (drv_ddr_hc_clk_dly( * ) << 11)));

    tmp = ((drv_ddr_round_up(, ) -1 ) << 1);
    ma_field = tmp & 0xFF;
    ba_field = (tmp >> 8) & 0x03;

    /* PD fast exit, WR REC = tWR in clocks -1, DLL reset, CAS = RL, burst = 4 */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_105, (DRV_DDR_LOAD_MODE_CMD | ((( << 4) | 2) << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_205, (ma_field | (ba_field << 8) | (drv_ddr_hc_clk_dly( * ) << 11)));

    /* issue precharge all command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_106, DRV_DDR_PRECH_ALL_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_206, (0x04 | (0x00 << 8) | (drv_ddr_hc_clk_dly( + ) << 11)));

    /* issue refresh command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_107, DRV_DDR_REF_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_207, (0x00 | (0x00 << 8) | (drv_ddr_hc_clk_dly() << 11)));

    /* issue refresh command */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_108, DRV_DDR_REF_CMD);
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_208, (0x00 | (0x00 << 8) | (drv_ddr_hc_clk_dly() << 11)));

    /* Mode register programming as before without DLL reset */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_109, (DRV_DDR_LOAD_MODE_CMD | ((( << 4) | 3) << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_209, (ma_field | (ba_field << 8) | (drv_ddr_hc_clk_dly( * ) << 11)));

    /* extended mode register same as before with OCD default */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_110, (DRV_DDR_LOAD_MODE_CMD | (0xC0 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_210, (0x00 | (0x00 << 8) | (drv_ddr_hc_clk_dly() << 11)));

    /* extended mode register same as before with OCD exit */
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_111, (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24)));
    PLIB_DDR_CmdDataWrite(DDR_ID_0, DDR_HOST_CMD_REG_211, (0x00 | (0x01 << 8) | (drv_ddr_hc_clk_dly(140 * ) << 11)));

    PLIB_DDR_CmdDataValid(DDR_ID_0);
    PLIB_DDR_CmdDataSend(DDR_ID_0);
    while (PLIB_DDR_CmdDataIsComplete(DDR_ID_0));
    PLIB_DDR_ControllerEnable(DDR_ID_0);
#endif


	*(volatile uint32_t*)(0xBF8E8050) = 0x00000008; /* MEM_WIDTH */

	*(volatile uint32_t*)(0xBF8E8000) = 0x00000000; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8004) = 0x0000001F; /* MIN_LIMIT */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000000; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8008) = 0x000000FF; /* RQST_PERIOD */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000000; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E800C) = 0x00000004; /* MIN_CMD_ACPT */

	*(volatile uint32_t*)(0xBF8E8000) = 0x00000005; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8004) = 0x0000001F; /* MIN_LIMIT */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000008; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8008) = 0x000000FF; /* RQST_PERIOD */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000008; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E800C) = 0x00000010; /* MIN_CMD_ACPT */

	*(volatile uint32_t*)(0xBF8E8000) = 0x0000000A; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8004) = 0x0000001F; /* MIN_LIMIT */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000010; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8008) = 0x000000FF; /* RQST_PERIOD */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000010; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E800C) = 0x00000010; /* MIN_CMD_ACPT */

	*(volatile uint32_t*)(0xBF8E8000) = 0x0000000F; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8004) = 0x00000004; /* MIN_LIMIT */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000018; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8008) = 0x000000FF; /* RQST_PERIOD */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000018; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E800C) = 0x00000004; /* MIN_CMD_ACPT */

	*(volatile uint32_t*)(0xBF8E8000) = 0x00000014; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8004) = 0x00000004; /* MIN_LIMIT */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000020; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E8008) = 0x000000FF; /* RQST_PERIOD */
	*(volatile uint32_t*)(0xBF8E8000) = 0x00000020; /* ARB_AGENT_SEL */
	*(volatile uint32_t*)(0xBF8E800C) = 0x00000004; /* MIN_CMD_ACPT */

	*(volatile uint32_t*)(0xBF8E8014) = 0x20000A0D; /* MEM_CONFIG_0 */
	*(volatile uint32_t*)(0xBF8E8018) = 0x00001FFF; /* MEM_CONFIG_1 */
	*(volatile uint32_t*)(0xBF8E801C) = 0x00000000; /* MEM_CONFIG_2 */
	*(volatile uint32_t*)(0xBF8E8020) = 0x000003FF; /* MEM_CONFIG_3 */
	*(volatile uint32_t*)(0xBF8E8024) = 0x00000007; /* MEM_CONFIG_4 */

	*(volatile uint32_t*)(0xBF8E8028) = 0x07180616; /* REF_CONFIG */

	*(volatile uint32_t*)(0xBF8E802C) = 0x00011080; /* PWR_SAVE_ECC_CONFIG */

	*(volatile uint32_t*)(0xBF8E8030) = 0x04112467; /* DDRDLYCFG0 */
	*(volatile uint32_t*)(0xBF8E8034) = 0x00016202; /* DDRDLYCFG1 */
	*(volatile uint32_t*)(0xBF8E8038) = 0x06229303; /* DDRDLYCFG2 */
	*(volatile uint32_t*)(0xBF8E803C) = 0x00090B08; /* DDRDLYCFG3 */

	*(volatile uint32_t*)(0xBF8E8040) = 0x00000000; /* DDRODTCFG */
	*(volatile uint32_t*)(0xBF8E804C) = 0x00010000; /* DDRODT_EN_CFG */
	*(volatile uint32_t*)(0xBF8E8040) = 0x00121200; /* DDRODTCFG */

	*(volatile uint32_t*)(0xBF8E8044) = 0x73020042; /* DDRXFERCFG */

	*(volatile uint32_t*)(0xBF8E8080) = 0x00FFFFFF; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80C0) = 0x0004F000; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8084) = 0x00FFF401; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80C4) = 0x00002004; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8088) = 0x00FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80C8) = 0x00000200; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E808C) = 0x00FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80CC) = 0x00000300; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8090) = 0x40FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80D0) = 0x00000100; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8094) = 0x52FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80D4) = 0x0000000B; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8098) = 0x00FFF401; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80D8) = 0x00002004; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E809C) = 0x00FFF801; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80DC) = 0x00018800; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E80A0) = 0x00FFF801; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80E0) = 0x00018800; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E80A4) = 0x53FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80E4) = 0x0000000A; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E80A8) = 0xC0FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80E8) = 0x00000103; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E80AC) = 0x40FFF001; /* HOST_CMD1 */
	*(volatile uint32_t*)(0xBF8E80EC) = 0x00045100; /* HOST_CMD2 */

	*(volatile uint32_t*)(0xBF8E8048) = 0x0000001B; /* HOST_CMD_ISSUE */
	*(volatile uint32_t*)(0xBF8E8010) = 0x00000001; /* MEM_START */

	while ((*(volatile uint32_t*)(0xBF8E8048) & 0x00000010) == 0x00000010);

	*(volatile uint32_t*)(0xBF8E8010) = 0x00000003; /* MEM_START */

	*(volatile uint32_t*)(0xBF8E9100) = 0x14000000; /* MEM_START */
	while ((*(volatile uint32_t*)(0xBF8E9100) & 0x00000003) != 0x00000003);
	
}


static void DDR_PHY_Init(void)
{
#if 0
    PLIB_DDR_PHY_OdtDisable(DDR_ID_0);
    PLIB_DDR_PHY_DriveStrengthSet(DDR_ID_0, );
    PLIB_DDR_PHY_OdtCal(DDR_ID_0, , );
    PLIB_DDR_PHY_DrvStrgthCal(DDR_ID_0, , );
    PLIB_DDR_PHY_ExtraClockDisable(DDR_ID_0);
    PLIB_DDR_PHY_ExternalDllEnable(DDR_ID_0);
    PLIB_DDR_PHY_PadReceiveDisable(DDR_ID_0);
    PLIB_DDR_PHY_PreambleDlySet(DDR_ID_0, );
    PLIB_DDR_PHY_DllRecalibDisable(DDR_ID_0);
    PLIB_DDR_PHY_DllMasterDelayStartSet(DDR_ID_0, );
    PLIB_DDR_PHY_SCLTestBurstModeSet(DDR_ID_0, );
    PLIB_DDR_PHY_DDRTypeSet(DDR_ID_0, );
    PLIB_DDR_PHY_ReadCASLatencySet(DDR_ID_0, );
    PLIB_DDR_PHY_WriteCASLatencySet(DDR_ID_0, );
    PLIB_DDR_PHY_OdtCSDisable(DDR_ID_0);
    PLIB_DDR_PHY_SCLDelay(DDR_ID_0, );
#endif

	*(volatile uint32_t*)(0xBF8E9124) = 0x30001000; /* DDRPHYDLLR */
	*(volatile uint32_t*)(0xBF8E9120) = 0x50EE62E3; /* DDRPHYPADCON */
	*(volatile uint32_t*)(0xBF8E9118) = 0x01000053; /* DDRSCLCFG0 */
	*(volatile uint32_t*)(0xBF8E911C) = 0x00000401; /* DDRSCLCFG1 */
	*(volatile uint32_t*)(0xBF8E910C) = 0x00000043; /* DDRSCLLAT */
}

void DRV_DDR_Initialize(void)
{
	DDR_PMD_Init();
	DDR_MPLL_Init();
    DDR_PHY_Init();
    DDR_Init();
}

/*******************************************************************************
 End of File
*/
