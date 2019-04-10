<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
// <editor-fold defaultstate="collapsed" desc="PIC32WK Wi-Fi Driver Initialization Data">
<#if CONFIG_USE_DRV_WIFI_WK!false == true>
/****************************************************************************************
 * Function:        void MAC_Interrupt_enable()
 *
 * Overview:        The function Enable WLAN MAC ISR
****************************************************************************************/
void MAC_Interrupt_enable(){
    IEC2bits.RFMACIE = 1; //Enable MAC ISR
}
/****************************************************************************************
 * Function:        void MAC_Interrupt_disable()
 *
 * Overview:        The function Disable WLAN MAC ISR
****************************************************************************************/
void MAC_Interrupt_disable(){
    IEC2bits.RFMACIE = 0; //Disable MAC ISR
}
/****************************************************************************************
 * Function:        void MAC_Timer0_enable()
 *
 * Overview:        The function Enable WLAN Timer 0 ISR
****************************************************************************************/
void MAC_Timer0_enable(){
    IEC2bits.RFTM0IE = 1; //Enable Timer0 ISR
    //0xBF8100E0
}
/****************************************************************************************
 * Function:        void MAC_Timer0_disable()
 *
 * Overview:        The function Disable WLAN Timer 0 ISR
****************************************************************************************/
void MAC_Timer0_disable(){
    IEC2bits.RFTM0IE = 0; //Disable Timer0 ISR
}
/****************************************************************************************
 * Function:        void MAC_Timer0_enable()
 *
 * Overview:        The function Enable WLAN Timer 1 ISR
****************************************************************************************/

void MAC_Timer1_enable(){
    IEC2bits.RFTM1IE = 1; //Enable Timer1 ISR
}
/****************************************************************************************
 * Function:        void MAC_Timer0_enable()
 *
 * Overview:        The function Disable WLAN Timer 1 ISR
****************************************************************************************/

void MAC_Timer1_disable(){
    IEC2bits.RFTM1IE = 0; //Disable Timer1 ISR
}
/****************************************************************************************
 * Function:        void WLAN_default_FEM_config()
 *
 * Overview:        The function WLAN FEM setting
****************************************************************************************/
void WLAN_default_FEM_config()
{
    TRISKCLR = 0x0B03;
    LATKbits.LATK0 = 0;
    LATKbits.LATK1 = 0;
    LATKbits.LATK8 = 0;
    LATKbits.LATK9 = 0;
    LATKbits.LATK11 = 0;
}
/****************************************************************************************
 * Function:        void WLAN_special_FEM_config()
 *
 * Overview:        The function WLAN FEM setting
****************************************************************************************/
void WLAN_special_FEM_config()
{
    TRISKSET = 0x0B03;
}
/****************************************************************************************
 * Function:        void postMacEvent(unsigned short qid)
 *
 * Overview:        WLAN MAC post events
****************************************************************************************/
void WF_UserEventsSet(uint16_t event, uint16_t eventInfo, bool isMgmt);
void postMacEvent(unsigned short qid)
{
	switch(qid)
	{
		case 0://WLAN_RX_EVENT_QID:

			WF_UserEventsSet(TCPIP_EV_RX_PKTPEND, 0, false);  // will result in WLANMAC_MACProcess being called
			break;
		case 1://WLAN_RX_EVENT_QID:

			WF_UserEventsSet(TCPIP_EV_RX_DONE, 0, false);  // will result in WLANMAC_MACProcess being called
			break;
		case 2:
		case 3:
		case 4:

			WF_UserEventsSet(TCPIP_EV_TX_DONE, 0, false);  // will result in WLANMAC_MACProcess being called
			break;
		default:

			WF_UserEventsSet(TCPIP_EV_RX_PKTPEND, 0, false);  // will result in WLANMAC_MACProcess being called
			break;
	}
}
/****************************************************************************************
 * Function:        int get_CPU_freq()
 *
 * Overview:        This function will return system CPU frequency
****************************************************************************************/
unsigned int get_CPU_freq()
{
	return SYS_CLK_FREQ;
}
</#if><#-- CONFIG_USE_DRV_WIFI_WK!false -->
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
