/*************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wdrv_mrf24wn_api.h

  Summary:
    MRF24WN Interface Functions

  Description:
    MRF24WN Interface Functions
 *************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

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
// DOM-IGNORE-END

#ifndef _WDRV_MRF24WN_API_H
#define _WDRV_MRF24WN_API_H

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_common.h"

//*******************************************************************************
/*
  Function:
        int32_t WDRV_EXT_Initialize(const WDRV_CALLBACKS *const CB)

  Summary:
    Initializes the MRF24WN Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the MRF24WN Wi-Fi driver, making it ready for
    clients to use.

  Precondition:
    None.

  Parameters:
    CB - pointer to callback functions

  Returns:
    - 0              - Indicates success
    - non-zero value - Indicates failure

  Remarks:
    None.
 */
int32_t WDRV_EXT_Initialize(const WDRV_CALLBACKS *const CB);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_Deinitialize(void)

  Summary:
    Deinitializes the MRF24WN Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes the MRF24WN driver.

  Precondition:
    None.

  Returns:
    None.

  Remarks:
    None
 */
void WDRV_EXT_Deinitialize(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdNetModeBSSSet(void)

  Summary:
    Sets the Wi-Fi network type to Infrastructure.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi network type to Infrastructure.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdNetModeBSSSet(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdNetModeIBSSSet(void)

  Summary:
    Sets the Wi-Fi network type to Adhoc.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi network type to Adhoc.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdNetModeIBSSSet(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdNetModeAPSet(void)

  Summary:
    Sets the Wi-Fi network type to SoftAP.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi network type to SoftAP.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdNetModeAPSet(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdMacAddressGet(uint8_t *MacAddr)

  Summary:
    Retrieves the MRF24WN MAC address.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the MRF24WN MAC address.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    MacAddr - Pointer where MAC address will be written (must point to a
              6 bytes buffer)

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdMacAddressGet(uint8_t *MacAddr);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdConnectContextBssidGet(uint8_t *bssId)

  Summary:
    Gets the BSSID.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current AP's BSSID.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    bssId - pointer where the current AP's BSSID will be written

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdConnectContextBssidGet(uint8_t *bssId);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdConnectContextChannelGet(uint16_t *bssChannel)

  Summary:
    Gets the AP channel
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current AP channel.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    bssChannel - pointer where the current AP channel will be written

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdConnectContextChannelGet(uint16_t *bssChannel);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdTxPowerSet(uint32_t dbm)

  Summary:
    Sets the Tx Power.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    The function sets the module's Tx power.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    dbm - value of tx power

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdTxPowerSet(uint32_t dbm);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdPowerSaveGet(bool *enabled)

  Summary:
    Retrieves current power save status.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the current power save status.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    enabled - pointer where the current power save status will be written

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdPowerSaveGet(bool *enabled);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdPowerSaveSet(bool enable)

  Summary:
    Sets the module in IEEE power save mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    The function sets the module in IEEE power save mode.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    enable - true will set the module in IEEE power save mode

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    This works only with Infrastructure mode. Do not call this in other modes.
 */
uint32_t WDRV_EXT_CmdPowerSaveSet(bool enable);

//*******************************************************************************
/*
  Function:
        WDRV_EXT_CmdChannelSet(uint16_t channel)

  Summary:
    Sets the channel on which to operate.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the channel on which to operate.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    channel - target channel

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    This works only with SoftAP mode. Do not call this in other modes.
 */
uint32_t WDRV_EXT_CmdChannelSet(uint16_t channel);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdConnect(void)

  Summary:
    Directs the MRF24WN to connect to a Wi-Fi network.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function causes the MRF24WN to connect to a Wi-Fi network. Upon
    connection, or a failure to connect, an event will be generated.

  Precondition:
    Wi-Fi initialization must be complete and relevant connection parameters
    must have been set.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdConnect(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdDisconnect(void)

  Summary:
    Directs the MRF24WN to disconnect from a Wi-Fi network.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function causes the MRF24WN to disconnect from a Wi-Fi network.

  Precondition:
    Wi-Fi initialization must be complete and a connection must be in
    progress.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdDisconnect(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_DataSend(uint16_t segSize, uint8_t *p_segData)

  Summary:
    Sends data packets to MRF24WN module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sends data packets to the MRF24WN module.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    seqSize - data size
    p_seqData - pointer to the data buffer

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_DataSend(uint16_t segSize, uint8_t *p_segData);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSecNoneSet(void)

  Summary:
    Sets Wi-Fi security to open (no security).
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to open. One can only connect to
    an AP that is running in open mode.

  Precondition:
    Wi-Fi initialization must be complete and in an unconnected state.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdSecNoneSet(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSecWEPSet(uint8_t *key)

  Summary:
    Sets Wi-Fi security to WEP.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WEP. One can only connect to
    an AP that is running the same WEP mode.

  Precondition:
    Wi-Fi initialization must be complete and in an unconnected state.

  Parameters:
    key - pointer to the WEP key buffer

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdSecWEPSet(uint8_t *key);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSecWPASet(uint8_t *key, uint16_t len)

  Summary:
    Sets Wi-Fi security to WPA.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WPA. One can only
    connect to an AP that is running the same WPA mode.

  Precondition:
    Wi-Fi initialization must be complete and in an unconnected state.

  Parameters:
    key - pointer to the WPA key buffer
    len - WPA key length

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdSecWPASet(uint8_t *key, uint16_t len);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSecWPA2Set(uint8_t *key, uint16_t len)

  Summary:
    Sets Wi-Fi security to WPA2.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WPA2. One can only
    connect to an AP that is running the same WPA2 mode.

  Precondition:
    Wi-Fi initialization must be complete and in an unconnected state.

  Parameters:
    key - pointer to the WPA2 key buffer
    len - WPA2 key length

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdSecWPA2Set(uint8_t *key, uint16_t len);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdTKIPEncryptSet(void)

  Summary:
    Sets Wi-Fi encryption to TKIP.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi encryption to TKIP. TKIP is the default
    encryption of WPA. But it will be sometimes needed in WPA2 as well. Hence
    this function is mostly used for this purpose.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdTKIPEncryptSet(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdAESEncryptSet(void)

  Summary:
    Sets Wi-Fi encryption to AES.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi encryption to AES. AES is the default
    encryption of WPA2. But it will be sometimes needed in WPA as well. Hence
    this function is mostly used for this purpose.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdAESEncryptSet(void);

//*******************************************************************************
/*
  Function:
        int32_t WDRV_EXT_CmdSecWPSSet(bool pinMode, uint8_t *key)

  Summary:
    Sets Wi-Fi security to WPS.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WPS. One can only connect to
    an AP that supports WPS.

  Precondition:
    Wi-Fi initialization must be complete and in an unconnected state.

  Parameters:
    pinMode - 0: PBC mode; 1: PIN mode
    key - pointer of the PIN buffer

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None
 */
uint32_t WDRV_EXT_CmdSecWPSSet(bool pinMode, uint8_t *key);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_WPSResultsRead(WDRV_CONFIG *p_wdrv_config)

  Summary:
    Reads the WPS process results back from the MRF24WN module and updates the
    configuration data.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    After the WPS process has completed, this function is used to read the WPS process
    results from the MRF24WN module and update the configuration data.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    p_wdrv_config - pointer to where configuration data will be updated

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_EXT_WPSResultsRead(WDRV_CONFIG *p_wdrv_config);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSSIDGet(uint8_t *ssid, uint8_t *length)

  Summary:
    Gets the SSID.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function returns the SSID and SSID Length.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    ssid - pointer to buffer where SSID will be written
    length - number of bytes in SSID

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdSSIDGet(uint8_t *ssid, uint8_t *length);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdSSIDSet(uint8_t *ssid)

  Summary:
    Sets the SSID.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the SSID.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    ssid - pointer to SSID buffer

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    SSID is case sensitive.
 */
uint32_t WDRV_EXT_CmdSSIDSet(uint8_t *ssid);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdScanStart(void)

  Summary:
    Directs the MRF24WN module to start a scan.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function directs the MRF24WN module to start a scan.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdScanStart(void);

//*******************************************************************************
/*
  Function:
        uint32_t WDRV_EXT_CmdScanGet(uint16_t *numOfResults)

  Summary:
    Reads the number of scan results from the MRF24WN module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function reads the number of scan results from the MRF24WN module.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    numOfResults - pointer where the number of scan results will be written

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
uint32_t WDRV_EXT_CmdScanGet(uint16_t *numOfResults);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_ScanResultGet(uint8_t listIndex, WDRV_SCAN_RESULT *p_scanResult)

  Summary:
    Reads the selected scan results back from the MRF24WN module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    After a scan has completed this function is used to read one scan
    result at a time from the MRF24WN module.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    listIndex - index (0 based list) of the scan entry to retrieve
    p_scanResult - pointer to where scan result is written

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
void WDRV_EXT_ScanResultGet(uint16_t idx, WDRV_SCAN_RESULT *p_scanResult);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_ScanDoneSet(void)

  Summary:
    Indicates when a scan has completed.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function indicates when a scan has completed.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_EXT_ScanDoneSet(void);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_PrivConfig(uint32_t *config)

  Summary:
    Configures g_wdrvext_priv parameter.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function configures g_wdrvext_priv parameter.

  Precondition:
    None.

  Parameters:
    config - pointer to the parameter array

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_EXT_PrivConfig(uint32_t *config);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_CmdFWUpdate(void)

  Summary:
    Directs the module to start firmware download and upgrade.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function directs the module to start the firmware download and upgrade.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_EXT_CmdFWUpdate(void);

//*******************************************************************************
/*
  Function:
        void WDRV_EXT_HWInterruptHandler(void)

  Summary:
    Wi-Fi driver (MRF24WN-specific) interrupt service routine.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is the Wi-Fi driver (MRF24WN-specific) interrupt service routine.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_EXT_HWInterruptHandler(void);

//*******************************************************************************
/*
  Function:
        void WDRV_INTR_SourceEnable(void)

  Summary:
    Enables interrupts from the module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables interrupts from the module.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_INTR_SourceEnable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_INTR_SourceDisable(void)

  Summary:
    Disables interrupts from the module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables interrupts from the module.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_INTR_SourceDisable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_INTR_Init(void)

  Summary:
    Initializes interrupts for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes interrupts for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_INTR_Init(void);

//*******************************************************************************
/*
  Function:
        void WDRV_INTR_Deinit(void)

  Summary:
    Deinitializes interrupts for Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes interrupts for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_INTR_Deinit(void);

//*******************************************************************************
/*
  Function:
        void WDRV_SPI_In(uint8_t const *const OutBuf, uint16_t OutSize,
            uint8_t *const InBuf, uint16_t InSize)

  Summary:
    Receives data from the module through the SPI bus.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function receives data from the module through the SPI bus.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    bufOut - buffer pointer of output command
    OutSize - the command size
    InBuf - buffer pointer of input data
    InSize - the input data size

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_SPI_In(uint8_t *const OutBuf, uint16_t OutSize, uint8_t *const InBuf, uint16_t InSize);

//*******************************************************************************
/*
  Function:
        void WDRV_SPI_Out(uint8_t const *const bufOut, uint16_t OutSize)

  Summary:
    Sends data out to the module through the SPI bus.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sends data out to the module through the SPI bus.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    bufOut - buffer pointer of output data
    OutSize - the data size

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_SPI_Out(uint8_t *const bufOut, uint16_t OutSize);

//*******************************************************************************
/*
  Function:
        void WDRV_SPI_Init(void)

  Summary:
    Initializes the SPI object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the SPI object for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_SPI_Init(void);

//*******************************************************************************
/*
  Function:
        void WDRV_SPI_Deinit(void)

  Summary:
    Deinitializes the SPI object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes the SPI object for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_SPI_Deinit(void);

//*******************************************************************************
/*
  Function:
        void WDRV_GPIO_Init(void)

  Summary:
    Initializes the GPIO objects for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the GPIO objects for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_GPIO_Init(void);

//*******************************************************************************
/*
  Function:
        void WDRV_GPIO_DeInit(void)

  Summary:
    Deinitializes the GPIO objects for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes the GPIO objects for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_GPIO_DeInit(void);

//*******************************************************************************
/*
  Function:
        void WDRV_GPIO_PowerOff(void)

  Summary:
    Powers off the MRF24WN module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function powers off the MRF24WN module.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_GPIO_PowerOff(void);

//*******************************************************************************
/*
  Function:
        void WDRV_GPIO_PowerOn(void)

  Summary:
    Powers on the MRF24WN module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function powers on the MRF24WN module.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_GPIO_PowerOn(void);

//*******************************************************************************
/*
  Function:
        bool WDRV_IsPowerOff(void)

  Summary:
    Checks if MRF24WN is turned off.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function checks if MRF24WN is turned off.

  Parameters:
    None.

  Returns:
    - 0              - Indicates that MRF24WN is turned off
    - Non-zero value - Indicates that MRF24WN is on

  Remarks:
    None.
 */
bool WDRV_IsPowerOff(void);

//*******************************************************************************
/*
  Function:
        bool WDRV_CLI_Init(void)

  Summary:
    Initializes the console CLI interface.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the console CLI interface.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    - 0              - Indicates success
    - Non-zero value - Indicates failure

  Remarks:
    None.
 */
bool WDRV_CLI_Init(void);

//*******************************************************************************
/*
  Function:
        void WDRV_MRF24WN_ISR(void)

  Summary:
    Wi-Fi driver (MRF24WN-specific) interrupt service routine.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is the Wi-Fi driver (MRF24WN-specific) interrupt service routine.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_MRF24WN_ISR(void);

//*******************************************************************************
/*
Wi-Fi Commands Help

iwconfig
  Wi-Fi configuration
  Usage:
    iwconfig
    iwconfig ssid <ssid>
    iwconfig mode <mode>
    iwconfig security <security_mode> <key>/<pin>
    iwconfig power <enable/disable>
    iwconfig scan
    iwconfig scanget <scan_index>
  <ssid>:
    32 characters string - no blank or space allowed in this demo
  <mode>:
    managed/idle
  <security_mode>:
    open/wep40/wep104/wpa/wpa2/pbc/pin
    No blank or space allowed in <key> in current console commands
    Ex: iwconfig security open
        iwconfig security wep40 5AFB6C8E77
        iwconfig security wep104 90E96780C739409DA50034FCAA
        iwconfig security wpa microchip_psk
        iwconfig security wpa2 microchip_psk
        iwconfig security pbc
        iwconfig security pin 12390212

rftest
  RF test

mac
  Get MAC address

readconf
  Read from storage

erase conf
  Erase storage

saveconf
  Save storage
 */

#endif /* _WDRV_MRF24WN_API_H */
