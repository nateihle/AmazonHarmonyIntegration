/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wdrv_winc1500_stub.h

  Summary:
    WINC1500 wireless driver stub APIs.

  Description:
    WINC1500 wireless driver stub APIs.
 *******************************************************************************/

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

#ifndef _WDRV_WINC1500_STUB_H
#define _WDRV_WINC1500_STUB_H

#include "system/common/sys_module.h"

#include "system/command/sys_command.h"
#include "system/console/sys_console.h"

#define WDRV_STUB_Print(x) SYS_CONSOLE_PRINT x

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_INTR_SourceEnable(void)

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
void WDRV_STUB_INTR_SourceEnable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_INTR_SourceDisable(void)

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
void WDRV_STUB_INTR_SourceDisable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_INTR_Init(void (*isr)(void))

  Summary:
    Initializes interrupts for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes interrupts for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    isr - function pointer to the interrupt service handler.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_INTR_Init(void (*isr)(void));

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_INTR_Deinit(void)

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
void WDRV_STUB_INTR_Deinit(void);

//*******************************************************************************
/*
  Function:
      bool WDRV_STUB_SPI_Out(unsigned char const *buf, uint32_t size)

  Summary:
    Sends data out to the module through the SPI bus.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sends data out to the module through the SPI bus.

  Precondition:
    SPI driver should be initialized.

  Parameters:
    buf - buffer pointer of output data
    size - the output data size

  Returns:
    True - Indicates success
    False - Indicates failure

  Remarks:
    None.
 */
bool WDRV_STUB_SPI_Out(unsigned char *const buf, uint32_t size);

//*******************************************************************************
/*
  Function:
    bool WDRV_STUB_SPI_In(unsigned char *const buf, uint32_t size)

  Summary:
    Receives data from the module through the SPI bus.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function receives data from the module through the SPI bus.

  Precondition:
    SPI driver should be initialized.

  Parameters:
    buf - buffer pointer of input data
    size - the input data size

  Returns:
    None.

  Remarks:
    None.
 */
bool WDRV_STUB_SPI_In(unsigned char *const buf, uint32_t size);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_SPI_Initialize(void)

  Summary:
    Initializes the SPI object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the SPI object for the Wi-Fi driver.

  Precondition:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_SPI_Initialize(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_SPI_Deinitialize(void)

  Summary:
    Deinitializes the SPI object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes the SPI object for the Wi-Fi driver.

  Precondition:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_SPI_Deinitialize(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_Initialize(void)

  Summary:
    Initializes the GPIO object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the GPIO object for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_Initialize(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_DeInitialize(void)

  Summary:
    Deinitializes the GPIO object for the Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function deinitializes the GPIO object for the Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_DeInitialize(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_ChipEnable(void)

  Summary:
    Enables the WINC1500 chip.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables the WINC1500 chip.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_ChipEnable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_ChipDisable(void)

  Summary:
    Disables the WINC1500 chip.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables the WINC1500 chip.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_ChipDisable(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_ModuleReset(void)

  Summary:
    Resets the WINC1500 module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function resets the WINC1500 module.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_ModuleReset(void);

//*******************************************************************************
/*
  Function:
        void WDRV_STUB_GPIO_ModuleUnreset(void)

  Summary:
    Unresets the WINC1500 module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function unresets the WINC1500 module.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    board - Microchip development kit type (i.e., PIC32MZ EC Starter Kit,
            PIC32 Ethernet Starter Kit, etc.)

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_GPIO_ModuleUnreset(void);

//*******************************************************************************
/*
  Function:
        WDRV_STUB_Assert(int condition, const char *msg, const char *file, int line)

  Summary:
    Dumps out an error message on serial console and resets itself
    when the driver asserts.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Dumps out an error message on serial console and resets itself
    when the driver asserts.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    condition - asserts if false
    msg - error message
    file - file name
    line - line number where driver asserts.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_Assert(int condition, const char *msg, const char *file, int line);

//*******************************************************************************
/*
  Function:
        WDRV_STUB_HardDelay(uint16_t delay)

  Summary:
    Waits spinning for the delay milliseconds.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function has driver wait spinining for the delay milliseconds.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    board - duration to spin.

  Returns:
    None.

  Remarks:
    None.
 */
void WDRV_STUB_HardDelay(uint16_t delay);

#endif /* _WDRV_WINC1500_STUB_H */
