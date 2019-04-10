/*******************************************************************************
  BT Serial Port Interface

  Company:
    Microchip Technology Inc.

  File Name:
    btport.h

  Summary:
    Contains the BT Serial Port Interface specific defintions and function prototypes.

  Description:
    This file contains the BT Serial Port Interface specific defintions and function
    prototypes.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef __BTPORT_H_INCLUDED__
#define __BTPORT_H_INCLUDED__

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define PIC32_BC7_PS_VALUES_LOCATION        (0x9D000000+0x40000-0x200)
#define BTX_BT_ADDRESS_NAP                  ((BT_DEVICE_DESIGN_ID>>16) & 0xFFFF)
#define BTX_BT_ADDRESS_UAP_LAP              ((((BT_DEVICE_DESIGN_ID & 0x0000FFFF) << 16)) | BT_DEVICE_ID_2LSB)
#define BTX_BT_ADDRESS_4B                   (BT_DEVICE_DESIGN_ID & 0xFF)

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef void (*btport_StartCallback)(void);

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************
void bttask_pal_initBluetoothPort(void);
bt_byte bttask_pal_getAddrsAssigned(void);
void bttask_pal_setetAddrsAssigned(void);
void bttask_pal_initBluetoothPort(void);
void bttask_pal_handleRxSignal(void);
void bttask_pal_handleTxSignal(void);
void bt_oem_send(const bt_byte* buffer, bt_uint len, bt_oem_send_callback_fp callback);
void bt_oem_recv(bt_byte* buffer, bt_uint len, bt_oem_recv_callback_fp callback);
void APP_btx_csr_set_ps_vars(void);
void bttask_pal_startBluetoothPort_2(void);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************


#ifdef __cplusplus
}
#endif

#endif // __BTPORT_H_INCLUDED__
/*******************************************************************************
 End of File
*/
