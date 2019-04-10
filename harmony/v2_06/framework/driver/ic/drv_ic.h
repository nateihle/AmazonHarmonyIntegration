/*******************************************************************************
  Input Capture Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    help_drv_ic.h

  Summary:
    Input Capture driver interface declarations for the static single instance 
	driver.

  Description:
    The Input Capture device driver provides a simple interface to manage the 
	Input Capture modules on Microchip microcontrollers. This file defines 
	the interface declarations for the IC driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _DRV_IC_H
#define _DRV_IC_H

#include "system_config.h"
#include "driver/driver_common.h"
#include "system/common/sys_module.h"

// *****************************************************************************
/* IC Driver Module Index Numbers

  Summary:
    IC driver index definitions.

  Description:
    These constants provide IC Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_IC_Initialize and
    DRV_IC_Open routines to identify the driver instance in use.
*/

#define DRV_IC_INDEX_0         0
#define DRV_IC_INDEX_1         1
#define DRV_IC_INDEX_2         2
#define DRV_IC_INDEX_3         3
#define DRV_IC_INDEX_4         4
#define DRV_IC_INDEX_5         5
#define DRV_IC_INDEX_6         6
#define DRV_IC_INDEX_7         7
#define DRV_IC_INDEX_8         8
#define DRV_IC_INDEX_9         9
#define DRV_IC_INDEX_10        10
#define DRV_IC_INDEX_11        11
#define DRV_IC_INDEX_12        12
#define DRV_IC_INDEX_13        13
#define DRV_IC_INDEX_14        14
#define DRV_IC_INDEX_15        15

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for the static driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_IC_Initialize(const SYS_MODULE_INDEX index,
                                     const SYS_MODULE_INIT * const init);

  Summary:
    Initializes the Input Capture instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine initializes the Input Capture driver instance for the specified 
	driver instance, making it ready for clients to use it. The initialization 
	routine is specified by the MHC parameters. The driver instance index is
    independent of the Input Capture module ID. For example, driver instance 0 
	can be assigned to Input Capture 2.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    This routine must be called before any other Input Capture routine is called.
    This routine should only be called once during system initialization. 
*/
SYS_MODULE_OBJ DRV_IC_Initialize(const SYS_MODULE_INDEX index,const SYS_MODULE_INIT * const init);


// *****************************************************************************
/* Function:
DRV_HANDLE DRV_IC_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)

  Summary:
    Opens the Input Capture instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine starts the Input Capture driver for the specified driver
    index, starting an input capture.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
DRV_HANDLE DRV_IC_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent);

// *****************************************************************************
/* Function:
     void DRV_IC_Close(DRV_HANDLE handle)

  Summary:
    Closes the Input Capture instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine stops the Input Capture driver for the specified driver
    index, stopping an input capture.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None. 
*/
void DRV_IC_Close(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
DRV_HANDLE DRV_IC_Start(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)

  Summary:
    Starts the Input Capture instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine starts the Input Capture driver for the specified driver
    index, starting an input capture.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
DRV_HANDLE DRV_IC_Start(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent);


// *****************************************************************************
/* Function:
     void DRV_IC_Stop(DRV_HANDLE handle)

  Summary:
    Stops the Input Capture instance for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine stops the Input Capture driver for the specified driver
    index, stopping an input capture.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None. 
*/
void DRV_IC_Stop(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
     uint32_t DRV_IC_Capture32BitDataRead(DRV_HANDLE handle)


  Summary:
    Reads the 32-bit Input Capture for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine reads the 32-bit data for the specified driver index

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    uint32_t value of the data read from the Input Capture.

  Remarks:
    None.
*/
uint32_t DRV_IC_Capture32BitDataRead(DRV_HANDLE handle);


// *****************************************************************************
/* Function:
     uint16_t DRV_IC_Capture16BitDataRead(DRV_HANDLE handle)

  Summary:
    Reads the 16-bit Input Capture for the specified driver index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    This routine reads the 16-bit data for the specified driver index.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    uint16_t value of the data read from the Input Capture.

  Remarks:
    None.
*/
uint16_t DRV_IC_Capture16BitDataRead(DRV_HANDLE handle);


// *****************************************************************************
/* Function:
     bool DRV_IC_BufferIsEmpty(DRV_HANDLE handle)

  Summary:
    Returns the Input Capture instance buffer empty status for the specified driver 
	index.
    <p><b>Implementation:</b> Static</p>	

  Description:
    Returns the Input Capture instance buffer empty status for the specified driver 
	index. The function should be called to determine whether or not the IC buffer 
	has data.

  Precondition:
    DRV_IC_Initialize has been called.

  Parameters:
    None.

  Returns:
    Boolean
	- 1 - Buffer is empty
	- 0 - Buffer is not empty

  Remarks:
    None. 
*/
bool DRV_IC_BufferIsEmpty(DRV_HANDLE handle);

#ifdef DRV_IC_DRIVER_MODE_STATIC
#include "framework/driver/ic/drv_ic_static.h"
#endif

#endif // #ifndef _DRV_IC_H

/*******************************************************************************
 End of File
*/
