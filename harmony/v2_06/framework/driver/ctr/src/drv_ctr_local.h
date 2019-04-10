/*******************************************************************************
  CTR Driver Local Data Structures

  Company:
  Microchip Technology Inc.

  File Name:
  drv_ctr_local.h

  Summary:
  CTR Driver Local Data Structures

  Description:
  Driver Local Data Structures
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

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
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_CTR_LOCAL_H
#define _DRV_CTR_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/ctr/drv_ctr.h"
#include "driver/ctr/src/drv_ctr_variant_mapping.h"
#include "osal/osal.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constant Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Maximum number of latches used for any use-case in the CTR driver.

  Summary:
    Maximum number of latches used for any use-case

  Description:
    These constants provide Maximum number of latches used for any use-case in 
	CTR driver. The use-cases include wifi, USB or GPIO.
	As of current Implementation, Wifi uses 4 latches, USB uses 1 latch and
	GPIO uses 1 latch. So, maxumum number of latch used out of all usecases is 4.
	
  Remarks:
    These constants should be used in place of hard-coded numeric literals.
*/

#define 	 DRV_CTR_MAX_LATCH_USED  4

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* CTR Driver Global Instances Object

  Summary:
  Object used to keep track of data that is common to all instances of the
  CTR driver.

  Description:
  This object is used to keep track of any data that is common to all
  instances of the CTR driver.

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
} DRV_CTR_COMMON_DATA_OBJ;

// *****************************************************************************
/* CTR Driver Instance Object

  Summary:
  Object used to keep any data required for an instance of the CTR 
   driver.

  Description:
  This object is used to keep track of any data that must be maintained to 
  manage a single instance of the driver.

  Remarks:
  None.
*/

typedef struct
{
  /*  The module index of associated  driver */
  SYS_MODULE_INDEX  ctrModuleIndex;
  
  /* Identifies the CTR peripheral instance */
  CTR_MODULE_ID ctrID;
  
  /* The status of the driver */
  SYS_STATUS status;

  /* Flag to indicate this object is in use  */
  bool inUse;

  /* Flag to indicate that driver has been opened exclusively. */
  bool isExclusive;

  /* Keeps track of the number of clients that have opened this driver */
  uint32_t nClients;

  /* Interrupt source for CTR Event interrupt */
  INT_SOURCE ctrEventInterruptSource;
  
  /* Interrupt source for CTR Trigger interrupt */
  INT_SOURCE ctrTriggerInterruptSource;
  
  /* CTR Event Interrupt Mode */
  CTR_LATCH_INT_MODE ctrLatchEventMode;

  /* Pointer to the client object */
  DRV_CTR_CLIENT_OBJ *clientObj;
  
  /* State of the task */
 // DRV_CTR_DATA_OBJECT_STATE  	state; /* TBD */

  /* Driver Mode */
  DRV_MODE drvMode;
  
  uint8_t latmax;
  
  uint8_t bufmax;
 
  /* Latch Callback queue occupancy counter */
  uint8_t latchCallBackOccupancy[DRV_CTR_LATCH_NUM];
  
  /* Hardware instance mutex */
  OSAL_MUTEX_DECLARE(mutexDriverInstance);

} DRV_CTR_OBJ;

// *****************************************************************************
/* CTR Driver Client Object

  Summary:
  Object used to track a single client.

  Description:
  This object is used to keep the data necesssary to keep track of a single 
  client.

  Remarks:
  None.
*/

typedef struct
{
  /* The hardware instance object associated with the client */
  DRV_CTR_OBJ * hDriver;

  /* This flags indicates if the object is in use or is available */
  bool inUse;

  /* To configure for onetime or multiple interrupts */
  bool oneTimeCallback;
  
  /* Client Status */
  DRV_CTR_CLIENT_STATUS clientStatus; /* TBD */

  /* Application Context associated with this client */
  uintptr_t context;
  
  /* client Callback */
  DRV_CTR_CALLBACK timeStampCallback;
  
} DRV_CTR_CLIENT_OBJ;


#endif //#ifndef _DRV_CTR_LOCAL_H

/*******************************************************************************
 End of File
*/

