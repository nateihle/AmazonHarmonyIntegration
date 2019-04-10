/*******************************************************************************
  SAMPLE Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_local.h

  Summary:
    SAMPLE driver local declarations and definitions

  Description:
    This file contains the SAMPLE driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SAMPLE_LOCAL_H
#define _DRV_SAMPLE_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "sample/drv_sample.h"
#include "sample/src/drv_sample_variant_mapping.h"



// *****************************************************************************
// *****************************************************************************
// Section: SAMPLE Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SAMPLE_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name

  Description:
     This macro creates the instance-specific name of the given static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)     name ## id

#define _DRV_STATIC_OBJ_NAME_A(name,id)     _DRV_STATIC_OBJ_NAME_B(name,id)

#define _DRV_SAMPLE_OBJ_MAKE_NAME(name)    _DRV_STATIC_OBJ_NAME_A(name, DRV_SAMPLE_INDEX)


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SAMPLE Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance

  Description:
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None
*/

typedef struct _DRV_SAMPLE_OBJ_STRUCT
{
    /* Required : The status of the driver */
    SYS_STATUS                                              status;

    /* Required : The peripheral Id associated with the object */
    SAMPLE_MODULE_ID                                        sampleId;

    /* Required : Flag to indicate in use  */
    bool                                                    inUse;

    /* Optional: number of clients possible with the hardware instance */
    uint8_t                                                 numClients;

    /* TODO : Other members to support driver operations */

} DRV_SAMPLE_OBJ;


typedef unsigned short int DRV_SAMPLE_OBJ_HANDLE;


// *****************************************************************************
/* SAMPLE Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None
*/

typedef struct _DRV_SAMPLE_CLIENT_OBJ_STRUCT
{
    /* Optional : Maintains Client Status  */
    DRV_SAMPLE_CLIENT_STATUS                                status;

    /* Required : Driver Object associated with the client */
    DRV_SAMPLE_OBJ_HANDLE                                   driverHandle;

    /* Required : Flag to indicate in use  */
    bool                                                    inUse;

    /* TODO : Other members to maintain client state */

} DRV_SAMPLE_CLIENT_OBJ;


typedef unsigned short int DRV_SAMPLE_CLIENT_OBJ_HANDLE;


// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware Objects for the dynamic driver
*/

extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj[];


// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj[];


// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj0;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj1;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj2;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj3;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj4;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj5;
extern DRV_SAMPLE_OBJ            gDrvSAMPLEObj6;


// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj0;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj1;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj2;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj3;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj4;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj5;
extern DRV_SAMPLE_CLIENT_OBJ     gDrvSAMPLEClientObj6;


#endif //#ifndef _DRV_SAMPLE_LOCAL_H

/*******************************************************************************
 End of File
*/

