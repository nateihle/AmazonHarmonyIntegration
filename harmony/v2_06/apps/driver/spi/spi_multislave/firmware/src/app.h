/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef unsigned char SPI_DATA_TYPE;

#define MAX_NUM_OF_BYTES                64

#define MAX_NUM_OF_BYTES_IN_BUF         (MAX_NUM_OF_BYTES + 4)

/* SS1 is controlled by this GPIO */
#define SPI_SLAVE_1_CS_PORT_ID          PORT_CHANNEL_H

#define SPI_SLAVE_1_CS_PORT_PIN         PORTS_BIT_POS_10

/* SS2 is controlled by this GPIO */
#define SPI_SLAVE_2_CS_PORT_ID          PORT_CHANNEL_H

#define SPI_SLAVE_2_CS_PORT_PIN         PORTS_BIT_POS_15

#define APP_SPI_CS_SELECT(x,y)     \
                                        SYS_PORTS_PinClear(PORTS_ID_0,x,y)

#define APP_SPI_CS_DESELECT(x,y)   \
                                        SYS_PORTS_PinSet(PORTS_ID_0,x,y)
// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* In this state, the application opens the driver. */
    APP_STATE_INIT,
    
    /* Application initiates first transfer between SPI1 to SPI2. */
    APP_STATE_START_TRANSFER_1,
            
    /* Application initiates second transfer between SPI1 to SPI3. */
    APP_STATE_START_TRANSFER_2,

    /* Application let the Master write to Slave */
    APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE,

    /* SPI1 writes the data to the SPI2. */
    APP_STATE_MASTER_WR_SLAVE_1,

    /*SPI1 waits for write operation to complete on SPI2*/
    APP_STATE_MASTER_WAIT_FOR_SLAVE_1_WR_COMPLETION,
            
    /* SPI1 writes the data to the SPI3. */
    APP_STATE_MASTER_WR_SLAVE_2,

    /*SPI1 waits for write operation to complete on SPI3*/
    APP_STATE_MASTER_WAIT_FOR_SLAVE_2_WR_COMPLETION,

    /* SPI1 Idle Mode*/
    APP_STATE_MASTER_IDLE,

    /* SPI2 reads the data from SPI1 */
    APP_STATE_SLAVE_1_RD,

    /*SPI2 waits for read operation to complete*/
    APP_STATE_SLAVE_1_WAIT_FOR_RD_COMPLETION,
            
    /* SPI2 reads the data from SPI1 */
    APP_STATE_SLAVE_2_RD,

    /*SPI3 waits for read operation to complete*/
    APP_STATE_SLAVE_2_WAIT_FOR_RD_COMPLETION,

    /* SPI2 and SPI3 will be idle, Slave Select pins of both SPIs are inactive*/
    APP_STATE_SLAVES_IDLE,

    /* Verify the received data on SPI2 and SPI3. */
    APP_STATE_VERIFY_SLAVES_DATA,

    /* Application is in IDLE state after completion. */
    APP_STATE_IDLE

} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Current state of Application */
    APP_STATES  state;

    /* Current state of SPI Master task */
    APP_STATES  masterState;

    /* Current state of SPI Slave task*/
    APP_STATES  slaveState;

    /* SPI1 Driver Handle  */
    DRV_HANDLE  drvMasterHandle;

    /* SPI2 Driver Handle  */
    DRV_HANDLE  drvSlave1Handle;
    
    /* SPI3 Driver Handle  */
    DRV_HANDLE  drvSlave2Handle;

    /* Write buffer handle for SPI1 */
    DRV_SPI_BUFFER_HANDLE   drvMasterTxBufHandle;

    /* Read buffer handle for SPI2 */
    DRV_SPI_BUFFER_HANDLE   drvSlave1RxBufHandle;

    /* Read buffer handle for SPI3 */
    DRV_SPI_BUFFER_HANDLE   drvSlave2RxBufHandle;
    
    /* Master configuration data */
    DRV_SPI_CLIENT_DATA  * masterCfgData;

} APP_DATA;

/* Driver Buffer Context

  Summary:
    This structure holds the Driver Buffer Context (reference to buffer's passed to Driver)

  Description:
    The reference of the buffer gets passed in as the context into BufferAddRead/write. 
    The same is passed as the context to applicationBufferEventHandler() function.  
*/
typedef struct 
{
	void * MasterTxBufferRef1;
	void * MasterTxBufferRef2;
	void * Slave1RxBufferRef;
	void * Slave2RxBufferRef;

}context;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


/*******************************************************************************
  Function:
    void APP_SPI_MASTER_Task ( void )

  Summary:
    Application SPI MASTER tasks function

  Description:
    This routine is the SPI1 as MASTER application's tasks function.  It
    defines the master's state machine and core logic.

  Precondition:
    APP_Tasks() should be called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This routine must be called from APP_Tasks() routine.
 */
void APP_SPI_MASTER_Task(void);

/*******************************************************************************
  Function:
    void APP_SPI_SLAVE_Task ( void )

  Summary:
    Application SPI SLAVE tasks function

  Description:
    This routine is the SPI2 and SPI3 as SLAVE application's tasks function.  It
    defines the slave 's state machine and core logic.

  Precondition:
    APP_Tasks() should be called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This routine must be called from APP_Tasks() routine.
 */
void APP_SPI_SLAVE_Task(void);


/*******************************************************************************
  Function:
    void APP_BufferEventHandlerMaster ( void )

  Summary:
    Application buffer event handler call back function for SPI1.

  Description:
    This routine is the call back function for SPI1 buffer events.
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead() function along with the context reference.

  Precondition:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().

  Parameters:
    DRV_SPI_BUFFER_EVENT buffEvent: To be populated by the driver. 
    DRV_SPI_BUFFER_HANDLE hBufferEven: Driver client handler
    void* context: Buffer reference for the event.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().
 */
void APP_BufferEventHandlerMaster(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context );

/*******************************************************************************
  Function:
    void APP_BufferEventHandlerSlave1 ( void )

  Summary:
    Application buffer event handler call back function for SPI2.

  Description:
    This routine is the call back function for SPI2 buffer events.
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead() function along with the context reference.

  Precondition:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().

  Parameters:
    DRV_SPI_BUFFER_EVENT buffEvent: To be populated by the driver.
    DRV_SPI_BUFFER_HANDLE hBufferEven: Driver client handler
    void* context: Buffer reference for the event.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().
 */
void APP_BufferEventHandlerSlave1(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context );

/*******************************************************************************
  Function:
    void APP_BufferEventHandlerSlave2 ( void )

  Summary:
    Application buffer event handler call back function for SPI3.

  Description:
    This routine is the call back function for SPI3 buffer events.
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead() function along with the context reference.

  Precondition:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().

  Parameters:
    DRV_SPI_BUFFER_EVENT buffEvent: To be populated by the driver.
    DRV_SPI_BUFFER_HANDLE hBufferEven: Driver client handler
    void* context: Buffer reference for the event.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This routine to be registered with the DRV_SPI_BufferAddWrite()
    or DRV_SPI_BufferAddRead().
 */
void APP_BufferEventHandlerSlave2(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context );

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

