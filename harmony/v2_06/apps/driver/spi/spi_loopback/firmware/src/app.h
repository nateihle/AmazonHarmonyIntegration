/*******************************************************************************
  Sample Application Header

  File Name:
    app.h

  Summary:
    Sample application definitions and prototypes

  Description:
    This file contains the sample application's definitions and prototypes.
 ******************************************************************************/

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
 ******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H


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
#include "system/system.h"
#include "driver/spi/drv_spi.h"
#include "system/clk/sys_clk.h"
#include "system/int/sys_int.h"
#include "system/devcon/sys_devcon.h"
#include "system/ports/sys_ports.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

typedef unsigned char SPI_DATA_TYPE;

// *****************************************************************************
// *****************************************************************************
// Section: Application Macros
// *****************************************************************************
// *****************************************************************************

#define MAX_NUM_OF_BYTES        64

#define MAX_NUM_OF_BYTES_IN_BUF (MAX_NUM_OF_BYTES + 4)


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
    
    /* Check if the driver is opened and ready  */
    APP_STATE_CHECK_DRVR_STATE,
    
    /* Application: Master writes the data to the Slave. */
    APP_STATE_SPI1_MASTER_WR,

    /*SPI1 MASTER waits for Write operation to complete*/
    APP_STATE_SPI1_WAIT_FOR_WR_COMPLETION,

    /* SPI1 Idle Mode*/
    APP_STATE_SPI1_MASTER_IDLE,

    /* Application: Slave reads/gets the data from the Master */
    APP_STATE_SPI2_SLAVE_RD,

    /*SPI2 Slave waits for read operation to complete*/
    APP_STATE_SPI2_WAIT_FOR_RD_COMPLETION,

    /* Slave Read operations are complete: Slave Idle State*/
    APP_STATE_SPI2_SLAVE_IDLE,

    /* Application let the master write to slave */
    APP_STATE_TRANSFER_DATA_BTWN_MASTER_SLAVE,

    /* Application verifies the transmitted and received data. */
    APP_STATE_VERIFY_DATA,

    /* In this state, application is in IDLE state after completion. */
    APP_STATE_IDLE,

    /* Application error state */
    APP_STATE_ERROR,
            
    /* In this state, application is successfully completed */
    APP_STATE_SUCCESS

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
    DRV_HANDLE  drvSPI1Handle;

    /* SPI2 Driver Handle  */
    DRV_HANDLE  drvSPI2Handle;

    /* Write buffer handle */
    DRV_SPI_BUFFER_HANDLE   drvSPI1TxBufHandle;

    /* Read buffer handle */
    DRV_SPI_BUFFER_HANDLE   drvSPI2RxBufHandle;

    /* SPI1 Driver TX buffer  */
    //SPI_DATA_TYPE drvSPI1TXbuffer[MAX_NUM_OF_BYTES_IN_BUF];

    /* SPI2 Driver RX buffer  */
    //SPI_DATA_TYPE drvSPI2RXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
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
	void * SPI1TxBufferRef;
	void * SPI2RxBufferRef;

}context;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     Sample application's initialization routine

  Description:
    This routine initializes sample application's state machine.

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
    Sample application tasks function

  Description:
    This routine is the sample application's tasks function.  It
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

void APP_Tasks ( void );

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
    This routine is the SPI2 as SLAVE application's tasks function.  It
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
void APP_SPI_SLAVE_Task(void);


/*******************************************************************************
  Function:
    void APP_BufferEventHandlerSPI1 ( void )

  Summary:
    Application buffer event handler call back functiuon for SPI1.

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
void APP_BufferEventHandlerSPI1(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context );

/*******************************************************************************
  Function:
    void APP_BufferEventHandlerSPI2 ( void )

  Summary:
    Application buffer event handler call back functiuon for SPI2.

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
void APP_BufferEventHandlerSPI2(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context );

/* Extern Application data*/
extern APP_DATA appData;

#endif /* _APP_HEADER_H */
/*******************************************************************************
 End of File
*/
