// *****************************************************************************
/* USART Interrupt Mode Operation Control

  Summary:
    Macro controls operation of the driver in the interrupt mode

  Description:
    This macro controls the operation of the driver in the interrupt
    mode of operation.The possible values it can take are
    - true - Enables the interrupt mode
    - false - Enables the polling mode

*/

#define DRV_USART_INTERRUPT_MODE    true


#define DRV_USART_XFER_BUFFER_NUMBER    2

#warning "The definition of DRV_USART_INTERRUPT_SOURCE_TX is incorrect.   Fix drv_usart_dynamic.c line 867"


// *****************************************************************************
/* DRV_USART_INTERRUPT_SOURCE_RX = INT_SOURCE_USART_x_RECEIVE, where x =
   the peripheral number

  Summary:
    Macro to define the Rx interrupt source in case of static driver

  Description:
    Macro to define the Rx interrupt source in case of static driver

  Remarks:
    Refer to the INT PLIB document for more information on INT_SOURCE
    enumeration
*/


// *****************************************************************************
/* DRV_USART_INTERRUPT_SOURCE_ERROR = INT_SOURCE_USART_x_ERROR, x=peripheral
   number

  Summary:
    Defines the interrupt source for the error interrupt

  Description:
    This macro defines the interrupt source for the error interrupt. This is the
    static override of the dynamic intialization passed using DRV_USART_INIT.
*/


// *****************************************************************************
/* DRV_USART_INDEX = DRV_USART_INDEX_x, where x = driver index number

  Summary:
    USART Static Index selection

  Description:
    USART Static Index selection for the driver object reference

  Remarks:
    This index is required to make a reference to the driver object

*/


// *****************************************************************************
/* DRV_USART_PERIPHERAL_ID = USART_ID_x, where x=peripheral number

  Summary:
    Configures the PLIB id

  Description:
    This macro configures the PLIB Id for the driver. This is the static
    override of the dynamic intialization passed using DRV_USART_INIT.

  Remarks:
    Defined in project configurations (on the compiler command line).
*/



