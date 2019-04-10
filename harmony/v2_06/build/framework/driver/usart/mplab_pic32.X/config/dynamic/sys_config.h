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


// *****************************************************************************
/* USART driver objects configuration

  Summary
    Sets up the maximum number of hardware instances that can be supported

  Description
    Sets up the maximum number of hardware instances that can be supported

  Remarks:
    None
*/

#define DRV_USART_INSTANCES_NUMBER                      6


// *****************************************************************************
/* USART Client Count Configuration

  Summary
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Remarks:
    3 clients per instance.
*/

#define DRV_USART_CLIENTS_NUMBER        (DRV_USART_INSTANCES_NUMBER * 2)


#define DRV_USART_XFER_BUFFER_NUMBER    (DRV_USART_CLIENTS_NUMBER * 2)

#warning "The definition of DRV_USART_INTERRUPT_SOURCE_TX is incorrect.   Fix drv_usart_dynamic.c line 867"
#define DRV_USART_INTERRUPT_SOURCE_TX 0
