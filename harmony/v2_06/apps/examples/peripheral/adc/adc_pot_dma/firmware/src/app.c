/*******************************************************************************
  MPLAB Harmony Application

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the application logic.
 *******************************************************************************/


// DOM-IGNORE-BEGIN
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
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/adc/plib_adc.h"
#include "system/system.h"
#include "system/devcon/sys_devcon.h"
#include "framework/driver/adc/drv_adc_static.h"
#include "peripheral/dma/plib_dma.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

uint16_t adc_buffer[256];

SYS_DMA_CHANNEL_HANDLE channelHandle;

/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData = {
    0,
    0,
    0
};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
 */


APP_DRV_OBJECTS appDrvObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void)
{
 
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.buffer_flag = 0;
    appData.avgValue = 0;
    
    /* Allocate a DMA channel */
    channelHandle = SYS_DMA_ChannelAllocate(DMA_CHANNEL_0);
}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks(void)
{

    /* check the application state*/
    switch (appData.state)
    {
        /* Application's initial state. */
    case APP_STATE_INIT:
        /* configure the DMA channel 0*/
        SYS_DMA_ChannelTransferAdd(channelHandle, (const void*)&ADC1BUF0, 2, &adc_buffer, 512, 2 );
        SYS_DMA_ChannelSetup(channelHandle,SYS_DMA_CHANNEL_OP_MODE_AUTO, DMA_TRIGGER_ADC_1);
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE);
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL);
        PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_HALF_FULL);
        PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_DESTINATION_DONE);
        SYS_DMA_ChannelEnable(channelHandle);
        /* Enable ADC */
        DRV_ADC_Open();
        DRV_ADC_Start();
        appData.state = APP_STATE_RUN;
        break;

    case APP_STATE_RUN:

        if (appData.buffer_flag == 1)
        {
            appData.buffer_flag = 0;
        }
        if (appData.buffer_flag == 2)
        {
            appData.buffer_flag = 0;
        }
        break;

    default:
        break;
    }
}


/*******************************************************************************
 End of File
 */

