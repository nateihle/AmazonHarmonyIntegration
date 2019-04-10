/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Macro Definitions
// *****************************************************************************
// *****************************************************************************

#define BUFFER_SIZE  512

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

unsigned int i;
volatile unsigned int a_source_buffer[BUFFER_SIZE];
// This spreads the buffers out in memory.  It ensures that the source and 
// destination buffers are in different cache rows.
volatile unsigned int Unused_source_buffer[64*BUFFER_SIZE];
volatile unsigned int a_source_buffer2[BUFFER_SIZE];
// This spreads the buffers out in memory.  It ensures that the source and 
// destination buffers are in different cache rows.
volatile unsigned int Unused_source_buffer[64*BUFFER_SIZE];
volatile unsigned int a_dest_buffer[BUFFER_SIZE];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Setup DMA - disable transfers on interrupt and set the transfer to be started manually */
   PLIB_DMA_Enable(DMA_ID_0);
   PLIB_DMA_ChannelXAutoDisable(DMA_ID_0, DMA_CHANNEL_0);
   PLIB_DMA_ChannelXDestinationStartAddressSet(DMA_ID_0, DMA_CHANNEL_0, (uint32_t)a_dest_buffer);
   PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, DMA_CHANNEL_0, sizeof(a_source_buffer));
   PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DMA_CHANNEL_0, sizeof(a_dest_buffer));
   PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DMA_CHANNEL_0, sizeof(a_dest_buffer));
   PLIB_DMA_ChannelXTriggerEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_CHANNEL_TRIGGER_TRANSFER_START);
   PLIB_DMA_ChannelXStartIRQSet(DMA_ID_0, DMA_CHANNEL_0, DMA_TRIGGER_SOURCE_NONE);

   appData.state = APP_INIT;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    switch (appData.state)
    {
        case APP_INIT:

            /* Fill RAM with zeros to start */
            for (i = 0; i < BUFFER_SIZE; i++)
            {
                *(volatile unsigned int *)KVA0_TO_KVA1(a_source_buffer + i) = 0x00000000;
                *(volatile unsigned int *)KVA0_TO_KVA1(a_source_buffer2 + i) = 0x00000000;
                *(volatile unsigned int *)KVA0_TO_KVA1(a_dest_buffer + i) = 0x00000000;
            }

            /* Fill the cache with some data */
            for (i = 0; i < BUFFER_SIZE; i++)
            {
                a_source_buffer[i] = 0xDEADBEEF;
                a_source_buffer2[i] = 0xF005BA11;
            }

            /* Write it back to ram to prepare the buffers for DMA transfer */
            SYS_DEVCON_DataCacheClean((uint32_t) a_source_buffer, sizeof (a_source_buffer));
            SYS_DEVCON_DataCacheClean((uint32_t) a_source_buffer2, sizeof (a_source_buffer2));

            appData.state = NON_COHERENT_DMA_TRANSFER_TO_BUFFER;
            break;

        /* This will transfer data over DMA without using the cache functions */
        case NON_COHERENT_DMA_TRANSFER_TO_BUFFER:

            /* Copy a_source_buffer to a_dest_buffer using DMA */
            PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DMA_CHANNEL_0, (uint32_t)a_source_buffer);
            PLIB_DMA_ChannelXEnable(DMA_ID_0, DMA_CHANNEL_0);
            PLIB_DMA_StartTransferSet(DMA_ID_0, DMA_CHANNEL_0);

            appData.state = NON_COHERENT_CPU_READS_VALID_DATA;
            break;

        case NON_COHERENT_CPU_READS_VALID_DATA:

             /* Wait for transfer to finish, then check to see if the copied data is the same */
            if (!PLIB_DMA_ChannelXBusyIsBusy(DMA_ID_0, DMA_CHANNEL_0))
            {
                for (i = 0; i < BUFFER_SIZE; i++)
                {
                    if (a_dest_buffer[i] != a_source_buffer[i])
                    {
                        /* if the program gets here, the data isn't the same - light up the
                           yellow LED (program shouldn't go here) */
                        BSP_LEDOn(BSP_LED_2);
                        appData.state = COHERENT_DMA_TRANSFER_TO_BUFFER;
                        return;
                    }
                }
                appData.state = NON_COHERENT_DMA_TRANSFERS_AGAIN;
                return;
            }

            appData.state = NON_COHERENT_CPU_READS_VALID_DATA;
            break;

        case NON_COHERENT_DMA_TRANSFERS_AGAIN:

            /* Copy a_source_buffer2 to a_dest_buffer using DMA */
            PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DMA_CHANNEL_0, (uint32_t)a_source_buffer2);
            PLIB_DMA_ChannelXEnable(DMA_ID_0, DMA_CHANNEL_0);
            PLIB_DMA_StartTransferSet(DMA_ID_0, DMA_CHANNEL_0);

            appData.state = NON_COHERENT_CPU_READS_STALE_DATA;
            break;

        case NON_COHERENT_CPU_READS_STALE_DATA:

            /* Wait for transfer to finish, then check to see if the copied data is the same */
            if (!PLIB_DMA_ChannelXBusyIsBusy(DMA_ID_0, DMA_CHANNEL_0))
            {
                for (i = 0; i < BUFFER_SIZE; i++)
                {
                    if (a_dest_buffer[i] != a_source_buffer2[i])
                    {
                        /* if the program gets here, the data isn't the same - light up the
                           red LED (the program SHOULD go here). After the first DMA transfer and
                           read is performed on a_dest_buffer, the data contained in a_dest_buffer is
                           pulled into the cache. The second DMA transfer then updates the data contained
                           in a_dest_buffer (in ram), but the CPU isn't aware of this and the cache still contains
                           the stale data. So when comparing the source and destination buffers, they appear to
                           be different since the CPU is reading stale data from the cache. Before doing a read,
                           the cache must be invalidated so the new data is pulled into the cache from ram
                           (demonstrated in the next section). */
                        BSP_LEDOn(BSP_LED_1);
                        appData.state = COHERENT_DMA_TRANSFER_TO_BUFFER;
                        return;
                    }
                }
                appData.state = COHERENT_DMA_TRANSFER_TO_BUFFER;
                return;
            }

            appData.state = NON_COHERENT_CPU_READS_STALE_DATA;
            break;

        /* This will transfer data over DMA using the cache functions */
        case COHERENT_DMA_TRANSFER_TO_BUFFER:

            /* Fill RAM with zeros to start */
            for (i = 0; i < BUFFER_SIZE; i++)
            {
                *(volatile unsigned int *)KVA0_TO_KVA1(a_dest_buffer + i) = 0x00000000;
            }

            /* Copy a_source_buffer to a_dest_buffer using DMA */
            PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DMA_CHANNEL_0, (uint32_t)a_source_buffer);
            PLIB_DMA_ChannelXEnable(DMA_ID_0, DMA_CHANNEL_0);
            PLIB_DMA_StartTransferSet(DMA_ID_0, DMA_CHANNEL_0);

            appData.state = COHERENT_CPU_READS_VALID_DATA;
            break;

        case COHERENT_CPU_READS_VALID_DATA:

            /* Wait for transfer to finish, then check to see if the copied data is the same */
            if (!PLIB_DMA_ChannelXBusyIsBusy(DMA_ID_0, DMA_CHANNEL_0))
            {
                for (i = 0; i < BUFFER_SIZE; i++)
                {
                    if (a_dest_buffer[i] != a_source_buffer[i])
                    {
                        /* if the program gets here, the copied data isn't the same - light up the
                           yellow LED (the program shouldn't go here) */
                        BSP_LEDOn(BSP_LED_2);
                        appData.state = APP_IDLE;
                        return;
                    }
                }
                appData.state = COHERENT_DMA_TRANSFERS_AGAIN;
                return;
            }

            appData.state = COHERENT_CPU_READS_VALID_DATA;
            break;

        case COHERENT_DMA_TRANSFERS_AGAIN:

            /* Copy a_source_buffer2 to a_dest_buffer using DMA */
            PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DMA_CHANNEL_0, (uint32_t)a_source_buffer2);
            PLIB_DMA_ChannelXEnable(DMA_ID_0, DMA_CHANNEL_0);
            PLIB_DMA_StartTransferSet(DMA_ID_0, DMA_CHANNEL_0);

            appData.state = COHERENT_CPU_READS_VALID_DATA_AGAIN;
            break;

        case COHERENT_CPU_READS_VALID_DATA_AGAIN:

            /* Wait for transfer to finish, then check to see if the copied data is the same */
            if (!PLIB_DMA_ChannelXBusyIsBusy(DMA_ID_0, DMA_CHANNEL_0))
            {
                /* This is where we make the cache and ram coherent. The cache entries
                   containing data from a_dest_buffer are invalidated. This forces the cache
                   to fetch new data for those entries from ram, when doing a read on
                   a_dest_buffer. */
                SYS_DEVCON_DataCacheInvalidate((uint32_t)a_dest_buffer, sizeof(a_dest_buffer));

                for (i = 0; i < BUFFER_SIZE; i++)
                {
                    if (a_dest_buffer[i] != a_source_buffer2[i])
                    {
                        /* if the program gets here, the data isn't the same - light up the
                           yellow LED (the program shouldn't go here) */
                        BSP_LEDOn(BSP_LED_2);
                        appData.state = APP_IDLE;
                        return;
                    }
                }
                /* if the program gets here, the data is the same - light up the
                   green LED (the program SHOULD go here). */
                BSP_LEDOn(BSP_LED_3);
                appData.state = APP_IDLE;
                return;
            }

            appData.state = COHERENT_CPU_READS_VALID_DATA_AGAIN;
            break;

        case APP_IDLE:
            /* do nothing */
            break;

        default:
            break;
    }
}
 

/*******************************************************************************
 End of File
 */

