/*******************************************************************************
 OVM7690 Camera Driver Interface File

  File Name:
    drv_ovm7690_static.h

  Summary:
    OVM7690 camera driver interface declarations for the
    static single instance driver.

  Description:
    The OVM7690 camera device driver provides a simple interface to manage
    the OVM7690 camera cube interfacing to Microchip microcontrollers. This
    file defines the interface declarations for the OVM7690 driver.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

<#macro DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE>

#include "framework/driver/camera/ovm7690/drv_ovm7690_static.h"
#include <sys/kmem.h>
#include "framework/system/dma/sys_dma.h"

#define DRV_OVM7690_I2C_MASTER_WRITE_ID          0x42

#define DRV_OVM7690_I2C_MASTER_READ_ID           0xB9

#define OVM7690_READ_ADDRESS                0x0

static SYS_DMA_CHANNEL_HANDLE dmaHandle = SYS_DMA_CHANNEL_HANDLE_INVALID;

// *****************************************************************************
// *****************************************************************************
// Section: Local prototypes
// *****************************************************************************
// *****************************************************************************
volatile uint16_t horizontalSize;

void _delayMS ( unsigned int delayMs );
static void _camera_i2c_init(uint16_t hSize, uint16_t vSize, uint16_t vStart,
                     uint16_t hStart, bool subSample, bool greyscale);

extern volatile uint8_t frameBuffer[1][(272)][(480)];


// *****************************************************************************
// *****************************************************************************
// Section: Instancestatic driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_OVM7690_Initialize(void * frame)
{
    /* Initialize DRV_CAMERA_OVM7690  */
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;
    ANSELH = 0;
    ANSELJ = 0;
    ANSELK = 0;

    //Turn on Camera Clock source
    DRV_TMR0_Start();
    DRV_OC0_Enable();
}


void DRV_OVM7690_DeInitialize(void)
{
    /* DeInitialize DRV_OVM7690 */
}

void DRV_OVM7690_Tasks ( void )
{
    /* Tasks DRV_OVM7690  */

}

void DRV_OVM7690_Open(uint16_t hSize, uint16_t vSize, uint16_t vStart,
                     uint16_t hStart, bool subSample, bool greyScale)
{
    static uint8_t buffer[9];

    buffer[2] = 0x06;
    buffer[7] = 0x01;

    if(subSample == true)
        buffer[8] = 0x56;  //SubSample = 0x46
    else
        buffer[8] = 0x16;

    TRISJbits.TRISJ7 = 0;
    LATJbits.LATJ7 = 0;

     /*Software Reset*/
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x12, 0x80)== false);

    _delayMS(100);

    /*Horizontal Mirror Enable HSYNC/VSYCNC/PCLK and Color*/
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x0c, buffer[2]) == false);
    _delayMS(100);

    /*Slow down Pixel Clock*/
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x11, buffer[7]) == false);
    _delayMS(100);

    /*SubSample and RGB color*/
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x12, buffer[8]) == false);
    _delayMS(100);

    //vSize
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x1A, (vSize>>1))== false);
    _delayMS(100);

    //Shift Pixels by hStart
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x17, (0x69 + (hStart)))== false);
    _delayMS(100);

    //Shift Lines by vStart
    while(_DRV_CAMERA_OVM7690_RegisterSet(0x19, (0x0E + vStart)) == false);
    _delayMS(100);

    //GreyScale
    if(greyScale == true)
    {
        while(_DRV_CAMERA_OVM7690_RegisterSet(0xd2, 0x20) == false);
        _delayMS(100);
    }

    if(subSample == true)
    {
        //Change to horizontal 320
        while(_DRV_CAMERA_OVM7690_RegisterSet(0xcc, 0x01) == false);
        _delayMS(100);

        //Change to Horizontal 320
        //while(I2C_WriteBlock(2, 0x42, 0xcd, 0x40, 1) == false);
        while(_DRV_CAMERA_OVM7690_RegisterSet(0xcd, 0x40) == false);
        _delayMS(100);
    }

    horizontalSize = hSize;

       LATJbits.LATJ7 = 0;

   CNENAbits.CNIEA1 = 1;
   TRISAbits.TRISA1 = 1; //HSYNC IO Pin

   CNENJbits.CNIEJ2 = 1; //VSYNC
   TRISJbits.TRISJ2 = 1; //VSYNC IO Pin
   CNCONJbits.ON = 1;
   CNCONAbits.ON = 1;

   IFS3bits.CNJIF = 0; //Clear intterupt flag

   IPC31bits.CNJIP = 3;
   IPC29bits.CNAIP = 6;

   IEC3bits.CNJIE = 1;
   IEC3bits.CNAIE = 1;

   // set the transfer event control: what event is to start the DMA transfer
   TRISFbits.TRISF5 = 1; //Input for CLK

   TRISK = 0xff; //Input for portk

   dmaHandle = SYS_DMA_ChannelAllocate( DMA_CHANNEL_0 );
   SYS_DMA_ChannelSetup( dmaHandle, 
                         SYS_DMA_CHANNEL_OP_MODE_BASIC, 
                         DMA_TRIGGER_EXTERNAL_2 );
   SYS_DMA_ChannelTransferAdd( dmaHandle, 
                               (void *)KVA_TO_PA(&PORTK), 
                               1, 
                               (void *)KVA_TO_PA(&frameBuffer[0][20][0]),
                               (horizontalSize << 1), 
                               1);

}

bool _DRV_CAMERA_OVM7690_RegisterSet ( uint8_t regIndex, uint8_t regValue )
{
    uint8_t frame[3] = {DRV_OVM7690_I2C_MASTER_WRITE_ID, 0, 0};
    uint32_t frameIndex = 0;
    
    frame[1] = regIndex;
    frame[2] = regValue;
    
    while(DRV_I2C0_MasterBusIdle() == false);
    
    if(DRV_I2C0_MasterStart() == false )
    {
        /* I2C Master Start failed */
        return(false);
    }
    
    for( frameIndex = 0; frameIndex < 3; frameIndex++ )
    {
        if(DRV_I2C0_ByteWrite(frame[frameIndex]) == false)
        {
            DRV_I2C0_MasterStop();
            return (false);
        }
        
        DRV_I2C0_WaitForByteWriteToComplete();
        
        if(DRV_I2C0_WriteByteAcknowledged() == false)
        {
            DRV_I2C0_MasterStop();
            return (false);
        }
        
    }
       
    if(DRV_I2C0_MasterStop() == false)
    {
        /* I2C Master stop failed */
        return(false);
    }
   
    return (true);
}

void _delayMS ( unsigned int delayMs )
{
    if(delayMs)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk/2000)*delayMs);
    }
}

</#macro>
<#if CONFIG_DRV_I2C_INST_IDX0 == true && CONFIG_PIC32MX == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="0"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX0 == true && CONFIG_PIC32MZ == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="0"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true && CONFIG_PIC32MX == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="1"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true && CONFIG_PIC32MZ == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="1"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true && CONFIG_PIC32MX == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="2"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true && CONFIG_PIC32MZ == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="2"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX3 == true && CONFIG_PIC32MX == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="3"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX3 == true && CONFIG_PIC32MZ == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="3"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX4 == true && CONFIG_PIC32MX == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="4"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX4 == true && CONFIG_PIC32MZ == true>
<@DRV_OVM7690_STATIC_FUNCTIONS DRV_INSTANCE="4"/>
</#if>


/*******************************************************************************
 End of File
*/
