/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"  //KEEP THIS : 
                  // Included here because I removed it from system definitions.
                  // system_definitions.h has no dependency on app.h",
                  // while the app does depend on system_definitions.h CAL


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      ON
#pragma config JTAGEN =     ON
#pragma config ICESEL =     ICS_PGx2
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       ON
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_2
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_25
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_3
#pragma config FPLLMUL =    MUL_24
#pragma config FPLLODIV =   DIV_1
#pragma config UPLLIDIV =   DIV_3
#pragma config UPLLEN =     ON
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FSRSSEL =    PRIORITY_7
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON
#pragma config FVBUSONIO =  ON
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK4384 Initialization Data">
/*** CODEC Driver Initialization Data ***/
const DRV_AK4384_INIT drvak4384Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
    .i2sDriverModuleIndex = DRV_AK4384_I2S_DRIVER_MODULE_INDEX_IDX0,
    .volume = DRV_AK4384_VOLUME,
    .mclkMode = DRV_AK4384_MCLK_MODE_MACRO,
    .delayDriverInitialization = DRV_AK4384_DELAY_INITIALIZATION,
};
// </editor-fold>
 // <editor-fold defaultstate="collapsed" desc="DRV_I2S Initialization Data">
/*** I2S Driver Initialization Data ***/
const DRV_I2S_INIT drvI2S0InitData =
{
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX0,
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX0, 
    .usageMode = DRV_I2S_USAGE_MODE_IDX0,
    .baudClock = SPI_BAUD_RATE_CLK_IDX0,
    .baud = DRV_I2S_BAUD_RATE,
    .clockMode = DRV_I2S_CLK_MODE_IDX0,
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX0,
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX0,
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX0,
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0,
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX0,
    .queueSizeReceive = QUEUE_SIZE_RX_IDX0,
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX0,
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX0,    
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX0,    
    .dmaChannelReceive = DMA_CHANNEL_NONE,
};





// </editor-fold>
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0, 
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_IDX0,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};
const DRV_TMR_INIT drvTmr1InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX1,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX1,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX1, 
    .prescale = DRV_TMR_PRESCALE_IDX1,
    .mode = DRV_TMR_OPERATION_MODE_16_BIT,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX1,
    .asyncWriteEnable = false,
};
const DRV_TMR_INIT drvTmr2InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX2,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX2,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX2, 
    .prescale = DRV_TMR_PRESCALE_IDX2,
    .mode = DRV_TMR_OPERATION_MODE_IDX2,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX2,
    .asyncWriteEnable = false,
};

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="SYS_CLK Initialization Data">
// *****************************************************************************
/* System Clock Initialization Data
*/
const SYS_CLK_INIT sysClkInit =
{
    .moduleInit = {0},
    .systemClockFrequencyHz = SYS_CLK_FREQ,
    .waitTillComplete = true,
    .onWaitInstruction = SYS_CLK_ON_WAIT,
};
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_DMA Initialization Data">
/*** System DMA Initialization Data ***/

const SYS_DMA_INIT sysDmaInit =
{
	.sidl = SYS_DMA_SIDL_DISABLE,

};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_DEVCON_JTAGDisable();
    //KEEP THE FOLLOWING LINE
    SYS_PORTS_Initialize();

    /* Board Support Package Initialization */
    BSP_Initialize();        


    /* Initialize Drivers */
    sysObj.drvI2S0 = DRV_I2S_Initialize(DRV_I2S_INDEX_0, (SYS_MODULE_INIT *)&drvI2S0InitData);




    /*Initialize PMP0 */
    DRV_PMP0_Initialize();
    DRV_PMP0_ModeConfig();

    sysObj.sysDma = SYS_DMA_Initialize((SYS_MODULE_INIT *)&sysDmaInit);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA2, INT_PRIORITY_LEVEL2);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA2, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_2);



    /* Initialize ADC */
    DRV_ADC_Initialize();

    sysObj.drvak4384Codec0 = DRV_AK4384_Initialize(DRV_AK4384_INDEX_0, (SYS_MODULE_INIT *)&drvak4384Codec0InitData);


    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);
    sysObj.drvTmr1 = DRV_TMR_Initialize(DRV_TMR_INDEX_1, (SYS_MODULE_INIT *)&drvTmr1InitData);
    sysObj.drvTmr2 = DRV_TMR_Initialize(DRV_TMR_INDEX_2, (SYS_MODULE_INIT *)&drvTmr2InitData);


    SYS_INT_VectorPrioritySet(INT_VECTOR_T4, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T4, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_T3, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T3, INT_SUBPRIORITY_LEVEL0);
 
 
 
    // initialize the GFX hardware abstraction layer
    GFX_Initialize();


    /* Initialize System Services */
    //DON'T UNCOMMENT THE FOLLOWING LINE
    //SYS_PORTS_Initialize();    

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();
  
    /* Initialize Middleware */

    // initialize UI library
    LibAria_Initialize();


    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

