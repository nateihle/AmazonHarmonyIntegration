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


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_2X
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_2
#pragma config FPLLRNG =    RANGE_8_16_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_33
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     OFF
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK4953 Initialization Data">
/*** CODEC Driver Initialization Data ***/
const DRV_AK4953_INIT drvak4953Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
    .i2sDriverModuleIndex = DRV_AK4953_I2S_DRIVER_MODULE_INDEX_IDX0,
    .i2cDriverModuleIndex = DRV_AK4953_I2C_DRIVER_MODULE_INDEX_IDX0,

    .volume = DRV_AK4953_VOLUME,



};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_I2C Initialization Data">
// *****************************************************************************
/* I2C Driver Initialization Data
*/

const DRV_I2C_INIT drvI2C0InitData =
{
    .i2cId = DRV_I2C_PERIPHERAL_ID_IDX0,
    .i2cPerph = DRV_I2C_BIT_BANG_IDX0,
    .i2cMode = DRV_I2C_OPERATION_MODE_IDX0,
    .portSCL = DRV_SCL_PORT_IDX0,
	.pinSCL  = DRV_SCL_PIN_POSITION_IDX0,
	.portSDA = DRV_SDA_PORT_IDX0,
	.pinSDA  = DRV_SDA_PIN_POSITION_IDX0,
    .baudRate = DRV_I2C_BIT_BANG_BAUD_RATE_IDX0,
	.tmrSource = DRV_I2C_BIT_BANG_TMR_MODULE_IDX0,
	.tmrInterruptSource = DRV_I2C_BIT_BANG_INT_SRC_IDX0,
    .busspeed = DRV_I2C_SLEW_RATE_CONTROL_IDX0,
    .buslevel = DRV_I2C_SMBus_SPECIFICATION_IDX0,
};


const DRV_I2C_INIT drvI2C1InitData =
{
    .i2cId = DRV_I2C_PERIPHERAL_ID_IDX1,
    .i2cPerph = DRV_I2C_BIT_BANG_IDX1,
    .i2cMode = DRV_I2C_OPERATION_MODE_IDX1,
    .portSCL = DRV_SCL_PORT_IDX1,
	.pinSCL  = DRV_SCL_PIN_POSITION_IDX1,
	.portSDA = DRV_SDA_PORT_IDX1,
	.pinSDA  = DRV_SDA_PIN_POSITION_IDX1,
    .baudRate = DRV_I2C_BIT_BANG_BAUD_RATE_IDX1,
	.tmrSource = DRV_I2C_BIT_BANG_TMR_MODULE_IDX1,
	.tmrInterruptSource = DRV_I2C_BIT_BANG_INT_SRC_IDX1,
    .buslevel = DRV_I2C_SMBus_SPECIFICATION_IDX1,
    .busspeed = DRV_I2C_SLEW_RATE_CONTROL_IDX1,
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
// <editor-fold defaultstate="collapsed" desc="DRV_SDCARD Initialization Data">
/*** SDCARD Driver Initialization Data ***/

const DRV_SDCARD_INIT drvSDCardInit =
{
    .spiId = SPI_ID_2,
    .spiIndex = 0,
    .sdcardSpeedHz = 20000000,
    .spiClk = CLK_BUS_PERIPHERAL_2,
    .writeProtectPort = PORT_CHANNEL_F,
    .writeProtectBitPosition = PORTS_BIT_POS_1,
    .chipSelectPort = PORT_CHANNEL_B,
    .chipSelectBitPosition = PORTS_BIT_POS_14,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_SPI Initialization Data"> 
 /*** SPI Driver Initialization Data ***/
  /*** Index 0  ***/
 DRV_SPI_INIT drvSpi0InitData =
 {
    .spiId = DRV_SPI_SPI_ID_IDX0,
    .taskMode = DRV_SPI_TASK_MODE_IDX0,
    .spiMode = DRV_SPI_SPI_MODE_IDX0,
    .allowIdleRun = DRV_SPI_ALLOW_IDLE_RUN_IDX0,
    .spiProtocolType = DRV_SPI_SPI_PROTOCOL_TYPE_IDX0,
    .commWidth = DRV_SPI_COMM_WIDTH_IDX0,
    .baudClockSource = DRV_SPI_CLOCK_SOURCE_IDX0,
    .spiClk = DRV_SPI_SPI_CLOCK_IDX0,
    .baudRate = DRV_SPI_BAUD_RATE_IDX0,
    .bufferType = DRV_SPI_BUFFER_TYPE_IDX0,
    .clockMode = DRV_SPI_CLOCK_MODE_IDX0,
    .inputSamplePhase = DRV_SPI_INPUT_PHASE_IDX0,
    .txInterruptSource = DRV_SPI_TX_INT_SOURCE_IDX0,
    .rxInterruptSource = DRV_SPI_RX_INT_SOURCE_IDX0,
    .errInterruptSource = DRV_SPI_ERROR_INT_SOURCE_IDX0,
    .dummyByteValue = DRV_SPI_TRANSMIT_DUMMY_BYTE_VALUE_IDX0,
    .queueSize = DRV_SPI_QUEUE_SIZE_IDX0,
    .jobQueueReserveSize = DRV_SPI_RESERVED_JOB_IDX0,
 };
// </editor-fold>
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0,
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_16_BIT,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};
// <editor-fold defaultstate="collapsed" desc="DRV_TOUCH_MTCH6301 Initialization Data">
/*** MTCH6301 Driver Initialization Data ***/

const DRV_TOUCH_INIT drvTouchInitData =
{
    .moduleInit                  = {0},
    .touchId                     = DRV_TOUCH_INDEX_0,
    .drvInitialize               = NULL,
    .drvOpen                     = DRV_I2C_Open,
    .orientation                 = 0,
    .horizontalResolution        = 480,
    .verticalResolution          = 272,
    .pReadFunc                   = NULL,
    .pWriteFunc                  = NULL,
    .pSectorErase                = NULL,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
const DRV_USBHS_INIT drvUSBHSInit =
{
    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* Interrupt Source for USB module */
    .interruptSourceUSBDma = INT_SOURCE_USB_1_DMA,

    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},

    /* Operation Mode */
    .operationMode = DRV_USBHS_OPMODE_HOST,

    /* Operation Speed */ 
    .operationSpeed = USB_SPEED_HIGH,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USBHS_ID_0,
    
    /* Root Hub Port indication */
    .portIndication = NULL,
        
     /* Power Enable */ 
    .portPowerEnable = BSP_USBVBUSPowerEnable,
    
    /* Over Current detection */ 
    .portOverCurrentDetect = BSP_USBVBUSSwitchOverCurrentDetect,
     
     /* Available Current */
    .rootHubAvailableCurrent = 500,
};
// </editor-fold>

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
// <editor-fold defaultstate="collapsed" desc="SYS_FS Initialization Data">
/*** File System Initialization Data ***/


const SYS_FS_MEDIA_MOUNT_DATA sysfsMountTable[SYS_FS_VOLUME_NUMBER] = 
{
    
    {
		.mountName = SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX0,
		.fsType   = SYS_FS_TYPE_IDX0   
    },
    
    {
		.mountName = SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX1,
		.fsType   = SYS_FS_TYPE_IDX1   
    },



     
};




const SYS_FS_REGISTRATION_TABLE sysFSInit [ SYS_FS_MAX_FILE_SYSTEM_TYPE ] =
{
    {
        .nativeFileSystemType = FAT,
        .nativeFileSystemFunctions = &FatFsFunctions
    }
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_MSG Initialization Data">
/*** Message System Initialization Data ***/

static uint16_t queuePriorities0[2] = { 100, 100 };
SYS_MSG_INIT msg0Init =
{
    .nMaxMsgsDelivered = 1,
    .nMessagePriorities = 1,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_TOUCH Initialization Data">
// *****************************************************************************
/* System Touch Initialization Data
*/

const DRV_TOUCH_INIT sysTouchInit0 =
{
    .drvInitialize           = DRV_TOUCH_MTCH6301_Initialize,
    .drvOpen                 = DRV_TOUCH_MTCH6301_Open,
    .drvTouchStatus          = DRV_TOUCH_MTCH6301_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_MTCH6301_TouchDataRead,
    .drvTouchGetX            = DRV_TOUCH_MTCH6301_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_MTCH6301_TouchGetY,
    .drvTouchPenGet          = NULL,
    .orientation             = 0,
    .horizontalResolution    = 480,
    .verticalResolution      = 272,
	.minTouchDetectDelta     = 2,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_TMR Initialization Data">
/*** TMR Service Initialization Data ***/
const SYS_TMR_INIT sysTmrInitData =
{
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    .drvIndex = DRV_TMR_INDEX_0,
    .tmrFreq = 10, 
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Host Initialization Data"> 



const USB_HOST_TPL_ENTRY USBTPList[ 1 ] =
{
	
    TPL_INTERFACE_CLASS_SUBCLASS_PROTOCOL(0x08, 0x06, 0x50, NULL,  USB_HOST_MSD_INTERFACE) ,






};


const USB_HOST_HCD hcdTable = 
{
    .drvIndex = DRV_USBHS_INDEX_0,
    .hcdInterface = DRV_USBHS_HOST_INTERFACE
};

const USB_HOST_INIT usbHostInitData = 
{
    .nTPLEntries = 1 ,
    .tplList = (USB_HOST_TPL_ENTRY *)USBTPList,
    .hostControllerDrivers = (USB_HOST_HCD *)&hcdTable
    
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="emWin Touch Initialization Data">
/*** emWin Touch Initialization Data ***/

const EMWIN_TOUCH_INIT emWinTouchInitData = 
{

};
// </editor-fold>

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

    /* Board Support Package Initialization */
    BSP_Initialize();        

    /* Initialize Drivers */
    sysObj.drvI2C0 = DRV_I2C_Initialize(DRV_I2C_INDEX_0, (SYS_MODULE_INIT *)&drvI2C0InitData);
    sysObj.drvI2C1 = DRV_I2C_Initialize(DRV_I2C_INDEX_1, (SYS_MODULE_INIT *)&drvI2C1InitData);


    SYS_INT_VectorPrioritySet(INT_VECTOR_T8, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T8, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_VectorPrioritySet(INT_VECTOR_T9, INT_PRIORITY_LEVEL4);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T9, INT_SUBPRIORITY_LEVEL0);

    sysObj.drvI2S0 = DRV_I2S_Initialize(DRV_I2S_INDEX_0, (SYS_MODULE_INIT *)&drvI2S0InitData);


    sysObj.sysDma = SYS_DMA_Initialize((SYS_MODULE_INIT *)&sysDmaInit);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA2, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA2, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_2);


    sysObj.drvak4953Codec0 = DRV_AK4953_Initialize(DRV_AK4953_INDEX_0, (SYS_MODULE_INIT *)&drvak4953Codec0InitData);

    sysObj.drvSDCard = DRV_SDCARD_Initialize(DRV_SDCARD_INDEX_0,(SYS_MODULE_INIT *)&drvSDCardInit);

    /*** SPI Driver Index 0 initialization***/

    SYS_INT_VectorPrioritySet(DRV_SPI_TX_INT_VECTOR_IDX0, DRV_SPI_TX_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_SPI_TX_INT_VECTOR_IDX0, DRV_SPI_TX_INT_SUB_PRIORITY_IDX0);
    SYS_INT_VectorPrioritySet(DRV_SPI_RX_INT_VECTOR_IDX0, DRV_SPI_RX_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_SPI_RX_INT_VECTOR_IDX0, DRV_SPI_RX_INT_SUB_PRIORITY_IDX0);
    SYS_INT_VectorPrioritySet(DRV_DRV_SPI_ERROR_INT_VECTOR_IDX0, DRV_SPI_ERROR_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_DRV_SPI_ERROR_INT_VECTOR_IDX0, DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX0);
    sysObj.spiObjectIdx0 = DRV_SPI_Initialize(DRV_SPI_INDEX_0, (const SYS_MODULE_INIT  * const)&drvSpi0InitData);

    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);


    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
 
 
     sysObj.drvMtch6301 = DRV_TOUCH_MTCH6301_Initialize(DRV_TOUCH_MTCH6301_INDEX_0,
                                    (SYS_MODULE_INIT *)&drvTouchInitData);
 
    // initialize the GFX hardware abstraction layer
    GFX_Initialize();

    /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBHS_Initialize(DRV_USBHS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBHSInit);
    
    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);
    
    /* Set the priority of the USB DMA Interrupt */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1_DMA, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of the USB DMA Interrupt */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1_DMA, INT_SUBPRIORITY_LEVEL0);
    

    /* Initialize System Services */
    SYS_PORTS_Initialize();

    /*** File System Service Initialization Code ***/
    SYS_FS_Initialize( (const void *) sysFSInit );

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /*Setup the INT_SOURCE_EXTERNAL_1 and Enable it*/
    SYS_INT_VectorPrioritySet(INT_VECTOR_INT1, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_INT1, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1,INT_EDGE_TRIGGER_RISING);
    SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_1);






    /*** Message Service Initialization Code ***/
    msg0Init.nQSizes = queuePriorities0;
    sysObj.sysMsg0 = SYS_MSG_Initialize(SYS_MSG_0, (SYS_OBJ_HANDLE)&msg0Init);

    /*** Random Service Initialization Code ***/
    SYS_RANDOM_Initialize(0, 0);
    sysObj.sysTouchObject0 = SYS_TOUCH_Initialize(SYS_TOUCH_INDEX_0, (SYS_MODULE_INIT*)&sysTouchInit0);


    /*** TMR Service Initialization Code ***/
    sysObj.sysTmr  = SYS_TMR_Initialize(SYS_TMR_INDEX_0, (const SYS_MODULE_INIT  * const)&sysTmrInitData);

    /* Initialize Middleware */
 
    /* Initialize the USB Host layer */
    sysObj.usbHostObject0 = USB_HOST_Initialize (( SYS_MODULE_INIT *)& usbHostInitData );





    emWin_GuiInitialize( );


    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

