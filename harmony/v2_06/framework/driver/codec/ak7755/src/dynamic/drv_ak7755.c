/*******************************************************************************
  AK7755 CODEC Driver Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_AK7755_local_usb_audio.h

  Summary:
    AK7755 CODEC Driver Dynamic implemention.

  Description:
    This file contains the Dynamic mode implementation of the AK7755 driver.
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
// DOM-IGNORE-END

/*************************************************************
 * Include files.
 ************************************************************/
//#define READWRITE

#include "driver/codec/ak7755/src/drv_ak7755_local.h"

//TODO:  The specific APIs used should be here, not all system APIs
//       Copy them from system_definitions.
#include "system_definitions.h"


// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects for the AK7755 CODEC

  Description:
    This data type defines the hardware instance objects that are available for
    AK7755 CODEC, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_AK7755_OBJ gDrvak7755Obj[DRV_AK7755_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the client instances objects

  Description:
    This data type defines the client instance objects that are available on
    AK7755, so as to capture the client state of the instance.
    It uses the configuration of maximum number of clients which can get
    registered per hardware instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_AK7755_CLIENT_OBJ gDrvak7755ClientObj[DRV_AK7755_CLIENTS_NUMBER];


// *****************************************************************************
/* Driver common data object

  Summary:
    Defines the common data object

  Description:
    This object maintains data that is required by all AK7755
   driver instances

  Remarks:
    None
 */
DRV_AK7755_COMMON_DATA_OBJ gDrvak7755CommonDataObj;


static AK7755_ControlRegisters ak7755_ControlRegisters;

// gDrvCommandBuffer is a queue container for AK7755 commands
AK7755_COMMAND_QUEUE gDrvCommandBuffer;
uint8_t command_array[10];

static uint8_t regcur = 0;
static uint8_t control_register = AK7755_CONTROL_REG_CONT01 + AK7755_WRITE_OFFSET;
static uint8_t initRegValues[] = {
    0x00, //ak7755_ControlRegisters.cont01.value,
    0x00, //ak7755_ControlRegisters.cont02.value,
    0x04, //    ak7755_ControlRegisters.cont03.value,
    0x40, //    ak7755_ControlRegisters.cont04.value,
    0x03, //    ak7755_ControlRegisters.cont05.value,
    0x33, //    ak7755_ControlRegisters.cont06.value,
    0xc3, //    ak7755_ControlRegisters.cont07.value,
    0x00, //    ak7755_ControlRegisters.cont08.value,
    0x00, //    ak7755_ControlRegisters.cont09.value,
    0x00, //    ak7755_ControlRegisters.cont0A.value,
    0x00, //    0x00,
    0x00, //    ak7755_ControlRegisters.cont0C.value,
    0x80, //    reg 0D
    0xcf, //    ak7755_ControlRegisters.cont0E.value,
    0x20, //    ak7755_ControlRegisters.cont0F.value,
    0x00, //    reg 0x10
    0x00, //    reg 0x11
    0xff, //    ak7755_ControlRegisters.cont12.value,
    0xd0, //    ak7755_ControlRegisters.cont13.value,
    0xff, //    ak7755_ControlRegisters.cont14.value,
    0x0c, //    ak7755_ControlRegisters.cont15.value,
    0x0c, //    ak7755_ControlRegisters.cont16.value,
    0x26  //    ak7755_ControlRegisters.cont17.value
};

//#define DRIVER __attribute__((section("Driver")))


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Driver System Routine Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

/*   Function:
    SYS_MODULE_OBJ  DRV_AK7755_Initialize
    (
        const SYS_MODULE_INDEX drvIndex,
        const SYS_MODULE_INIT *const init
    );

  Summary:
    Initializes hardware and data for the instance of the AK7755 CODEC module

  Description:
    This routine initializes the AK7755 driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized.

  Precondition:


  Remarks:
    This routine must be called before any other AK7755 routine is called.

    This routine should only be called once during system initialization
    unless DRV_AK7755_Deinitialize is called to de-initialize the driver
    instance. This routine will NEVER block for hardware access.

 */
SYS_MODULE_OBJ  DRV_AK7755_Initialize(const SYS_MODULE_INDEX drvIndex, 
                                      const SYS_MODULE_INIT *const init)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_INIT *ak7755Init;
    uint32_t regValue = 0;
    uint8_t index = 0;
    
    
    /* Validate the driver index */
    if (drvIndex >= DRV_AK7755_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid driver index \r\n");
        return SYS_MODULE_OBJ_INVALID;
    }

    if (true == gDrvak7755Obj[drvIndex].inUse)
    {
        /* Cannot initialize an object that is already in use. */
        SYS_DEBUG(0, "Instance already in use \r\n");
        return SYS_MODULE_OBJ_INVALID;
    }

    ak7755Init = (DRV_AK7755_INIT *) init;
    drvObj = (DRV_AK7755_OBJ *)&gDrvak7755Obj[drvIndex];

    /* Populate the driver object with the required data */
    drvObj->inUse                           = true;
    drvObj->status                          = SYS_STATUS_UNINITIALIZED;
    drvObj->numClients                      = 0;
    drvObj->i2sDriverModuleIndex            = ak7755Init->i2sDriverModuleIndex;
    
    drvObj->i2cDriverModuleIndex            = ak7755Init->i2cDriverModuleIndex;
    drvObj->samplingRate                    = DRV_AK7755_AUDIO_SAMPLING_RATE;
//    drvObj->audioDataFormat                 = DRV_AK7755_AUDIO_DATA_FORMAT_MACRO;
    
    drvObj->drvI2CMasterHandle              = DRV_I2C_Open(DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX0,
                                                            DRV_IO_INTENT_WRITE );
    
    if (drvObj->drvI2CMasterHandle == DRV_HANDLE_INVALID)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    /*Assigning the init volume to all supported audio channels*/
    for(index=0; index < DRV_AK7755_NUMBER_OF_CHANNELS; index++)
    {
        drvObj->volume[index] = ak7755Init->volume;
    }
  
    drvObj->isInInterruptContext            = false;


    /* Initialize */
    drvObj->commandCompleteCallback = (DRV_AK7755_COMMAND_EVENT_HANDLER)0;
    drvObj->commandContextData = -1;

    drvObj->mclk_multiplier = DRV_AK7755_MCLK_SAMPLE_FREQ_MULTPLIER;

    /* Create the hardware instance mutex. */
     if (OSAL_MUTEX_Create(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
     {
        return SYS_MODULE_OBJ_INVALID;
     }

    /* Check if the global mutexes have been created. If not
       then create these. */
     if (!gDrvak7755CommonDataObj.membersAreInitialized)
     {
         /* This means that mutexes where not created. Create them. */
         if ((OSAL_MUTEX_Create(&(gDrvak7755CommonDataObj.mutexClientObjects)) != OSAL_RESULT_TRUE))
         {
            return SYS_MODULE_OBJ_INVALID;
         }
         /* Set this flag so that global mutexes get allocated only once */
         gDrvak7755CommonDataObj.membersAreInitialized = true;
     }

    drvObj->status = SYS_STATUS_BUSY;

//    /* Initiate AK7755 Command */
    drvObj->command = DRV_AK7755_COMMAND_INIT_CLK_PDN_SET;

    /* Config AK7755 control registers */


    ak7755_ControlRegisters.cont00.bits.CKM = AK7755_CKM_CLOCK_MODE;
    
    switch(drvObj->samplingRate){
        case 8000:
            regValue = 0;
            break;
        case 12000:
            regValue = 1;
            break;
        case 16000:
            regValue = 2;
            break;
        case 24000:
            regValue = 3;
            break;
        case 32000:
            regValue = 4;
            break;
        case 48000:
            regValue = 5;
            break;
        case 96000:
            regValue = 6;
            break;
        default:
            break;
            
    }
    
    ak7755_ControlRegisters.cont00.bits.DFS = regValue;
    
    ak7755_ControlRegisters.cont01.bits.BITFS = DRV_AK7755_CLOCK_BICK_FS;

    ak7755_ControlRegisters.cont02.bits.LRIF = DRV_AK7755_LRCK_IF_FORMAT_MACRO;
    
    ak7755_ControlRegisters.cont03.bits.DIF2 = AK7755_DSP_DIN2_FORMAT;
    ak7755_ControlRegisters.cont03.bits.DOF2 = AK7755_DSP_DOUT2_FORMAT;
 // select EC DSP or not
    if (drvObj->samplingRate <= 16000)
    {
        ak7755_ControlRegisters.cont03.bits.BANK = AK7755_DLRAM_MODE_EC_DSP;
        ak7755_ControlRegisters.cont04.value = AK7755_CONT04_DATARAM_CRAM_EC;
    }else
    {
        ak7755_ControlRegisters.cont03.bits.BANK = AK7755_DLRAM_MODE_NORMAL_DSP;
        ak7755_ControlRegisters.cont04.value = AK7755_CONT04_DATARAM_CRAM;
    }

    
    ak7755_ControlRegisters.cont05.value = 3;
    
    ak7755_ControlRegisters.cont06.bits.DIFDA = DRV_AK7755_DAC_INPUT_FORMAT_MACRO;
    ak7755_ControlRegisters.cont06.bits.DIF1 = DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO;
    
    ak7755_ControlRegisters.cont07.bits.DOF4 = DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO;
    ak7755_ControlRegisters.cont07.bits.DOF3 = AK7755_DSP_DOUT3_FORMAT;
    ak7755_ControlRegisters.cont07.bits.DOF1 = DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO;
    
#if DRV_AK7755_ENABLE_MICROPHONE
    ak7755_ControlRegisters.cont08.bits.SELDAI = 3; // SDIN1
#else
    ak7755_ControlRegisters.cont08.bits.SELDAI = 0; // DSP DOUT4, default
#endif
    
    ak7755_ControlRegisters.cont08.bits.SELDO3 = AK7755_SDOUT3_OUTPUT_SELECT;
    ak7755_ControlRegisters.cont08.bits.SELDO2 = AK7755_SDOUT2_OUTPUT_SELECT;
    
    ak7755_ControlRegisters.cont0A.bits.OUT1E = AK7755_SDOUT1_ENABLE;
    ak7755_ControlRegisters.cont0A.bits.OUT2E = AK7755_SDOUT2_ENABLE;
    ak7755_ControlRegisters.cont0A.bits.OUT3E = AK7755_SDOUT3_ENABLE;
    
    
    ak7755_ControlRegisters.cont0C.bits.SELDO1 = AK7755_SDOUT1_OUTPUT_SELECT;
    
    ak7755_ControlRegisters.cont0E.bits.PMADR = AK7755_PMADR_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMADL = AK7755_PMADL_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMADM = AK7755_PMAD2L_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMLO3 = AK7755_PMLO3_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMLO2 = AK7755_PMLO2_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMLO1 = AK7755_PMLO1_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMDAR = AK7755_PMDAR_POWER_MANAGEMENT;
    ak7755_ControlRegisters.cont0E.bits.PMDAL = AK7755_PMDAL_POWER_MANAGEMENT;
    
    ak7755_ControlRegisters.cont0F.bits.PMLI = AK7755_PML1_POWER_MANAGEMENT;
    
    ak7755_ControlRegisters.cont12.value = AK7755_MICROPHONE_GAIN_SETTING;
    
    ak7755_ControlRegisters.cont13.bits.LIGN = AK7755_LINE_IN_VOLUME_SETTING;

    // Volume Settings
    ak7755_ControlRegisters.cont14.value = AK7755_LINE_OUT_VOLUME_SETTING;
    
    ak7755_ControlRegisters.cont15.value = AK7755_ADC_LCH_DIGITAL_VOLUME_SETTING;
    ak7755_ControlRegisters.cont16.value = AK7755_ADC_RCH_DIGITAL_VOLUME_SETTING;
    ak7755_ControlRegisters.cont17.value = AK7755_ADC2_LCH_DIGITAL_VOLUME_SETTING;
    
  
    gDrvCommandBuffer.queueIn = 0;
    gDrvCommandBuffer.queueOut = 0;
    
    /* Return the object structure */
    return ((SYS_MODULE_OBJ) drvObj);

} /* DRV_AK7755_Initialize */



// *****************************************************************************
/* Function:
    void DRV_AK7755_Deinitialize( SYS_MODULE_OBJ object)

  Summary:
    Deinitialize the specified instance of the AK7755 driver module

  Description:
    Deinitialize the specified instance of the AK7755 driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Precondition:
    Function DRV_AK7755_Initialize should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK7755_Initialize routine

  Returns:
    None.

  Remarks:
    Once the Initialize operation has been called, the De-initialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
*/
void DRV_AK7755_Deinitialize(SYS_MODULE_OBJ object)
{
    DRV_AK7755_OBJ * drvObj; 

    if (object == SYS_MODULE_OBJ_INVALID)
    {
        /* Invalid object */
        SYS_DEBUG(0, "Invalid object \r\n");
        return;
    }
    drvObj = (DRV_AK7755_OBJ *) object;

    if (false == drvObj->inUse)
    {
        /* Cannot deinitialize an object that is
         * not already in use. */
        SYS_DEBUG(0, "Instance not in use \r\n");
        return;
    }

    /* Deallocate all the mutexes */
     if ((OSAL_MUTEX_Delete(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE))
     {
        SYS_DEBUG(0, "Unable to delete client handle mutex \r\n");
        return;
     }

 
//    DRV_I2S_Close (drvObj->i2sDriverClientHandleWrite);
//
//    DRV_I2S_Close (drvObj->i2sDriverClientHandleRead);

    /* Indicate that this object is not is use */
    drvObj->inUse = false;

    /* Set number of clients to zero */
    drvObj->numClients = 0;
    drvObj->status = SYS_STATUS_UNINITIALIZED;
    
    return;

} //End DRV_AK7755_Deinitialize()


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_AK7755_Status( SYS_MODULE_OBJ object)

  Summary:
    Gets the current status of the AK7755 driver module.

  Description:
    This routine provides the current status of the AK7755 driver module.

  Precondition:
    Function DRV_AK7755_Initialize should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK7755_Initialize routine

  Returns:
    SYS_STATUS_DEINITIALIZED  - Indicates that the driver has been
                                de-initialized

    SYS_STATUS_READY          - Indicates that any previous module operation
                                for the specified module has completed

    SYS_STATUS_BUSY           - Indicates that a previous module operation for
                                the specified module has not yet completed

    SYS_STATUS_ERROR          - Indicates that the specified module is in an
                                error state

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
*/
SYS_STATUS DRV_AK7755_Status(SYS_MODULE_OBJ object)
{
    DRV_AK7755_OBJ * drvObj;

    if (object == SYS_MODULE_OBJ_INVALID)
    {
        /* Invalid object */
        SYS_DEBUG(0, "Invalid object \r\n");
        return SYS_STATUS_ERROR;
    }
    drvObj = (DRV_AK7755_OBJ *) object;
    
    /* Return the status of the driver object */
    return drvObj->status;

} /* DRV_AK7755_Status */


// *****************************************************************************
/* Function:
    void  DRV_AK7755_Tasks(SYS_MODULE_OBJ object);

  Summary:
    Maintains the driver's control and data interface state machine.

  Description:
    This routine is used to maintain the driver's internal control and data
    interface state machine and implement its control and data interface
    implementations.
    This function should be called from the SYS_Tasks() function.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK7755_Initialize)

  Returns:
    None.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks).

*/
void DRV_AK7755_Tasks(SYS_MODULE_OBJ object)
{

    DRV_AK7755_OBJ *drvObj;

    drvObj = (DRV_AK7755_OBJ *)object;

    if ((false == drvObj->inUse))
    {
        /* This intance of the driver is not initialized. Dont
         * do anything */
        return;
    }
    
    _DRV_AK7755_ControlTasks(drvObj);
}


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_AK7755_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT    ioIntent
    )

  Summary:
    Opens the specified AK7755 driver instance and returns a handle to it

  Description:
    This routine opens the specified AK7755 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options are not relevant to this driver. All the data transfer functions
    of this driver are non blocking.

    Only DRV_IO_INTENT_WRITE is a valid ioIntent option as AK7755 is DAC only.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    Function DRV_AK7755_Initialize must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    ioIntent    - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "or'd" together to indicate the intended use
                  of the driver. See function description for details.

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Error can occur
    - if the number of client objects allocated via DRV_AK7755_CLIENTS_NUMBER is insufficient.
    - if the client is trying to open the driver but driver has been opened exclusively by another client.
    - if the driver hardware instance being opened is not initialized or is invalid.
    - if the ioIntent options passed are not relevant to this driver.

  Remarks:
    The handle returned is valid until the DRV_AK7755_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
*/
DRV_HANDLE DRV_AK7755_Open(const SYS_MODULE_INDEX drvIndex, 
                           const DRV_IO_INTENT ioIntent
)
{
    DRV_AK7755_CLIENT_OBJ *hClient;
    DRV_AK7755_OBJ *drvObj;
    uint32_t iClient;

    /* The drvIndex value should be valid. It should be
     * less the number of driver object instances.
     */
    if (drvIndex >= DRV_AK7755_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Bad Driver Index \r\n");
        return DRV_HANDLE_INVALID;
    }
    drvObj = (DRV_AK7755_OBJ *)&gDrvak7755Obj[drvIndex];
    if (drvObj->status != SYS_STATUS_READY)
    {
        /* The AK7755  module should be ready */
        SYS_DEBUG(0, "Was the driver initialized? \r\n");
        return DRV_HANDLE_INVALID;
    }

    if ((drvObj->numClients > 0) && (true == drvObj->isExclusive))
    {
        /* Driver already opened in exclusive mode. Cannot open a new client. */
        SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
        return DRV_HANDLE_INVALID;
    }

    if ((drvObj->numClients > 0) &&
        (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        /*  A client Instance of driver is open.
            Cannot open the new client in exclusive mode */
            SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
            return DRV_HANDLE_INVALID;
    }

    iClient = 0;
    hClient = (DRV_AK7755_CLIENT_OBJ *)&gDrvak7755ClientObj[iClient];

    /* Grab client object mutex here */
    if (OSAL_MUTEX_Lock(&(gDrvak7755CommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Setup client operations */
        /* Find available slot in array of client objects */
        for (iClient = 0; iClient < DRV_AK7755_CLIENTS_NUMBER; iClient++)
        { 
            if (false == hClient->inUse)
            {
                /* Set the exlusive mode for the driver instance */
                if (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
                {
                    drvObj->isExclusive = true;
                }

                if (DRV_IO_INTENT_READWRITE == (ioIntent & DRV_IO_INTENT_READWRITE))
                {
                    hClient->ioIntent = DRV_IO_INTENT_READWRITE;
                    drvObj->i2sDriverHandle = DRV_I2S_Open(drvObj->i2sDriverModuleIndex,
                        (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING));
                }
                else if (DRV_IO_INTENT_WRITE == (ioIntent & DRV_IO_INTENT_READWRITE))
                {
                    hClient->ioIntent = DRV_IO_INTENT_WRITE;
                    drvObj->i2sDriverClientHandleWrite = DRV_I2S_Open(drvObj->i2sDriverModuleIndex,
                        (DRV_IO_INTENT_WRITE | DRV_IO_INTENT_NONBLOCKING));
                    drvObj->i2sDriverHandle = drvObj->i2sDriverClientHandleWrite;
                }
                else if (DRV_IO_INTENT_READ == (ioIntent & DRV_IO_INTENT_READWRITE))
                {
                    hClient->ioIntent = DRV_IO_INTENT_READ;
                    drvObj->i2sDriverClientHandleRead = DRV_I2S_Open(drvObj->i2sDriverModuleIndex,
                        (DRV_IO_INTENT_READ | DRV_IO_INTENT_NONBLOCKING));
                    drvObj->i2sDriverHandle = drvObj->i2sDriverClientHandleRead;
                }
                else
                {
                    SYS_DEBUG(0, "i2s DRV_I2S_Open Error");
                }

                DRV_I2S_TransmitErrorIgnore(drvObj->i2sDriverHandle, true);
                DRV_I2S_ReceiveErrorIgnore(drvObj->i2sDriverHandle, true);
                DRV_I2S_BaudSet(drvObj->i2sDriverHandle,
                            (drvObj->samplingRate * (DRV_AK7755_BCLK_BIT_CLK_DIVISOR)),
                            drvObj->samplingRate);
                hClient->ioIntent |= DRV_IO_INTENT_NONBLOCKING;

                

                /* Remember which AK7755 driver instance owns me */
                hClient->inUse  = true;
                hClient->hDriver = drvObj;
                hClient->pEventCallBack = NULL;
                //hClient->pReadEventCallBack = NULL;
                drvObj->numClients++;
                /* We have found a client object
                 * Release the mutex and return with
                 * the driver handle */
                /* An operation mode is needed */
                if ((OSAL_MUTEX_Unlock(&(gDrvak7755CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
                    return DRV_HANDLE_INVALID;
                }
                /* Return the client object */
                return (DRV_HANDLE) hClient;
            }
            hClient++;
        }
        /* Could not find a client object. Release the mutex and
         * return with an invalid handle. */
        if ((OSAL_MUTEX_Unlock(&(gDrvak7755CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
        }
    }
    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_AK7755_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the AK7755 driver

  Description:
    This routine closes an opened-instance of the AK7755 driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  A new handle must
    be obtained by calling DRV_AK7755_Open before the caller may use the driver
    again

  Remarks:

    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
*/
void DRV_AK7755_Close( const DRV_HANDLE handle)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    DRV_AK7755_OBJ *drvObj;

    if (handle == DRV_HANDLE_INVALID || 
        (DRV_HANDLE)NULL == handle)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    drvObj = (DRV_AK7755_OBJ *) clientObj->hDriver;

    if (DRV_IO_INTENT_READ == (clientObj->ioIntent & DRV_IO_INTENT_READWRITE))
    {
        DRV_I2S_Close(drvObj->i2sDriverClientHandleRead);
        DRV_I2S_Close(drvObj->i2sDriverClientHandleWrite);
    }
    else if (DRV_IO_INTENT_WRITE == (clientObj->ioIntent & DRV_IO_INTENT_READWRITE))
    {
        DRV_I2S_Close(drvObj->i2sDriverClientHandleWrite);

    }
    else if (DRV_IO_INTENT_READWRITE == (clientObj->ioIntent & DRV_IO_INTENT_READWRITE))
    {
        DRV_I2S_Close(drvObj->i2sDriverClientHandleRead);
        DRV_I2S_Close(drvObj->i2sDriverClientHandleWrite);
    }

    /* De-allocate the object */
    clientObj->inUse = false;
    /* Reduce the number of clients */
    drvObj->numClients--;
    return;
} /* DRV_AK7755_Close */


// *****************************************************************************
/*
Function:
    void DRV_AK7755_BufferAddWrite
    (
        const DRV_HANDLE handle,
        DRV_AK7755_BUFFER_HANDLE *bufferHandle,
        void *buffer, size_t size
    )

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK7755_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK7755_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK7755_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 device instance and the DRV_AK7755_Status must have returned
    SYS_STATUS_READY.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE ust have been specified in the DRV_AK7755_Open call.

  Parameters:
    handle       - Handle of the AK7755 instance as return by the
                   DRV_AK7755_Open function.
    buffer       - Data to be transmitted.
    size         - Buffer size in bytes.
    bufferHandle - Pointer to an argument that will contain the
                   return buffer handle.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK7755_BUFFER_HANDLE_INVALID if the function was not successful.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK7755 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK7755 driver instance. It should not otherwise be called directly in an ISR.

*/
void DRV_AK7755_BufferAddWrite(const DRV_HANDLE handle,
                               DRV_AK7755_BUFFER_HANDLE *bufferHandle,
                               void *buffer, size_t size
)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    DRV_AK7755_OBJ *drvObj;

    /* The Client and driver objects from the handle */
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj    = (DRV_AK7755_OBJ *) clientObj->hDriver;

    /* We first check the arguments and initialize the
     * buffer handle */
    if (bufferHandle != NULL)
    {
        *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
    }

    /* See if the handle is still valid */
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    /* Grab a mutex. (BLOCKING)  */
    if (OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE) 
    {
        ;
    } 
    else 
    {
        /* The mutex acquisition timed out. Return with an
         * invalid handle. This code will not execute
         * if there is no RTOS. */
        return;
    }

    //I2S driver
    {
        DRV_I2S_BUFFER_HANDLE i2sBufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
        DRV_I2S_BufferAddWrite(drvObj->i2sDriverClientHandleWrite, &i2sBufferHandle,
                                    (uint8_t *) buffer, size);

            if (i2sBufferHandle != DRV_I2S_BUFFER_HANDLE_INVALID)
            {
                *bufferHandle = (DRV_AK7755_BUFFER_HANDLE)i2sBufferHandle;
            }
            else
            {
                *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
            }
    }

    /* Release mutex (UNBLOCK) */
    if ((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG(0, "Unable to DriverInstance mutex \r\n");
    }
    return;
} /* DRV_AK7755_BufferAddWrite */


// *****************************************************************************
/*
Function:
    void DRV_AK7755_BufferAddRead
    (
        const DRV_HANDLE handle,
        DRV_AK7755_BUFFER_HANDLE *bufferHandle,
        void *buffer, size_t size
    )

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK7755_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK7755_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK7755_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 device instance and the DRV_AK7755_Status must have returned
    SYS_STATUS_READY.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE ust have been specified in the DRV_AK7755_Open call.

  Parameters:
    handle       - Handle of the AK7755 instance as return by the
                   DRV_AK7755_Open function.
    buffer       - Data to be transmitted.
    size         - Buffer size in bytes.
    bufferHandle - Pointer to an argument that will contain the
                   return buffer handle.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK7755_BUFFER_HANDLE_INVALID if the function was not successful.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK7755 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK7755 driver instance. It should not otherwise be called directly in an ISR.

*/
void DRV_AK7755_BufferAddRead(const DRV_HANDLE handle,
                              DRV_AK7755_BUFFER_HANDLE *bufferHandle,
                              void *buffer, size_t size
)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    
    DRV_AK7755_OBJ *drvObj;

    /* The Client and driver objects from the handle */
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *) clientObj->hDriver;

    /* We first check the arguments and initialize the
     * buffer handle */
    if (bufferHandle != NULL)
    {
        *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
    }

    /* See if the handle is still valid */
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    /* Grab a mutex. */
    //if (OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    //{
    //   ;
    //}
    //else
    //{
    //    /* The mutex acquisition timed out. Return with an
    //     * invalid handle. This code will not execute
    //     * if there is no RTOS. */
    //    return;
    //}

    //I2S
    {
        DRV_I2S_BUFFER_HANDLE i2sBufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
        DRV_I2S_BufferAddRead(drvObj->i2sDriverClientHandleRead, 
                              &i2sBufferHandle,
                              (uint8_t *) buffer, size);

            if (i2sBufferHandle != DRV_I2S_BUFFER_HANDLE_INVALID)
            {
                *bufferHandle = (DRV_AK7755_BUFFER_HANDLE)i2sBufferHandle;
            }
            else
            {
                *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
            }
    }
} /* DRV_AK7755_BufferAddRead */


// *****************************************************************************
/* Function:
    void DRV_AK7755_BufferAddWriteRead
        (
                const DRV_HANDLE handle,
                DRV_AK4642_BUFFER_HANDLE *bufferHandle,
                void *transmitBuffer,
                void *receiveBuffer,
                size_t size
        )

  Summary:
    Schedule a non-blocking driver write-read operation.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking write-read operation. The function
    returns with a valid buffer handle in the bufferHandle argument if the
    write-read request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. While the request is
    in the queue, the application buffer is owned by the driver and should not
    be modified. The function returns DRV_AK4642_BUFFER_EVENT_COMPLETE:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only or write only
    - if the buffer size is 0
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK4642_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK4642_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK4642 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK4642 driver instance. It should not otherwise be called directly in an ISR.

    This function is useful when there is valid read expected for every
    AK4642 write. The transmit and receive size must be same.

*/
void DRV_AK7755_BufferAddWriteRead(const DRV_HANDLE handle,
                                   DRV_AK7755_BUFFER_HANDLE    *bufferHandle,
                                   void *transmitBuffer, void *receiveBuffer,
                                   size_t size)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;

    DRV_AK7755_OBJ *drvObj;

    /* The Client and driver objects from the handle */
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *) clientObj->hDriver;

    /* We first check the arguments and initialize the
     * buffer handle */
    if (bufferHandle != NULL)
    {
        *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
    }


    /* See if the handle is still valid */
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;

    }

    /* Grab a mutex. (BLOCK) */
    if (OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        ;
    }
    else
    {
        /* The mutex acquisition timed out. Return with an
         * invalid handle. This code will not execute
         * if there is no RTOS. */
        return;
    }
    {

        DRV_I2S_BUFFER_HANDLE i2sBufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
        DRV_I2S_BufferAddWriteRead(drvObj->i2sDriverHandle, &i2sBufferHandle,
                                    (uint8_t *) transmitBuffer, (uint8_t *) receiveBuffer, size);

            if (i2sBufferHandle != DRV_I2S_BUFFER_HANDLE_INVALID)
            {
                *bufferHandle = (DRV_AK7755_BUFFER_HANDLE)i2sBufferHandle;
            }
            else
            {
                *bufferHandle = DRV_AK7755_BUFFER_HANDLE_INVALID;
            }
    }

    /* Release mutex (UNBLOCK) */
    if ((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG(0, "Unable to DriverInstance mutex \r\n");
    }

} /* DRV_AK4642_BufferAddWriteRead */

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_BufferEventHandlerSet
    (
        DRV_HANDLE handle,
        const DRV_AK7755_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t contextHandle
    )

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls DRV_AK7755_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    eventHandler - Pointer to the event handler function.
    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Remarks:
    If the client does not want to be notified when the queued buffer transfer
    has completed, it does not need to register a callback.
*/
void DRV_AK7755_BufferEventHandlerSet(DRV_HANDLE handle,
                                      const DRV_AK7755_BUFFER_EVENT_HANDLER eventHandler,
                                      const uintptr_t contextHandle)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    DRV_AK7755_OBJ *drvObj;

    if ((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }


    /* Assing the event handler and the context */
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    drvObj = clientObj->hDriver;
    /* Set the Event Handler and context */
    clientObj->pEventCallBack = eventHandler;
    clientObj->hClientArg = contextHandle;

    if ((clientObj->ioIntent & DRV_IO_INTENT_READWRITE) == DRV_IO_INTENT_READWRITE)
    {
        DRV_I2S_BufferEventHandlerSet(drvObj->i2sDriverHandle,
        (DRV_I2S_BUFFER_EVENT_HANDLER) _DRV_AK7755_I2SBufferEventHandler,
        (uintptr_t)(clientObj));
    }
    else if ((clientObj->ioIntent & DRV_IO_INTENT_READWRITE) == DRV_IO_INTENT_WRITE)
    {
        DRV_I2S_BufferEventHandlerSet(drvObj->i2sDriverClientHandleWrite,
        (DRV_I2S_BUFFER_EVENT_HANDLER) _DRV_AK7755_I2SBufferEventHandler,
        (uintptr_t)(clientObj));
    }
    else if ((clientObj->ioIntent & DRV_IO_INTENT_READWRITE) == DRV_IO_INTENT_READ)
    {
        DRV_I2S_BufferEventHandlerSet(drvObj->i2sDriverClientHandleRead,
        (DRV_I2S_BUFFER_EVENT_HANDLER) _DRV_AK7755_I2SBufferEventHandler,
        (uintptr_t)(clientObj));
    }

} //End DRV_AK7755_BufferEventHandlerSet()


void DRV_AK7755_I2SBufferHandlerSet(DRV_HANDLE handle,
                                    DRV_I2S_BUFFER_EVENT_HANDLER 
                                        I2SBufferEventHandler)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    DRV_AK7755_OBJ *drvObj;

    if ((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    /* Check the event handler and the context */
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    drvObj = clientObj->hDriver;

    DRV_I2S_BufferEventHandlerSet(drvObj->i2sDriverHandle,
                                  (DRV_I2S_BUFFER_EVENT_HANDLER) 
                                      I2SBufferEventHandler,
                                  (uintptr_t)(drvObj));

}  //End  DRV_AK7755_I2SBufferHandlerSet()


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Specific Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/*
  Function:
    void DRV_AK7755_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)

  Summary:
    This function sets the sampling rate of the media stream.

  Description:
    This function sets the media sampling rate for the client handle.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;

    _DRV_AK7755_MasterClockSet(samplingRate, drvObj->mclk_multiplier);

    DRV_I2S_BaudSet(drvObj->i2sDriverHandle,
                        (samplingRate*(DRV_AK7755_BCLK_BIT_CLK_DIVISOR)),
                        samplingRate);
                        
    drvObj->samplingRate = samplingRate;
   
    return;
}

// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK7755_SamplingRateGet(DRV_HANDLE handle)

  Summary:
    This function gets the sampling rate set on the Ak7755

  Description:
    This function gets the sampling rate set on the Ak7755.

  Remarks:
    None.
 */
uint32_t DRV_AK7755_SamplingRateGet(DRV_HANDLE handle)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;

    /* Return the sampling rate */
    return drvObj->samplingRate;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_VolumeSet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel, uint8_t volume)

  Summary:
    This function sets the volume for AK7755 CODEC.

  Description:
    This functions sets the volume value from 0-255 which can attenuate
    from -115dB to +12dB. All decibles below -50dB are inbaudible


  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_VolumeSet(DRV_HANDLE handle, 
                          DRV_AK7755_CHANNEL channel, 
                          uint8_t volume)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK7755_COMMAND_SEND;
    
    if (!drvObj->isVolumeSetUnderProcess)
    {
        AK7755_COMMAND *volumeSetCmd;
        volumeSetCmd = _DRV_AK7755_CommandQueueGetSlot();
        // no more free command slot
        if (volumeSetCmd == NULL)
        {
            return;
        }
        
        drvObj->isVolumeSetUnderProcess = true;
        if (DRV_AK7755_CHANNEL_LEFT == channel)
        {
            DRV_AK7755_VolumeReMapping( drvObj, channel, volume);

            volumeSetCmd->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT_ONLY;
            volumeSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET);
            volumeSetCmd->control_data[1] = (uint8_t)(drvObj->volume[DRV_AK7755_CHANNEL_LEFT]&0xFF);
            volumeSetCmd->array_size = 2;
        }
        else if (DRV_AK7755_CHANNEL_RIGHT == channel)
        {
            DRV_AK7755_VolumeReMapping( drvObj, channel, volume);

            volumeSetCmd->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_RIGHT_ONLY;
            volumeSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT19+AK7755_WRITE_OFFSET);
            volumeSetCmd->control_data[1] = (uint8_t)(drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]&0xFF);
            volumeSetCmd->array_size = 2;
        }
        else
        {
            AK7755_COMMAND *volumeSetCmdRight;
            volumeSetCmdRight = _DRV_AK7755_CommandQueueGetSlot();
            DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_LEFT, volume);
            DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_RIGHT, volume);
            
            volumeSetCmd->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT;
            volumeSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET);
            volumeSetCmd->control_data[1] = (uint8_t)(drvObj->volume[DRV_AK7755_CHANNEL_LEFT]&0xFF);
            volumeSetCmd->array_size = 2;
            
            if (volumeSetCmdRight != NULL){
                volumeSetCmdRight->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_RIGHT;
                volumeSetCmdRight->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT19+AK7755_WRITE_OFFSET);
                volumeSetCmdRight->control_data[1] = (uint8_t)(drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]&0xFF);
                volumeSetCmdRight->array_size = 2;
            }
        }
        drvObj->isVolumeSetUnderProcess = false;
    }
    else
    {
        ; // Volume set under process
    }


    return;
}

// *****************************************************************************
/*
  Function:
    uint8_t DRV_AK7755_VolumeGet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel)

  Summary:
    This function gets the volume for AK7755 CODEC.

  Description:
    This functions gets the current volume programmed to the DAC AK7755.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    

  Returns:
    None

  Example:
    <code>
    // myAppObj is an application specific object.
    MY_APP_OBJ myAppObj;
    uint8_t volume;

    // myAK7755Handle is the handle returned
    // by the DRV_AK7755_Open function.

      volume = DRV_AK7755_VolumeGet(myAK7755Handle, DRV_AK7755_CHANNEL_LEFT);
    </code>

  Remarks:
    None.
 */
uint8_t DRV_AK7755_VolumeGet(DRV_HANDLE handle, DRV_AK7755_CHANNEL channel)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;

    /* Return the volume */
    return drvObj->volume[channel];
}
// *****************************************************************************
/*
  Function:
    void DRV_AK7755_MuteOn(DRV_HANDLE handle);

  Summary:
    This function allows AK7755 output for soft mute on.

  Description:
    This function Enables AK7755 output for soft mute.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_MuteOn(DRV_HANDLE handle)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;
    
    // Mute on is executed already, no need to send a I2C command
    if ((drvObj->lastRegValue[AK7755_CONTROL_REG_CONT1A-0x40] & 0x20) >> 5 == 0x01){
        return;
    }
    drvObj->command = DRV_AK7755_COMMAND_SEND;
    
    AK7755_COMMAND *muteOnCmd;
    muteOnCmd = _DRV_AK7755_CommandQueueGetSlot();
    if (muteOnCmd == NULL)
    {
        return;
    }
    regValue = _DRV_AK7755_CONTROL_REG_BIT_WRITE_Wrapper(drvObj,
               AK7755_CONTROL_REG_CONT1A+AK7755_WRITE_OFFSET,
               5,
               0x1
               );
    
    muteOnCmd->command = DRV_AK7755_COMMAND_MUTE_ON;
    muteOnCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT1A+AK7755_WRITE_OFFSET);
    muteOnCmd->control_data[1] = (uint8_t)(regValue&0xFF);
    muteOnCmd->array_size = 2;
    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_IntExtMicSet(DRV_HANDLE handle);

  Summary:
    This function sets up the codec for the internal or the external microphone use.

  Description:
    This function sets up the codec for the internal or the external microphone use.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    micInput     - Internal vs External mic input
  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_IntExtMicSet(DRV_HANDLE handle, DRV_AK7755_INT_EXT_MIC micInput)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;
    

    AK7755_COMMAND *micSetCmd;
    micSetCmd = _DRV_AK7755_CommandQueueGetSlot();
    if (micSetCmd == NULL)
    {
        return;
    }
    drvObj->command = DRV_AK7755_COMMAND_SEND;
    switch (micInput)
    {
        case INT_MIC :
            if (drvObj->lastRegValue[AK7755_CONTROL_REG_CONT0F-0x40] == (DRV_AK7755_CONTROL_REG_FIELD_WRITE(AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET, 0x20, 5, 1)))
            {
                return;
            }
            regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                        AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET,
                        0x20,
                        5, 
                        1);
            micSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET);
        break;
        case EXT_MIC:
            if (drvObj->lastRegValue[AK7755_CONTROL_REG_CONT0E-0x40] == (DRV_AK7755_CONTROL_REG_FIELD_WRITE(AK7755_CONTROL_REG_CONT0E+AK7755_WRITE_OFFSET, 0xC0, 6, 3)))
            {
                return;
            }
            regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                        AK7755_CONTROL_REG_CONT0E+AK7755_WRITE_OFFSET,
                        0xC0,
                        6, 
                        3);
            micSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT0E+AK7755_WRITE_OFFSET);
        break;
        default:
            if (drvObj->lastRegValue[AK7755_CONTROL_REG_CONT0F-0x40] == (DRV_AK7755_CONTROL_REG_FIELD_WRITE(AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET, 0x20, 5, 1)))
            {
                return;
            }
            regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                        AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET,
                        0x20,
                        5, 
                        1);
            micSetCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET);
        break;

    }
    
   
    micSetCmd->command = DRV_AK7755_COMMAND_INT_EXT_MIC_SET;
   
    micSetCmd->control_data[1] = (uint8_t)(regValue&0xFF);
    micSetCmd->array_size = 2;
     return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_MonoStereoMicSet(DRV_HANDLE handle);

  Summary:
    This function sets up the codec for the Mono or Stereo microphone mode.

  Description:
    This function sets up the codec for the Mono or Stereo microphone mode.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
//                   open routine

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_MonoStereoMicSet(DRV_HANDLE handle, DRV_AK7755_MONO_STEREO_MIC mono_stereo_mic)
{

    switch(mono_stereo_mic){
        case STEREO:
            DRV_AK7755_IntExtMicSet(handle, EXT_MIC);
            break;
        case MONO_LEFT_CHANNEL:
        case MONO_RIGHT_CHANNEL:
        default:
            DRV_AK7755_IntExtMicSet(handle, INT_MIC);
            break;
    }
    return;
}
// *****************************************************************************
/*
  Function:
    void DRV_AK7755_MuteOff(DRV_HANDLE handle)

  Summary:
    This function disables AK7755 output for soft mute.

  Description:
    This function disables AK7755 output for soft mute.

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_MuteOff(DRV_HANDLE handle)
{
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK7755_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK7755_OBJ *)clientObj->hDriver;
    
    // Mute off is executed already, no need to send a I2C command
    if ((drvObj->lastRegValue[AK7755_CONTROL_REG_CONT1A-0x40] & 0x20) >> 5 == 0x00){
        return;
    }
    drvObj->command = DRV_AK7755_COMMAND_SEND;

    regValue = _DRV_AK7755_CONTROL_REG_BIT_WRITE_Wrapper(drvObj,
               AK7755_CONTROL_REG_CONT1A+AK7755_WRITE_OFFSET,
               5,
               0x0
               );
    
    AK7755_COMMAND *muteOffCmd;
    muteOffCmd = _DRV_AK7755_CommandQueueGetSlot();
    if (muteOffCmd == NULL)
    {
        return;
    }
    
    muteOffCmd->command = DRV_AK7755_COMMAND_MUTE_OFF;
    muteOffCmd->control_data[0] = (uint8_t)(AK7755_CONTROL_REG_CONT1A+AK7755_WRITE_OFFSET);
    muteOffCmd->control_data[1] = (uint8_t)(regValue&0xFF);
    muteOffCmd->array_size = 2;

    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK7755_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl
)

  Summary:
    This function provides a run time audio format configuration

  Description:
    This function sets up audio mode in I2S protocol

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    dl           - Data length for I2S audio interface
    sl           - Left/Right Sample Length for I2S audio interface
  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK7755_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl)
{
    
    DRV_AK7755_OBJ *drvObj;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }
    
    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    
    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;

    // initialize with a mostly used one
    SPI_AUDIO_COMMUNICATION_WIDTH spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL;
    if(sl == SAMPLE_LENGTH_32)
    {
        switch(dl)
        {
            case DATA_LENGTH_16:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL;
                break;
            case DATA_LENGTH_24:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_24DATA_32FIFO_32CHANNEL;
                break;
            case DATA_LENGTH_32:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_32DATA_32FIFO_32CHANNEL;
                break;
            default:
                 // should never reach this branch
                break;
        };
    }else
    {
        // no mater what dl is, the mode can only be
        spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_16CHANNEL;
    }
    
    DRV_I2S_SetAudioCommunicationMode(drvObj->i2sDriverHandle, spi_audio_mode);
}


// *****************************************************************************
/*
  Function:
    void DRV_AK7755_CommandEventHandlerSet
    (
        DRV_HANDLE handle,
        const DRV_AK7755_COMMAND_EVENT_HANDLER eventHandler,
        const uintptr_t contextHandle
    )

  Summary:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

    When a client calls DRV_AK7755_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "AK7755 CODEC
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_AK7755_Initialize routine must have been called for the specified
    AK7755 driver instance.

    DRV_AK7755_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    eventHandler - Pointer to the event handler function.
    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Remarks:
    If the client does not want to be notified when the command
    has completed, it does not need to register a callback.
*/
void DRV_AK7755_CommandEventHandlerSet
(
    DRV_HANDLE handle,
    const DRV_AK7755_COMMAND_EVENT_HANDLER eventHandler,
    const uintptr_t contextHandle
)
{
    DRV_AK7755_CLIENT_OBJ *clientObj;
    DRV_AK7755_OBJ *drvObj;

    if ((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }


    clientObj = (DRV_AK7755_CLIENT_OBJ *) handle;
    /* Assing the event handler and the context */
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }

    drvObj = (DRV_AK7755_OBJ *)clientObj->hDriver;
    drvObj->commandCompleteCallback = eventHandler;
    drvObj->commandContextData = contextHandle;
    return;
}


// *****************************************************************************
// *****************************************************************************
// Section: AK7755 CODEC Version Information Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/*
  Function:
    int8_t* DRV_AK7755_VersionStrGet(void)

  Summary:
    This function returns the version of AK7755 driver in string format.

  Description:
    The DRV_AK7755_VersionStrGet function returns a string in the format:
    "<major>.<minor>[.<patch>][<type>]"
    Where:
        <major> is the AK7755 driver's version number.
        <minor> is the AK7755 driver's version number.
        <patch> is an optional "patch" or "dot" release number (which is not
        included in the string if it equals "00").
        <type> is an optional release type ("a" for alpha, "b" for beta ?
        not the entire word spelled out) that is not included if the release
        is a production version (I.e. Not an alpha or beta).

        The String does not contain any spaces.

        Example:
        "0.03a"
        "1.00"

  Precondition:
    None

  Parameters:
    None

  Returns: returns a string containing the version of AK7755 driver.

  Example:
    <code>
        int8_t *AK7755string;
        AK7755string = DRV_AK7755_VersionStrGet();
    </code>

  Remarks:
    None
 */
int8_t* DRV_AK7755_VersionStrGet(void)
{
    return (int8_t*) _DRV_AK7755_VERSION_STR;
}


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK7755_VersionGet( void )

  Summary:
    This function returns the version of AK7755 driver

  Description:
    The version number returned from the DRV_AK7755_VersionGet function is an
    unsigned integer in the following decimal format.
    <major> * 10000 + <minor> * 100 + <patch>

    Where the numbers are represented in decimal and the meaning is the same as
    above.  Note that there is no numerical representation of release type.

    Example:
    For version "0.03a", return:  0 * 10000 + 3 * 100 + 0
    For version "1.00", return:  1 * 100000 + 0 * 100 + 0


  Precondition:
    None

  Parameters:
    None

  Returns: returns the version of AK7755 driver.

  Example:
    <code>
        uint32_t AK7755version;
        AK7755version = DRV_AK7755_VersionGet();
    </code>

  Remarks:
    None
 */
uint32_t DRV_AK7755_VersionGet(void)
{
    return (_DRV_AK7755_VERSION_MAJOR * 10000 +  \
            _DRV_AK7755_VERSION_MINOR * 100 + \
            _DRV_AK7755_VERSION_PATCH);
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
/* Function:
    static void DRV_AK7755_VolumeReMapping( DRV_AK7755_OBJ* drvObj, DRV_AK7755_CHANNEL channel,uint8_t volume)

  Summary:
    Volume remapping to reverse the codec volume value to dB mapping which currently works reverse

  Description:
    Volume remapping to reverse the codec volume value to dB mapping which currently works reverse
 The ak7755 codec has DAC value to volume range mapping as :-
 00 H : +12dB
 FF H : -115dB
 In order to provide to the user a more intuitive DAC value to dB correspondance, this remapping fucntion reverses the volume value to dB mapping
 so that 00 H : -115 dB
         FF H : +12 dB
  Precondition:
    DRV_AK7755_OBJ - driver object should be available

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_AK7755_Initialize routine
    volume          - 0 - 255 value supported by the codec for volume adjustment

  Returns:
    None.
  Remarks:
    Note that the allowed range of the codec stretches from +12dB to -115 dB.
    However for most applications, the entire dB range is not audible.
    It might be obersved that values 80 and below for the volume input might not have audible dB values.

*/
static void DRV_AK7755_VolumeReMapping( DRV_AK7755_OBJ* drvObj, DRV_AK7755_CHANNEL channel, uint8_t volume)
{
    uint8_t gainValue;
    uint8_t gainDACBits;

    gainValue = ( ((MAX_VOLUME_CODE-MIN_VOLUME_CODE)*((int16_t)volume) )>>8 ) + MIN_VOLUME_CODE;
    gainDACBits = 0xFF - gainValue; // 0x00 = max gain, 0xFF = mute

    int volCtrl = gainDACBits;
    drvObj->volume[channel] = volCtrl;
}/*DRV_AK7755_VolumeReMapping*/

// *****************************************************************************
 /*
  Function:
    static void _DRV_AK7755_MasterClockSet(uint32_t samplingRate)

  Summary:
    Generates the master clock(to AK7755) from REFCLOCK  for the given
    sampling rate.

  Description:
    Generates the master clock(to AK7755) from REFCLOCK  for the given
    sampling rate.

  Remarks:
    None
*/
static void _DRV_AK7755_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier)
{
uint32_t mclkInHertz, achievedFrequencyHz;

    /* 256fs MCLK is 256 times sampling rate */
    mclkInHertz = mclk_multiplier*samplingRate;
    achievedFrequencyHz = SYS_CLK_ReferenceFrequencySet(
                                CLK_BUS_REFERENCE_1,
                                DRV_AK7755_INPUT_REFCLOCK,
                                mclkInHertz, true );
    if (achievedFrequencyHz == 0)
    {
        SYS_DEBUG(0, "Frequency not set properly. check what is the problem \r\n");
    }
    

    return;

}

// *****************************************************************************

/*  Function:
        static void _DRV_AK7755_ConrolRegisterSet
        (
            DRV_AK7755_OBJ *drvObj,
            DRV_AK7755_CONTROL_REGISTER contRegister,
            uint8_t value
        )

  Summary:
    Writes a byte to AK7755 register .
 
  Description:
    Writes a byte to AK7755 register .

  Remarks:
    None
*/
static uintptr_t _DRV_AK7755_ConrolRegisterSet
(
    DRV_AK7755_OBJ *drvObj,
    uint8_t *controlData,
    uint32_t size
)
{   
    uintptr_t drvI2CBuffHandle = DRV_I2C_Transmit( drvObj->drvI2CMasterHandle,
                                                    AK7755_I2C_ADDR,
                                                    (I2C_DATA_TYPE *)controlData, 
                                                    size, 
                                                    NULL);
    drvObj->controlCommandStatus = false;
    return drvI2CBuffHandle;
}



// *****************************************************************************
 /*
  Function:
    static void _DRV_AK7755_ControlTasks(DRV_AK7755_OBJ *drvObj)

  Summary:
    Implements the state maching for the Audio control interface of AK7755

  Description:
    Implements the state maching for the Audio control interface of AK7755

  Remarks:
    None
*/


static uint32_t mTimerStart, mTimerEnd;
bool readTimerStart = true;

static void _DRV_AK7755_ControlTasks(DRV_AK7755_OBJ *drvObj)
{
    volatile uint32_t regValue;
    CONT0F_REGISTER regCONT0F;

    uint16_t nData;
    bool skipCheck = false;
   
    switch (drvObj->command)
    {
        case DRV_AK7755_COMMAND_NONE:
        {
            /* Do nothing. No Control Command executed */
            
            ;
        }
        break;

        case DRV_AK7755_COMMAND_INIT_CLK_PDN_SET:
        {
             /* Generate master clock from REFCLOCK for the given sampling rate */
             _DRV_AK7755_MasterClockSet(drvObj->samplingRate, drvObj->mclk_multiplier);
             
            /* Initiate AK7755 Command */
             
            if (BSP_AK7755_PDNStateGet() == 1)
            {
                //do nothing
            }else{
                /* Reset Values for Control interface  and power down pins */
                BSP_AK7755_PDNOff();
                /* Bring power down out of reset (Set PDN High) */
                BSP_AK7755_PDNOn();
            }
     
            // Control register must be made after 1ms from the PDN pin="H"
            
            regcur = 0;
            control_register = AK7755_CONTROL_REG_CONT01 + AK7755_WRITE_OFFSET;
            drvObj->command = DRV_AK7755_COMMAND_INIT_COUNTER;
            
//            command_array[0] = AK7755_CONTROL_REG_DEVNO+AK7755_READ_OFFSET;
//            drvObj->drvI2CBuffHandle = DRV_I2C_TransmitThenReceive(drvObj->drvI2CMasterHandle, 
//                                                        AK7755_I2C_ADDR,
//                                                        command_array,
//                                                        1,
//                                                        read,
//                                                        1,
//                                                        NULL);

        }
        break;

        case DRV_AK7755_COMMAND_INIT_COUNTER:
        {
            if (readTimerStart){
                asm volatile("mtc0   $0,$9");
                asm volatile("mfc0   %0, $9" : "=r"(mTimerStart));
                readTimerStart = false;
            }
            
            asm volatile("mfc0   %0, $9" : "=r"(mTimerEnd));
            //wait 1ms
            if (mTimerEnd - mTimerStart > SYS_CLK_FREQ/1000){
                drvObj->command = DRV_AK7755_COMMAND_INIT_CLOCK_MODE;
            }
        }
        break;
        case DRV_AK7755_COMMAND_INIT_CLOCK_MODE:
        {
            drvObj->command = DRV_AK7755_COMMAND_INIT_START;
            regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT00 + AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            ak7755_ControlRegisters.cont00.value);

            command_array[0] = AK7755_CONTROL_REG_CONT00 + AK7755_WRITE_OFFSET;
            command_array[1] = (uint8_t)(regValue&0xFF);
            drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet(drvObj, command_array, 2);
        }
        break;
       
        case DRV_AK7755_COMMAND_INIT_START:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                
                // Init registers
                // I2C interface now seems to work...           
                // Execute register initialization contained in _AK7755_ControlRegInits ******************
                
                if (regcur == sizeof(initRegValues)-1){
                    drvObj->command = DRV_AK7755_COMMAND_DSP_READY;
                }
                
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            control_register,
                            0xFF,
                            0,
                            initRegValues[regcur]
                            );
                command_array[0] = control_register;
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet(drvObj, command_array, 2);
                regcur++;
                control_register++;
            }
            else
            {
                /* Do Nothing. Remain in this state until
                 * the INIT_START command is transferred successfully */
                ;
            }

        }
        break;
        case DRV_AK7755_COMMAND_DSP_READY:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                
                drvObj->command = DRV_AK7755_COMMAND_CLOCK_RELEASE_RESET;
             // Program DSP PRAM *******************************************************
                regCONT0F.value = 0;
                regCONT0F.bits.PMLI  = 1;
                regCONT0F.bits.DLRDY = 1; // Set DSP Download Ready

                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT0F + AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            regCONT0F.value
                            );
                
                command_array[0] = AK7755_CONTROL_REG_CONT0F + AK7755_WRITE_OFFSET;
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 2);
            }
        }
        break;
        case DRV_AK7755_COMMAND_CLOCK_RELEASE_RESET:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                
                drvObj->command = DRV_AK7755_COMMAND_PROGRAM_DSP;
                ak7755_ControlRegisters.cont01.value = 0;
                ak7755_ControlRegisters.cont01.bits.CKRESETN = 1;
                
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT01 + AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            ak7755_ControlRegisters.cont01.value
                            );
                command_array[0] = AK7755_CONTROL_REG_CONT01 + AK7755_WRITE_OFFSET;
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 2);
                // need wait at least 10ms 
            }
        }
        break;
        case DRV_AK7755_COMMAND_PROGRAM_DSP:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_PROGRAM_CRAM;
                
                
                // Now write values to PRAM
                
                if (drvObj->samplingRate <= 16000)
                {
                    nData = ( sizeof _AK7755_PRAM_EC ) / sizeof(uint8_t);
                    drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, (uint8_t *)_AK7755_PRAM_EC, nData);
//                    _DRV_AK7755_I2CDataWrite(drvObj, 0xB8,addrMem,(uint8_t *)_AK7755_PRAM_EC,nData);
                }
                else
                {
                    nData = ( sizeof _AK7755_PRAM ) / sizeof(uint8_t);
                    
                    drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, (uint8_t *)_AK7755_PRAM, nData);
//                    _DRV_AK7755_I2CDataWrite(drvObj,0xB8,addrMem,(uint8_t *)_AK7755_PRAM,nData);
                }

                
//                ak7755_ControlRegisters.cont01.bits.CKRESETN = 1;
//                _DRV_AK7755_ControlRegisterSet( drvObj, AK7755_CONTROL_REG_CONT01 + AK7755_WRITE_OFFSET, ak7755_ControlRegisters.cont01.value);
//                regValue = _DRV_AK7755_ControlRegisterGet(drvObj, AK7755_CONTROL_REG_CONT01 + AK7755_READ_OFFSET);
//                assert(regValue == ak7755_ControlRegisters.cont01.value);
            }
        }
        break;
        case DRV_AK7755_COMMAND_PROGRAM_CRAM:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                skipCheck = false;
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_RESET_REGISTERS;
                if (drvObj->samplingRate <= 16000)
                 {
                    if ( NULL != _AK7755_CRAM_EC ) //If non-trivial coefficient RAM
                    {
                        nData = ( sizeof _AK7755_CRAM_EC ) / sizeof(uint8_t);
                        drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, (uint8_t *)_AK7755_CRAM_EC, nData);
//                        _DRV_AK7755_I2CDataWrite(drvObj,0xB4,addrMem,(uint8_t *)_AK7755_CRAM_EC,nData);
                    }
                 }
                 // Program Coefficient RAM ***********************************************
                else{
                    if ( NULL != _AK7755_CRAM ) //If non-trivial coefficient RAM
                    {
                        nData = ( sizeof _AK7755_CRAM ) / sizeof(uint8_t);
                        drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, (uint8_t *)_AK7755_CRAM, nData);
//                        _DRV_AK7755_I2CDataWrite(drvObj,0xB4,addrMem,(uint8_t *)_AK7755_CRAM,nData);
                    }else{
                        skipCheck = true;
                        drvObj->controlCommandStatus = false;
                    }
                 }
            }
        }
        break;
        case DRV_AK7755_COMMAND_RESET_REGISTERS:
        {
            if ((skipCheck || DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE)
                && false == drvObj->controlCommandStatus)
            {
                
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT;
                
                regCONT0F.value = 0;
                regCONT0F.bits.PMLI  = 1;
                regCONT0F.bits.CRESETN = 1; 
                regCONT0F.bits.DSPRESETN = 1;
                
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT0F+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            regCONT0F.value
                            );  
                command_array[0] =AK7755_CONTROL_REG_CONT0F+ AK7755_WRITE_OFFSET;
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 2);
            }
        }
        break;
        
        case DRV_AK7755_COMMAND_INIT_AUDIO_FORMAT:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT;
                
                // DAC Input Format
                ak7755_ControlRegisters.cont06.bits.DIFDA = DRV_AK7755_DAC_INPUT_FORMAT_MACRO;
                
                // DSP DIN1 Format
                ak7755_ControlRegisters.cont06.bits.DIF1 = DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO;

                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT06+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            ak7755_ControlRegisters.cont06.value
                            );
                command_array[0] = (uint8_t)(AK7755_CONTROL_REG_CONT06+AK7755_WRITE_OFFSET);
                command_array[1] = (uint8_t)(regValue&0xFF);
                
                
                ak7755_ControlRegisters.cont07.bits.DOF4 = DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO;
                
                
#if DRV_AK7755_ENABLE_MICROPHONE
                ak7755_ControlRegisters.cont07.bits.DOF1 = DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO;
                
#endif

                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT07+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            ak7755_ControlRegisters.cont07.value
                            );
               command_array[2] = (uint8_t)(regValue&0xFF);
                
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 3);


            }
            else
            {
                /* Do Nothing. Remain in this state until
                 * the INIT_SAMPLING_RATE command is transferred successfully */
                ;
            }
        }
        break;
        
        case DRV_AK7755_COMMAND_VOLUME_SET_CHANNELS_INIT:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_INIT_END;
                DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_LEFT, drvObj->volume[DRV_AK7755_CHANNEL_LEFT]);
                DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_RIGHT, drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]);
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            drvObj->volume[DRV_AK7755_CHANNEL_LEFT]
                            );
                
                command_array[0] = (uint8_t)(AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET);
                command_array[1] = (uint8_t)(regValue&0xFF);
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT19+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]
                            );
                command_array[2] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 3);
            }
        }
        break;

        case DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_RIGHT;
                DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_LEFT, drvObj->volume[DRV_AK7755_CHANNEL_LEFT]);
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            drvObj->volume[DRV_AK7755_CHANNEL_LEFT]
                            );
                command_array[0] = (uint8_t)(AK7755_CONTROL_REG_CONT18+AK7755_WRITE_OFFSET);
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 2);

            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;
        
        case DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_RIGHT:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_INIT_END;
                DRV_AK7755_VolumeReMapping( drvObj, DRV_AK7755_CHANNEL_RIGHT, drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]);
                regValue = _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(drvObj,
                            AK7755_CONTROL_REG_CONT19+AK7755_WRITE_OFFSET,
                            0xFF,
                            0,
                            drvObj->volume[DRV_AK7755_CHANNEL_RIGHT]
                            );
                
                command_array[0] = (uint8_t)(AK7755_CONTROL_REG_CONT19+AK7755_WRITE_OFFSET);
                command_array[1] = (uint8_t)(regValue&0xFF);
                drvObj->drvI2CBuffHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, command_array, 2);
            }
            
        }
        break;
        case DRV_AK7755_COMMAND_INIT_END:
        {
            if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, drvObj->drvI2CBuffHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE 
                    && false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_NONE;
                drvObj->status = SYS_STATUS_READY;
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;
        case DRV_AK7755_COMMAND_SEND:
        {
            AK7755_COMMAND* cmd = _DRV_AK7755_CommandQueueTop();
            // Command Queue is Empty, no need to send control command
            if (cmd == NULL){
                drvObj->command = DRV_AK7755_COMMAND_NONE;
            }else{
                cmd->drvI2CBufHandle = _DRV_AK7755_ConrolRegisterSet( drvObj, cmd->control_data, cmd->array_size);
                drvObj->command = DRV_AK7755_COMMAND_COMPLETE;
            }
        }
        break;
        case DRV_AK7755_COMMAND_COMPLETE:
        {
            AK7755_COMMAND* cmd = _DRV_AK7755_CommandQueueTop();
            // safety check, cmd should never be NULL
            if (cmd == NULL){
                drvObj->command = DRV_AK7755_COMMAND_NONE;
            }else{
                if (DRV_I2C_TransferStatusGet(drvObj->drvI2CMasterHandle, cmd->drvI2CBufHandle) == DRV_I2C_BUFFER_EVENT_COMPLETE){
                    drvObj->command = DRV_AK7755_COMMAND_SEND;
                    // Release this command slot
                    _DRV_AK7755_CommandQueuePop();
                }
                /*
                else{
                    // otherwise continously check if this command completes
                }*/
            }
        }
        break;

        case DRV_AK7755_COMMAND_SAMPLING_RATE_SET:
        case DRV_AK7755_COMMAND_MUTE_ON:
        case DRV_AK7755_COMMAND_MUTE_OFF:
        case DRV_AK7755_COMMAND_DIGITAL_BLOCK_CONTROL_SET:
        case DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_LEFT_ONLY:
        case DRV_AK7755_COMMAND_VOLUME_SET_CHANNEL_RIGHT_ONLY:
        case DRV_AK7755_COMMAND_INT_EXT_MIC_SET:
        case DRV_AK7755_COMMAND_MONO_STEREO_MIC_SET:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK7755_COMMAND_NONE;
                
                if (drvObj->commandCompleteCallback != (DRV_AK7755_COMMAND_EVENT_HANDLER) 0)
                {
                    drvObj->commandCompleteCallback(drvObj->commandContextData);
                }
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the command is transferred successfully */
                ;
            }
        }
        break;
    }
    return;
}


/// *****************************************************************************
 /*
  Function:
        static void _DRV_AK7755_I2SBufferEventHandler
        (
            DRV_I2S_BUFFER_EVENT event,
            DRV_I2S_BUFFER_HANDLE bufferHandle,
            uintptr_t contextHandle
        )

  Summary:
    Implements the handler for i2s buffer request completion.

  Description:
    Implements the handler for i2s buffer request completion.

  Remarks:
    None
*/

static void _DRV_AK7755_I2SBufferEventHandler
(
    DRV_I2S_BUFFER_EVENT event,
    DRV_I2S_BUFFER_HANDLE bufferHandle,
    uintptr_t contextHandle
)
{
    //DRV_AK7755_OBJ *drvObj;
    //DRV_AK7755_BUFFER_OBJECT *bufObject;
    DRV_AK7755_CLIENT_OBJ *clientObj;

    if (DRV_I2S_BUFFER_HANDLE_INVALID == bufferHandle || 
            0 == contextHandle )
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    clientObj = (DRV_AK7755_CLIENT_OBJ *)contextHandle;
    if (DRV_I2S_BUFFER_EVENT_COMPLETE == event)
    {
        clientObj->pEventCallBack(DRV_AK7755_BUFFER_EVENT_COMPLETE,
            (DRV_AK7755_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else if (DRV_I2S_BUFFER_EVENT_ABORT == event)
    {
        clientObj->pEventCallBack(DRV_AK7755_BUFFER_EVENT_ABORT,
            (DRV_AK7755_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else if (DRV_I2S_BUFFER_EVENT_ERROR == event)
    {
        clientObj->pEventCallBack(DRV_AK7755_BUFFER_EVENT_ERROR,
            (DRV_AK7755_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else
    {
        ;
    }
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: AK7755 Command Queue Implementations
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
 /*
  Function:
        static void _DRV_AK7755_CommandQueueGetSlot
        (
        )

  Summary:
    Get a free slot from AK7755 command queue.

  Description:
    Get a free slot from AK7755 command queue.

  Remarks:
    None
*/
static AK7755_COMMAND* _DRV_AK7755_CommandQueueGetSlot(){
    
    if (gDrvCommandBuffer.status == 1)
    {
        return NULL;
    }
    AK7755_COMMAND* ret = NULL;
    ret = &gDrvCommandBuffer.commandBuffer[gDrvCommandBuffer.queueIn];
    gDrvCommandBuffer.queueIn++;
    if (gDrvCommandBuffer.queueIn == AK7755_COMMAND_QUEUE_BUFFER){
        gDrvCommandBuffer.queueIn = 0;
    }
    if (gDrvCommandBuffer.queueIn == gDrvCommandBuffer.queueOut)
    {
        // Buffer is Full
        gDrvCommandBuffer.status = 1;
    }else{
        gDrvCommandBuffer.status = 2;
    }
    return ret;
}
 /*
  Function:
        static void _DRV_AK7755_CommandQueuePop
        (
        )

  Summary:
    Pop up the top slot of AK7755 command queue.

  Description:
    Pop up the top slot of AK7755 command queue.

  Remarks:
    None
*/
static AK7755_COMMAND* _DRV_AK7755_CommandQueuePop(){
    if (gDrvCommandBuffer.status == 0)
    {
        return NULL;
    }
    AK7755_COMMAND *ret = &gDrvCommandBuffer.commandBuffer[gDrvCommandBuffer.queueOut];
    gDrvCommandBuffer.queueOut++;
    if (gDrvCommandBuffer.queueOut == AK7755_COMMAND_QUEUE_BUFFER){
        gDrvCommandBuffer.queueOut = 0;
    }
    if (gDrvCommandBuffer.queueOut == gDrvCommandBuffer.queueIn){
        gDrvCommandBuffer.status = 0;
    }else{
        gDrvCommandBuffer.status = 2;
    }
    return ret;
}
 /*
  Function:
        static void _DRV_AK7755_CommandQueueTop
        (
        )

  Summary:
    Return the top slot of AK7755 command queue.

  Description:
    Return the top slot of AK7755 command queue.

  Remarks:
    None
*/
static AK7755_COMMAND* _DRV_AK7755_CommandQueueTop(){
    if (gDrvCommandBuffer.status == 0)
    {
        return NULL;
    }
    AK7755_COMMAND *ret = &gDrvCommandBuffer.commandBuffer[gDrvCommandBuffer.queueOut];
    
    return ret;
}

// *****************************************************************************

 /*
  Function:
        static uint8_t _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper
        (
            DRV_AK7755_OBJ *drvObj, 
            uint8_t reg_addr, 
            uint8_t mask, 
            uint8_t pos, 
            uint8_t val
        )

  Summary:
    A wrapper function of DRV_AK7755_CONTROL_REG_FIELD_WRITE macro.

  Description:
    A wrapper function of DRV_AK7755_CONTROL_REG_FIELD_WRITE macro.

  Remarks:
    None
*/
static uint8_t _DRV_AK7755_CONTROL_REG_FIELD_WRITE_Wrapper(DRV_AK7755_OBJ *drvObj, uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t val){
    uint8_t regValue; 
    regValue = DRV_AK7755_CONTROL_REG_FIELD_WRITE(
                                drvObj->lastRegValue[reg_addr-0xC0],
                                mask,
                                pos,
                                val);
    
    drvObj->lastRegValue[reg_addr-0xC0] = regValue;
    return regValue;
}
// *****************************************************************************
 /*
  Function:
        static uint8_t _DRV_AK7755_CONTROL_REG_BIT_WRITE_Wrapper
        (
            DRV_AK7755_OBJ *drvObj, 
            uint8_t reg_addr, 
            uint8_t pos, 
            uint8_t val
        )

  Summary:
    A wrapper function of DRV_AK7755_CONTROL_REG_BIT_WRITE macro.

  Description:
    A wrapper function of DRV_AK7755_CONTROL_REG_BIT_WRITE macro.

  Remarks:
    None
*/
static uint8_t _DRV_AK7755_CONTROL_REG_BIT_WRITE_Wrapper(DRV_AK7755_OBJ *drvObj, uint8_t reg_addr, uint8_t pos, uint8_t val)
{
    uint8_t regValue;
    regValue = DRV_AK7755_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[reg_addr-0xC0],
                    pos,
                    val);
    drvObj->lastRegValue[reg_addr-0xC0] = regValue;
    return regValue;
}


/*******************************************************************************
 End of File
*/

