/*******************************************************************************
  AK4384 CODEC Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak4384_local.h

  Summary:
    AK4384 CODEC driver local declarations and definitions

  Description:
    This file contains the AK4384 CODEC driver's local declarations and definitions.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_AK4384_LOCAL_H
#define _DRV_AK4384_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "osal/osal.h"
#include "driver/codec/ak4384/drv_ak4384.h"
#include "driver/tmr/drv_tmr.h"


// *****************************************************************************
// *****************************************************************************
// Section: Constants and Macros
// *****************************************************************************
// *****************************************************************************
/* AK4384 Driver Version Macros

  Summary:
    AK4384 driver version.

  Description:
    These constants provide AK4384 driver version information. The driver
    version is:
    DRV_AK4384_VERSION_MAJOR.DRV_AK4384_VERSION_MINOR[.<DRV_AK4384_VERSION_PATCH>][DRV_AK4384_VERSION_TYPE]
    It is represented in DRV_I2S_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_AK4384_VERSION_STR.
    DRV_AK4384_VERSION_TYPE provides the type of the release when the
    release is a(alpha) or b(beta). The interfaces DRV_AK4384_VersionGet
    and DRV_AK4384_VersionStrGet provide interfaces to the access the
    version and the version string.

  Remarks:
    Modify the return value of DRV_I2S_VersionStrGet and the
    DRV_AK4384_VERSION_MAJOR, DRV_AK4384_VERSION_MINOR,
    DRV_AK4384_VERSION_PATCH, and DRV_AK4384_VERSION_TYPE.
 */

#define _DRV_AK4384_VERSION_MAJOR         0
#define _DRV_AK4384_VERSION_MINOR         1
#define _DRV_AK4384_VERSION_PATCH         0
#define _DRV_AK4384_VERSION_TYPE          a
#define _DRV_AK4384_VERSION_STR           "0.01a"


// *****************************************************************************
/* AK4384 DMA transfer abort

  Summary:
    AK4384 DMA transfer abort

  Description:
    This constant indicates that the AK4384 DMA transfer is aborted.

  Remarks:
    None.
*/
#define DRV_AK4384_DMA_TRANSFER_ABORT /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/


// *****************************************************************************
/* AK4384 DMA transfer error

  Summary:
    AK4384 DMA transfer error

  Description:
    This constant indicates that the AK4384 DMA transfer has an address error.

  Remarks:
    None.
*/
#define DRV_AK4384_DMA_TRANSFER_ERROR /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/


// *****************************************************************************
/* AK4384 Master Clock from REFCLOCK

  Summary:
    AK4384 Master Clock from REFCLOCK

  Description:
    The following constants gives computation of divisor (RODIV) and
    trim value (ROTRIM) constants for generating master clock value to
    AK4384. The source t0 generate master clock is REFCLOCK.

  Remarks:
    None.
*/
#define DRV_AK4384_MCLK_RODIV(source, rate, ratio)  (source / rate / (ratio << 1))
#define DRV_AK4384_MCLK_ROTRIM(source, rate, ratio) ((source / rate) & ((ratio << 1)-1))


// *****************************************************************************
/* AK4384 Control register bit fields

  Summary:
    AK4384 Control register bit fields

  Description:
    The following constants gives computation for writing to a particular bit or
    field in the AK4384 control registers

  Remarks:
    None.
*/
#define DRV_AK4384_CONTROL_REG_BIT_WRITE(regValue, pos, newvale)    \
                            ((regValue & ~(1<<(pos)) ) | ((0x1&(newvale))<<(pos)))
#define DRV_AK4384_CONTROL_REG_FIELD_WRITE(reg,mask,pos,val)        \
                            ((reg) & ~(mask)) | ((mask)&((val)<<(pos)))


// *****************************************************************************
/* AK4384 COntrol Register 1

  Summary:
    AK4384 COntrol Register 1

  Description:
    The following constants gives position, mask and values for control
    register 1 of AK4384.

  Remarks:
    None.
*/
#define DRV_AK4384_CTRL1_RSTN_RST_POS               (0)
#define DRV_AK4384_CTRL1_RSTN_RST_VAL               (0)
#define DRV_AK4384_CTRL1_RSTN_NORM_POS              (0)
#define DRV_AK4384_CTRL1_RSTN_NORM_VAL              (1)
#define DRV_AK4384_CTRL1_PW_PDN_POS                 (1)
#define DRV_AK4384_CTRL1_PW_PDN_VAL                 (0)
#define DRV_AK4384_CTRL1_PW_NORM_POS                (1)
#define DRV_AK4384_CTRL1_PW_NORM_VAL                (1)
#define DRV_AK4384_CTRL1_DIF_MASK                   0x1C
#define DRV_AK4384_CTRL1_DIF_POS                    (2)
#define DRV_AK4384_CTRL1_ACKS_MANUAL_POS            (7)
#define DRV_AK4384_CTRL1_ACKS_MANUAL_VAL            (0)
#define DRV_AK4384_CTRL1_ACKS_AUTO_POS              (7)
#define DRV_AK4384_CTRL1_ACKS_AUTO_VAL              (1)


// *****************************************************************************
/* AK4384 COntrol Register 2

  Summary:
    AK4384 COntrol Register 2

  Description:
    The following constants gives position, mask and values for control
    register 2 of AK4384.

  Remarks:
    None.
*/
#define DRV_AK4384_CTRL2_SMUTE_NORM_POS             (0)
#define DRV_AK4384_CTRL2_SMUTE_NORM_VAL             (0)
#define DRV_AK4384_CTRL2_SMUTE_MUTE_POS             (0)
#define DRV_AK4384_CTRL2_SMUTE_MUTE_VAL             (1)
#define DRV_AK4384_CTRL2_DEM_MASK                   0x6
#define DRV_AK4384_CTRL2_DEM_44_1KHZ_POS            (1)
#define DRV_AK4384_CTRL2_DEM_44_1KHZ_VAL            (0)
#define DRV_AK4384_CTRL2_DEM_OFF_POS                (1)
#define DRV_AK4384_CTRL2_DEM_OFF_VAL                (1)
#define DRV_AK4384_CTRL2_DEM_48KHZ_POS              (1)
#define DRV_AK4384_CTRL2_DEM_48KHZ_VAL              (2)
#define DRV_AK4384_CTRL2_DEM_32KHZ_POS              (1)
#define DRV_AK4384_CTRL2_DEM_32KHZ_VAL              (3)
#define DRV_AK4384_CTRL2_DFS_MASK                   0x18
#define DRV_AK4384_CTRL2_DFS_NORM_POS               (4)
#define DRV_AK4384_CTRL2_DFS_NORM_VAL               (0)
#define DRV_AK4384_CTRL2_DFS_DOUBLE_POS             (4)
#define DRV_AK4384_CTRL2_DFS_DOUBLE_VAL             (1)
#define DRV_AK4384_CTRL2_DFS_QUAD_POS               (4)
#define DRV_AK4384_CTRL2_DFS_QUAD_VAL               (2)
#define DRV_AK4384_CTRL2_SLOW_SHARP_POS             (5)
#define DRV_AK4384_CTRL2_SLOW_SHARP_VAL             (0)
#define DRV_AK4384_CTRL2_SLOW_SLOW_POS              (5)
#define DRV_AK4384_CTRL2_SLOW_SLOW_VAL              (1)
#define DRV_AK4384_CTRL2_DZFM_SEPA_POS              (6)
#define DRV_AK4384_CTRL2_DZFM_SEPA_VAL              (0)
#define DRV_AK4384_CTRL2_DZFM_ANDED_POS             (6)
#define DRV_AK4384_CTRL2_DZFM_ANDED_VAL             (1)
#define DRV_AK4384_CTRL2_DZFE_DISABLE_POS           (7)
#define DRV_AK4384_CTRL2_DZFE_DISABLE_VAL           (0)
#define DRV_AK4384_CTRL2_DZFE_ENABLE_POS            (7)
#define DRV_AK4384_CTRL2_DZFE_ENABLE_VAL            (1)

// *****************************************************************************
/* AK4384 COntrol Register 3

  Summary:
    AK4384 Control Register 3

  Description:
    The following constants gives position, mask and values for control
    register 3 of AK4384.

  Remarks:
    None.
*/
#define DRV_AK4384_CTRL3_DZFB_HIGH_POS              (2)
#define DRV_AK4384_CTRL3_DZFB_HIGH_VAL              (0)
#define DRV_AK4384_CTRL3_DZFB_LOW_POS               (2)
#define DRV_AK4384_CTRL3_DZFB_LOW_VAL               (1)
#define DRV_AK4384_CTRL3_INVR_NORM_POS              (3)
#define DRV_AK4384_CTRL3_INVR_NORM_VAL              (0)
#define DRV_AK4384_CTRL3_INVR_INV_POS               (3)
#define DRV_AK4384_CTRL3_INVR_INV_VAL               (1)
#define DRV_AK4384_CTRL3_INVL_NORM_POS              (4)
#define DRV_AK4384_CTRL3_INVL_NORM_VAL              (0)
#define DRV_AK4384_CTRL3_INVL_INV_POS               (4)
#define DRV_AK4384_CTRL3_INVL_INV_VAL               (1)


// *****************************************************************************
/* AK4384 Channel Attenuation

  Summary:
    AK4384 Channel Attenuation

  Description:
    The following constants left and right channel attenuation

  Remarks:
    None.
*/
#define DRV_AK4384_LATT_ATT(n)                      (n)
#define DRV_AK4384_RATT_ATT(n)                      (n)
#define DRV_AK4384_ATT_MASK                         0xFF


// *****************************************************************************
/* AK4384 Control commands and data

  Summary:
   AK4384 Control commands and data

  Description:
    The following constants defines AK4384 Control commands and data

  Remarks:
    None.
*/
#define DRV_AK4384_CHIP_ADDRESS             (1 << 14)
#define DRV_AK4384_CONTROL_READ             (0 << 13)
#define DRV_AK4384_CONTROL_WRITE            (1 << 13)
#define DRV_AK4384_REG_ADDRESS(n)           (n << 8)
#define DRV_AK4384_REG_DATA(n)              (n << 0)
#define DRV_AK4384_COMMAND_SHIFT_BIT(n)     ((((uint32_t)(drvObj->controlCommand) << (n+1)) & 0x10000) ? true : false)


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* AK4384 Supported control commands

  Summary:
    AK4384 Supported control commands

  Description:
    This enumeration identifies AK4384 supported control commands

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4384_COMMAND_NONE,
    DRV_AK4384_COMMAND_INIT_OPEN_TIMER,
    DRV_AK4384_COMMAND_INIT_CLK_PDN_SET,
    DRV_AK4384_COMMAND_INIT_START,
    DRV_AK4384_COMMAND_INIT_SAMPLING_RATE,
    DRV_AK4384_COMMAND_INIT_END,
    DRV_AK4384_COMMAND_SAMPLING_RATE_SET,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT_INIT,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT_INIT,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT_ONLY,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT_ONLY,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT,
    DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT,
    DRV_AK4384_COMMAND_SET_CONTROL1_INIT,
    DRV_AK4384_COMMAND_MUTE_ON,
    DRV_AK4384_COMMAND_MUTE_OFF,
    DRV_AK4384_COMMAND_ZERO_DETECT_ENABLE,
    DRV_AK4384_COMMAND_ZERO_DETECT_DISABLE,
    DRV_AK4384_COMMAND_ZERO_DETECT_MODE_SET,
    DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_ENABLE,
    DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_DISABLE,
    DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_ENABLE,
    DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_ENABLE,
    DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_DISABLE,
    DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_DISABLE,
    DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_ENABLE,
    DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_DISABLE,
    DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_ENABLE,
    DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_DISABLE,
    DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_ENABLE,
    DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_DISABLE,
    DRV_AK4384_COMMAND_DEEMPHASIS_FILTER_SET,
}DRV_AK4384_COMMAND;


// *****************************************************************************
/* AK4384 supported sampling rates

  Summary:
    AK4384 supported sampling rates

  Description:
    This enumeration identifies AK4384 supported sampling rates

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4384_SAMPLERATE_192000HZ = 192000,
    DRV_AK4384_SAMPLERATE_176400HZ = 176400,
    DRV_AK4384_SAMPLERATE_120000HZ = 120000,
    DRV_AK4384_SAMPLERATE_96000HZ =  96000,
    DRV_AK4384_SAMPLERATE_88200HZ =  88200,
    DRV_AK4384_SAMPLERATE_60000HZ =  60000,
    DRV_AK4384_SAMPLERATE_48000HZ =  48000,
    DRV_AK4384_SAMPLERATE_44100HZ =  44100,
    DRV_AK4384_SAMPLERATE_32000HZ =  32000,
    DRV_AK4384_SAMPLERATE_24000HZ =  24000,
    DRV_AK4384_SAMPLERATE_16000HZ =  16000,
    DRV_AK4384_SAMPLERATE_8000HZ =   8000,
    DRV_AK4384_SAMPLERATE_DEFAULT =  44100
} DRV_AK4384_SAMPLERATE;

// *****************************************************************************
/* AK4384 supported master clock multipliers

  Summary:
    AK4384 supported master clock multipliers

  Description:
    This enumeration identifies AK4384 supported master clock multipliers

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4384_MCLK_MULTIPLIER_128FS = 128,                        
    DRV_AK4384_MCLK_MULTIPLIER_192FS = 192,                                    
    DRV_AK4384_MCLK_MULTIPLIER_256FS = 256,            
    DRV_AK4384_MCLK_MULTIPLIER_384FS = 384,
    DRV_AK4384_MCLK_MULTIPLIER_512FS = 512,            
    DRV_AK4384_MCLK_MULTIPLIER_768FS = 768,                        
    DRV_AK4384_MCLK_MULTIPLIER_1152FS = 1152     
} DRV_AK4384_MCLK_MULTIPLIER;


// *****************************************************************************

// *****************************************************************************
/* AK4384 Control registers

  Summary:
    AK4384 Control registers

  Description:
    This enumeration identifies AK4384 Control registers

  Remarks:
    None.
*/
typedef enum
{
    DRV_AK4384_CONTROL_REGISTER_1,
    DRV_AK4384_CONTROL_REGISTER_2,
    DRV_AK4384_CONTROL_REGISTER_3,
    DRV_AK4384_CONTROL_REGISTER_LATT,
    DRV_AK4384_CONTROL_REGISTER_RATT

} DRV_AK4384_CONTROL_REGISTER;


// *****************************************************************************
/* AK4384 Sampling Speed Control

  Summary:
    Identifies Sampling Speed Control

  Description:
    This enumeration identifies sampling mode.
	The enumerator is applicable when the MCLK is configured  in Manual Setting Mode.

  Remarks:
    None.
*/
typedef enum
{
    /* Sampling Speed control Normal Mode.
       This is the default control.
       Supports frequency range 8kHz~48kHz */
    DRV_AK4384_SAMPLING_SPEED_NORMAL,

    /* Sampling Speed control Double Mode.
       Supports frequency range 60kHz~96kHz */
    DRV_AK4384_SAMPLING_SPEED_DOUBLE,

    /* Sampling Speed control Double Mode.
       Supports frequency range 120kHz~192kHz */
    DRV_AK4384_SAMPLING_SPEED_QUAD

} DRV_AK4384_SAMPLING_SPEED;


// *****************************************************************************
/* AK4384 Driver task states

  Summary
    Lists the different states that AK4384 Driver task routine can have.

  Description
    This enumeration lists the different states that AK4384 Driver task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AK4384_TASK_STATE_CONTROL,
    DRV_AK4384_TASK_STATE_DATA
} DRV_AK4384_TASK;


/**********************************************
 * Driver Client Obj
 **********************************************/
typedef struct _DRV_AK4384_CLIENT_OBJ_STRUCT
{
    /* Indicates that this object is in use */
    bool inUse;

    /* Indicate whether the client is open in
     * read,write or read/write mode */
    DRV_IO_INTENT ioIntent;

    /* Call back function for this client */
    DRV_AK4384_BUFFER_EVENT_HANDLER  pEventCallBack;

    /* Client data(Event Context) that will be
     * returned at callback */
    uintptr_t hClientArg;

    /* pointer to the driver that own this object */
    void* hDriver;

} DRV_AK4384_CLIENT_OBJ;

/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_AK4384_OBJ_STRUCT
{

    /* Status of this driver instance */
    SYS_STATUS status;

    /* Indicates this object is in use */
    bool inUse;

    /* Flag to indicate that the hardware instance is used
     *  in exclusive access mode */
    bool isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t numClients;

    /* Identifies data module(I2S) driver ID for
     * data interface of CODEC */
    SYS_MODULE_INDEX i2sDriverModuleIndex;

    /* Identifies data module(I2S) driver open handle */
    DRV_HANDLE i2sDriverHandle;

    /* Identifies control module timer ID for
     * control interface of CODEC */
    SYS_MODULE_INDEX tmrDriverModuleIndex;

    /* Identifies control module(Timer) driver open handle */
    DRV_HANDLE tmrDriverHandle;

    /* Sampling rate */
    uint32_t samplingRate;

    /* MCLK mode. */
    DRV_AK4384_MCLK_MODE mclkMode;

    /* Identifies the Audio data format */
    DRV_AK4384_AUDIO_DATA_FORMAT audioDataFormat;

    /* Keeps track if the driver is in interrupt
     * context */
    bool isInInterruptContext;

    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE(mutexDriverInstance);

    /* ----------------------------------------------------*/
    /* Control interface specific Implementation variables */
    /* ----------------------------------------------------*/

    /* Volume for volume command */
    uint8_t volume[DRV_AK4384_NUMBER_OF_CHANNELS];

    /* Command under execution */
    DRV_AK4384_COMMAND command;

    /* Command value being transmitted */
    uint32_t controlCommand;

    /* Status of command under execution */
    volatile bool controlCommandStatus;

    /* Number of bits transfered for a control command */
    uint8_t countBit;

    /* Array holding the last programmed command value */
    uint8_t lastRegValue[5];

    /* Command complete callback function */
    DRV_AK4384_COMMAND_EVENT_HANDLER commandCompleteCallback;

    /* command complete event context */
    uintptr_t commandContextData;

    uint16_t mclk_multiplier;
    uint16_t bclk_divider;
    
    bool delayDriverInitialization;

} DRV_AK4384_OBJ;


// *****************************************************************************
/* AK4384 Driver Global Instances Object

  Summary:
    Object used to keep track of data that is common to all instances of the
    AK4384 driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the AK4384 driver.

  Remarks:
    None.
*/

typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool membersAreInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexClientObjects);

} DRV_AK4384_COMMON_DATA_OBJ;


/**************************************
 * Local functions.
 *************************************/
// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier)

  Summary:
    Generates the master clock(to AK4384) from REFCLOCK  for the given
    sampling rate.

  Description:
    Generates the master clock(to AK4384) from REFCLOCK  for the given
    sampling rate.

  Remarks:
    None
*/
static void _DRV_AK4384_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier);


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_ConrolRegisterSet (DRV_AK4384_OBJ *drvObj,
                DRV_AK4384_CONTROL_REGISTER contRegister, uint8_t value )

  Summary:
    Prepares the control command to be sent to AK4384. Also starts the
    timer to initiate the control command transfer.

  Description:
    Prepares the control command to be sent to AK4384. Also starts the
    timer to initiate the control command transfer.

  Remarks:
    None
*/
static void _DRV_AK4384_ConrolRegisterSet (DRV_AK4384_OBJ *drvObj,
                DRV_AK4384_CONTROL_REGISTER contRegister, uint8_t value );


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_ControlTasks(DRV_AK4384_OBJ *drvObj)

  Summary:
    Implements the state maching for the Audio control interface of AK4384

  Description:
    Implements the state maching for the Audio control interface of AK4384

  Remarks:
    None
*/
static void _DRV_AK4384_ControlTasks(DRV_AK4384_OBJ *drvObj);

// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_TimerCallbackHandler(uintptr_t context,  uint32_t currTick)

  Summary:
    Implements the bit banging SPI implementation

  Description:
    Implements the bit banging SPI implementation for the control interface
    commands

  Remarks:
    None
*/
static void _DRV_AK4384_TimerCallbackHandler(uintptr_t context,  uint32_t currTick);


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_I2SBufferEventHandler(DRV_I2S_BUFFER_EVENT event,
        DRV_I2S_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)

  Summary:
    Implements the handler for i2s buffer request completion.

  Description:
    Implements the handler for i2s buffer request completion.

  Remarks:
    None
*/
static void _DRV_AK4384_I2SBufferEventHandler(DRV_I2S_BUFFER_EVENT event,
        DRV_I2S_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle);

#endif // #ifndef _DRV_AK4384_LOCAL_H
/*******************************************************************************
 End of File
*/