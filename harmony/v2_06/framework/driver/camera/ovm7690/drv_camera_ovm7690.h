/*******************************************************************************
  OVM7690 Camera Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_camera_ovm7690.h

  Summary:
    OVM7690 Camera Driver local data structures.

  Description:
    This header file provides the local data structures for the OVM7690 Camera
	Driver Library.
*******************************************************************************/

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

#ifndef _DRV_CAMERA_OVM7690_H
#define _DRV_CAMERA_OVM7690_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "driver/camera/drv_camera.h"
#include "system/system.h"
#include "system/int/sys_int.h"
#include "system/dma/sys_dma.h"
#include "system/ports/sys_ports.h"
#include "system/debug/sys_debug.h"
#include <sys/kmem.h>

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* OVM7690 Camera Driver Module Index

  Summary:
    OVM7690 driver index definitions.

  Description:
    These constants provide OVM7690 Camera Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_CAMERA_OVM7690_Initialize and
    DRV_CAMERA_OVM7690_Open routines to identify the driver instance in use.
*/

#define DRV_CAMERA_OVM7690_INDEX_0        0
#define DRV_CAMERA_OVM7690_INDEX_1        1

// *****************************************************************************
/* OVM7690 Camera Driver SCCB Write ID

  Summary:
    OVM7690 Camera SCCB Interface device Write Slave ID.

  Description:
    This macro provides a definition of the OVM7690 Camera SCCB Interface device Write 
    Slave ID.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_CAMERA_OVM7690_RegisterSet 
    function to identify the OVM7690 Camera SCCB Interface device Write Slave ID.
   
*/

#define DRV_CAMERA_OVM7690_SCCB_WRITE_ID /*DOM-IGNORE-BEGIN*/ 0x42 /*DOM-IGNORE-END*/

// *****************************************************************************
/* OVM7690 Camera Driver SCCB Read ID

  Summary:
    OVM7690 Camera SCCB Interface device Read Slave ID.
    
  Description:
    This macro provides a definition of the OVM7690 Camera SCCB Interface device Read 
    Slave ID.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
*/

#define DRV_CAMERA_OVM7690_SCCB_READ_ID /*DOM-IGNORE-BEGIN*/ 0x43 /*DOM-IGNORE-END*/

// *****************************************************************************
/* OVM7690 Camera Driver Soft reset flag.

  Summary:
    OVM7690 Camera Driver Register 0x12 Soft reset flag.

  Description:
    This macro provides a definition of the OVM7690 Camera Register 0x12 Soft reset 
    flag.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
*/

#define DRV_CAMERA_OVM7690_REG12_SOFT_RESET /*DOM-IGNORE-BEGIN*/ 0x80 /*DOM-IGNORE-END*/

// *****************************************************************************
/* OVM7690 Camera Error flag

  Summary:
    Identifies OVM7690 Camera possible errors.

  Description:
    This enumeration defines possible OVM7690 Camera errors.

  Remarks:
    This enumeration values are returned by driver interfaces in case of errors.
*/
typedef enum
{
    /* OVM7690 Camera Driver Invalid Handle */
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE,

    /* OVM7690 Camera Driver error none */
    DRV_CAMERA_OVM7690_ERROR_NONE,
    
} DRV_CAMERA_OVM7690_ERROR;

// *****************************************************************************
/* OVM7690 Camera Client Status.

  Summary:
    Identifies OVM7690 Camera possible client status.

  Description:
    This enumeration defines possible OVM7690 Camera Client Status.

  Remarks:
    This enumeration values are set by driver interfaces:
    DRV_CAMERA_OVM7690_Open and DRV_CAMERA_OVM7690_Close.
*/
typedef enum
{
    /* An error has occurred.*/
    DRV_CAMERA_OVM7690_CLIENT_STATUS_ERROR    = DRV_CLIENT_STATUS_ERROR,

    /* The driver is closed, no operations for this client are ongoing,
    and/or the given handle is invalid. */
    DRV_CAMERA_OVM7690_CLIENT_STATUS_CLOSED   = DRV_CLIENT_STATUS_CLOSED,

    /* The driver is currently busy and cannot start additional operations. */
    DRV_CAMERA_OVM7690_CLIENT_STATUS_BUSY     = DRV_CLIENT_STATUS_BUSY,

    /* The module is running and ready for additional operations */
    DRV_CAMERA_OVM7690_CLIENT_STATUS_READY    = DRV_CLIENT_STATUS_READY

} DRV_CAMERA_OVM7690_CLIENT_STATUS;

// *****************************************************************************
/* OVM7690 Camera Device Register Addresses.

  Summary:
    Lists OVM7690 Camera device register addresses.
 
  Description:
    This enumeration defines the list of device register addresses.

  Remarks:
    These constants should be used in place of hard-coded numeric literals. 
    These values should be passed into the DRV_CAMERA_OVM7690_RegisterSet 
    function. Refer to the specific device data sheet for more information.
*/
typedef enum
{
    /* AGC Gain Control */
    DRV_CAMERA_OVM7690_GAIN_REG_ADDR   /*DOM-IGNORE-BEGIN*/  = 0x00, /*DOM-IGNORE-END*/
    
    /* AWB Blue Gain Control */
    DRV_CAMERA_OVM7690_BGAIN_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x01, /*DOM-IGNORE-END*/
    
    /* AWB Red Gain Control */
    DRV_CAMERA_OVM7690_RGAIN_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x02, /*DOM-IGNORE-END*/
         
    /* AWB Green Gain Control */
    DRV_CAMERA_OVM7690_GGAIN_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x03, /*DOM-IGNORE-END*/
    
    /* Frame Average Level */
    DRV_CAMERA_OVM7690_YAVG_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x04, /*DOM-IGNORE-END*/
    
    /* B Pixel Average */
    DRV_CAMERA_OVM7690_BAVG_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x05, /*DOM-IGNORE-END*/
    
    /* R Pixel Average */
    DRV_CAMERA_OVM7690_RAVG_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x06, /*DOM-IGNORE-END*/
    
    /* G Pixel Average */
    DRV_CAMERA_OVM7690_GAVG_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x07, /*DOM-IGNORE-END*/
    
    /* Product ID Number MSB */
    DRV_CAMERA_OVM7690_PIDH_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x0A, /*DOM-IGNORE-END*/
    
    /* Product ID Number LSB */
    DRV_CAMERA_OVM7690_PIDL_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x0B, /*DOM-IGNORE-END*/
    
    /* Register 0x0C */
    DRV_CAMERA_OVM7690_REG0C_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x0C, /*DOM-IGNORE-END*/
    
    /* Register 0x0D */
    DRV_CAMERA_OVM7690_REG0D_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x0D, /*DOM-IGNORE-END*/
    
    /* Register 0x0E */
    DRV_CAMERA_OVM7690_REG0E_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x0E, /*DOM-IGNORE-END*/
    
    /* Automatic Exposure Control MSBs */
    DRV_CAMERA_OVM7690_AECH_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x0F, /*DOM-IGNORE-END*/
            
    /* Automatic Exposure Control LSBs */
    DRV_CAMERA_OVM7690_AECL_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x10, /*DOM-IGNORE-END*/
            
    /* Clock Control Register */
    DRV_CAMERA_OVM7690_CLKRC_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x11, /*DOM-IGNORE-END*/
            
    /* Register 0x12 */
    DRV_CAMERA_OVM7690_REG12_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x12, /*DOM-IGNORE-END*/
            
    /* Register 0x13 */
    DRV_CAMERA_OVM7690_REG13_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x13, /*DOM-IGNORE-END*/
            
    /* Register 0x14 */
    DRV_CAMERA_OVM7690_REG14_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x14, /*DOM-IGNORE-END*/
            
    /* Register 0x15 */
    DRV_CAMERA_OVM7690_REG15_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x15, /*DOM-IGNORE-END*/
            
    /* Register 0x16 */
    DRV_CAMERA_OVM7690_REG16_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x16, /*DOM-IGNORE-END*/
            
    /* Horizontal Window start point control */
    DRV_CAMERA_OVM7690_HSTART_REG_ADDR /*DOM-IGNORE-BEGIN*/ = 0x17, /*DOM-IGNORE-END*/
            
    /* Horizontal Sensor Size */
    DRV_CAMERA_OVM7690_HSIZE_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x18, /*DOM-IGNORE-END*/
            
    /* Vertical Window start Line control */
    DRV_CAMERA_OVM7690_VSTART_REG_ADDR /*DOM-IGNORE-BEGIN*/ = 0x19, /*DOM-IGNORE-END*/
            
    /* Vertical sensor size */
    DRV_CAMERA_OVM7690_VSIZE_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x1A, /*DOM-IGNORE-END*/
            
    /* Pixel Shift */
    DRV_CAMERA_OVM7690_SHFT_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x1B, /*DOM-IGNORE-END*/
            
    /* Manufacturer ID Byte - High */
    DRV_CAMERA_OVM7690_MIDH_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x1C, /*DOM-IGNORE-END*/
            
    /* Manufacturer ID Byte - Low */
    DRV_CAMERA_OVM7690_MIDL_REG_ADDR   /*DOM-IGNORE-BEGIN*/ = 0x1D, /*DOM-IGNORE-END*/
            
    /* Register 0x20 */
    DRV_CAMERA_OVM7690_REG20_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x20, /*DOM-IGNORE-END*/

    /* Register 0x28 */
    DRV_CAMERA_OVM7690_REG28_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x28, /*DOM-IGNORE-END*/
            
    /* Register 0x3E */
    DRV_CAMERA_OVM7690_REG3E_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0x3E, /*DOM-IGNORE-END*/
            
    /* Register 0xB4 */
    DRV_CAMERA_OVM7690_REGB4_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB4, /*DOM-IGNORE-END*/
            
    /* Register 0xB5 */
    DRV_CAMERA_OVM7690_REGB5_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB5, /*DOM-IGNORE-END*/
            
    /* Register 0xB6 */
    DRV_CAMERA_OVM7690_REGB6_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB6, /*DOM-IGNORE-END*/
            
    /* Register 0xB7 */
    DRV_CAMERA_OVM7690_REGB7_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB7, /*DOM-IGNORE-END*/
            
    /* Register 0xB8 */
    DRV_CAMERA_OVM7690_REGB8_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB8, /*DOM-IGNORE-END*/
            
    /* Register 0xB9 */
    DRV_CAMERA_OVM7690_REGB9_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xB9, /*DOM-IGNORE-END*/
            
    /* Register 0xBA */
    DRV_CAMERA_OVM7690_REGBA_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBA, /*DOM-IGNORE-END*/
            
    /* Register 0xBB */
    DRV_CAMERA_OVM7690_REGBB_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBB, /*DOM-IGNORE-END*/
            
    /* Register 0xBC */
    DRV_CAMERA_OVM7690_REGBC_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBC, /*DOM-IGNORE-END*/
            
    /* Register 0xBD */
    DRV_CAMERA_OVM7690_REGBD_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBD, /*DOM-IGNORE-END*/
            
    /* Register 0xBE */
    DRV_CAMERA_OVM7690_REGBE_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBE, /*DOM-IGNORE-END*/
            
    /* Register 0xBF */
    DRV_CAMERA_OVM7690_REGBF_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xBF, /*DOM-IGNORE-END*/
            
    /* Register 0xC0 */
    DRV_CAMERA_OVM7690_REGC0_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC0, /*DOM-IGNORE-END*/
            
    /* Register 0xC1 */
    DRV_CAMERA_OVM7690_REGC1_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC1, /*DOM-IGNORE-END*/
            
    /* Register 0xC2 */
    DRV_CAMERA_OVM7690_REGC2_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC2, /*DOM-IGNORE-END*/
            
    /* Register 0xC3 */
    DRV_CAMERA_OVM7690_REGC3_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC3, /*DOM-IGNORE-END*/
            
    /* Register 0xC4 */
    DRV_CAMERA_OVM7690_REGC4_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC4, /*DOM-IGNORE-END*/
            
    /* Register 0xC5 */
    DRV_CAMERA_OVM7690_REGC5_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC5, /*DOM-IGNORE-END*/
            
    /* Register 0xC6 */
    DRV_CAMERA_OVM7690_REGC6_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC6, /*DOM-IGNORE-END*/
            
    /* Register 0xC7 */
    DRV_CAMERA_OVM7690_REGC7_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC7, /*DOM-IGNORE-END*/
            
    /* Register 0xC8 */
    DRV_CAMERA_OVM7690_REGC8_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC8, /*DOM-IGNORE-END*/
            
    /* Register 0xC9 */
    DRV_CAMERA_OVM7690_REGC9_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xC9, /*DOM-IGNORE-END*/
            
    /* Register 0xCA */
    DRV_CAMERA_OVM7690_REGCA_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCA, /*DOM-IGNORE-END*/
            
    /* Register 0xCB */
    DRV_CAMERA_OVM7690_REGCB_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCB, /*DOM-IGNORE-END*/
            
    /* Register 0xCC */
    DRV_CAMERA_OVM7690_REGCC_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCC, /*DOM-IGNORE-END*/
            
    /* Register 0xCD */
    DRV_CAMERA_OVM7690_REGCD_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCD, /*DOM-IGNORE-END*/
            
    /* Register 0xCE */
    DRV_CAMERA_OVM7690_REGCE_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCE, /*DOM-IGNORE-END*/        
            
    /* Register 0xCF */
    DRV_CAMERA_OVM7690_REGCF_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xCF, /*DOM-IGNORE-END*/
            
    /* Register 0xD0 */
    DRV_CAMERA_OVM7690_REGD0_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD0, /*DOM-IGNORE-END*/
            
    /* Register 0xD1 */
    DRV_CAMERA_OVM7690_REGD1_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD1, /*DOM-IGNORE-END*/
            
    /* Register 0xD2 */
    DRV_CAMERA_OVM7690_REGD2_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD2, /*DOM-IGNORE-END*/
            
    /* Register 0xD3 */
    DRV_CAMERA_OVM7690_REGD3_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD3, /*DOM-IGNORE-END*/
            
    /* Register 0xD4 */
    DRV_CAMERA_OVM7690_REGD4_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD4, /*DOM-IGNORE-END*/
            
    /* Register 0xD5 */
    DRV_CAMERA_OVM7690_REGD5_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD5, /*DOM-IGNORE-END*/
            
    /* Register 0xD6 */
    DRV_CAMERA_OVM7690_REGD6_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD6, /*DOM-IGNORE-END*/
            
    /* Register 0xD7 */
    DRV_CAMERA_OVM7690_REGD7_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD7, /*DOM-IGNORE-END*/
            
    /* Register 0xD8 */
    DRV_CAMERA_OVM7690_REGD8_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD8, /*DOM-IGNORE-END*/
            
    /* Register 0xD9 */
    DRV_CAMERA_OVM7690_REGD9_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xD9, /*DOM-IGNORE-END*/
            
    /* Register 0xDA */
    DRV_CAMERA_OVM7690_REGDA_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDA, /*DOM-IGNORE-END*/
            
    /* Register 0xDB */
    DRV_CAMERA_OVM7690_REGDB_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDB, /*DOM-IGNORE-END*/
            
    /* Register 0xDC */
    DRV_CAMERA_OVM7690_REGDC_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDC, /*DOM-IGNORE-END*/
            
    /* Register 0xDD */
    DRV_CAMERA_OVM7690_REGDD_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDD, /*DOM-IGNORE-END*/
            
    /* Register 0xDE */
    DRV_CAMERA_OVM7690_REGDE_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDE, /*DOM-IGNORE-END*/        
            
    /* Register 0xDF */
    DRV_CAMERA_OVM7690_REGDF_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xDF, /*DOM-IGNORE-END*/
            
    /* Register 0xE0 */
    DRV_CAMERA_OVM7690_REGE0_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xE0, /*DOM-IGNORE-END*/
            
    /* Register 0xE1 */
    DRV_CAMERA_OVM7690_REGE1_REG_ADDR  /*DOM-IGNORE-BEGIN*/ = 0xE1, /*DOM-IGNORE-END*/
            
} DRV_CAMERA_OVM7690_REGISTER_ADDRESS;

// *****************************************************************************
/* OVM7690 Camera output pixel formats

  Summary:
    Lists output pixel formats supported by the OVM7690 Camera.
 
  Description:
    This enumeration provides a list of output formats supported by the OVM7690 
	Camera.

  Remarks:
    These constants should be used in place of hard-coded numeric literals. 
    These values should be passed into the DRV_CAMERA_OVM7690_RegisterSet 
    function. Refer to the specific device data sheet for more information.
*/

typedef enum
{
    /* YUV output format */
    DRV_CAMERA_OVM7690_REG12_OP_FORMAT_YUV   /*DOM-IGNORE-BEGIN*/ = 0x00, /*DOM-IGNORE-END*/
            
    /* Bayer RAW Format */
    DRV_CAMERA_OVM7690_REG12_OP_FORMAT_RAW_1 /*DOM-IGNORE-BEGIN*/ = 0x01, /*DOM-IGNORE-END*/
            
    /* RGB Format */
    DRV_CAMERA_OVM7690_REG12_OP_FORMAT_RGB   /*DOM-IGNORE-BEGIN*/ = 0x10, /*DOM-IGNORE-END*/
            
    /* Bayer Raw Format */
    DRV_CAMERA_OVM7690_REG12_OP_FORMAT_RAW_2 /*DOM-IGNORE-BEGIN*/ = 0x11, /*DOM-IGNORE-END*/

} DRV_CAMERA_OVM7690_REG12_OP_FORMAT;


// *****************************************************************************
/* OVM7690 Camera Window Rect

  Summary:
    OVM7690 Camera window rectangle coordinates.
 
  Description:
    This structure defines window rectangle co-ordinates as left, right, top, and
    bottom. 

  Remarks:
    These values should be passed into the DRV_CAMERA_OVM7690_FrameRectSet 
    function.
*/

typedef struct
{
    /* OVM7690 Camera Window left coordinate */
    uint32_t left;
            
    /* OVM7690 Camera Window top coordinate */
    uint32_t top;
            
    /* OVM7690 Camera Window right coordinate */
    uint32_t right;
            
    /* OVM7690 Camera Window bottom coordinate */
    uint32_t bottom;
    
} DRV_CAMERA_OVM7690_RECT;

// *****************************************************************************
/* OVM7690 Camera Initialization parameters

  Summary:
    OVM7690 Camera Driver initialization parameters.

  Description:
    This structure defines OVM7690 Camera Driver initialization parameters.

  Remarks:
    These values should be passed into the DRV_CAMERA_OVM7690_Initialize 
    function.
*/

typedef struct
{
    /* Camera module ID */
    CAMERA_MODULE_ID        cameraID;
    
    /* Source Port Address */
    void *                  sourcePort;
    
    /* HSYNC pin channel */
    PORTS_CHANNEL           hsyncChannel;
    
    /* HSYNC pin bit position */
    PORTS_BIT_POS           hsyncPosition;
            
    /* VSYNC pin channel */
    PORTS_CHANNEL           vsyncChannel;

    /* VSYNC pin bit position */
    PORTS_BIT_POS           vsyncPosition;
    
    /* HSYNC Interrupt Source */
    INT_SOURCE              hsyncInterruptSource;
    
    /* VSYNC Interrupt Source */
    INT_SOURCE              vsyncInterruptSource;

    /* DMA channel */
    DMA_CHANNEL             dmaChannel;
    
    /* DMA trigger source */
    DMA_TRIGGER_SOURCE      dmaTriggerSource;
    
    /* Bits per pixel */
    uint16_t                bpp;
    
} DRV_CAMERA_OVM7690_INIT;

// *****************************************************************************
/* OVM7690 Camera Driver Instance Object 

  Summary:
    OVM7690 Camera Driver instance object.

  Description:
    This structure provides a definition of the OVM7690 Camera Driver instance 
	object.

  Remarks:
    These values are been updated into the DRV_CAMERA_OVM7690_Initialize 
    function.
*/

typedef struct
{
    /* The module index associated with the object*/
    CAMERA_MODULE_ID moduleId;
    
    /* The status of the driver */
    SYS_STATUS status;
    
    /* Flag to indicate this object is in use  */
    bool inUse;

    /* Flag to indicate that driver has been opened exclusively. */
    bool isExclusive;

    /* Keeps track of the number of clients
     * that have opened this driver */
    size_t nClients;

    /* HSYNC pin channel */
    PORTS_CHANNEL hsyncChannel;
    
    /* HSYNC pin bit position */
    PORTS_BIT_POS hsyncPosition;
            
    /* VSYNC pin channel */
    PORTS_CHANNEL vsyncChannel;

    /* VSYNC pin bit position */
    PORTS_BIT_POS vsyncPosition;
    
    /* HSYNC Interrupt Source */
    INT_SOURCE hsyncInterruptSource;
    
    /* VSYNC Interrupt Source */
    INT_SOURCE vsyncInterruptSource;
    
    /* DMA Handle */
    SYS_DMA_CHANNEL_HANDLE dmaHandle;
    
    /* Read DMA channel  */
    DMA_CHANNEL dmaChannel;
    
    /* DMA Trigger Source */
    DMA_TRIGGER_SOURCE dmaTriggerSource;
    
    /* DMA Transfer Complete Flag */
    bool dmaTransferComplete;
    
    /* Source Port Address */
    void * sourcePort;

    /* Frame Line Count */
    uint32_t frameLineCount;
    
    /* Frame Line Size */
    uint32_t frameLineSize;
    
    /* Frame Line Address */
    void * frameLineAddress;
    
    /* Framebuffer Address */
    void * frameBufferAddress;
    
    /* Window Rectangle*/
    DRV_CAMERA_OVM7690_RECT rect;
    
    /* Bits per pixel supported */
    uint16_t bpp;
    
} DRV_CAMERA_OVM7690_OBJ;

// *****************************************************************************
/* OVM7690 Camera Driver Client Object.

  Summary:
    OVM7690 Camera Driver client object.
    
  Description:
    This structure provides a definition of the OVM7690 Camera Driver client 
	object. 

  Remarks:
    These values are been updated into the DRV_CAMERA_OVM7690_Open 
    function.
*/

typedef struct
{
    /* The hardware instance object associated with the client */
    DRV_CAMERA_OVM7690_OBJ * hDriver;

    /* The I/O intent with which the client was opened */
    DRV_IO_INTENT   ioIntent;

    /* This flags indicates if the object is in use or is available */
    bool inUse;
    
    /* Driver Error */
    DRV_CAMERA_OVM7690_ERROR  error;
    
    /* Client status */
    DRV_CAMERA_OVM7690_CLIENT_STATUS status;

} DRV_CAMERA_OVM7690_CLIENT_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for the driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
     SYS_MODULE_OBJ DRV_CAMERA_OVM7690_Initialize
     (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
     )

  Summary:
    Initializes the OVM7690 Camera instance for the specified driver index.

  Description:
    This function initializes the OVM7690 Camera Driver instance for the 
    specified driver index, making it ready for clients to open and use it. The 
    initialization data is specified by the init parameter. The initialization 
    may fail if the number of driver objects allocated are insufficient or if 
    the specified driver instance is already initialized. The driver instance 
    index is independent of the OVM7690 Camera module ID. Refer to
    the description of the DRV_CAMERA_OVM7690_INIT data structure for more 
    details on which members on this data structure are overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized
    init   - Pointer to a data structure containing any data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.  
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // The following code snippet shows an example OVM7690 driver initialization.
    
    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other OVM7690 Camera Driver function 
	is called.

    This function should only be called once during system initialization
    unless DRV_CAMERA_OVM7690_Deinitialize is called to deinitialize the driver 
    instance. This function will NEVER block for hardware access.
 */
SYS_MODULE_OBJ DRV_CAMERA_OVM7690_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT * const init
);

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the OVM7690 Camera Driver module.

  Description:
    This function deinitializes the specified instance of the OVM7690 Camera 
	Driver module, disabling its operation (and any hardware), and invalidates 
	all of the internal data.

  Precondition:
    Function DRV_CAMERA_OVM7690_Initialize should have been called before 
    calling this function.

  Parameters:
    object          - Driver object handle, returned from the 
                      DRV_CAMERA_OVM7690_Initialize function

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object; //  Returned from DRV_CAMERA_OVM7690_Initialize
    SYS_STATUS          status;

    DRV_CAMERA_OVM7690_Deinitialize(object);

    status = DRV_CAMERA_OVM7690_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know 
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This 
    function will NEVER block waiting for hardware.
 */

void DRV_CAMERA_OVM7690_Deinitialize(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_CAMERA_OVM7690_Open
    (
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified OVM7690 Camera Driver instance and returns a handle to 
    it.

  Description:
    This function opens the specified OVM7690 Camera Driver instance and provides
    a handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent 
    parameter defines how the client interacts with this driver instance.

  Precondition:
    Function DRV_CAMERA_OVM7690_Initialize must have been called before calling 
    this function.

  Parameters:
    index   - Identifier for the object instance to be opened
    intent  - Zero or more of the values from the enumeration
              DRV_IO_INTENT "ORed" together to indicate the intended use
              of the driver. See function description for details.

  Returns:
    If successful, the function returns a valid open instance handle (a number
    identifying both the caller and the module instance).
    
    If an error occurs, the return value is DRV_HANDLE_INVALID. Errors can occur:
    - if the number of client objects allocated via DRV_CAMERA_OVM7690_CLIENTS_NUMBER 
      is insufficient
    - if the client is trying to open the driver but driver has been opened
      exclusively by another client
    - if the driver hardware instance being opened is not initialized or is
      invalid
    - if the client is trying to open the driver exclusively, but has already
      been opened in a non exclusive mode by another client
    - if the driver is not ready to be opened, typically when the initialize
      function has not completed execution

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_CAMERA_OVM7690_Close function is 
    called. This function will NEVER block waiting for hardware.If the requested 
    intent flags are not supported, the function will return DRV_HANDLE_INVALID. 
    This function is thread safe in a RTOS application.

 */
DRV_HANDLE DRV_CAMERA_OVM7690_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_Close( DRV_Handle handle )

  Summary:
    Closes an opened instance of the OVM7690 Camera Driver.

  Description:
    This function closes an opened instance of the OVM7690 Camera Driver, 
    invalidating the handle. Any buffers in the driver queue that were submitted
    by this client will be removed. After calling this function, the handle 
    passed in "handle" must not be used with any of the remaining driver 
    routines (with one possible exception described in the "Remarks" section).
    A new handle must be obtained by calling DRV_CAMERA_OVM7690_Open before the 
    caller may use the driver again

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   Open function

  Returns:
    None.
                
  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_USART_Open
    DRV_CAMERA_OVM7690_Close(handle);
    </code>

  Remarks:
    Usually there is no need for the client to verify that the Close operation
    has completed.  The driver will abort any ongoing operations when this
    function is called.
 */

void DRV_CAMERA_OVM7690_Close ( DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameBufferAddressSet
    ( 
     DRV_HANDLE handle,
     void * frameBuffer
    )

  Summary:
    Sets the framebuffer address.

  Description:
    This function will set the framebuffer address. This framebuffer address 
	will point to the location at which frame data is to be rendered. This buffer 
	is shared with the display controller to display the frame on the display.
 
  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle  - A valid open instance handle, returned from the driver's
              Open function

  Returns:
    * DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    * DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>
      
        DRV_HANDLE handle;
        uint16_t frameBuffer[DISP_VER_RESOLUTION][DISP_HOR_RESOLUTION];

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameBufferAddressSet( handle, (void *) frameBuffer ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    </code>

  Remarks:
    This function is mandatory. A valid framebuffer address must be set to 
    display the camera data.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameBufferAddressSet
( 
    DRV_HANDLE handle,
    void * frameBuffer
);

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Start
    ( 
        DRV_HANDLE handle
    );

  Summary:
    Starts camera rendering to the display.

  Description:
    This function starts the camera rendering to the display by writing the pixel
    data to the framebuffer. The framebuffer is shared between the OVM7690 Camera and
    the display controller. 

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    DRV_CAMERA_OVM7690_FrameBufferAddressSet must have been called to set a valid
    framebuffer address.

  Parameters:
    handle  - A valid open instance handle, returned from the driver's
              Open function

  Returns:
    * DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    * DRV_CAMERA_OVM7690_ERROR_NONE - No error.
 
  Example:
    <code>
        DRV_HANDLE handle;
        uint16_t frameBuffer[DISP_VER_RESOLUTION][DISP_HOR_RESOLUTION];

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameBufferAddressSet( handle, (void *) frameBuffer ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }

        if ( DRV_CAMERA_OVM7690_Start( handle ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    </code>

  Remarks:
    This function is mandatory. Camera module will not update the framebuffer
    without calling this function.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Start
( 
    DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Stop
    ( 
        DRV_HANDLE handle
    );
 
  Summary:
    Stops rendering the camera Pixel data.

  Description:
    This function starts the camera rendering to the display by writing the pixel
    data to the framebuffer. The framebuffer is shared between the OVM7690 Camera and
    the display controller.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle  - A valid open instance handle, returned from the driver's
              Open function.

  Returns:
    * DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    * DRV_CAMERA_OVM7690_ERROR_NONE - No error.
 
  Example:
    <code>
        DRV_HANDLE handle;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_Stop( handle ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    </code>

  Remarks:
    This function only disables the interrupt for HSYNC and VSYNC. To stop the 
    camera the power-down pin needs to be toggled to an active-high value., which 
	will stop the camera internal clock and maintain the register values.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Stop
( 
    DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_RegisterSet 
    ( 
        DRV_CAMERA_OVM7690_REGISTER_ADDRESS regIndex, 
        uint8_t regValue 
    )

  Summary:
    Sets the camera OVM7690 configuration registers.

  Description:
    This function sets the OVM7690 Camera configuration registers using the SCCB
    interface. 

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    The SCCB interface also must have been initialized to configure the OVM7690 
	Camera Driver.

  Parameters:
    regIndex - Defines the OVM7690 configuration register addresses.
    regValue - Defines the register value to be set.

  Returns:
    * DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    * DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>
       DRV_HANDLE handle;
       uint8_t reg12 = DRV_CAMERA_OVM7690_REG12_SOFT_RESET;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG12_REG_ADDR,
                                             reg12 ) != 
                                             DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    </code>

  Remarks:
    This function can be used separately or within an interface.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_RegisterSet 
( 
    DRV_CAMERA_OVM7690_REGISTER_ADDRESS regIndex, 
    uint8_t regValue 
);

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameRectSet
    ( 
        DRV_HANDLE handle,
        uint32_t   left,
        uint32_t   top,
        uint32_t   right,
        uint32_t   bottom
    )

  Summary:
    Sets the frame rectangle set.

  Description:
    This function sets the frame rectangle coordinates. The frame within the 
    rectangle is copied to the framebuffer. The left and top values are 
    expected to be less than right and bottom respectively. Left, top, right, and 
    bottom values are also expected to be within range of screen coordinates.
    Internally it calls the DRV_CAMERA_OVM7690_RegisterSet function to set the 
	respective registers. The rectangle coordinates are also maintained in the 
	driver object.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    The SCCB interface also must have been initialized to configure the OVM7690 
	Camera Driver.

  Parameters:
    handle - A valid open instance handle, returned from the driver's Open function
    left   - left frame coordinate
    top    - top frame coordinate
    right  - right frame coordinate
    bottom - bottom frame coordinate 

  Returns:
    * DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    * DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>
        DRV_HANDLE handle;
        uint32_t left   = 0x69;
        uint32_t top    = 0x0E;
        uint32_t right  = DISP_HOR_RESOLUTION + 0x69;
        uint32_t bottom = DISP_VER_RESOLUTION + 0x69;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameRectSet( handle, left, top, right, bottom ) != 
                                             DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    </code>

  Remarks:
    This function is optional if default values are expected to be used.    
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameRectSet
( 
    DRV_HANDLE handle,
    uint32_t   left,
    uint32_t   top,
    uint32_t   right,
    uint32_t   bottom
);

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_HsyncEventHandler(SYS_MODULE_OBJ object)
 
  Summary:
    Horizontal synchronization event handler. 

  Description:
    This function is called when the OVM7690 Camera sends a Horizontal Sync Pulse 
	on the HSYNC line. It sets the next line address in the DMA module.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    object - Driver object handle, returned from the 
             DRV_CAMERA_OVM7690_Initialize function

  Returns:
    None.

  Example:
    <code>
    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        //error
        return;    
    }

    void __ISR( HSYNC_ISR_VECTOR) _Ovm7690HSyncHandler(void)
    {
        DRV_CAMERA_OVM7690_HsyncEventHandler(objectHandle);

        SYS_INT_SourceStatusClear(HSYNC_INTERRUPT_SOURCE);
    }
    </code>

  Remarks:
    This function is mandatory.

 */

void DRV_CAMERA_OVM7690_HsyncEventHandler(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_VsyncEventHandler(SYS_MODULE_OBJ object)
 
  Summary:
    Vertical synchronization event handler .

  Description:
    This function is called when the OVM7690 Camera sends a Vertical Sync Pulse on
    the VSYNC line. It clears the number of lines drawn variable.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize function must have been called for the 
    specified OVM7690 Camera Driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    object - Driver object handle, returned from the 
             DRV_CAMERA_OVM7690_Initialize function

  Returns:
    None.

  Example:
    <code>
    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        //error
        return;    
    }

    void __ISR( VSYNC_ISR_VECTOR) _Ovm7690VSyncHandler(void)
    {
        DRV_CAMERA_OVM7690_VsyncEventHandler(objectHandle);

        SYS_INT_SourceStatusClear(VSYNC_INTERRUPT_SOURCE);
    }
    </code>

  Remarks:
    This function is mandatory.

 */

void DRV_CAMERA_OVM7690_VsyncEventHandler(SYS_MODULE_OBJ object);

/*******************************************************************************
  Function:
    void DRV_CAMERA_OVM7690_Tasks(SYS_MODULE_OBJ object );

  Summary:
    Maintains the OVM7690 state machine.
 */

void DRV_CAMERA_OVM7690_Tasks(SYS_MODULE_OBJ object);

// *****************************************************************************
// *****************************************************************************
// Section: Local Interface Headers for the driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

void _DRV_CAMERA_OVM7690_HardwareSetup( DRV_CAMERA_OVM7690_OBJ * dObj );

void _DRV_CAMERA_OVM7690_DMAEventHandler( SYS_DMA_TRANSFER_EVENT event,
                                          SYS_DMA_CHANNEL_HANDLE handle, 
                                          uintptr_t contextHandle );

void _DRV_CAMERA_OVM7690_delayMS ( unsigned int delayMs );

#ifdef __cplusplus
    }
#endif
    
#endif // #ifndef _DRV_CAMERA_OVM7690_H

/*******************************************************************************
 End of File
*/