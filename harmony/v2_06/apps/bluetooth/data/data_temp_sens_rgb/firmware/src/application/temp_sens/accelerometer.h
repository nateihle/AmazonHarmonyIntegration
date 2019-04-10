/*******************************************************************************
  Accelerometer Library Interface

  Company:
    Microchip Technology Inc.

  File Name:
    accelerometer.h

  Summary:
    Contains the Accelerometer Interface specific defintions and function
    prototypes.

  Description:
    This file contains the Accelerometer Interface specific defintions and 
    function prototypes.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _ACCELEROMETER_H_FILE
#define _ACCELEROMETER_H_FILE

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

#include "app.h"
#define ACCEL_BMA250_I2C_MODULE             I2C_ID_1
#define ACCEL_BMA250_I2C_VECTOR_LOCATION    _I2C_1_VECTOR
#define ACCEL_BMA250_I2C_SOURCE             INT_SOURCE_I2C_1_MASTER
#define ACCEL_BMA250_I2C_VECTOR             INT_VECTOR_I2C1
/********************************************************************
 Section: Private Macros
********************************************************************/
#define     ACCEL_BMA250_WRITE_ADDR     (0x30)
#define     ACCEL_BMA250_READ_ADDR      (0x31)

#define     ACCEL_BMA250_CHIP_ID_ADDR   (0x00)
#define     ACCEL_BMA250_VERSION_ADDR   (0x01)
#define     ACCEL_BMA250_ACC_X_LSB_ADDR (0x02)
#define     ACCEL_BMA250_ACC_X_MSB_ADDR (0x03)
#define     ACCEL_BMA250_ACC_Y_LSB_ADDR (0x04)
#define     ACCEL_BMA250_ACC_Y_MSB_ADDR (0x05)
#define     ACCEL_BMA250_ACC_Z_LSB_ADDR (0x06)
#define     ACCEL_BMA250_ACC_Z_MSB_ADDR (0x07)
#define     ACCEL_BMA250_TEMP           (0x08)
#define     ACCEL_BMA250_FIFO_SET_ADDR  (0x3e)
#define     ACCEL_BMA250_FIFO_DATA_ADDR (0x3f)

#define     ACCEL_BMA250_ADDR15         (0x15)
#define     ACCEL_BMA250_ADDR14         (0x14)
#define     ACCEL_BMA250_ADDR13         (0x13)
#define     ACCEL_BMA250_ADDR12         (0x12)
#define     ACCEL_BMA250_ADDR11         (0x11)
#define     ACCEL_BMA250_ADDR10         (0x10)
#define     ACCEL_BMA250_ADDR0F         (0x0F)
#define     ACCEL_BMA250_ADDR0E         (0x0E)
#define     ACCEL_BMA250_ADDR0D         (0x0D)
#define     ACCEL_BMA250_ADDR0C         (0x0C)
#define     ACCEL_BMA250_ADDR0B         (0x0B)
#define     ACCEL_BMA250_ADDR0A         (0x0A)
#define     ACCEL_BMA250_ADDR09         (0x09)

#define     ACCEL_BMA250_CHIP_ID        (0xF9)
#define     ACCEL_BMA250_THES           (0x00)

#define     ACCEL_BMA250_RANGE_2G       (3)
#define     ACCEL_BMA250_RANGE_4G       (5)
#define     ACCEL_BMA250_RANGE_8G       (8)
#define     ACCEL_BMA250_RANGE_16G      (12)

#define     ACCEL_BMA250_BW_25          (0x00)
#define     ACCEL_BMA250_BW_50          (0x01)
#define     ACCEL_BMA250_BW_100         (0x02)
#define     ACCEL_BMA250_BW_190         (0x03)
#define     ACCEL_BMA250_BW_375         (0x04)
#define     ACCEL_BMA250_BW_750         (0x05)
#define     ACCEL_BMA250_BW_1500        (0x06)

#define    ACCEL_BMA250_FIFO_MODE_BYPASS (0x00)
#define    ACCEL_BMA250_FIFO_MODE_FIFO   (0x01)
#define    ACCEL_BMA250_FIFO_MODE_STREAM (0x02)

#define    ACCEL_BMA250_FIFO_DATA_SELECT_XYZ (0x00 << 6)
#define    ACCEL_BMA250_FIFO_DATA_SELECT_X   (0x01 << 6)
#define    ACCEL_BMA250_FIFO_DATA_SELECT_Y   (0x02 << 6)
#define    ACCEL_BMA250_FIFO_DATA_SELECT_Z   (0x03 << 6)


#define     ACCEL_CMD_START             0x00
#define     ACCEL_CMD_RESTART           0x01
#define     ACCEL_CMD_STOP              0x02
#define     ACCEL_CMD_TX_BYTE           0x03
#define     ACCEL_CMD_RX_BYTE           0x04
#define     ACCEL_CMD_ACK               0x05
#define     ACCEL_CMD_EN_RX             0x06
#define     ACCEL_CMD_ACK_POLL          0x07
#define     ACCEL_DONE                  0xFF
#define     ACCEL_INVALID_IDX           0xFFFFFFFF

#define     ACCEL_BMA250_AXIS_BIT       (10)
void AccelInit(void);
/********************************************************************
 Section: Private structures
********************************************************************/
typedef struct
{
    uint8_t data;
    uint8_t cmd;
}ACCEL_CMD_DATA;

typedef union
{
    //Address 0x00
    struct
    {
        uint8_t chip_id :8;
    } ;

    //Address 0x01
    struct
    {
        uint8_t ml_version :4;
        uint8_t al_version :4;
    };

    //Address 0x02
    struct
    {
        uint8_t new_data_x :1;
        uint8_t :5;
        uint8_t acc_x :2;
    } ;

    //Address 0x03
    struct
    {
        uint8_t acc_x_MSB :8;
    };

    //Address 0x04
    struct
    {
        uint8_t new_data_y :1;
        uint8_t :5;
        uint8_t acc_y :2;
    } __attribute__((packed));

    //Address 0x05
    struct
    {
        uint8_t acc_y_MSB :8;
    };

    //Address 0x06
    struct
    {
        uint8_t new_data_z :1;
        uint8_t :5;
        uint8_t acc_z :2;
    };

    //Address 0x07
    struct
    {
        uint8_t acc_z_MSB :8;
    };

    //Address 0x08
    struct
    {
        int acc_Temp :8;
    };

    //Address 0x14
    struct
    {
        uint8_t bandwidth :3;
        uint8_t range :2;
		uint8_t  :3;
    } ;

    uint8_t val;
} ACCEL_BMA250_REG;

typedef struct
{
    ACCEL_BMA250_REG    acc_x_lsb;
    ACCEL_BMA250_REG    acc_x_msb;
    ACCEL_BMA250_REG    acc_y_lsb;
    ACCEL_BMA250_REG    acc_y_msb;
    ACCEL_BMA250_REG    acc_z_lsb;
    ACCEL_BMA250_REG    acc_z_msb;
}ACCEL_X_Y_Z_DATA;

typedef struct
{
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t z_lsb;
    uint8_t z_msb;
}ACCEL_X_Y_Z_BUF;
typedef enum
{
    ACCEL_FALSE = 0,
    ACCEL_TRUE
}ACCEL_BOOL;
typedef enum
{
    ACCEL_PRIORIY,
    EEPROM_PRIOFITY,
    FREE_PRIORITY
}I2C_PRIORITY;

/*********************************************************
  Summary:
    Acceleration device return result.

  Description:
    Acceleration device return result
  ********************************************************/
typedef enum
{
    /* Acceleration routine result invalid. */
    ACCEL_INVALID,
    /* Acceleration routine result valid. */
    ACCEL_VALID         
}ACCEL_RESULT;

typedef enum
{
    FAHRENHEIT,
    CELSIUS,
    KELVIN, /*K=C+273.15*/
    RAW_DATA
}RETURN_TEMP;
/*************************************************
  Summary:
    Accleration initialization device structure.

  Description:
    Accleration initialization device structure.
  ************************************************/
#ifndef ACCEL_INIT
typedef union
{
    struct
    {
        /* System clock rate in Hertz */
        uint32_t sourceClock;
        /* data clock rate in Hertz */
        uint32_t dataRate;
    };
}ACCEL_INIT;
#endif

/***************************************************
  Summary:
    Accleration data type.

  Description:
    Accleration data type.
  **************************************************/
typedef int8_t ACCEL_DATA;  // Acceleration data type.

/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELInitialize(ACCEL_INIT *initialization)

  Summary:
    Initializes the Accelerometer's I/O and communication interface.

  Description:
    This routine configures the I/O (i.e., chip select) and communication 
    interface to the external Accelerometer.

  Precondition:
    None.

  Parameters:
    initialization    - pointer to the device specific initialization structure.

  Returns:
    * ACCEL_INVALID - Initialization was not complete properly
    * ACCEL_VALID - Initalization was completed

  Example:
    <code>
    ACCEL_INIT init;

    init.sourceClock = SOUCE_CLK_80MHz;
    init.dataRate    = ACCEL_DATARATE_25MHz;

    // Initialize the Accelerometer using 80MHz source clock
    if(ACCELEROMETERInit(&init) == ACCEL_INVALID)
    {
        // handle initialization error
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELInitialize(ACCEL_INIT *initialization, uint8_t range);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELGetXYZAxis(ACCEL_DATA *acc_x, ACCEL_DATA *acc_y, ACCEL_DATA *acc_z)

  Summary:
    Get the 3-axis (X-Y-Z) acceleration.

  Description:
    This routine gets the 3-axis accerelation.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    acc_x           - the X-axis acceration
    acc_y           - the Y-axis acceration
    acc_z           - the Z-axis acceration

  Returns:
    * ACCEL_VALID - if the acceration data is valid
    * ACCEL_INVALID - if the acceration data is invalid

  Example:
    <code>
    ACCEL_DATA  acc_x, acc_y, acc_z;

    // Get the 3-axis acceration
    if(ACCELGetXYZAxis(&acc_x, &acc_y, &acc_z) == ACCEL_VALID)
    {
        // valid acceration data
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELGetXYZAxis(ACCEL_DATA *acc_x, ACCEL_DATA *acc_y, ACCEL_DATA *acc_z);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELGetXYAxis(ACCEL_DATA *acc_x, ACCEL_DATA *acc_y)

  Summary:
    Get the 2-axis (X-Y) acceleration.

  Description:
    This routine gets the 2-axis accerelation.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    acc_x           - the X-axis acceration
    acc_y           - the Y-axis acceration

  Returns:
    * ACCEL_VALID - if the acceration data is valid
    * ACCEL_INVALID - if the acceration data is invalid

  Example:
    <code>
    ACCEL_DATA  acc_x, acc_y;

    // Get the 3-axis acceration
    if(ACCELGetXYAxis(&acc_x, &acc_y) == ACCEL_VALID)
    {
        // valid acceration data
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELGetXYAxis(ACCEL_DATA *acc_x, ACCEL_DATA *acc_y);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELGetXAxis(ACCEL_DATA *acc_x)

  Summary:
    Get the X-axis acceleration.

  Description:
    This routine gets the X-axis accerelation.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    acc_x           - the X-axis acceration

  Returns:
    * ACCEL_VALID - if the acceration data is valid
    * ACCEL_INVALID - if the acceration data is invalid

  Example:
    <code>
    ACCEL_DATA  acc_x;

    // Get the X-axis acceration
    if(ACCELGetXAxis(&acc_x) == ACCEL_VALID)
    {
        // valid acceration data
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELGetXAxis(ACCEL_DATA *acc_x);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELGetYAxis(ACCEL_DATA *acc_y)

  Summary:
    Get the Y-axis acceleration.

  Description:
    This routine gets the Y-axis accerelation.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    acc_y           - the Y-axis acceration

  Returns:
    * ACCEL_VALID - if the acceration data is valid
    * ACCEL_INVALID - if the acceration data is invalid

  Example:
    <code>
    ACCEL_DATA  acc_y;

    // Get the Y-axis acceration
    if(ACCELGetYAxis(&acc_y) == ACCEL_VALID)
    {
        // valid acceration data
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELGetYAxis(ACCEL_DATA *acc_y);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELGetZAxis(ACCEL_DATA *acc_z)

  Summary:
    Get the Z-axis acceleration.

  Description:
    This routine gets the Z-axis accerelation.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    acc_z           - the Z-axis acceration

  Returns:
    * ACCEL_VALID - if the acceration data is valid
    * ACCEL_INVALID - if the acceration data is invalid

  Example:
    <code>
    ACCEL_DATA  acc_z;

    // Get the Z-axis acceration
    if(ACCELGetZAxis(&acc_z) == ACCEL_VALID)
    {
        // valid acceration data
    }
    </code>

  Remarks:
    None
  *****************************************************************************/
ACCEL_RESULT ACCELGetZAxis(ACCEL_DATA *acc_z);


/*******************************************************************************
  Function:
    ACCEL_RESULT ACCELTask(void)

  Summary:
    Handle the Accelerometer tasks.

  Description:
    This routine handles the Accelerometer tasks.

  Precondition:
    The accelerometer needs to be properly initialized by calling ACCELInitialize.

  Parameters:
    None.

  Returns:
    * ACCEL_VALID - the accelerometer is being accessed
    * ACCEL_INVALID - the accelerometer is not being accessed.

  Example:
    <code>
    if(ACCELGetZAxis(&acc_z) == ACCEL_INVALID)
    {
        // do something on a shared bus
    }
    </code>

  Remarks:
    This routine can be used to determine to the communication bus is free.  It
    can also be used for polling.
  *****************************************************************************/
ACCEL_RESULT ACCELTask(void);
ACCEL_RESULT  ReadTemp(RETURN_TEMP ReturnType);

extern volatile uint8_t I2CPriority;


#ifdef __cplusplus
}
#endif

#endif /* _ACCELEROMETER_H_FILE */
/*******************************************************************************
 End of File
*/
