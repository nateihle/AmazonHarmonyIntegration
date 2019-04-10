/*******************************************************************************
  MPLAB Harmony Application

  Company:
    Microchip Technology Inc.

  File Name:
    volume_btadk.c

  Summary:
    A volume control interface for bluetooth audio development kit.

  Description:
    A volume control interface for bluetooth audio development kit.
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

#include "volume.h"
#include "system_definitions.h"
#include "../src/audio_codec/audio_codec.h"
#define MAKE_VOLUME(reading)    (reading * 255 / 1023) + (reading % 1023 >= 511 ? 1 : 0)

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************


/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

VOL_DATA volData =
{
    //TODO - Initialize appData structure.

};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/


VOL_DRV_OBJECTS volDrvObject;

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
    void APP_VolumeInitialize ( void )

  Remarks:
    See prototype in app.h.
 */
static int mVolume;
static int mNewVolume;

void APP_VolumeInitialize ( void )
{
      SYS_PORTS_PinModeSelect(PORTS_ID_0,PORTS_ANALOG_PIN_11,PORTS_PIN_MODE_ANALOG);

    // AVdd, AVss, MUX A only, scan mode, 2 channels
    PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS);
    PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA);
    PLIB_ADC_ResultBufferModeSelect(ADC_ID_1, ADC_BUFFER_MODE_ONE_16WORD_BUFFER);
    PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_2SAMPLES_PER_INTERRUPT);
    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);

    // 31 Tad auto-sample, Tad = 6*Tcy
    PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 0x1F);
    PLIB_ADC_ConversionClockSet(ADC_ID_1,SYS_CLK_BUS_PERIPHERAL_1, 5);

    // Set MUX A negative input
    PLIB_ADC_MuxChannel0InputNegativeSelect(ADC_ID_1,ADC_MUX_A,ADC_INPUT_NEGATIVE_VREF_MINUS);

    PLIB_ADC_InputScanMaskAdd(ADC_ID_1, ADC_INPUT_SCAN_AN11);

    // Turn on, auto sampling, auto convert
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
    PLIB_ADC_Enable(ADC_ID_1);

    // Set volume
    mVolume = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0);
    mVolume = MAKE_VOLUME(mVolume);
    mNewVolume = mVolume;
    /* Place the App state machine in its initial state. */
    volData.state = VOL_STATE_INIT;
}




/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_VolumeTasks ( void )
{
//    int i;
    uint8_t resultBufIndex = 0;

    /* check the application state*/
    switch ( volData.state )
    {
        /* Application's initial state. */
        case VOL_STATE_INIT:
            /* Enable ADC */
            DRV_ADC_Open();

            volData.state = VOL_STATE_START_CAPTURE;
            break;

        case VOL_STATE_START_CAPTURE:
            DRV_ADC_Start();

            volData.state = VOL_STATE_WAIT_FOR_ADC;
            break;

        case VOL_STATE_WAIT_FOR_ADC:
            if (DRV_ADC_SamplesAvailable())
                volData.state = VOL_STATE_OUTPUT_RESULT;
            break;

        case VOL_STATE_OUTPUT_RESULT:
            volData.potValue = DRV_ADC_SamplesRead(resultBufIndex);

             mNewVolume = MAKE_VOLUME(volData.potValue );
             if(abs(mNewVolume - mVolume) < 5)
                 return;
         
            volData.potValue >>=7; /* 10-bit value to 3-bit value */
            mVolume = mNewVolume;
            if(CodecData.codecClient.handle!=DRV_HANDLE_INVALID)
            {
                DRV_CODEC_VolumeSet(CodecData.codecClient.handle,DRV_CODEC_CHANNEL_LEFT_RIGHT,mVolume);
                APP_PlayerEventHandler(PLAYER_EVENT_VOLUME_CHANGE, mVolume);
                
            }
            volData.state = VOL_STATE_START_CAPTURE;
            break;

        /* The default state should never be executed. */
        default:
            break;
	}
}


/*******************************************************************************
 End of File
 */