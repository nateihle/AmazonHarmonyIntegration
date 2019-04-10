/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    calibrate.c

  Summary:
    This file contains the source code for the MPLAB Harmony application calibration.

  Description:
    This file contains the source code for the MPLAB Harmony application calibration.  It 
    implements the logic of the application's calibration state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 released Microchip Technology Inc.  All rights reserved.

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

#include "calibrate.h"
#include "system_definitions.h"
#include "sst25vf016.h"
#include "drv_nvm_flash_sqi_sst26.h"
#include "driver/cpld/xc2c64a/drv_xc2c64a.h"
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define SAMPLE_POINTS   4

// defines addresses for storage of calibration and version values in SPI Flash on MX
#define SST25_CAL_VERSION	(unsigned long)0xFFFFFFFE
#define SST25_CAL_ULX       (unsigned long)0xFFFFFFFC
#define SST25_CAL_ULY       (unsigned long)0xFFFFFFFA
#define SST25_CAL_URX       (unsigned long)0xFFFFFFF8
#define SST25_CAL_URY       (unsigned long)0xFFFFFFF6
#define SST25_CAL_LLX       (unsigned long)0xFFFFFFF4
#define SST25_CAL_LLY       (unsigned long)0xFFFFFFF2
#define SST25_CAL_LRX       (unsigned long)0xFFFFFFF0
#define SST25_CAL_LRY       (unsigned long)0xFFFFFFEE

#define CALIBRATIONINSET   20       // range 0 <= CALIBRATIONINSET <= 40 
const uint16_t Version = 0xF110 | CALIBRATIONINSET;
/*
 User start calibration input button
 */

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

APP_CALIBRATION_DATA appCalData;
laImageWidget* imageWidget[SAMPLE_POINTS];
laLabelWidget* labelWidget[SAMPLE_POINTS];
static char * location[SAMPLE_POINTS] = { "UL", "UR", "LR", "LL" };

short xRawTouch[SAMPLE_POINTS]; // = { 0x00C0, 0x037C, 0x035A, 0x00B9 };
short yRawTouch[SAMPLE_POINTS]; // = { 0x00D4, 0x00D7, 0x0333, 0x0350 };

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
//bool APP_CalibrationCheck(void);
void _app_CalibrationStore(void);
void _app_CalibrationLoad(void);

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


void APP_CalibrationInitialize( void )
{    
#if defined (MEB_BOARD)
    SST25_CS_LAT = 1;
    SST25_CS_TRIS = 0;
    CPLDInitialize();
    CPLDSetGraphicsConfiguration(CPLD_GFX_CONFIG_16BIT);
    CPLDSetSPIFlashConfiguration(SPI_FLASH_CHANNEL);
    SST25Init();
#elif defined (GFX_PICTAIL_V3) || defined (GFX_PICTAIL_V3E) || defined(GFX_PICTAIL_LCC)
    SST25_CS_LAT = 1;
    SST25_CS_TRIS = 0;
    SST25Init();
#endif
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Calibration_Tasks ( void )

  Remarks:
    See prototype in calibration.h.
 */

bool APP_Calibration_Tasks ( void )
{    
    static short counter = 0;
    bool status = false;
    char charBuff[16] = {0};
    laString str;
    static short xpos = 0;
    static short ypos = 0;
    
    /* Check the application's current state. */
    switch ( appCalData.state )
    {
        
        /* Application's initial state. */
        case APP_CAL_STATE_INIT:
        {
            imageWidget[0] = ImageWidget1; imageWidget[1] = ImageWidget2;
            imageWidget[2] = ImageWidget3; imageWidget[3] = ImageWidget4;
            labelWidget[0] = LabelWidget5; labelWidget[1] = LabelWidget6;
            labelWidget[2] = LabelWidget7; labelWidget[3] = LabelWidget8;

            counter=0;
            
            if ((DRV_TOUCH_ADC_TouchGetRawX() == -1) && (DRV_TOUCH_ADC_TouchGetRawY() == -1))
            { 
                laWidget_SetVisible((laWidget*)imageWidget[0], true);
                appCalData.state = APP_CAL_WAIT_FOR_PRESS;
            }
            break;
        }
                
        case APP_CAL_WAIT_FOR_PRESS:
        {
            if ((DRV_TOUCH_ADC_TouchGetRawX() != -1) && (DRV_TOUCH_ADC_TouchGetRawY() != -1))
            {
                xpos = DRV_TOUCH_ADC_TouchGetRawX();
                ypos = DRV_TOUCH_ADC_TouchGetRawY();
                appCalData.state = APP_CAL_WAIT_FOR_RELEASE;
            }
            
            break;
        }
              
        case APP_CAL_WAIT_FOR_RELEASE:
        {
            if ((DRV_TOUCH_ADC_TouchGetRawX() == -1) && (DRV_TOUCH_ADC_TouchGetRawY() == -1))
            {    
                if ( counter < SAMPLE_POINTS )
                {
                    #ifdef TOUCHSCREEN_RESISTIVE_FLIP_Y
                    yRawTouch[3 - counter] = ypos;
                    #else
                    yRawTouch[counter] = ypos;
                    #endif

                    #ifdef TOUCHSCREEN_RESISTIVE_FLIP_X
                    xRawTouch[3 - counter] = xpos;
                    #else
                    xRawTouch[counter] = xpos;
                    #endif

                    laImageWidget_SetImage((laImageWidget*)imageWidget[counter], (GFXU_ImageAsset*)&green_crosshair);
            
                    sprintf(charBuff, "%s: (%04X, %04X)", location[counter], xpos, ypos);

                    str = laString_CreateFromCharBuffer(charBuff, 
                        GFXU_StringFontIndexLookup(&stringTable, string_Results, 0));

                    laLabelWidget_SetText(labelWidget[counter], str);

                    laString_Destroy(&str);
                        
                    counter++;
                    
                    if ( counter < SAMPLE_POINTS) 
                    {
                        laWidget_SetVisible((laWidget*)imageWidget[counter], true);
                        laImageWidget_SetImage((laImageWidget*)imageWidget[counter-1], (GFXU_ImageAsset*)&green_crosshair);
                    } 

                    appCalData.state = APP_CAL_WAIT_FOR_PRESS;
                }
                if ( counter == SAMPLE_POINTS )
                {
                     appCalData.state = APP_CAL_LOAD;
                     
                }
            }
            break;
        }
        
        case APP_CAL_LOAD:
        {

            DRV_TOUCH_ADC_CoefficientSet(xRawTouch, yRawTouch);
            
            laWidget_SetVisible((laWidget*)Continue, true);

            appCalData.state = APP_CAL_SAVE;
            
            break;
        }
        
        case APP_CAL_SAVE:
        {
            _app_CalibrationStore();
            appCalData.state = APP_CAL_DONE;
        }
     
        case APP_CAL_DONE:
        {
            // there was a touch for 1s now wait for release

            xpos = DRV_TOUCH_ADC_TouchGetRawX();
            ypos = DRV_TOUCH_ADC_TouchGetRawY();

            if ( (ypos != -1) && (xpos != -1) )
            {
                // return true (we are done)
                status = true; 
                laWidget_SetVisible((laWidget*)Continue, false);
                appCalData.state = APP_CAL_STATE_INIT;
            }
            
            break;
        }
                
        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    
    return status;
}

void _app_CalibrationLoad(void)
{
#if defined (GFX_PICTAIL_V3)                || \
		  defined (GFX_PICTAIL_V3E)             || \
          defined (GFX_PICTAIL_LCC)             || \
	      defined (MEB_BOARD) 
    // read from 
    xRawTouch[0] = SST25ReadWord(SST25_CAL_ULX);
    yRawTouch[0] = SST25ReadWord(SST25_CAL_ULY);

    xRawTouch[1] = SST25ReadWord(SST25_CAL_URX);
    yRawTouch[1] = SST25ReadWord(SST25_CAL_URY);

    xRawTouch[3] = SST25ReadWord(SST25_CAL_LLX);
    yRawTouch[3] = SST25ReadWord(SST25_CAL_LLY);

    xRawTouch[2] = SST25ReadWord(SST25_CAL_LRX);
    yRawTouch[2] = SST25ReadWord(SST25_CAL_LRY);

    // set them in ADC driver
    DRV_TOUCH_ADC_CoefficientSet(xRawTouch, yRawTouch);
#endif
}

void _app_CalibrationStore(void)
{
#if defined (GFX_PICTAIL_V3)                || \
		  defined (GFX_PICTAIL_V3E)             || \
          defined (GFX_PICTAIL_LCC)             || \
	      defined (MEB_BOARD)     // the upper left X sample address is used since it is the first one
    // and this assumes that all stored values are located in one 
    // sector
    SST25SectorErase(SST25_CAL_ULX);

    SST25WriteWord(xRawTouch[0], SST25_CAL_ULX);
    SST25WriteWord(yRawTouch[0], SST25_CAL_ULY);

    SST25WriteWord(xRawTouch[1], SST25_CAL_URX);
    SST25WriteWord(yRawTouch[1], SST25_CAL_URY);

    SST25WriteWord(xRawTouch[3], SST25_CAL_LLX);
    SST25WriteWord(yRawTouch[3], SST25_CAL_LLY);

    SST25WriteWord(xRawTouch[2], SST25_CAL_LRX);
    SST25WriteWord(yRawTouch[2], SST25_CAL_LRY);

    SST25WriteWord(Version, SST25_CAL_VERSION);
#endif
}


bool APP_CalibrationLoadFromFlash(void)
{
#if defined (GFX_PICTAIL_V3)                || \
		  defined (GFX_PICTAIL_V3E)             || \
          defined (GFX_PICTAIL_LCC)             || \
	      defined (MEB_BOARD) 
    // check for stored calibration data
    if (SST25ReadWord(SST25_CAL_VERSION) != Version)
    {
        return false;
    }    
    _app_CalibrationLoad();
    
    return true;
#else
    return false;
#endif
}

bool APP_CalibrationLoadFromSram(void)
{
    if ( xRawTouch[0] == 0x0 ) return false;
    
    // set them in ADC driver
    DRV_TOUCH_ADC_CoefficientSet(xRawTouch, yRawTouch);
    
    return true;
}

bool APP_CalibrationLoadFromUser(void)
{
	uint16_t count;
              
	// check for calibration
	// this tests any touches on the touch screen, user has to touch the screen for more than 1
	// second to make the calibration work 
	count = 0;
	while(1)
	{
		SYS_TMR_DelayMS(100);
		// check if there is a touch
		if ((DRV_TOUCH_ADC_TouchGetRawX() == -1) && (DRV_TOUCH_ADC_TouchGetRawY() == -1))
			return false;
		else
			count++;
		if (count == 10)
		{
            laWidget_SetVisible((laWidget*)imageWidget[0], true);
            appCalData.state = APP_CAL_STATE_INIT;                   
            return true;
  		}      	
 	}  		
}

    
/*******************************************************************************
 End of File
 */
