/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    dtd.c

  Summary:
    Double Talk Detector (DTD)

  Description:
    Double Talk Detector (DTD)
   
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#include "dtd.h"
#include "system_config.h"
#include "system/debug/sys_debug.h"

static int _refDetectHoldoff = 0;
static int _dtDetectHoldoff = 0;
static q31 _xFrPowerQ1d31 = 0;
static q31 _yFrPowerQ1d31 = 0;

DtdConfig * config;

/*******************************************************************************
 * Functions: dtdProc
 *
 * Summary:
 *   Detects near speech activity occurring at the same time as loudspeaker
 *   echo.
 *
 * Description:
 *   Detects near speech activity occurring at the same time as loudspeaker
 *   echo.
 *
 * Parameters:
 *   Dtd * dtd - Double talke detections object pointer 
 *
 * Returns:
 *   None
 */
void dtdInit(Dtd * dtd, DtdConfig config)
{
    dtd->dtDetected = false;
    dtd->echoDetected = false;
    dtd->refFrPowerQ1d31 = 0;
    dtd->micFrPowerQ1d31 = 0;
    dtd->refFrExp8AvPowerQ1d31 = 0;
    dtd->micFrExp8AvPowerQ1d31 = 0;

    dtd->config.frameLength            = config.frameLength;
    dtd->config.pathLossQ15            = config.pathLossQ15; 
    dtd->config.refThreshQ31           = config.refThreshQ31;
    dtd->config.echoThreshQ31          = config.echoThreshQ31; 
    dtd->config.micThreshQ31           = config.micThreshQ31;  
    dtd->config.refDetectHoldoffFrames = config.refDetectHoldoffFrames;
    dtd->config.dtDetectHoldoffFrames  = config.dtDetectHoldoffFrames; 
    dtd->config.dtdWindowSamples       = config.dtdWindowSamples;
}
 
/*******************************************************************************
 * Functions: dtdProc
 *
 * Summary:
 *   Detects near speech activity occurring at the same time as loudspeaker
 *   echo.
 *
 * Description:
 *   Detects near speech activity occurring at the same time as loudspeaker
 *   echo.
 *
 * Parameters:
 *   Dtd * dtd - Double talke detections object pointer 
 *   refInQ1d15 *  - echo reference signal value
 *   micInQ1d15 *  - echo + near + noise signal value
 *
 * Returns:
 *   None
 */

void dtdProc(Dtd * dtd, q15 * refInQ1d15, q15 * micInQ1d15)
{

    config = &(dtd->config);

    // TODO:  Coherence DTD
    // Geigel algorithm (N is FIR length)
    //|s(i)| = |y(i) + echo(i)=h*x(i)|  ?  c * max{|x(i)|, |x(i-1)|, ?, |x(i-N)|},
    // i.e  the input signal is > the echo reference signal reduced by the
    // path loss (.5 is -6dB).  NOTE:  needs a larger frame than 8.

    //Short term Exp Window Average Power for frame
    int kk;
    _xFrPowerQ1d31 = dtd->refFrPowerQ1d31; 
    _yFrPowerQ1d31 = dtd->micFrPowerQ1d31; 
    for (kk=0; kk<dtd->config.frameLength; kk++) 
    {
        _xFrPowerQ1d31 = ExpAvgPower(_xFrPowerQ1d31, refInQ1d15[kk], POWEREXP8);
        _yFrPowerQ1d31 = ExpAvgPower(_yFrPowerQ1d31, micInQ1d15[kk], POWEREXP8);
    }
    dtd->refFrPowerQ1d31 = _xFrPowerQ1d31;
    dtd->micFrPowerQ1d31 = _yFrPowerQ1d31;

    //Long term Exp Window Average Power (256 Frames (256 ms)
    dtd->refFrExp8AvPowerQ1d31 = ExpAvgPower(
            dtd->refFrExp8AvPowerQ1d31, _xFrPowerQ1d31 , POWEREXP256);

    dtd->micFrExp8AvPowerQ1d31 = ExpAvgPower(
            dtd->micFrExp8AvPowerQ1d31, _yFrPowerQ1d31, POWEREXP256);

    dtd->dtDetected   = false;
    dtd->echoDetected = false;

    if ((dtd->refFrPowerQ1d31 > config->refThreshQ31) &&
        (dtd->echoDetected == false))
    {
        _refDetectHoldoff = config->refDetectHoldoffFrames;
        dtd->echoDetected = true;
    }
    else if (dtd->echoDetected == false && _refDetectHoldoff > 0 )
    {
        if (_refDetectHoldoff>0) _refDetectHoldoff--;
    }
    else
    {
        dtd->echoDetected = false;
    }

    //TODO:  Implement DTD, Coherance or simple Geigel
    if ((dtd->micFrPowerQ1d31 > dtd->micFrExp8AvPowerQ1d31) &&
        (dtd->micFrPowerQ1d31 > config->micThreshQ31) &&
        (dtd->dtDetected == false))
    {
        _dtDetectHoldoff = config->dtDetectHoldoffFrames;
        dtd->dtDetected = true;
    }
    else if (dtd->dtDetected == false && _dtDetectHoldoff > 0 )
    {
        if (_dtDetectHoldoff>0) _dtDetectHoldoff--;
    }
    else
    {
        dtd->dtDetected = false;
    }
} //End dtdProc())

/*******************************************************************************
 End of File
 */