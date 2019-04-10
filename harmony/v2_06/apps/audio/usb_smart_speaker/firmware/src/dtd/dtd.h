/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    dtd.h

  Summary:
    Double Talk Detector (DTD)

  Description:
    Double Talk Detector (DTD)

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DTD_H
#define _DTD_H

#include <stdbool.h>
#include <stdint.h>
#include "libq_c.h"
#include "libq.h"
#include "dsp.h"

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END


//Exp Avg Filter - Factor used for est of rect. window average (N=256???)
#define POWEREXP256   7    
#define POWEREXP8     3


typedef struct _DtdConfig
{
    int frameLength;  //#samples in frame 
    q31 refThreshQ31;
    q31 echoThreshQ31; 
    q31 micThreshQ31;  
    q15 dtdPathLossQ15; 
    q15 pathLossQ15;
    int refDetectHoldoffFrames;
    int dtDetectHoldoffFrames; 
    int dtdWindowSamples;
} DtdConfig;

typedef struct _Dtd
{
    bool dtDetected;
    bool echoDetected;
    q31  refFrPowerQ1d31;
    q31  micFrPowerQ1d31;
    q31  refFrExp8AvPowerQ1d31;
    q31  micFrExp8AvPowerQ1d31;
    DtdConfig config;
} Dtd;


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
void dtdInit(Dtd * dtd, DtdConfig config);
 

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
 */
void dtdProc(Dtd * dtd, q15 * xQ1d15, q15 * yQ1d15);


//*****************************************************************************
// ExpAveragePower()
//
// Summary:
//   Update the power estimate using a exponential average filter on the current
//   value, based on the formula:
//
//   p(k) = p(k-1)(1-2^-N) + (2^-N)*x(k)^2
//
// Arguments:
//   q31 powerQ1d31 - [in/out] p(k-1) in Q1.31 format
//   q15 xQ1d15 - [in] x(k)) in Q1.16format
//   int16_t powerExp - N
//
// Return Value:
//   p(k) in Q1d31 format
//
// Return Value: //   None
//*****************************************************************************
q31     ExpAvgPower(q31 powerQ1d31, q15 xQ1d15, int16_t lambExp);
q31     ExpAvgPowerN(q31 powerQ1d31, q15 *xQ1d15, int16_t pexp);
int16_t DSP_VectorSumSquares16_32(int16_t *indata, int16_t N, int16_t scale);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

