//*****************************************************************************
//
// File: FrssAbs.c
//
// Description: Creates a saturated Absolute value.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrssAbs()
 *
 * Description:
 *   Creates a saturated Absolute value.  It takes the absolute value of the
 *   16-bit 2s-complement fractional input with saturation. The saturation is
 *   for handling the case where taking the absolute value of MINFRACT16 is
 *   greater than MAXFRACT16, or the allowable range of 16-bit values.
 *   This function relates to the ETSI abs function.
 *
 * Arguments:
 *   Fract16 a
 *     [in] input argument
 *
 * Return Value:
 *   Fract16 result
 *     [return]  abs(input) <= MAXFRACT16
 *
 ******************************************************************************/
Fract16 FrssAbs(Fract16 a)
{
  Fract16 result;                   /* Value returned */

  /* If input is a positive 2's complement value, then 
   * simply take the input 2's complement value */
  if (a >= (Fract16)0)
  {
    result = (Fract16)(a);
  }
  /* Else the Input is a Negative value, so check for min value (-1) */
  else
  {
    /* If not the min value (-1), just negate the input for absolute value */ 
    if (a != (Fract16)MINFRACT16)
    {
      result = (Fract16)(-a);
    }
    /* Else it was the min value (-1), so saturate to max value (1) */
    else
    {
      result = (Fract16)MAXFRACT16;
    }
  }
  return (result);
}
