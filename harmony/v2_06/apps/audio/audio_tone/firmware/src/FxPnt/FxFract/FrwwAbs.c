//******************************************************************************
// File: FrwwAbs.c
//
// Description: Creates a saturated Absolute value.
//
//******************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


//******************************************************************************
//
// FrwwAbs()
//
// Description:
//   Creates a saturated Absolute value.  It takes the absolute value of the
//   32-bit 2s-complement fractional input with saturation. The saturation is
//   for handling the case where taking the absolute value of ATIMINFRACT32 is
//   greater than ATIMAXFRACT32, or the allowable range of 32-bit values.
//   This function relates to the ETSI L_abs function.
//
// Arguments:
//   AtiFract32 a
//     [in] input argument
//
// Return Value:
//   AtiFract32 result
//     [return]  abs(input) <= ATIMAXFRACT32
//
//*****************************************************************************
Fract32 FrwwAbs(Fract32 a)
{
  Fract32 result;                   /* Value returned */

  /* If input is a positive 2's complement value then,
   * simply take the input 2's complement value */
  if (a >= (Fract32)0)
  {
    result = a;
  }
  /* Else the Input is a Negative value, so check for min value (-1) */
  else
  {   
    /* If not the min value (-1), just negate the input for absolute value */ 
    if (a != (Fract32)MINFRACT32)
    {
      result = (Fract32)(-a);
    }
    /* Else it was the min value (-1), so saturate to max value (1) */
    else
    {
      result = (Fract32)MAXFRACT32;
    }
  }
  return (result);
}
