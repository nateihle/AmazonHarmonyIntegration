//*****************************************************************************
// File: frssssExpAvg.c
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"
#include "FxMath.h"

//*****************************************************************************
//
// FrssssExpAvg()
//
// Description:
//   Exponential averaging implements a smoothing function based on the form:
//       avg[i+1] = avg[i] * lamda + new * (1-lamda)
//   In this implementation, is has been optimized as follows.
//       avg[i+1] = (avg[i] - new) * lamda + new
//
//   The optimization precludes accurate processing of new numbers that differ
//   from the current average by more than unity. If the difference is greater
//   than unity or less than negative unity, the difference is saturated.
//
//   The effect is akin to a smaller lambda, e.g., the new value will have a
//   greater weight than expected. If the smoothing is of data that is entirely
//   positive or entirely negative, then the saturation will not be an issue.
//
// Arguments:
//   FxPnt16 prevAvg16
//     [in] Previous exponential average
//   FxPnt16 newMeas16
//     [in] New value to be averaged in
//   Fract16 lamdaQ1d15
//     [in] exponential averaging constant
//
// Return Value:
//   FxPnt16 newAvg16
//
//*****************************************************************************/
FxPnt16 FrssssExpAvg(FxPnt16 prevAvg16,
                     FxPnt16 newMeas16,
                     Fract16 lamdaQ1d15)
{
  FxPnt16 newAvg16;   /* The new average to be returned. */

  /* Do exponential average */
  /* avg[i+1] = (avg[i] - new) * lamda + new */
  newAvg16 = FrsssSub(prevAvg16, newMeas16);
  newAvg16 = FrsssMultR(lamdaQ1d15, newAvg16);
  newAvg16 = FrsssAdd(newAvg16, newMeas16);

  /* Return the product */
  return(newAvg16);

}
