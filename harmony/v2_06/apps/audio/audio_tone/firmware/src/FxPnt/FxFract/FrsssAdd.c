//*****************************************************************************
//
// File: FrsssAdd.c
//
// Description: Add (op1 + op2) or subtract (op1 - op2) two 16-bit 
//   2s-complement fractional operands to produce a 16-bit 2s-complement 
//   fractional result with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


//*****************************************************************************
//
// FrwsSat()
//
// Description:
//   Private function used only by this fractional library to  
//   saturate a 32-bit value to 16-bits and returns the result.
//   The lower 16-bits of the 32-bit input are limited to within the
//   range of MINFRACT16 <= In <= MAXFRACT16.  It is assumed that 
//   the binary point in the 16-bit output is aligned with the LOWER 16-bits
//   of the 32 bit input. The function is used for overflow control to limit
//   values before they exceed their range.
//   This function does not produce an overflow status flag.
//
// Arguments:
//   Fract32 a
//     [in]        Input value to be saturated
//
// Return Value:
//   Fract16 result
//     [return]    Saturated value
//
//*****************************************************************************/
static Fract16 FrwsSat(Fract32 a)
{
  Fract16 result;                   /* Value returned */

  /* Take input's bottom 16-bits for output.  Modify this result next
   * only if input exceeds the output range */  
  result = (Fract16)a;
  /* Check for overflow with the 'Sign-Extended' 16-bit range limits. If it 
   * overflows in the positive direction, force to Max positive value.*/
  if (a > (Fract32)(MAXFRACT16)) 
  {
    result = (Fract16)MAXFRACT16;
  }
  /* If it overflows in the negative direction, force to MIN negative value.*/
  if (a < (Fract32)(MINFRACT16))
  {
    result = (Fract16)MINFRACT16;
  } 
  return (result);
}


//*****************************************************************************
//
// frsssAdd()
//
// Description:
//   Add two 16-bit 2s-complement fractional (op1 + op2) to produce a 16-bit
//   2s-complement fractional result with saturation. The saturation is for
//   handling the overflow/underflow cases, where the result is set to
//   MAX16 when an overflow occurs and the result is set to
//   MIN16 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 16-bit inputs and the resulting 16-bit output.  
//
//   This function calls the FrwsSat()
//
// Arguments:
//   Fract16 a
//     [in] Addition operand op1.
//   Fract16 b;
//     [in] Addition operand op2.
//
// Return Value:
//   Fract16 result
//     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
//
//*****************************************************************************
Fract16 FrsssAdd(Fract16 a, Fract16 b)
{
  Fract32  sumExtP;                   /* Value returned */

  /* Sign-Extend inputs to 32-bits before 'adding' so that the addition is
   * done with extended-precision to accomodate for any overflow bits. */
  sumExtP = (Fract32)a + (Fract32)b;
  /* Saturate the 32-bit sum to eliminate any overflow bits so that the 
   * output sum will be within the allowable 16-bit range. */
  return (FrwsSat(sumExtP));
}


/******************************************************************************
 *
 * FrsssSub()
 *
 * Description:
 *   Subtract two 16-bit 2s-complement fractional (op1 - op2) to produce a
 *   16-bit 2s-complement fractional difference result with saturation. The
 *   saturation is for handling the overflow/underflow cases, where the result
 *   is set to MAX16 when an overflow occurs and the result is set to
 *   MIN16 when an underflow occurs.  This function does not produce
 *   any status flag to indicate when an overflow or underflow has occured.
 *   It is assumed that the binary point is in exactly the same bit position
 *   for both 16-bit inputs and the resulting 16-bit output.  
 *   This function relates to the ETSI sub function.
 *
 *   This function calls the FrwsSat()
 *
 * Arguments:
 *   Fract16 a
 *     [in] Subtraction operand op1.
 *   Fract16 b;
 *     [in] Subtraction operand op2.
 *
 * Return Value:
 *   Fract16 result
 *     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
 *
 ******************************************************************************/
Fract16 FrsssSub(Fract16 a, Fract16 b)
{
  Fract32  diffExtP;                   /* Value returned */

  /* Sign-Extend inputs to 32-bits before 'subtracting' so that the subtraction
   * is done with extended-precision to accomodate for any overflow bits. */
  diffExtP = (Fract32)a - (Fract32)b;
  /* Saturate the 32-bit difference to eliminate any overflow bits so that 
   * the output difference will be within the allowable 16-bit range. */
  return (FrwsSat(diffExtP));
}
