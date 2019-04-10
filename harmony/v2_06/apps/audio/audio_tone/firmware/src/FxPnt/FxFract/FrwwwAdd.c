//*****************************************************************************
//
// File: FrwwwAdd.c
//
// Description: Add two 32-bit 2s-complement fractional (op1 + op2) to produce 
//   a 32-bit 2s-complement fractional result with saturation.
//
//*****************************************************************************/

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwwwAdd()
 *
 * Description:
 *   Add two 32-bit 2s-complement fractional (op1 + op2) to produce a 32-bit
 *   2s-complement fractional result with saturation. The saturation is for
 *   handling the overflow/underflow cases, where the result is set to
 *   ATIMAX32 when an overflow occurs and the result is set to
 *   ATIMIN32 when an underflow occurs.  This function does not produce
 *   any status flag to indicate when an overflow or underflow has occured.
 *   It is assumed that the binary point is in exactly the same bit position
 *   for both 32-bit inputs and the resulting 32-bit output.  
 *   This function relates to the ETSI L_add function.
 *
 * Arguments:
 *   Fract32 a
 *     [in] Addition operand op1.
 *   Fract32 b;
 *     [in] Addition operand op2.
 *
 * Return Value:
 *   Fract32 result
 *     [return]  Range: MINFRACT32 <= result <= MAXFRACT32
 *
 ******************************************************************************/
Fract32 FrwwwAdd(Fract32 a, Fract32 b)
{
  Fract32 result;                   /* Value returned */

  /* Add Operands to get default sum. Set overflow/underflow
   * values later if need be, otherwise this is correct. */
  result = a + b;
  /* Check to see if both 32-bit operands have the same sign.
   * Exclusive OR the operands then AND with the 32-bit sign bit flag
   * so that the answer will be zero if they were the same sign. */
  if (((a ^ b) & MSBBITFRACT32) == 0)
  {
    /* If operand a & b have the same sign bit, check the sign of
     * the result. If the sign of the result is different from both
     * the operands than an overflow or underflow has occurred. */
    if (((result ^ a) & MSBBITFRACT32) != 0)
    {
      /* If the operands where both negative, then an underflow has
       * occurred, so set the result to the most Negative number,
       * ATIMINFRACT32.  If the operands where both positive, than
       * an overflow has occurred, so set the result to the most
       * Positive number, ATIMAXFRACT32. */
      result = (a < (Fract32)0) ? (Fract32)MINFRACT32 : \
        (Fract32)MAXFRACT32;
    }
  }
  return (result);
}
