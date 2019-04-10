//=============================================================================
//
// File: FxTypes.h
//
// Description: Fixed and point data types.
//
//=============================================================================

#ifndef _FXTYPES_H_
#define _FXTYPES_H_

#include <stdint.h>

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// Fractional data types. 
typedef int16_t Fract16; //Q16-n.n
typedef int16_t FxPnt16; //Q?

typedef int32_t Fract32; //Q32-n.n
typedef int32_t FxPnt32; //Q??

typedef int64_t Fract64; //Q64-n.n
typedef int32_t FxPnt64; //Q????

// Fx floating point type (limited floating point)
// (extended FxQflExt32 used with f2Qfloat32) 
struct {
  Fract16   man;
  int16_t   exp;
} FxQFloat32;


// Maximum and minimum values for 16-bit data types. 
#define MAXINT16   ((int16_t) 0x7fff)
#define MININT16   ((int16_t) (-MAXINT16 - 1))
#define MAXFRACT16 MAXINT16       /* +0.999969 */
#define MINFRACT16 MININT16       /* -1.000000 */

#define MAXINT32   ((int32_t)0x7fffffffL)
#define MININT32   ((int32_t)(-MAXINT32 - 1))
#define MAXFRACT32 MAXINT32       /* +0.9999999995 */
#define MINFRACT32 MININT32       /* -1.0000000000 */

// minimum and maximum definitions for FX floating point data types 
#define MAXPFLOAT32 {MAXFRACT16, MAXINT16}
#define MINPFLOAT32 {MINFRACT16, MAXINT16}

#define UNITYFLOAT 1.0

// Null pointer value (normally, NULL is defined in stddef.h) 
//#define NULL ((void *)0)

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif  /* _FXTYPES_H_ */
