/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    sha512.h

  Summary:
    Crypto Framework Library header for cryptographic functions.

  Description:
    This header file contains function prototypes and definitions of
    the data types and constants that make up the Cryptographic Framework
    Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright © 2013-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
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

#ifndef WOLF_CRYPT_SHA512_H
#define WOLF_CRYPT_SHA512_H

#include "crypto/src/types.h"

#ifdef WOLFSSL_SHA512

/* for fips @wc_fips */
#ifdef HAVE_FIPS
    #define wc_Sha512             Sha512
    #define WC_SHA512             SHA512
    #define WC_SHA512_BLOCK_SIZE  SHA512_BLOCK_SIZE
    #define WC_SHA512_DIGEST_SIZE SHA512_DIGEST_SIZE
    #define WC_SHA512_PAD_SIZE    SHA512_PAD_SIZE
    #ifdef WOLFSSL_SHA384
        #define wc_Sha384             Sha384
        #define WC_SHA384             SHA384
        #define WC_SHA384_BLOCK_SIZE  SHA384_BLOCK_SIZE
        #define WC_SHA384_DIGEST_SIZE SHA384_DIGEST_SIZE
        #define WC_SHA384_PAD_SIZE    SHA384_PAD_SIZE
    #endif /* WOLFSSL_SHA384 */

    #define CYASSL_SHA512
    #if defined(WOLFSSL_SHA384)
        #define CYASSL_SHA384
    #endif
    #include "crypto/src/sha512.h"
#endif

#ifdef __cplusplus
    extern "C" {
#endif

#ifndef HAVE_FIPS /* avoid redefinition of structs */

#ifdef WOLFSSL_ASYNC_CRYPT
    #include "crypto/src/async.h"
#endif

#ifndef NO_OLD_WC_NAMES
    #define Sha512             wc_Sha512
    #define SHA512             WC_SHA512
    #define SHA512_BLOCK_SIZE  WC_SHA512_BLOCK_SIZE
    #define SHA512_DIGEST_SIZE WC_SHA512_DIGEST_SIZE
    #define SHA512_PAD_SIZE    WC_SHA512_PAD_SIZE
#endif

/* in bytes */
enum {
    WC_SHA512              =   4,   /* hash type unique */
    WC_SHA512_BLOCK_SIZE   = 128,
    WC_SHA512_DIGEST_SIZE  =  64,
    WC_SHA512_PAD_SIZE     = 112
};


/* Sha512 digest */
typedef struct wc_Sha512 {
    word64  digest[WC_SHA512_DIGEST_SIZE / sizeof(word64)];
    word64  buffer[WC_SHA512_BLOCK_SIZE  / sizeof(word64)];
    word32  buffLen;   /* in bytes          */
    word64  loLen;     /* length in bytes   */
    word64  hiLen;     /* length in bytes   */
    void*   heap;
#ifdef WOLFSSL_ASYNC_CRYPT
    WC_ASYNC_DEV asyncDev;
#endif /* WOLFSSL_ASYNC_CRYPT */
} wc_Sha512;

#endif /* HAVE_FIPS */

WOLFSSL_API int wc_InitSha512(wc_Sha512*);
WOLFSSL_API int wc_InitSha512_ex(wc_Sha512*, void*, int);
WOLFSSL_API int wc_Sha512Update(wc_Sha512*, const byte*, word32);
WOLFSSL_API int wc_Sha512Final(wc_Sha512*, byte*);
WOLFSSL_API void wc_Sha512Free(wc_Sha512*);

WOLFSSL_API int wc_Sha512GetHash(wc_Sha512*, byte*);
WOLFSSL_API int wc_Sha512Copy(wc_Sha512* src, wc_Sha512* dst);

#if defined(WOLFSSL_SHA384)

#ifndef HAVE_FIPS /* avoid redefinition of structs */

#ifndef NO_OLD_WC_NAMES
    #define Sha384             wc_Sha384
    #define SHA384             WC_SHA384
    #define SHA384_BLOCK_SIZE  WC_SHA384_BLOCK_SIZE
    #define SHA384_DIGEST_SIZE WC_SHA384_DIGEST_SIZE
    #define SHA384_PAD_SIZE    WC_SHA384_PAD_SIZE
#endif

/* in bytes */
enum {
    WC_SHA384              =   5,   /* hash type unique */
    WC_SHA384_BLOCK_SIZE   =   WC_SHA512_BLOCK_SIZE,
    WC_SHA384_DIGEST_SIZE  =   48,
    WC_SHA384_PAD_SIZE     =   WC_SHA512_PAD_SIZE
};

typedef wc_Sha512 wc_Sha384;
#endif /* HAVE_FIPS */

WOLFSSL_API int wc_InitSha384(wc_Sha384*);
WOLFSSL_API int wc_InitSha384_ex(wc_Sha384*, void*, int);
WOLFSSL_API int wc_Sha384Update(wc_Sha384*, const byte*, word32);
WOLFSSL_API int wc_Sha384Final(wc_Sha384*, byte*);
WOLFSSL_API void wc_Sha384Free(wc_Sha384*);

WOLFSSL_API int wc_Sha384GetHash(wc_Sha384*, byte*);
WOLFSSL_API int wc_Sha384Copy(wc_Sha384* src, wc_Sha384* dst);

#endif /* WOLFSSL_SHA384 */

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* WOLFSSL_SHA512 */
#endif /* WOLF_CRYPT_SHA512_H */

