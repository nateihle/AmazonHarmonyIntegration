/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    sha256.h

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




/* code submitted by raphael.huck@efixo.com */

#ifndef WOLF_CRYPT_SHA256_H
#define WOLF_CRYPT_SHA256_H

#include "crypto/src/types.h"

#ifndef NO_SHA256

#ifdef HAVE_FIPS
    #define wc_Sha256             Sha256
    #define WC_SHA256             SHA256
    #define WC_SHA256_BLOCK_SIZE  SHA256_BLOCK_SIZE
    #define WC_SHA256_DIGEST_SIZE SHA256_DIGEST_SIZE
    #define WC_SHA256_PAD_SIZE    SHA256_PAD_SIZE

    #ifdef WOLFSSL_SHA224
        #define wc_Sha224             Sha224
        #define WC_SHA224             SHA224
        #define WC_SHA224_BLOCK_SIZE  SHA224_BLOCK_SIZE
        #define WC_SHA224_DIGEST_SIZE SHA224_DIGEST_SIZE
        #define WC_SHA224_PAD_SIZE    SHA224_PAD_SIZE
    #endif

    /* for fips @wc_fips */
    #include "crypto/src/sha256.h"
#endif

#ifdef __cplusplus
    extern "C" {
#endif

#ifndef HAVE_FIPS /* avoid redefinition of structs */

#ifdef WOLFSSL_MICROCHIP_PIC32MZ
    #include "crypto/src/pic32mz-crypt.h"
#endif
#ifdef WOLFSSL_ASYNC_CRYPT
    #include "crypto/src/async.h"
#endif
#ifndef NO_OLD_SHA256_NAMES
    #define SHA256             WC_SHA256
#endif
#ifndef NO_OLD_WC_NAMES
    #define Sha256             wc_Sha256
    #define SHA256_BLOCK_SIZE  WC_SHA256_BLOCK_SIZE
    #define SHA256_DIGEST_SIZE WC_SHA256_DIGEST_SIZE
    #define SHA256_PAD_SIZE    WC_SHA256_PAD_SIZE
#endif

/* in bytes */
enum {
    WC_SHA256              =  2,   /* hash type unique */
    WC_SHA256_BLOCK_SIZE   = 64,
    WC_SHA256_DIGEST_SIZE  = 32,
    WC_SHA256_PAD_SIZE     = 56
};



/* wc_Sha256 digest */
typedef struct wc_Sha256 {
    /* alignment on digest and buffer speeds up ARMv8 crypto operations */
    ALIGN16 word32  digest[WC_SHA256_DIGEST_SIZE / sizeof(word32)];
    ALIGN16 word32  buffer[WC_SHA256_BLOCK_SIZE  / sizeof(word32)];
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    void*   heap;
#ifdef WOLFSSL_PIC32MZ_HASH
    hashUpdCache cache; /* cache for updates */
#endif
} wc_Sha256;



#endif /* HAVE_FIPS */

WOLFSSL_API int wc_InitSha256(wc_Sha256*);
WOLFSSL_API int wc_InitSha256_ex(wc_Sha256*, void*, int);
WOLFSSL_API int wc_Sha256Update(wc_Sha256*, const byte*, word32);
WOLFSSL_API int wc_Sha256Final(wc_Sha256*, byte*);
WOLFSSL_API void wc_Sha256Free(wc_Sha256*);

WOLFSSL_API int wc_Sha256GetHash(wc_Sha256*, byte*);
WOLFSSL_API int wc_Sha256Copy(wc_Sha256* src, wc_Sha256* dst);

#ifdef WOLFSSL_PIC32MZ_HASH
WOLFSSL_API void wc_Sha256SizeSet(wc_Sha256*, word32);
#endif

#ifdef WOLFSSL_SHA224
#ifndef HAVE_FIPS /* avoid redefinition of structs */

#ifndef NO_OLD_WC_NAMES
    #define Sha224             wc_Sha224
    #define SHA224             WC_SHA224
    #define SHA224_BLOCK_SIZE  WC_SHA224_BLOCK_SIZE
    #define SHA224_DIGEST_SIZE WC_SHA224_DIGEST_SIZE
    #define SHA224_PAD_SIZE    WC_SHA224_PAD_SIZE
#endif
/* in bytes */
enum {
    WC_SHA224              =   8,   /* hash type unique */
    WC_SHA224_BLOCK_SIZE   =   WC_SHA256_BLOCK_SIZE,
    WC_SHA224_DIGEST_SIZE  =   28,
    WC_SHA224_PAD_SIZE     =   WC_SHA256_PAD_SIZE
};

typedef wc_Sha256 wc_Sha224;
#endif /* HAVE_FIPS */

WOLFSSL_API int wc_InitSha224(wc_Sha224*);
WOLFSSL_API int wc_InitSha224_ex(wc_Sha224*, void*, int);
WOLFSSL_API int wc_Sha224Update(wc_Sha224*, const byte*, word32);
WOLFSSL_API int wc_Sha224Final(wc_Sha224*, byte*);
WOLFSSL_API void wc_Sha224Free(wc_Sha224*);

WOLFSSL_API int wc_Sha224GetHash(wc_Sha224*, byte*);
WOLFSSL_API int wc_Sha224Copy(wc_Sha224* src, wc_Sha224* dst);

#endif /* WOLFSSL_SHA224 */

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* NO_SHA256 */
#endif /* WOLF_CRYPT_SHA256_H */

