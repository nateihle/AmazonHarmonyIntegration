/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "system_definitions.h"
#include "system_config.h"
#include "crypto/src/settings.h"
#include "crypto/src/md5.h"
#include "crypto/src/sha.h"
#include "crypto/src/sha256.h"
#include "crypto/src/sha512.h"
#include "crypto/src/random.h"
#include "crypto/src/rsa.h"
#include "crypto/src/certs_test.h"
#include "crypto/src/des3.h"
#include "crypto/src/aes.h"
#include "crypto/src/hmac.h"
#ifdef HAVE_ECC
#include "crypto/src/ecc.h"
#endif
#ifdef HAVE_LIBZ
#include "crypto/src/compress.h"
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData ={
    .state = APP_STATE_INIT,

    .system_time = 0,

    .functionRet = 0,
};


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void TimerTick ( uintptr_t context, uint32_t alarmCount )
{
    appData.system_time += alarmCount;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

time_t time(time_t *tod)
{
    if (tod != NULL)
        *tod = appData.system_time;

    return appData.system_time;
}

int  md5_test(void);
int  sha_test(void);
int  sha256_test(void);
int  sha512_test(void);
int  sha384_test(void);
int  hmac_md5_test(void);
int  hmac_sha_test(void);
int  hmac_sha256_test(void);
int  hmac_sha384_test(void);
int  hmac_sha512_test(void);
int  des_test(void);
int  des3_test(void);
int  aes_test(void);
int  aesgcm_test(void);
int  gmac_test(void);
int  aesccm_test(void);
int  rsa_test(void);
int  random_test(void);
#ifdef HAVE_ECC
    int  ecc_test(void);
    #ifdef HAVE_ECC_ENCRYPT
        int  ecc_encrypt_test(void);
    #endif
#endif
#ifdef HAVE_BLAKE2
    int  blake2b_test(void);
#endif
#ifdef HAVE_LIBZ
    int compress_test(void);
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    DRV_TMR0_AlarmRegister(390625, true, 0, TimerTick);
    DRV_TMR0_AlarmEnable(true);
    DRV_TMR0_Start();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            BSP_LEDOn(BSP_LED_2); // Indicate that we're running
            appData.state = APP_STATE_TEST_MD5;
            break;
        }

        case APP_STATE_TEST_MD5:
#if !defined(NO_MD5)
            appData.functionRet = md5_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_SHA;
            break;

        case APP_STATE_TEST_SHA:
            appData.functionRet = sha_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_SHA256;
            break;

        case APP_STATE_TEST_SHA256:
#ifndef NO_SHA256
            appData.functionRet = sha256_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_SHA384;
            break;

        case APP_STATE_TEST_SHA384:
#ifdef WOLFSSL_SHA384
            appData.functionRet = sha384_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_SHA512;
            break;

        case APP_STATE_TEST_SHA512:
#ifdef WOLFSSL_SHA512
            appData.functionRet = sha512_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_HMAC_MD5;
            break;

        case APP_STATE_TEST_HMAC_MD5:
#if !defined(NO_HMAC) && !defined(NO_MD5)
            appData.functionRet = hmac_md5_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_HMAC_SHA;
            break;

        case APP_STATE_TEST_HMAC_SHA:
#if !defined(NO_HMAC) && !defined(NO_SHA)
            appData.functionRet = hmac_sha_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_HMAC_SHA256;
            break;

        case APP_STATE_TEST_HMAC_SHA256:
#if !defined(NO_HMAC) && !defined(NO_SHA256)
            appData.functionRet = hmac_sha256_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_HMAC_SHA384;
            break;

        case APP_STATE_TEST_HMAC_SHA384:
#if !defined(NO_HMAC) && defined(WOLFSSL_SHA384)
            appData.functionRet = hmac_sha384_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_HMAC_SHA512;
            break;

        case APP_STATE_TEST_HMAC_SHA512:
#if !defined(NO_HMAC) && defined(WOLFSSL_SHA512)
            appData.functionRet = hmac_sha512_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_DES;
            break;

        case APP_STATE_TEST_DES:
            appData.functionRet = des_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_DES3;
            break;

        case APP_STATE_TEST_DES3:
            appData.functionRet = des3_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_AES;
            break;

        case APP_STATE_TEST_AES:
            appData.functionRet = aes_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_RSA;
            break;

        case APP_STATE_TEST_RSA:
            appData.functionRet = rsa_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_RANDOM;
            break;

        case APP_STATE_TEST_RANDOM:
            appData.functionRet = random_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
                appData.state = APP_STATE_TEST_ECC;
            break;

        case APP_STATE_TEST_ECC:
#if defined(HAVE_ECC)
            appData.functionRet = ecc_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_TEST_COMPRESS;
            break;

        case APP_STATE_TEST_COMPRESS:
#if defined(HAVE_LIBZ)
            appData.functionRet = compress_test();
            if (appData.functionRet != 0)
                appData.state = APP_STATE_ERROR;
            else
#endif
                appData.state = APP_STATE_SPIN;
            break;

        case APP_STATE_ERROR:
            BSP_LEDOn(BSP_LED_1);
            break;

        case APP_STATE_SPIN:
            BSP_LEDOff(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);

            break;

        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

#ifndef NO_MD5
int md5_test(void)
{
    CRYPT_MD5_CTX  md5;
    byte hash[MD5_DIGEST_SIZE];

    testVector a, b, c, d, e;
    testVector test_md5[5];
    int times = sizeof(test_md5) / sizeof(testVector), i;

    a.input  = "abc";
    a.output = "\x90\x01\x50\x98\x3c\xd2\x4f\xb0\xd6\x96\x3f\x7d\x28\xe1\x7f"
               "\x72";
    a.inLen  = strlen(a.input);
    a.outLen = MD5_DIGEST_SIZE;

    b.input  = "message digest";
    b.output = "\xf9\x6b\x69\x7d\x7c\xb7\x93\x8d\x52\x5a\x2f\x31\xaa\xf1\x61"
               "\xd0";
    b.inLen  = strlen(b.input);
    b.outLen = MD5_DIGEST_SIZE;

    c.input  = "abcdefghijklmnopqrstuvwxyz";
    c.output = "\xc3\xfc\xd3\xd7\x61\x92\xe4\x00\x7d\xfb\x49\x6c\xca\x67\xe1"
               "\x3b";
    c.inLen  = strlen(c.input);
    c.outLen = MD5_DIGEST_SIZE;

    d.input  = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz012345"
               "6789";
    d.output = "\xd1\x74\xab\x98\xd2\x77\xd9\xf5\xa5\x61\x1c\x2c\x9f\x41\x9d"
               "\x9f";
    d.inLen  = strlen(d.input);
    d.outLen = MD5_DIGEST_SIZE;

    e.input  = "1234567890123456789012345678901234567890123456789012345678"
               "9012345678901234567890";
    e.output = "\x57\xed\xf4\xa2\x2b\xe3\xc9\x55\xac\x49\xda\x2e\x21\x07\xb6"
               "\x7a";
    e.inLen  = strlen(e.input);
    e.outLen = MD5_DIGEST_SIZE;

    test_md5[0] = a;
    test_md5[1] = b;
    test_md5[2] = c;
    test_md5[3] = d;
    test_md5[4] = e;

    CRYPT_MD5_Initialize(&md5);

    for (i = 0; i < times; ++i) {
        CRYPT_MD5_DataAdd(&md5, (byte*)test_md5[i].input, (word32)test_md5[i].inLen);
        CRYPT_MD5_Finalize(&md5, hash);

        if (memcmp(hash, test_md5[i].output, MD5_DIGEST_SIZE) != 0)
            return 0 - i;
    }

    return 0;
}
#endif /* NO_MD5 */

#ifndef NO_SHA

int sha_test(void)
{
    CRYPT_SHA_CTX  sha;
    byte hash[SHA_DIGEST_SIZE];

    testVector a, b, c, d;
    testVector test_sha[4];
    int times = sizeof(test_sha) / sizeof(struct testVector), i;

    a.input  = "abc";
    a.output = "\xA9\x99\x3E\x36\x47\x06\x81\x6A\xBA\x3E\x25\x71\x78\x50\xC2"
               "\x6C\x9C\xD0\xD8\x9D";
    a.inLen  = strlen(a.input);
    a.outLen = SHA_DIGEST_SIZE;

    b.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    b.output = "\x84\x98\x3E\x44\x1C\x3B\xD2\x6E\xBA\xAE\x4A\xA1\xF9\x51\x29"
               "\xE5\xE5\x46\x70\xF1";
    b.inLen  = strlen(b.input);
    b.outLen = SHA_DIGEST_SIZE;

    c.input  = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
               "aaaaaa";
    c.output = "\x00\x98\xBA\x82\x4B\x5C\x16\x42\x7B\xD7\xA1\x12\x2A\x5A\x44"
               "\x2A\x25\xEC\x64\x4D";
    c.inLen  = strlen(c.input);
    c.outLen = SHA_DIGEST_SIZE;

    d.input  = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
               "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
               "aaaaaaaaaa";
    d.output = "\xAD\x5B\x3F\xDB\xCB\x52\x67\x78\xC2\x83\x9D\x2F\x15\x1E\xA7"
               "\x53\x99\x5E\x26\xA0";
    d.inLen  = strlen(d.input);
    d.outLen = SHA_DIGEST_SIZE;

    test_sha[0] = d;
    test_sha[1] = b;
    test_sha[2] = c;
    test_sha[3] = a;

    CRYPT_SHA_Initialize(&sha);

    for (i = 0; i < times; ++i) {
        CRYPT_SHA_DataAdd(&sha, (byte*)test_sha[i].input, (word32)test_sha[i].inLen);
        CRYPT_SHA_Finalize(&sha, hash);

        if (memcmp(hash, test_sha[i].output, SHA_DIGEST_SIZE) != 0)
            return -5 - i;
    }

    return 0;
}

#endif /* NO_SHA */


#ifndef NO_SHA256
int sha256_test(void)
{
    CRYPT_SHA256_CTX sha;
    byte   hash[SHA256_DIGEST_SIZE];

    testVector a, b;
    testVector test_sha[2];
    int times = sizeof(test_sha) / sizeof(struct testVector), i;

    a.input  = "abc";
    a.output = "\xBA\x78\x16\xBF\x8F\x01\xCF\xEA\x41\x41\x40\xDE\x5D\xAE\x22"
               "\x23\xB0\x03\x61\xA3\x96\x17\x7A\x9C\xB4\x10\xFF\x61\xF2\x00"
               "\x15\xAD";
    a.inLen  = strlen(a.input);
    a.outLen = SHA256_DIGEST_SIZE;

    b.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    b.output = "\x24\x8D\x6A\x61\xD2\x06\x38\xB8\xE5\xC0\x26\x93\x0C\x3E\x60"
               "\x39\xA3\x3C\xE4\x59\x64\xFF\x21\x67\xF6\xEC\xED\xD4\x19\xDB"
               "\x06\xC1";
    b.inLen  = strlen(b.input);
    b.outLen = SHA256_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;

    CRYPT_SHA256_Initialize(&sha);

    for (i = 0; i < times; ++i) {
        CRYPT_SHA256_DataAdd(&sha, (byte*)test_sha[i].input,(word32)test_sha[i].inLen);
        CRYPT_SHA256_Finalize(&sha, hash);

        if (memcmp(hash, test_sha[i].output, SHA256_DIGEST_SIZE) != 0)
            return -10 - i;
    }

    return 0;
}
#endif


#ifdef WOLFSSL_SHA512
int sha512_test(void)
{
    CRYPT_SHA512_CTX sha;
    byte   hash[SHA512_DIGEST_SIZE];

    testVector a, b;
    testVector test_sha[2];
    int times = sizeof(test_sha) / sizeof(struct testVector), i;

    a.input  = "abc";
    a.output = "\xdd\xaf\x35\xa1\x93\x61\x7a\xba\xcc\x41\x73\x49\xae\x20\x41"
               "\x31\x12\xe6\xfa\x4e\x89\xa9\x7e\xa2\x0a\x9e\xee\xe6\x4b\x55"
               "\xd3\x9a\x21\x92\x99\x2a\x27\x4f\xc1\xa8\x36\xba\x3c\x23\xa3"
               "\xfe\xeb\xbd\x45\x4d\x44\x23\x64\x3c\xe8\x0e\x2a\x9a\xc9\x4f"
               "\xa5\x4c\xa4\x9f";
    a.inLen  = strlen(a.input);
    a.outLen = SHA512_DIGEST_SIZE;

    b.input  = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhi"
               "jklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";
    b.output = "\x8e\x95\x9b\x75\xda\xe3\x13\xda\x8c\xf4\xf7\x28\x14\xfc\x14"
               "\x3f\x8f\x77\x79\xc6\xeb\x9f\x7f\xa1\x72\x99\xae\xad\xb6\x88"
               "\x90\x18\x50\x1d\x28\x9e\x49\x00\xf7\xe4\x33\x1b\x99\xde\xc4"
               "\xb5\x43\x3a\xc7\xd3\x29\xee\xb6\xdd\x26\x54\x5e\x96\xe5\x5b"
               "\x87\x4b\xe9\x09";
    b.inLen  = strlen(b.input);
    b.outLen = SHA512_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;

    CRYPT_SHA512_Initialize(&sha);

    for (i = 0; i < times; ++i) {
        CRYPT_SHA512_DataAdd(&sha, (byte*)test_sha[i].input,(word32)test_sha[i].inLen);
        CRYPT_SHA512_Finalize(&sha, hash);

        if (memcmp(hash, test_sha[i].output, SHA512_DIGEST_SIZE) != 0)
            return -15 - i;
    }

    return 0;
}
#endif


#ifdef WOLFSSL_SHA384
int sha384_test(void)
{
    CRYPT_SHA384_CTX sha;
    byte   hash[SHA384_DIGEST_SIZE];

    testVector a, b;
    testVector test_sha[2];
    int times = sizeof(test_sha) / sizeof(struct testVector), i;

    a.input  = "abc";
    a.output = "\xcb\x00\x75\x3f\x45\xa3\x5e\x8b\xb5\xa0\x3d\x69\x9a\xc6\x50"
               "\x07\x27\x2c\x32\xab\x0e\xde\xd1\x63\x1a\x8b\x60\x5a\x43\xff"
               "\x5b\xed\x80\x86\x07\x2b\xa1\xe7\xcc\x23\x58\xba\xec\xa1\x34"
               "\xc8\x25\xa7";
    a.inLen  = strlen(a.input);
    a.outLen = SHA384_DIGEST_SIZE;

    b.input  = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhi"
               "jklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";
    b.output = "\x09\x33\x0c\x33\xf7\x11\x47\xe8\x3d\x19\x2f\xc7\x82\xcd\x1b"
               "\x47\x53\x11\x1b\x17\x3b\x3b\x05\xd2\x2f\xa0\x80\x86\xe3\xb0"
               "\xf7\x12\xfc\xc7\xc7\x1a\x55\x7e\x2d\xb9\x66\xc3\xe9\xfa\x91"
               "\x74\x60\x39";
    b.inLen  = strlen(b.input);
    b.outLen = SHA384_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;

    CRYPT_SHA384_Initialize(&sha);

    for (i = 0; i < times; ++i) {
        CRYPT_SHA384_DataAdd(&sha, (byte*)test_sha[i].input,(word32)test_sha[i].inLen);
        CRYPT_SHA384_Finalize(&sha, hash);

        if (memcmp(hash, test_sha[i].output, SHA384_DIGEST_SIZE) != 0)
            return -20 - i;
    }

    return 0;
}
#endif /* WOLFSSL_SHA384 */


#if !defined(NO_HMAC) && !defined(NO_MD5)
int hmac_md5_test(void)
{
    Hmac hmac;
    byte hash[MD5_DIGEST_SIZE];

    const char* keys[]=
    {
        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b",
        "Jefe",
        "\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA"
    };

    testVector a, b, c;
    testVector test_hmac[3];

    int times = sizeof(test_hmac) / sizeof(testVector), i;

    a.input  = "Hi There";
    a.output = "\x92\x94\x72\x7a\x36\x38\xbb\x1c\x13\xf4\x8e\xf8\x15\x8b\xfc"
               "\x9d";
    a.inLen  = strlen(a.input);
    a.outLen = MD5_DIGEST_SIZE;

    b.input  = "what do ya want for nothing?";
    b.output = "\x75\x0c\x78\x3e\x6a\xb0\xb5\x03\xea\xa8\x6e\x31\x0a\x5d\xb7"
               "\x38";
    b.inLen  = strlen(b.input);
    b.outLen = MD5_DIGEST_SIZE;

    c.input  = "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD";
    c.output = "\x56\xbe\x34\x52\x1d\x14\x4c\x88\xdb\xb8\xc7\x33\xf0\xe8\xb3"
               "\xf6";
    c.inLen  = strlen(c.input);
    c.outLen = MD5_DIGEST_SIZE;

    test_hmac[0] = a;
    test_hmac[1] = b;
    test_hmac[2] = c;

    for (i = 0; i < times; ++i) {
        wc_HmacSetKey(&hmac, MD5, (byte*)keys[i], (word32)strlen(keys[i]));
        wc_HmacUpdate(&hmac, (byte*)test_hmac[i].input,
                   (word32)test_hmac[i].inLen);
        wc_HmacFinal(&hmac, hash);

        if (memcmp(hash, test_hmac[i].output, MD5_DIGEST_SIZE) != 0)
            return -25 - i;
    }

    return 0;
}
#endif /* NO_HMAC && NO_MD5 */

#if !defined(NO_HMAC) && !defined(NO_SHA)
int hmac_sha_test(void)
{
    Hmac hmac;
    byte hash[SHA_DIGEST_SIZE];

    const char* keys[]=
    {
        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                                                                "\x0b\x0b\x0b",
        "Jefe",
        "\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA"
                                                                "\xAA\xAA\xAA"
    };

    testVector a, b, c;
    testVector test_hmac[3];

    int times = sizeof(test_hmac) / sizeof(testVector), i;

    a.input  = "Hi There";
    a.output = "\xb6\x17\x31\x86\x55\x05\x72\x64\xe2\x8b\xc0\xb6\xfb\x37\x8c"
               "\x8e\xf1\x46\xbe\x00";
    a.inLen  = strlen(a.input);
    a.outLen = SHA_DIGEST_SIZE;

    b.input  = "what do ya want for nothing?";
    b.output = "\xef\xfc\xdf\x6a\xe5\xeb\x2f\xa2\xd2\x74\x16\xd5\xf1\x84\xdf"
               "\x9c\x25\x9a\x7c\x79";
    b.inLen  = strlen(b.input);
    b.outLen = SHA_DIGEST_SIZE;

    c.input  = "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD";
    c.output = "\x12\x5d\x73\x42\xb9\xac\x11\xcd\x91\xa3\x9a\xf4\x8a\xa1\x7b"
               "\x4f\x63\xf1\x75\xd3";
    c.inLen  = strlen(c.input);
    c.outLen = SHA_DIGEST_SIZE;

    test_hmac[0] = a;
    test_hmac[1] = b;
    test_hmac[2] = c;

    for (i = 0; i < times; ++i) {
        wc_HmacSetKey(&hmac, SHA, (byte*)keys[i], (word32)strlen(keys[i]));
        wc_HmacUpdate(&hmac, (byte*)test_hmac[i].input,
                   (word32)test_hmac[i].inLen);
        wc_HmacFinal(&hmac, hash);

        if (memcmp(hash, test_hmac[i].output, SHA_DIGEST_SIZE) != 0)
            return -30 - i;
    }

    return 0;
}
#endif


#if !defined(NO_HMAC) && !defined(NO_SHA256)
int hmac_sha256_test(void)
{
    Hmac hmac;
    byte hash[SHA256_DIGEST_SIZE];

    const char* keys[]=
    {
        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                                                                "\x0b\x0b\x0b",
        "Jefe",
        "\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA"
                                                                "\xAA\xAA\xAA"
    };

    testVector a, b, c;
    testVector test_hmac[3];

    int times = sizeof(test_hmac) / sizeof(testVector), i;

    a.input  = "Hi There";
    a.output = "\xb0\x34\x4c\x61\xd8\xdb\x38\x53\x5c\xa8\xaf\xce\xaf\x0b\xf1"
               "\x2b\x88\x1d\xc2\x00\xc9\x83\x3d\xa7\x26\xe9\x37\x6c\x2e\x32"
               "\xcf\xf7";
    a.inLen  = strlen(a.input);
    a.outLen = SHA256_DIGEST_SIZE;

    b.input  = "what do ya want for nothing?";
    b.output = "\x5b\xdc\xc1\x46\xbf\x60\x75\x4e\x6a\x04\x24\x26\x08\x95\x75"
               "\xc7\x5a\x00\x3f\x08\x9d\x27\x39\x83\x9d\xec\x58\xb9\x64\xec"
               "\x38\x43";
    b.inLen  = strlen(b.input);
    b.outLen = SHA256_DIGEST_SIZE;

    c.input  = "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD";
    c.output = "\x77\x3e\xa9\x1e\x36\x80\x0e\x46\x85\x4d\xb8\xeb\xd0\x91\x81"
               "\xa7\x29\x59\x09\x8b\x3e\xf8\xc1\x22\xd9\x63\x55\x14\xce\xd5"
               "\x65\xfe";
    c.inLen  = strlen(c.input);
    c.outLen = SHA256_DIGEST_SIZE;

    test_hmac[0] = a;
    test_hmac[1] = b;
    test_hmac[2] = c;

    for (i = 0; i < times; ++i) {
        wc_HmacSetKey(&hmac, SHA256, (byte*)keys[i], (word32)strlen(keys[i]));
        wc_HmacUpdate(&hmac, (byte*)test_hmac[i].input,
                   (word32)test_hmac[i].inLen);
        wc_HmacFinal(&hmac, hash);

        if (memcmp(hash, test_hmac[i].output, SHA256_DIGEST_SIZE) != 0)
            return -35 - i;
    }

    return 0;
}
#endif


#if !defined(NO_HMAC) && defined(WOLFSSL_SHA384)
int hmac_sha384_test(void)
{
    Hmac hmac;
    byte hash[SHA384_DIGEST_SIZE];

    const char* keys[]=
    {
        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                                                                "\x0b\x0b\x0b",
        "Jefe",
        "\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA"
                                                                "\xAA\xAA\xAA"
    };

    testVector a, b, c;
    testVector test_hmac[3];

    int times = sizeof(test_hmac) / sizeof(testVector), i;

    a.input  = "Hi There";
    a.output = "\xaf\xd0\x39\x44\xd8\x48\x95\x62\x6b\x08\x25\xf4\xab\x46\x90"
               "\x7f\x15\xf9\xda\xdb\xe4\x10\x1e\xc6\x82\xaa\x03\x4c\x7c\xeb"
               "\xc5\x9c\xfa\xea\x9e\xa9\x07\x6e\xde\x7f\x4a\xf1\x52\xe8\xb2"
               "\xfa\x9c\xb6";
    a.inLen  = strlen(a.input);
    a.outLen = SHA384_DIGEST_SIZE;

    b.input  = "what do ya want for nothing?";
    b.output = "\xaf\x45\xd2\xe3\x76\x48\x40\x31\x61\x7f\x78\xd2\xb5\x8a\x6b"
               "\x1b\x9c\x7e\xf4\x64\xf5\xa0\x1b\x47\xe4\x2e\xc3\x73\x63\x22"
               "\x44\x5e\x8e\x22\x40\xca\x5e\x69\xe2\xc7\x8b\x32\x39\xec\xfa"
               "\xb2\x16\x49";
    b.inLen  = strlen(b.input);
    b.outLen = SHA384_DIGEST_SIZE;

    c.input  = "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD";
    c.output = "\x88\x06\x26\x08\xd3\xe6\xad\x8a\x0a\xa2\xac\xe0\x14\xc8\xa8"
               "\x6f\x0a\xa6\x35\xd9\x47\xac\x9f\xeb\xe8\x3e\xf4\xe5\x59\x66"
               "\x14\x4b\x2a\x5a\xb3\x9d\xc1\x38\x14\xb9\x4e\x3a\xb6\xe1\x01"
               "\xa3\x4f\x27";
    c.inLen  = strlen(c.input);
    c.outLen = SHA384_DIGEST_SIZE;

    test_hmac[0] = a;
    test_hmac[1] = b;
    test_hmac[2] = c;

    for (i = 0; i < times; ++i) {
        wc_HmacSetKey(&hmac, SHA384, (byte*)keys[i], (word32)strlen(keys[i]));
        wc_HmacUpdate(&hmac, (byte*)test_hmac[i].input,
                   (word32)test_hmac[i].inLen);
        wc_HmacFinal(&hmac, hash);

        if (memcmp(hash, test_hmac[i].output, SHA384_DIGEST_SIZE) != 0)
            return -40 - i;
    }

    return 0;
}
#endif


#if !defined(NO_HMAC) && defined(WOLFSSL_SHA512)
int hmac_sha512_test(void)
{
    Hmac hmac;
    byte hash[SHA512_DIGEST_SIZE];

    const char* keys[]=
    {
        "\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b\x0b"
                                                                "\x0b\x0b\x0b",
        "Jefe",
        "\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA"
                                                                "\xAA\xAA\xAA"
    };

    testVector a, b, c;
    testVector test_hmac[3];

    int times = sizeof(test_hmac) / sizeof(testVector), i;

    a.input  = "Hi There";
    a.output = "\x87\xaa\x7c\xde\xa5\xef\x61\x9d\x4f\xf0\xb4\x24\x1a\x1d\x6c"
               "\xb0\x23\x79\xf4\xe2\xce\x4e\xc2\x78\x7a\xd0\xb3\x05\x45\xe1"
               "\x7c\xde\xda\xa8\x33\xb7\xd6\xb8\xa7\x02\x03\x8b\x27\x4e\xae"
               "\xa3\xf4\xe4\xbe\x9d\x91\x4e\xeb\x61\xf1\x70\x2e\x69\x6c\x20"
               "\x3a\x12\x68\x54";
    a.inLen  = strlen(a.input);
    a.outLen = SHA512_DIGEST_SIZE;

    b.input  = "what do ya want for nothing?";
    b.output = "\x16\x4b\x7a\x7b\xfc\xf8\x19\xe2\xe3\x95\xfb\xe7\x3b\x56\xe0"
               "\xa3\x87\xbd\x64\x22\x2e\x83\x1f\xd6\x10\x27\x0c\xd7\xea\x25"
               "\x05\x54\x97\x58\xbf\x75\xc0\x5a\x99\x4a\x6d\x03\x4f\x65\xf8"
               "\xf0\xe6\xfd\xca\xea\xb1\xa3\x4d\x4a\x6b\x4b\x63\x6e\x07\x0a"
               "\x38\xbc\xe7\x37";
    b.inLen  = strlen(b.input);
    b.outLen = SHA512_DIGEST_SIZE;

    c.input  = "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD\xDD"
               "\xDD\xDD\xDD\xDD\xDD\xDD";
    c.output = "\xfa\x73\xb0\x08\x9d\x56\xa2\x84\xef\xb0\xf0\x75\x6c\x89\x0b"
               "\xe9\xb1\xb5\xdb\xdd\x8e\xe8\x1a\x36\x55\xf8\x3e\x33\xb2\x27"
               "\x9d\x39\xbf\x3e\x84\x82\x79\xa7\x22\xc8\x06\xb4\x85\xa4\x7e"
               "\x67\xc8\x07\xb9\x46\xa3\x37\xbe\xe8\x94\x26\x74\x27\x88\x59"
               "\xe1\x32\x92\xfb";
    c.inLen  = strlen(c.input);
    c.outLen = SHA512_DIGEST_SIZE;

    test_hmac[0] = a;
    test_hmac[1] = b;
    test_hmac[2] = c;

    for (i = 0; i < times; ++i) {
        wc_HmacSetKey(&hmac, SHA512, (byte*)keys[i], (word32)strlen(keys[i]));
        wc_HmacUpdate(&hmac, (byte*)test_hmac[i].input,
                   (word32)test_hmac[i].inLen);
        wc_HmacFinal(&hmac, hash);

        if (memcmp(hash, test_hmac[i].output, SHA512_DIGEST_SIZE) != 0)
            return -45 - i;
    }

    return 0;
}
#endif


#ifndef NO_DES3
int des_test(void)
{
    const byte vector[] = { /* "now is the time for all " w/o trailing 0 */
        0x6e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
        0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20,
        0x66,0x6f,0x72,0x20,0x61,0x6c,0x6c,0x20
    };

    byte plain[24];
    byte cipher[24];

    Des enc;
    Des dec;

    const byte key[] =
    {
        0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef
    };

    const byte iv[] =
    {
        0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef
    };

    const byte verify[] =
    {
        0x8b,0x7c,0x52,0xb0,0x01,0x2b,0x6c,0xb8,
        0x4f,0x0f,0xeb,0xf3,0xfb,0x5f,0x86,0x73,
        0x15,0x85,0xb3,0x22,0x4b,0x86,0x2b,0x4b
    };

    /* The above const allocates in RAM, but does not flush out of cache. Copy
       it back out so it is in physical memory. */
#if defined(HW_CRYPTO)
    SYS_DEVCON_DataCacheFlush();
#endif
    wc_Des_SetKey(&enc, key, iv, DES_ENCRYPTION);
    wc_Des_CbcEncrypt(&enc, cipher, vector, sizeof(vector));
    wc_Des_SetKey(&dec, key, iv, DES_DECRYPTION);
    wc_Des_CbcDecrypt(&dec, plain, cipher, sizeof(cipher));

    if (memcmp(plain, vector, sizeof(plain)))
        return -51;

    if (memcmp(cipher, verify, sizeof(cipher)))
        return -52;

    return 0;
}
#endif /* NO_DES3 */


#ifndef NO_DES3
int des3_test(void)
{
    const byte vector[] = { /* "Now is the time for all " w/o trailing 0 */
        0x4e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
        0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20,
        0x66,0x6f,0x72,0x20,0x61,0x6c,0x6c,0x20
    };

    byte plain[24];
    byte cipher[24];

    CRYPT_TDES_CTX enc;
    CRYPT_TDES_CTX dec;

    const byte key3[] =
    {
        0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,
        0xfe,0xde,0xba,0x98,0x76,0x54,0x32,0x10,
        0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67
    };
    const byte iv3[] =
    {
        0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,
        0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
        0x11,0x21,0x31,0x41,0x51,0x61,0x71,0x81

    };

    const byte verify3[] =
    {
        0x43,0xa0,0x29,0x7e,0xd1,0x84,0xf8,0x0e,
        0x89,0x64,0x84,0x32,0x12,0xd5,0x08,0x98,
        0x18,0x94,0x15,0x74,0x87,0x12,0x7d,0xb0
    };


    /* The above const allocates in RAM, but does not flush out of cache. Copy
       it back out so it is in physical memory. */
#if defined(HW_CRYPTO)
    SYS_DEVCON_DataCacheFlush();
#endif
    CRYPT_TDES_KeySet(&enc, key3, iv3, DES_ENCRYPTION);
    CRYPT_TDES_KeySet(&dec, key3, iv3, DES_DECRYPTION);
    CRYPT_TDES_CBC_Encrypt(&enc, cipher, vector, sizeof(vector));
    CRYPT_TDES_CBC_Decrypt(&dec, plain, cipher, sizeof(cipher));

    if (memcmp(plain, vector, sizeof(plain)))
        return -53;

    if (memcmp(cipher, verify3, sizeof(cipher)))
        return -54;

    return 0;
}
#endif /* NO_DES */


#ifndef NO_AES
int aes_test(void)
{
    CRYPT_AES_CTX enc;
    CRYPT_AES_CTX dec;

    const byte msg[] = { /* "now is the time for all " w/o trailing 0 */
        0x6e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
        0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20,
        0x66,0x6f,0x72,0x20,0x61,0x6c,0x6c,0x20
    };

    const byte verify[] =
    {
        0x95,0x94,0x92,0x57,0x5f,0x42,0x81,0x53,
        0x2c,0xcc,0x9d,0x46,0x77,0xa2,0x33,0xcb
    };

    byte key[] = "0123456789abcdef   ";  /* align */
    byte iv[]  = "1234567890abcdef   ";  /* align */

    byte cipher[AES_BLOCK_SIZE * 4];
    byte plain [AES_BLOCK_SIZE * 4];

    /* The above const allocates in RAM, but does not flush out of cache. Copy
       it back out so it is in physical memory. */
#if defined(HW_CRYPTO)
    SYS_DEVCON_DataCacheFlush();
#endif
    CRYPT_AES_KeySet(&enc, key, AES_BLOCK_SIZE, iv, AES_ENCRYPTION);
    CRYPT_AES_KeySet(&dec, key, AES_BLOCK_SIZE, iv, AES_DECRYPTION);

    CRYPT_AES_CBC_Encrypt(&enc, cipher, msg,   AES_BLOCK_SIZE);
    CRYPT_AES_CBC_Decrypt(&dec, plain, cipher, AES_BLOCK_SIZE);

    if (memcmp(plain, msg, AES_BLOCK_SIZE))
        return -60;

    if (memcmp(cipher, verify, AES_BLOCK_SIZE))
        return -61;

#ifdef WOLFSSL_AES_COUNTER
    {
        const byte ctrKey[] =
        {
            0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
            0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
        };

        const byte ctrIv[] =
        {
            0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
            0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
        };


        const byte ctrPlain[] =
        {
            0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
            0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
            0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,
            0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
            0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,
            0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
            0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,
            0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10
        };

        const byte ctrCipher[] =
        {
            0x87,0x4d,0x61,0x91,0xb6,0x20,0xe3,0x26,
            0x1b,0xef,0x68,0x64,0x99,0x0d,0xb6,0xce,
            0x98,0x06,0xf6,0x6b,0x79,0x70,0xfd,0xff,
            0x86,0x17,0x18,0x7b,0xb9,0xff,0xfd,0xff,
            0x5a,0xe4,0xdf,0x3e,0xdb,0xd5,0xd3,0x5e,
            0x5b,0x4f,0x09,0x02,0x0d,0xb0,0x3e,0xab,
            0x1e,0x03,0x1d,0xda,0x2f,0xbe,0x03,0xd1,
            0x79,0x21,0x70,0xa0,0xf3,0x00,0x9c,0xee
        };

        CRYPT_AES_KeySet(&enc, ctrKey, AES_BLOCK_SIZE, ctrIv, AES_ENCRYPTION);
        /* Ctr only uses encrypt, even on key setup */
        CRYPT_AES_KeySet(&dec, ctrKey, AES_BLOCK_SIZE, ctrIv, AES_ENCRYPTION);

        /* The above const allocates in RAM, but does not flush out of cache. Copy
           it back out so it is in physical memory. */
#if defined(HW_CRYPTO)
        SYS_DEVCON_DataCacheFlush();
#endif
        CRYPT_AES_CTR_Encrypt(&enc, cipher, ctrPlain, AES_BLOCK_SIZE*4);
        CRYPT_AES_CTR_Encrypt(&dec, plain, cipher, AES_BLOCK_SIZE*4);

        if (memcmp(plain, ctrPlain, AES_BLOCK_SIZE*4))
            return -66;

        if (memcmp(cipher, ctrCipher, AES_BLOCK_SIZE*4))
            return -67;
    }
#endif /* WOLFSSL_AES_COUNTER */

    return 0;
}

#endif // NO_AES


int random_test(void)
{
    CRYPT_RNG_CTX  rng;
    byte block[32];
    int ret;

    ret = CRYPT_RNG_Initialize(&rng);
    if (ret != 0) return -39;

    CRYPT_RNG_BlockGenerate(&rng, block, sizeof(block));

    return 0;
}

#ifndef NO_RSA

#if !defined(USE_CERT_BUFFERS_1024) && !defined(USE_CERT_BUFFERS_2048)
    #ifdef FREESCALE_MQX
        static const char* clientKey  = "a:\\certs\\client-key.der";
        static const char* clientCert = "a:\\certs\\client-cert.der";
        #ifdef WOLFSSL_CERT_GEN
            static const char* caKeyFile  = "a:\\certs\\ca-key.der";
            static const char* caCertFile = "a:\\certs\\ca-cert.pem";
            #ifdef HAVE_ECC
                static const char* eccCaKeyFile  = "a:\\certs\\ecc-key.der";
                static const char* eccCaCertFile = "a:\\certs\\server-ecc.pem";
            #endif
        #endif
    #elif defined(WOLFSSL_MKD_SHELL)
        static char* clientKey = "certs/client-key.der";
        static char* clientCert = "certs/client-cert.der";
        void set_clientKey(char *key) {  clientKey = key ; }
        void set_clientCert(char *cert) {  clientCert = cert ; }
        #ifdef WOLFSSL_CERT_GEN
            static char* caKeyFile  = "certs/ca-key.der";
            static char* caCertFile = "certs/ca-cert.pem";
            void set_caKeyFile (char * key)  { caKeyFile   = key ; }
            void set_caCertFile(char * cert) { caCertFile = cert ; }
            #ifdef HAVE_ECC
                static const char* eccCaKeyFile  = "certs/ecc-key.der";
                static const char* eccCaCertFile = "certs/server-ecc.pem";
                void set_eccCaKeyFile (char * key)  { eccCaKeyFile  = key ; }
                void set_eccCaCertFile(char * cert) { eccCaCertFile = cert ; }
            #endif
        #endif
    #else
        static const char* clientKey  = "./certs/client-key.der";
        static const char* clientCert = "./certs/client-cert.der";
        #ifdef WOLFSSL_CERT_GEN
            static const char* caKeyFile  = "./certs/ca-key.der";
            static const char* caCertFile = "./certs/ca-cert.pem";
            #ifdef HAVE_ECC
                static const char* eccCaKeyFile  = "./certs/ecc-key.der";
                static const char* eccCaCertFile = "./certs/server-ecc.pem";
            #endif
        #endif
    #endif
#endif



#define FOURK_BUF 4096

int rsa_test(void)
{
    byte*   tmp;
    size_t bytes;
    RsaKey key;
    RNG    rng;
    word32 idx = 0;
    int    ret;
    byte   in[] = "Everyone gets Friday off.";
    word32 inLen = (word32)strlen((char*)in);
    byte   out[256];
    byte   plain[256];
#if !defined(USE_CERT_BUFFERS_1024) && !defined(USE_CERT_BUFFERS_2048)
    FILE*  file, * file2;
#endif
#ifdef WOLFSSL_TEST_CERT
    DecodedCert cert;
#endif

    tmp = (byte*)malloc(FOURK_BUF);
    if (tmp == NULL)
        return -40;

#ifdef USE_CERT_BUFFERS_1024
    XMEMCPY(tmp, client_key_der_1024, sizeof_client_key_der_1024);
    bytes = sizeof_client_key_der_1024;
#elif defined(USE_CERT_BUFFERS_2048)
    XMEMCPY(tmp, client_key_der_2048, sizeof_client_key_der_2048);
    bytes = sizeof_client_key_der_2048;
#else
    file = fopen(clientKey, "rb");

    bytes = fread(tmp, 1, FOURK_BUF, file);
    fclose(file);
#endif /* USE_CERT_BUFFERS */

#ifdef HAVE_CAVIUM
    RsaInitCavium(&key, CAVIUM_DEV_ID);
#endif
    wc_InitRsaKey(&key, 0);
    ret = wc_RsaPrivateKeyDecode(tmp, &idx, &key, (word32)bytes);
    if (ret != 0) return -71;

    ret = wc_InitRng(&rng);
    if (ret != 0) return -72;

    ret = wc_RsaPublicEncrypt(in, inLen, out, sizeof(out), &key, &rng);
    if (ret < 0) return -73;

    ret = wc_RsaPrivateDecrypt(out, ret, plain, sizeof(plain), &key);
    if (ret < 0) return -74;

    if (memcmp(plain, in, inLen)) return -75;

    ret = wc_RsaSSL_Sign(in, inLen, out, sizeof(out), &key, &rng);
    if (ret < 0) return -76;

    memset(plain, 0, sizeof(plain));
    ret = wc_RsaSSL_Verify(out, ret, plain, sizeof(plain), &key);
    if (ret < 0) return -77;

    if (memcmp(plain, in, ret)) return -78;

#if defined(WOLFSSL_MDK_ARM)
    #define sizeof(s) strlen((char *)(s))
#endif

#ifdef USE_CERT_BUFFERS_1024
    XMEMCPY(tmp, client_cert_der_1024, sizeof_client_cert_der_1024);
    bytes = sizeof_client_cert_der_1024;
#elif defined(USE_CERT_BUFFERS_2048)
    XMEMCPY(tmp, client_cert_der_2048, sizeof_client_cert_der_2048);
    bytes = sizeof_client_cert_der_2048;
#else
    file2 = fopen(clientCert, "rb");
    if (!file2)
        return -49;

    bytes = fread(tmp, 1, FOURK_BUF, file2);
    fclose(file2);
#endif

#ifdef sizeof
		#undef sizeof
#endif

#ifdef WOLFSSL_TEST_CERT
    InitDecodedCert(&cert, tmp, (word32)bytes, 0);

    ret = ParseCert(&cert, CERT_TYPE, NO_VERIFY, 0);
    if (ret != 0) return -491;

    FreeDecodedCert(&cert);
#else
    (void)bytes;
#endif


#ifdef WOLFSSL_KEY_GEN
    {
        byte*  der;
        byte*  pem;
        int    derSz = 0;
        int    pemSz = 0;
        RsaKey derIn;
        RsaKey genKey;
        FILE* keyFile;
        FILE* pemFile;

        InitRsaKey(&genKey, 0);
        ret = MakeRsaKey(&genKey, 1024, 65537, &rng);
        if (ret != 0)
            return -301;

        der = (byte*)malloc(FOURK_BUF);
        if (der == NULL)
            return -307;
        pem = (byte*)malloc(FOURK_BUF);
        if (pem == NULL)
            return -308;

        derSz = RsaKeyToDer(&genKey, der, FOURK_BUF);
        if (derSz < 0)
            return -302;

        keyFile = fopen("./key.der", "wb");
        if (!keyFile)
            return -303;
        ret = (int)fwrite(der, derSz, 1, keyFile);
        fclose(keyFile);

        pemSz = DerToPem(der, derSz, pem, FOURK_BUF, PRIVATEKEY_TYPE);
        if (pemSz < 0)
            return -304;

        pemFile = fopen("./key.pem", "wb");
        if (!pemFile)
            return -305;
        ret = (int)fwrite(pem, pemSz, 1, pemFile);
        fclose(pemFile);

        InitRsaKey(&derIn, 0);
        idx = 0;
        ret = RsaPrivateKeyDecode(der, &idx, &derIn, derSz);
        if (ret != 0)
            return -306;

        FreeRsaKey(&derIn);
        FreeRsaKey(&genKey);
        free(pem);
        free(der);
    }
#endif /* WOLFSSL_KEY_GEN */


#ifdef WOLFSSL_CERT_GEN
    /* self signed */
    {
        Cert        myCert;
        byte*       derCert;
        byte*       pem;
        FILE*       derFile;
        FILE*       pemFile;
        int         certSz;
        int         pemSz;
#ifdef WOLFSSL_TEST_CERT
        DecodedCert decode;
#endif

        derCert = (byte*)malloc(FOURK_BUF);
        if (derCert == NULL)
            return -309;
        pem = (byte*)malloc(FOURK_BUF);
        if (pem == NULL)
            return -310;

        InitCert(&myCert);

        strncpy(myCert.subject.country, "US", CTC_NAME_SIZE);
        strncpy(myCert.subject.state, "OR", CTC_NAME_SIZE);
        strncpy(myCert.subject.locality, "Portland", CTC_NAME_SIZE);
        strncpy(myCert.subject.org, "yaSSL", CTC_NAME_SIZE);
        strncpy(myCert.subject.unit, "Development", CTC_NAME_SIZE);
        strncpy(myCert.subject.commonName, "www.yassl.com", CTC_NAME_SIZE);
        strncpy(myCert.subject.email, "info@yassl.com", CTC_NAME_SIZE);
        myCert.isCA    = 1;
        myCert.sigType = CTC_SHA256wRSA;

        certSz = MakeSelfCert(&myCert, derCert, FOURK_BUF, &key, &rng);
        if (certSz < 0)
            return -401;

#ifdef WOLFSSL_TEST_CERT
        InitDecodedCert(&decode, derCert, certSz, 0);
        ret = ParseCert(&decode, CERT_TYPE, NO_VERIFY, 0);
        if (ret != 0)
            return -402;
        FreeDecodedCert(&decode);
#endif
        derFile = fopen("./cert.der", "wb");
        if (!derFile)
            return -403;
        ret = (int)fwrite(derCert, certSz, 1, derFile);
        fclose(derFile);

        pemSz = DerToPem(derCert, certSz, pem, FOURK_BUF, CERT_TYPE);
        if (pemSz < 0)
            return -404;

        pemFile = fopen("./cert.pem", "wb");
        if (!pemFile)
            return -405;
        ret = (int)fwrite(pem, pemSz, 1, pemFile);
        fclose(pemFile);
        free(pem);
        free(derCert);
    }
    /* CA style */
    {
        RsaKey      caKey;
        Cert        myCert;
        byte*       derCert;
        byte*       pem;
        FILE*       derFile;
        FILE*       pemFile;
        int         certSz;
        int         pemSz;
        size_t      bytes3;
        word32      idx3 = 0;
			  FILE* file3 ;
#ifdef WOLFSSL_TEST_CERT
        DecodedCert decode;
#endif

        derCert = (byte*)malloc(FOURK_BUF);
        if (derCert == NULL)
            return -311;
        pem = (byte*)malloc(FOURK_BUF);
        if (pem == NULL)
            return -312;

        file3 = fopen(caKeyFile, "rb");

        if (!file3)
            return -412;

        bytes3 = fread(tmp, 1, FOURK_BUF, file3);
        fclose(file3);

        InitRsaKey(&caKey, 0);
        ret = RsaPrivateKeyDecode(tmp, &idx3, &caKey, (word32)bytes3);
        if (ret != 0) return -413;

        InitCert(&myCert);

        strncpy(myCert.subject.country, "US", CTC_NAME_SIZE);
        strncpy(myCert.subject.state, "OR", CTC_NAME_SIZE);
        strncpy(myCert.subject.locality, "Portland", CTC_NAME_SIZE);
        strncpy(myCert.subject.org, "yaSSL", CTC_NAME_SIZE);
        strncpy(myCert.subject.unit, "Development", CTC_NAME_SIZE);
        strncpy(myCert.subject.commonName, "www.yassl.com", CTC_NAME_SIZE);
        strncpy(myCert.subject.email, "info@yassl.com", CTC_NAME_SIZE);

        ret = SetIssuer(&myCert, caCertFile);
        if (ret < 0)
            return -405;

        certSz = MakeCert(&myCert, derCert, FOURK_BUF, &key, NULL, &rng);
        if (certSz < 0)
            return -407;

        certSz = SignCert(myCert.bodySz, myCert.sigType, derCert, FOURK_BUF,
                          &caKey, NULL, &rng);
        if (certSz < 0)
            return -408;


#ifdef WOLFSSL_TEST_CERT
        InitDecodedCert(&decode, derCert, certSz, 0);
        ret = ParseCert(&decode, CERT_TYPE, NO_VERIFY, 0);
        if (ret != 0)
            return -409;
        FreeDecodedCert(&decode);
#endif

        derFile = fopen("./othercert.der", "wb");
        if (!derFile)
            return -410;
        ret = (int)fwrite(derCert, certSz, 1, derFile);
        fclose(derFile);

        pemSz = DerToPem(derCert, certSz, pem, FOURK_BUF, CERT_TYPE);
        if (pemSz < 0)
            return -411;

        pemFile = fopen("./othercert.pem", "wb");
        if (!pemFile)
            return -412;
        ret = (int)fwrite(pem, pemSz, 1, pemFile);
        fclose(pemFile);
        free(pem);
        free(derCert);
        FreeRsaKey(&caKey);
    }
#ifdef HAVE_ECC
    /* ECC CA style */
    {
        ecc_key     caKey;
        Cert        myCert;
        byte*       derCert;
        byte*       pem;
        FILE*       derFile;
        FILE*       pemFile;
        int         certSz;
        int         pemSz;
        size_t      bytes3;
        word32      idx3 = 0;
			  FILE* file3 ;
#ifdef WOLFSSL_TEST_CERT
        DecodedCert decode;
#endif

        derCert = (byte*)malloc(FOURK_BUF);
        if (derCert == NULL)
            return -5311;
        pem = (byte*)malloc(FOURK_BUF);
        if (pem == NULL)
            return -5312;

        file3 = fopen(eccCaKeyFile, "rb");

        if (!file3)
            return -5412;

        bytes3 = fread(tmp, 1, FOURK_BUF, file3);
        fclose(file3);

        ecc_init(&caKey);
        ret = EccPrivateKeyDecode(tmp, &idx3, &caKey, (word32)bytes3);
        if (ret != 0) return -5413;

        InitCert(&myCert);
        myCert.sigType = CTC_SHA256wECDSA;

        strncpy(myCert.subject.country, "US", CTC_NAME_SIZE);
        strncpy(myCert.subject.state, "OR", CTC_NAME_SIZE);
        strncpy(myCert.subject.locality, "Portland", CTC_NAME_SIZE);
        strncpy(myCert.subject.org, "wolfSSL", CTC_NAME_SIZE);
        strncpy(myCert.subject.unit, "Development", CTC_NAME_SIZE);
        strncpy(myCert.subject.commonName, "www.wolfssl.com", CTC_NAME_SIZE);
        strncpy(myCert.subject.email, "info@wolfssl.com", CTC_NAME_SIZE);

        ret = SetIssuer(&myCert, eccCaCertFile);
        if (ret < 0)
            return -5405;

        certSz = MakeCert(&myCert, derCert, FOURK_BUF, NULL, &caKey, &rng);
        if (certSz < 0)
            return -5407;

        certSz = SignCert(myCert.bodySz, myCert.sigType, derCert, FOURK_BUF,
                          NULL, &caKey, &rng);
        if (certSz < 0)
            return -5408;

#ifdef WOLFSSL_TEST_CERT
        InitDecodedCert(&decode, derCert, certSz, 0);
        ret = ParseCert(&decode, CERT_TYPE, NO_VERIFY, 0);
        if (ret != 0)
            return -5409;
        FreeDecodedCert(&decode);
#endif

        derFile = fopen("./certecc.der", "wb");
        if (!derFile)
            return -5410;
        ret = (int)fwrite(derCert, certSz, 1, derFile);
        fclose(derFile);

        pemSz = DerToPem(derCert, certSz, pem, FOURK_BUF, CERT_TYPE);
        if (pemSz < 0)
            return -5411;

        pemFile = fopen("./certecc.pem", "wb");
        if (!pemFile)
            return -5412;
        ret = (int)fwrite(pem, pemSz, 1, pemFile);
        fclose(pemFile);
        free(pem);
        free(derCert);
        ecc_free(&caKey);
    }
#endif /* HAVE_ECC */
#ifdef HAVE_NTRU
    {
        RsaKey      caKey;
        Cert        myCert;
        byte*       derCert;
        byte*       pem;
        FILE*       derFile;
        FILE*       pemFile;
        FILE*       caFile;
        FILE*       ntruPrivFile;
        int         certSz;
        int         pemSz;
        size_t      bytes;
        word32      idx = 0;
#ifdef WOLFSSL_TEST_CERT
        DecodedCert decode;
#endif
        derCert = (byte*)malloc(FOURK_BUF);
        if (derCert == NULL)
            return -311;
        pem = (byte*)malloc(FOURK_BUF);
        if (pem == NULL)
            return -312;

        byte   public_key[557];          /* sized for EES401EP2 */
        word16 public_key_len;           /* no. of octets in public key */
        byte   private_key[607];         /* sized for EES401EP2 */
        word16 private_key_len;          /* no. of octets in private key */
        DRBG_HANDLE drbg;
        static uint8_t const pers_str[] = {
                'C', 'y', 'a', 'S', 'S', 'L', ' ', 't', 'e', 's', 't'
        };
        word32 rc = crypto_drbg_instantiate(112, pers_str, sizeof(pers_str),
                                            GetEntropy, &drbg);
        if (rc != DRBG_OK)
            return -450;

        rc = crypto_ntru_encrypt_keygen(drbg, NTRU_EES401EP2, &public_key_len,
                                        NULL, &private_key_len, NULL);
        if (rc != NTRU_OK)
            return -451;

        rc = crypto_ntru_encrypt_keygen(drbg, NTRU_EES401EP2, &public_key_len,
                                     public_key, &private_key_len, private_key);
        crypto_drbg_uninstantiate(drbg);

        if (rc != NTRU_OK)
            return -452;

        caFile = fopen(caKeyFile, "rb");

        if (!caFile)
            return -453;

        bytes = fread(tmp, 1, FOURK_BUF, caFile);
        fclose(caFile);

        InitRsaKey(&caKey, 0);
        ret = RsaPrivateKeyDecode(tmp, &idx, &caKey, (word32)bytes);
        if (ret != 0) return -454;

        InitCert(&myCert);

        strncpy(myCert.subject.country, "US", CTC_NAME_SIZE);
        strncpy(myCert.subject.state, "OR", CTC_NAME_SIZE);
        strncpy(myCert.subject.locality, "Portland", CTC_NAME_SIZE);
        strncpy(myCert.subject.org, "yaSSL", CTC_NAME_SIZE);
        strncpy(myCert.subject.unit, "Development", CTC_NAME_SIZE);
        strncpy(myCert.subject.commonName, "www.yassl.com", CTC_NAME_SIZE);
        strncpy(myCert.subject.email, "info@yassl.com", CTC_NAME_SIZE);

        ret = SetIssuer(&myCert, caCertFile);
        if (ret < 0)
            return -455;

        certSz = MakeNtruCert(&myCert, derCert, FOURK_BUF, public_key,
                              public_key_len, &rng);
        if (certSz < 0)
            return -456;

        certSz = SignCert(myCert.bodySz, myCert.sigType, derCert, FOURK_BUF,
                          &caKey, NULL, &rng);
        if (certSz < 0)
            return -457;


#ifdef WOLFSSL_TEST_CERT
        InitDecodedCert(&decode, derCert, certSz, 0);
        ret = ParseCert(&decode, CERT_TYPE, NO_VERIFY, 0);
        if (ret != 0)
            return -458;
        FreeDecodedCert(&decode);
#endif
        derFile = fopen("./ntru-cert.der", "wb");
        if (!derFile)
            return -459;
        ret = fwrite(derCert, certSz, 1, derFile);
        fclose(derFile);

        pemSz = DerToPem(derCert, certSz, pem, FOURK_BUF, CERT_TYPE);
        if (pemSz < 0)
            return -460;

        pemFile = fopen("./ntru-cert.pem", "wb");
        if (!pemFile)
            return -461;
        ret = fwrite(pem, pemSz, 1, pemFile);
        fclose(pemFile);

        ntruPrivFile = fopen("./ntru-key.raw", "wb");
        if (!ntruPrivFile)
            return -462;
        ret = fwrite(private_key, private_key_len, 1, ntruPrivFile);
        fclose(ntruPrivFile);
        free(pem);
        free(derCert);
        FreeRsaKey(&caKey);
    }
#endif /* HAVE_NTRU */
#endif /* WOLFSSL_CERT_GEN */

    wc_FreeRsaKey(&key);
#ifdef HAVE_CAVIUM
    RsaFreeCavium(&key);
#endif
    free(tmp);

    return 0;
}

#endif

#ifdef HAVE_ECC

int ecc_test(void)
{
    CRYPT_RNG_CTX     rng;
    byte    sharedA[1024];
    byte    sharedB[1024];
    byte    sig[1024];
    byte    digest[20];
    byte    exportBuf[1024];
    word32  x, y;
    int     i, verify, ret;
    ecc_key userA, userB, pubKey;

    ret = CRYPT_RNG_Initialize(&rng);
    if (ret != 0)
        return -1001;

    wc_ecc_init(&userA);
    wc_ecc_init(&userB);
    wc_ecc_init(&pubKey);

    ret = wc_ecc_make_key((struct RNG *)&rng, 32, &userA);
    ret = wc_ecc_make_key((struct RNG *)&rng, 32, &userB);

    if (ret != 0)
        return -1002;

    x = sizeof(sharedA);
    ret = wc_ecc_shared_secret(&userA, &userB, sharedA, &x);

    y = sizeof(sharedB);
    ret = wc_ecc_shared_secret(&userB, &userA, sharedB, &y);

    if (ret != 0)
        return -1003;

    if (y != x)
        return -1004;

    if (memcmp(sharedA, sharedB, x))
        return -1005;

    x = sizeof(exportBuf);
    ret = wc_ecc_export_x963(&userA, exportBuf, &x);
    if (ret != 0)
        return -1006;

    ret = wc_ecc_import_x963(exportBuf, x, &pubKey);

    if (ret != 0)
        return -1007;

    y = sizeof(sharedB);
    ret = wc_ecc_shared_secret(&userB, &pubKey, sharedB, &y);

    if (ret != 0)
        return -1008;

    if (memcmp(sharedA, sharedB, y))
        return -1010;

    /* test DSA sign hash */
    for (i = 0; i < (int)sizeof(digest); i++)
        digest[i] = i;

    x = sizeof(sig);
    ret = wc_ecc_sign_hash(digest, sizeof(digest), sig, &x, (struct RNG *)&rng, &userA);

    verify = 0;
    ret = wc_ecc_verify_hash(sig, x, digest, sizeof(digest), &verify, &userA);

    if (ret != 0)
        return -1011;

    if (verify != 1)
        return -1012;

    x = sizeof(exportBuf);
    ret = wc_ecc_export_private_only(&userA, exportBuf, &x);
    if (ret != 0)
        return -1013;

    wc_ecc_free(&pubKey);
    wc_ecc_free(&userB);
    wc_ecc_free(&userA);

    return 0;
}
#endif

#ifdef HAVE_LIBZ

const byte sample_text[] =
    "Biodiesel cupidatat marfa, cliche aute put a bird on it incididunt elit\n"
    "polaroid. Sunt tattooed bespoke reprehenderit. Sint twee organic id\n"
    "marfa. Commodo veniam ad esse gastropub. 3 wolf moon sartorial vero,\n"
    "plaid delectus biodiesel squid +1 vice. Post-ironic keffiyeh leggings\n"
    "selfies cray fap hoodie, forage anim. Carles cupidatat shoreditch, VHS\n"
    "small batch meggings kogi dolore food truck bespoke gastropub.\n"
    "\n"
    "Terry richardson adipisicing actually typewriter tumblr, twee whatever\n"
    "four loko you probably haven't heard of them high life. Messenger bag\n"
    "whatever tattooed deep v mlkshk. Brooklyn pinterest assumenda chillwave\n"
    "et, banksy ullamco messenger bag umami pariatur direct trade forage.\n"
    "Typewriter culpa try-hard, pariatur sint brooklyn meggings. Gentrify\n"
    "food truck next level, tousled irony non semiotics PBR ethical anim cred\n"
    "readymade. Mumblecore brunch lomo odd future, portland organic terry\n"
    "richardson elit leggings adipisicing ennui raw denim banjo hella. Godard\n"
    "mixtape polaroid, pork belly readymade organic cray typewriter helvetica\n"
    "four loko whatever street art yr farm-to-table.\n"
    "\n"
    "Vinyl keytar vice tofu. Locavore you probably haven't heard of them pug\n"
    "pickled, hella tonx labore truffaut DIY mlkshk elit cosby sweater sint\n"
    "et mumblecore. Elit swag semiotics, reprehenderit DIY sartorial nisi ugh\n"
    "nesciunt pug pork belly wayfarers selfies delectus. Ethical hoodie\n"
    "seitan fingerstache kale chips. Terry richardson artisan williamsburg,\n"
    "eiusmod fanny pack irony tonx ennui lo-fi incididunt tofu YOLO\n"
    "readymade. 8-bit sed ethnic beard officia. Pour-over iphone DIY butcher,\n"
    "ethnic art party qui letterpress nisi proident jean shorts mlkshk\n"
    "locavore.\n"
    "\n"
    "Narwhal flexitarian letterpress, do gluten-free voluptate next level\n"
    "banh mi tonx incididunt carles DIY. Odd future nulla 8-bit beard ut\n"
    "cillum pickled velit, YOLO officia you probably haven't heard of them\n"
    "trust fund gastropub. Nisi adipisicing tattooed, Austin mlkshk 90's\n"
    "small batch american apparel. Put a bird on it cosby sweater before they\n"
    "sold out pork belly kogi hella. Street art mollit sustainable polaroid,\n"
    "DIY ethnic ea pug beard dreamcatcher cosby sweater magna scenester nisi.\n"
    "Sed pork belly skateboard mollit, labore proident eiusmod. Sriracha\n"
    "excepteur cosby sweater, anim deserunt laborum eu aliquip ethical et\n"
    "neutra PBR selvage.\n"
    "\n"
    "Raw denim pork belly truffaut, irony plaid sustainable put a bird on it\n"
    "next level jean shorts exercitation. Hashtag keytar whatever, nihil\n"
    "authentic aliquip disrupt laborum. Tattooed selfies deserunt trust fund\n"
    "wayfarers. 3 wolf moon synth church-key sartorial, gastropub leggings\n"
    "tattooed. Labore high life commodo, meggings raw denim fingerstache pug\n"
    "trust fund leggings seitan forage. Nostrud ullamco duis, reprehenderit\n"
    "incididunt flannel sustainable helvetica pork belly pug banksy you\n"
    "probably haven't heard of them nesciunt farm-to-table. Disrupt nostrud\n"
    "mollit magna, sriracha sartorial helvetica.\n"
    "\n"
    "Nulla kogi reprehenderit, skateboard sustainable duis adipisicing viral\n"
    "ad fanny pack salvia. Fanny pack trust fund you probably haven't heard\n"
    "of them YOLO vice nihil. Keffiyeh cray lo-fi pinterest cardigan aliqua,\n"
    "reprehenderit aute. Culpa tousled williamsburg, marfa lomo actually anim\n"
    "skateboard. Iphone aliqua ugh, semiotics pariatur vero readymade\n"
    "organic. Marfa squid nulla, in laborum disrupt laboris irure gastropub.\n"
    "Veniam sunt food truck leggings, sint vinyl fap.\n"
    "\n"
    "Hella dolore pork belly, truffaut carles you probably haven't heard of\n"
    "them PBR helvetica in sapiente. Fashion axe ugh bushwick american\n"
    "apparel. Fingerstache sed iphone, jean shorts blue bottle nisi bushwick\n"
    "flexitarian officia veniam plaid bespoke fap YOLO lo-fi. Blog\n"
    "letterpress mumblecore, food truck id cray brooklyn cillum ad sed.\n"
    "Assumenda chambray wayfarers vinyl mixtape sustainable. VHS vinyl\n"
    "delectus, culpa williamsburg polaroid cliche swag church-key synth kogi\n"
    "magna pop-up literally. Swag thundercats ennui shoreditch vegan\n"
    "pitchfork neutra truffaut etsy, sed single-origin coffee craft beer.\n"
    "\n"
    "Odio letterpress brooklyn elit. Nulla single-origin coffee in occaecat\n"
    "meggings. Irony meggings 8-bit, chillwave lo-fi adipisicing cred\n"
    "dreamcatcher veniam. Put a bird on it irony umami, trust fund bushwick\n"
    "locavore kale chips. Sriracha swag thundercats, chillwave disrupt\n"
    "tousled beard mollit mustache leggings portland next level. Nihil esse\n"
    "est, skateboard art party etsy thundercats sed dreamcatcher ut iphone\n"
    "swag consectetur et. Irure skateboard banjo, nulla deserunt messenger\n"
    "bag dolor terry richardson sapiente.\n";


int compress_test(void)
{
    int ret = 0;
    word32 dSz = sizeof(sample_text);
    word32 cSz = (dSz + (word32)(dSz * 0.001) + 12);
    byte *c = NULL;
    byte *d = NULL;

    c = calloc(cSz, sizeof(byte));
    d = calloc(dSz, sizeof(byte));

    if (c == NULL || d == NULL)
        ret = -300;

    if (ret == 0 && (ret = wc_Compress(c, cSz, sample_text, dSz, 0)) < 0)
        ret = -301;

    if (ret > 0) {
        cSz = (word32)ret;
        ret = 0;
    }

    if (ret == 0 && wc_DeCompress(d, dSz, c, cSz) != (int)dSz)
        ret = -302;

    if (ret == 0 && memcmp(d, sample_text, dSz))
        ret = -303;

    if (c) free(c);
    if (d) free(d);

    return ret;
}

#endif /* HAVE_LIBZ */

/*******************************************************************************
 End of File
 */

