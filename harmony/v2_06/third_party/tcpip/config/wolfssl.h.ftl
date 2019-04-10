/*******************************************************************************
  Application Header

  File Name:
    config.h

  Summary:
 config file for CyaSSL to avoid custom build options

  Description:
 config file for CyaSSL to avoid custom build options
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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


#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif


#if defined(INLINE)
#undef INLINE
#define INLINE inline
#endif

#define MICROCHIP_MPLAB_HARMONY
#define MICROCHIP_TCPIP
#define MICROCHIP_PIC32
#define WOLFSSL_HAVE_MIN
#define WOLFSSL_HAVE_MAX

<#if CONFIG_USE_3RDPARTY_WOLFSSL>

<#if !CONFIG_WOLFSSL_USE_MZ_CRYPTO>
#define MICROCHIP_PIC32_RNG
#define NEED_AES_TABLES
<#else>
#define WOLFSSL_MICROCHIP_PIC32MZ_RNG
#define WOLFSSL_PIC32MZ_CE
#define WOLFSSL_PIC32MZ_CRYPT
#define MICROCHIP_PIC32_RNG
#define HAVE_AES_ENGINE
#define WOLFSSL_PIC32MZ_RNG
/* #define WOLFSSL_PIC32MZ_HASH */
#define WOLFSSL_AES_COUNTER
#define NO_BIG_INT
</#if>
#define SIZEOF_LONG_LONG 8
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define NO_DEV_RANDOM
#define NO_FILESYSTEM
#define WOLFSSL_STATIC_RSA


<#if !CONFIG_WOLFSSL_MULTI_THREAD_SUPPORT>
#define SINGLE_THREADED
</#if>

<#if CONFIG_WOLFSSL_USE_FAST_MATH>
#define USE_FAST_MATH
</#if>
#define TFM_TIMING_RESISTANT
<#if !CONFIG_WOLFSSL_WOLFSSL_CLIENT>
#define NO_WOLFSSL_CLIENT
</#if>
<#if !CONFIG_WOLFSSL_WOLFSSL_SERVER>
#define NO_WOLFSSL_SERVER
</#if>
<#if !CONFIG_WOLFSSL_DES3>
#define NO_DES3
</#if>
<#if !CONFIG_WOLFSSL_DH>
#define NO_DH
</#if>
<#if !CONFIG_WOLFSSL_AES>
#define NO_AES
</#if>
<#if !CONFIG_WOLFSSL_DSA>
#define NO_DSA
</#if>
<#if !CONFIG_WOLFSSL_ERROR_STRINGS>
#define NO_ERROR_STRINGS
</#if>
<#if !CONFIG_WOLFSSL_HMAC>
#define NO_HMAC
</#if>
<#if !CONFIG_WOLFSSL_MD4>
#define NO_MD4
</#if>
<#if !CONFIG_WOLFSSL_SHA256>
#define NO_SHA256
</#if>
<#if !CONFIG_WOLFSSL_PSK>
#define NO_PSK
</#if>
<#if !CONFIG_WOLFSSL_PWDBASED>
#define NO_PWDBASED
</#if>
<#if !CONFIG_WOLFSSL_RC4>
#define NO_RC4
</#if>
<#if !CONFIG_WOLFSSL_RABBIT>
#define NO_RABBIT
</#if>
<#if !CONFIG_WOLFSSL_HC128>
#define NO_HC128
</#if>
<#if !CONFIG_WOLFSSL_SESSION_CACHE>
#define NO_SESSION_CACHE
</#if>
<#if !CONFIG_WOLFSSL_TLS>
#define NO_TLS
</#if>
<#if CONFIG_WOLFSSL_SMALL_SESSION_CACHE>
#define SMALL_SESSION_CACHE
</#if>
<#if CONFIG_WOLFSSL_CERT_GEN>
#define WOLFSSL_CERT_GEN
</#if>
<#if CONFIG_WOLFSSL_DER_LOAD>
#define WOLFSSL_DER_LOAD
</#if>
<#if CONFIG_WOLFSSL_DTLS>
#define WOLFSSL_DTLS
</#if>
<#if CONFIG_WOLFSSL_KEY_GEN>
#define WOLFSSL_KEY_GEN
</#if>
<#if CONFIG_WOLFSSL_RIPEMD>
#define WOLFSSL_RIPEMD
</#if>
<#if CONFIG_WOLFSSL_SHA384>
#define WOLFSSL_SHA384
</#if>
<#if CONFIG_WOLFSSL_SHA512>
#define WOLFSSL_SHA512
</#if>
<#if CONFIG_WOLFSSL_HAVE_AESCCM>
#define HAVE_AESCCM
</#if>
<#if CONFIG_WOLFSSL_HAVE_AESGCM>
#define HAVE_AESGCM
</#if>
<#if CONFIG_WOLFSSL_HAVE_CAMELLIA>
#define HAVE_CAMELLIA
</#if>
<#if CONFIG_WOLFSSL_HAVE_CRL>
#define HAVE_CRL
</#if>
<#if CONFIG_WOLFSSL_HAVE_ECC>
#define HAVE_ECC
</#if>
<#if CONFIG_WOLFSSL_HAVE_LIBZ>
#define HAVE_LIBZ
</#if>
<#if CONFIG_WOLFSSL_HAVE_OCSP>
#define HAVE_OCSP
</#if>
<#if CONFIG_WOLFSSL_OPENSSL_EXTRA>
#define OPENSSL_EXTRA
</#if>

<#if CONFIG_WOLFSSL_DEBUG_SUPPORT>
#define DEBUG_WOLFSSL
</#if>

<#if !CONFIG_WOLFSSL_OLD_TLS_SUPPORT>
#define NO_OLD_TLS
</#if>

<#if CONFIG_WOLFSSL_HAVE_TLS13>
#define WOLFSSL_TLS13
#define HAVE_TLS_EXTENSIONS
</#if>

</#if>


<#if CONFIG_USE_3RDPARTY_WOLFMQTT>
#define WOLFMQTT_NONBLOCK

<#if CONFIG_WOLFMQTT_USE_TLS>
#define ENABLE_MQTT_TLS
</#if>
</#if>


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */
