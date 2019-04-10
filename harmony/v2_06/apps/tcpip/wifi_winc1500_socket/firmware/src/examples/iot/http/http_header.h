/*******************************************************************************
  File Name:
    http_header.h

  Summary:
    HTTP header definitions
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _HTTP_HEADER_H
#define _HTTP_HEADER_H

#ifdef __cplusplus
extern "C" {
#endif

#define HTTP_HEADER_ACCEPT       	    ("Accept: ")
#define HTTP_HEADER_ACCEPT_CHARSET   	("Accept-Charset: ")
#define HTTP_HEADER_ACCEPT_ENCODING  	("Accept-Encoding: ")
#define HTTP_HEADER_ACCEPT_LANGUAGE  	("Accept-Language: ")
#define HTTP_HEADER_ACCEPT_RANGES    	("Accept-Ranges: ")
#define HTTP_HEADER_AGE              	("Age: ")
#define HTTP_HEADER_ALLOW	            ("Allow: ")
#define HTTP_HEADER_AUTHORIZATION    	("Authorization: ")
#define HTTP_HEADER_CACHE_CONTROL    	("Cache-Control: ")
#define HTTP_HEADER_CONNECTION       	("Connection: ")
#define HTTP_HEADER_CONTENT_ENCODING 	("Content-Encoding: ")
#define HTTP_HEADER_CONTENT_LANGUAGE 	("Content-Language: ")
#define HTTP_HEADER_CONTENT_LENGTH   	("Content-Length: ")
#define HTTP_HEADER_CONTENT_LOCATION 	("Content-Location: ")
#define HTTP_HEADER_CONTENT_MD5      	("Content-MD5: ")
#define HTTP_HEADER_CONTENT_RANGE        ("Content-Range: ")
#define HTTP_HEADER_CONTENT_TYPE         ("Content-Type: ")
#define HTTP_HEADER_DATE                 ("Date: ")
#define HTTP_HEADER_ETAG                 ("ETag: ")
#define HTTP_HEADER_EXPECT               ("Expect: ")
#define HTTP_HEADER_EXPIRES              ("Expires: ")
#define HTTP_HEADER_FROM                 ("From: ")
#define HTTP_HEADER_HOST                 ("Host: ")
#define HTTP_HEADER_IF_MATCH             ("If-Match: ")
#define HTTP_HEADER_IF_MODIFIED_SINCE	("If-Modified-Since: ")
#define HTTP_HEADER_IF_NONE_MATCH        ("If-None-Match: ")
#define HTTP_HEADER_IF_RANGE             ("If-Range: ")
#define HTTP_HEADER_IF_UNMODIFIED_SINCE	("If-Unmodified-Since: ")
#define HTTP_HEADER_LAST_MODIFIED        ("Last-Modified: ")
#define HTTP_HEADER_LOCATION             ("Location: ")
#define HTTP_HEADER_MAX_FORWARDS         ("Max-Forwards: ")
#define HTTP_HEADER_PRAGMA               ("Pragma: ")
#define HTTP_HEADER_PROXY_AUTHENTICATE	("Proxy-Authenticate: ")
#define HTTP_HEADER_PROXY_AUTHORIZATION  ("Proxy-Authorization: ")
#define HTTP_HEADER_RANGE                ("Range: ")
#define HTTP_HEADER_REFERER              ("Referer: ")
#define HTTP_HEADER_RETRY_AFTER          ("Retry-After: ")
#define HTTP_HEADER_SERVER               ("Server: ")
#define HTTP_HEADER_TE                   ("TE: ")
#define HTTP_HEADER_TRAILER              ("Trailer: ")
#define HTTP_HEADER_TRANSFER_ENCODING	("Transfer-Encoding: ")
#define HTTP_HEADER_UPGRADE              ("Upgrade: ")
#define HTTP_HEADER_USER_AGENT           ("User-Agent: ")
#define HTTP_HEADER_VARY                 ("Vary: ")
#define HTTP_HEADER_VIA                  ("Via: ")
#define HTTP_HEADER_WARNING              ("Warning: ")
#define HTTP_HEADER_WWW_AUTHENTICATE     ("WWW-Authenticate: ")

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _HTTP_HEADER_H */
