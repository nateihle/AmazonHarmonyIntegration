/*******************************************************************************
  File Name:
    base64.c

  Summary:
    Helper Function Used by WINC1500 Email Sending Example

  Description:
    Helper Function Used by WINC1500 Email Sending Example
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

static const char g_ccB64Tbl[64]
	= "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void ConvertToBase64(char *pcOutStr, const char *pccInStr, int iLen)
{
	const char *pccIn = (const char *)pccInStr;
	char *pcOut;
	int iCount;
	pcOut = pcOutStr;

	/* Loop in for Multiple of 24 Bits and Convert to Base 64 */
	for (iCount = 0; iLen - iCount >= 3; iCount += 3, pccIn += 3) {
		*pcOut++ = g_ccB64Tbl[pccIn[0] >> 2];
		*pcOut++ = g_ccB64Tbl[((pccIn[0] & 0x03) << 4) | (pccIn[1] >> 4)];
		*pcOut++ = g_ccB64Tbl[((pccIn[1] & 0x0F) << 2) | (pccIn[2] >> 6)];
		*pcOut++ = g_ccB64Tbl[pccIn[2] & 0x3f];
	}

	/* Check if String is not multiple of 3 Bytes */
	if (iCount != iLen) {
		unsigned char ucLastByte;

		*pcOut++ = g_ccB64Tbl[pccIn[0] >> 2];
		ucLastByte = ((pccIn[0] & 0x03) << 4);

		if (iLen - iCount > 1) {
			/* If there are 2 Extra Bytes */
			ucLastByte |= (pccIn[1] >> 4);
			*pcOut++ = g_ccB64Tbl[ucLastByte];
			*pcOut++ = g_ccB64Tbl[((pccIn[1] & 0x0F) << 2)];
		} else {
			/* If there is only 1 Extra Byte */
			*pcOut++ = g_ccB64Tbl[ucLastByte];
			*pcOut++ = '=';
		}

		*pcOut++ = '=';
	}

	*pcOut  = '\0';
}

//DOM-IGNORE-END
