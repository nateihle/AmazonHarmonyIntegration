/*******************************************************************************
  File Name:
    growl.c

  Summary:
    Growl Client Interface
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

#include "app.h"
#include "growl.h"

#if SIMPLE_GROWL_EXAMPLE

#define MAX_REGISTERED_EVENTS               1
#define GROWL_CONNECT_RETRY                 3
#define GROWL_DNS_RETRY                     3

#define PROWL_DOMAIN_NAME                   "api.prowlapp.com"//"www.google.com.eg"//
#define PROWL_CLIENT_STRING_ID              "prowl"//"google"//
#define PROWL_API_KEY_SIZE                  40

#define NMA_DOMAIN_NAME                     "www.notifymyandroid.com"
#define NMA_CLIENT_STRING_ID                "notifymyandroid"
#define NMA_API_KEY_SIZE                    48

#define GROWL_HTTP_PORT                     80
#define GROWL_HTTPS_PORT                    443
#define GROWL_MSG_SIZE                      256

#define GROWL_STATE_IDLE                    ((uint8_t)0)
#define GROWL_STATE_REQ_PENDING             ((uint8_t)1)
#define GROWL_STATE_WAITING_RESPONSE        ((uint8_t)2)

#define GROWL_RX_TIMEOUT                    (25 * 1000)

typedef struct{
    uint32_t    serverIPAddress;
    uint16_t    port;
    SOCKET      Socket;
    uint8_t     state;
    uint8_t     *pApp;
    uint8_t     *pEvent;
    uint8_t     *pMsg;
}tstrNotification;

static volatile tstrNotification        gstrNotificationProwl;
static volatile tstrNotification        gstrNotificationNMA;
static const uint8_t                    hexDigits[] = "0123456789abcdef";
static volatile uint8_t                 *prwKey;
static volatile uint8_t                 *nmaKey;
static volatile uint8_t                 msg[GROWL_DESCRIPTION_MAX_LENGTH];

static uint16_t Encode(uint8_t *pcStr, uint8_t *pcEncoded)
{
    uint8_t     *pcTmp = pcStr;
    uint8_t     *pcbuf = pcEncoded;
    uint16_t    u16Count = 0;

    while (*pcTmp)
    {
        if (
            ((*pcTmp >= '0') && (*pcTmp <= '9')) ||
            ((*pcTmp >= 'a') && (*pcTmp <= 'z')) ||
            ((*pcTmp >= 'A') && (*pcTmp <= 'Z')) ||
            (*pcTmp == '-') ||
            (*pcTmp == '_') ||
            (*pcTmp == '.') ||
            (*pcTmp == '~')
        )
        {
            *pcbuf++ = *pcTmp;
        }
        else
        {
            *pcbuf++ = '%';
            *pcbuf++ = hexDigits[(*pcTmp>> 4) & 0x0F];
            *pcbuf++ = hexDigits[(*pcTmp & 0x0F)];
            u16Count += 2;
        }
        pcTmp++;
        u16Count++;
    }
    return u16Count;
}

static uint16_t FormatMsg(uint8_t clientName, uint8_t *pMsg)
{
    uint16_t    u16Tmp;
    uint16_t    u16MsgOffset = 0;
    tstrNotification *strNotification;

    if (clientName == NMA_CLIENT)
    {
        strNotification = (tstrNotification *)(&gstrNotificationNMA);
        /* Put the start of the HTTP Request message. */
        u16Tmp = sizeof("GET /publicapi/notify?apikey=") - 1;
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)"GET /publicapi/notify?apikey=", u16Tmp);
        u16MsgOffset += u16Tmp;

        /* Add the API Key to the message. */
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)nmaKey, NMA_API_KEY_SIZE);
        u16MsgOffset+= NMA_API_KEY_SIZE;
    }
    else if (clientName == PROWL_CLIENT)
    {
        strNotification = (tstrNotification *)(&gstrNotificationProwl);

        /* Put the start of the HTTP Request message. */
        u16Tmp = sizeof("GET /publicapi/add?apikey=") - 1;
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)"GET /publicapi/add?apikey=", u16Tmp);
        u16MsgOffset += u16Tmp;

        /* Add the API Key to the message. */
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)prwKey, PROWL_API_KEY_SIZE);
        u16MsgOffset+= PROWL_API_KEY_SIZE;
    }
    else
        return 0;

    /* Encode the Application name and append it to the message. */
    u16Tmp = sizeof("&application=") - 1;
    memcpy(&pMsg[u16MsgOffset], (uint8_t *)"&application=", u16Tmp);
    u16MsgOffset += u16Tmp;
    u16Tmp = Encode((uint8_t *)strNotification->pApp, &pMsg[u16MsgOffset]);
    u16MsgOffset += u16Tmp;

    /* Encode the Event name and append it to the message. */
    u16Tmp = sizeof("&event=") - 1;
    memcpy(&pMsg[u16MsgOffset], (uint8_t *)"&event=", u16Tmp);
    u16MsgOffset += u16Tmp;
    u16Tmp = Encode((uint8_t *)strNotification->pEvent, &pMsg[u16MsgOffset]);
    u16MsgOffset += u16Tmp;

    /* Encode the Description message and append it to the message. */
    u16Tmp = sizeof("&description=") - 1;
    memcpy(&pMsg[u16MsgOffset], (uint8_t *)"&description=", u16Tmp);
    u16MsgOffset += u16Tmp;
    u16Tmp = Encode((uint8_t *)strNotification->pMsg, &pMsg[u16MsgOffset]);
    u16MsgOffset += u16Tmp;

    u16Tmp = sizeof(" HTTP/1.1\r\nHost: ") - 1;
    memcpy(&pMsg[u16MsgOffset], (uint8_t *)" HTTP/1.1\r\nHost: ", u16Tmp);
    u16MsgOffset += u16Tmp;

    if (clientName == NMA_CLIENT)
    {
        u16Tmp = sizeof(NMA_DOMAIN_NAME) - 1;
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)NMA_DOMAIN_NAME, u16Tmp);
        u16MsgOffset += u16Tmp;
    }
    else if (clientName == PROWL_CLIENT)
    {
        u16Tmp = sizeof(PROWL_DOMAIN_NAME) - 1;
        memcpy(&pMsg[u16MsgOffset], (uint8_t *)PROWL_DOMAIN_NAME, u16Tmp);
        u16MsgOffset += u16Tmp;
    }

    u16Tmp = sizeof("\r\n\r\n") - 1;
    memcpy(&pMsg[u16MsgOffset], (uint8_t *)"\r\n\r\n", u16Tmp);
    u16MsgOffset += u16Tmp;
    pMsg[u16MsgOffset] = '\0';
    u16MsgOffset++;

    return u16MsgOffset;
}

static uint8_t GetResponseCode(uint8_t *p_rxBuf, uint16_t bufferSize)
{
    uint8_t u8Code = 0xFF;

    if ((p_rxBuf != NULL) && (bufferSize > 0))
    {
        uint16_t u16Offset = 0;
        do
        {
            if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"HTTP/1.1 ", 9))
            {
                u16Offset += 9;
                if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"200", 3))
                {
                    u8Code = 20;
                }
                else if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"400", 3))
                {
                    u8Code = 40;
                }
                else if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"401", 3))
                {
                    u8Code = 41;
                }
                else if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"402", 3))
                {
                    u8Code = 42;
                }
                else if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"406", 3))
                {
                    u8Code = 46;
                }
                else if (!memcmp(&p_rxBuf[u16Offset], (uint8_t *)"409", 3))
                {
                    u8Code = 49;
                }
                else
                {
                    u8Code = 50;
                }
                break;
            }
            u16Offset++;
        } while (u16Offset < (bufferSize - 9));
    }
    return u8Code;
}

/*
 * Growl notification callback. Pointer to a function delivering Growl events.
 *
 * u8Code: possible error codes could be returned by the NMA server and refer to the comments in the growl.h.
 *   [20] GROWL_SUCCESS
 *   [40] GROWL_ERR_BAD_REQUEST
 *   [41] GROWL_ERR_NOT_AUTHORIZED
 *   [42] GROWL_ERR_NOT_ACCEPTED
 *   [46] GROWL_ERR_API_EXCEED
 *   [49] GROWL_ERR_NOT_APPROVED
 *   [50] GROWL_ERR_SERVER_ERROR
 *   [30] GROWL_ERR_LOCAL_ERROR
 *   [10] GROWL_ERR_CONN_FAILED
 *   [11] GROWL_ERR_RESOLVE_DNS
 *   [12] GROWL_RETRY
 * clientID: client ID returned by the NMA server.
 */
static void growl_cb(uint8_t u8Code, uint8_t clientID)
{
    SYS_CONSOLE_PRINT("growl_cb: %d \r\n", u8Code);
}

static void resolve_cb(uint8_t *p_HostName, uint32_t serverIP)
{
    uint8_t                 clientID = 0;
    tstrNotification        *pstrNotification = NULL;
    struct sockaddr_in      strAddr;

    if (strstr((const char *)p_HostName, (const char *)NMA_CLIENT_STRING_ID))
    {
        clientID = NMA_CLIENT;
        pstrNotification = (tstrNotification *)(&gstrNotificationNMA);
    }
    else if (strstr((const char *)p_HostName, (char *)PROWL_CLIENT_STRING_ID))
    {
        clientID = PROWL_CLIENT;
        pstrNotification = (tstrNotification *)(&gstrNotificationProwl);
    }

    if (serverIP != 0)
    {
        if (pstrNotification->serverIPAddress == 0)
        {
            pstrNotification->serverIPAddress = serverIP;
        }

        strAddr.sin_family = AF_INET;
        strAddr.sin_port = pstrNotification->port;
        strAddr.sin_addr.s_addr = serverIP;

        connect(pstrNotification->Socket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
    }
    else
    {
        static uint8_t      u8Retry = GROWL_DNS_RETRY;
        if (u8Retry--)
        {
            SYS_CONSOLE_PRINT("Retry resolving DNS\n");
            if (strstr((const char *)p_HostName, (const char *)NMA_CLIENT_STRING_ID))
                gethostbyname((uint8_t *)NMA_DOMAIN_NAME);
            else if (strstr((const char *)p_HostName, (const char *)PROWL_CLIENT_STRING_ID))
                gethostbyname((uint8_t *)PROWL_DOMAIN_NAME);
        }
        else
        {
            close(pstrNotification->Socket);
            pstrNotification->Socket = -1/*0xFF*/;
            pstrNotification->state = GROWL_STATE_IDLE;
            u8Retry = GROWL_DNS_RETRY;
            SYS_CONSOLE_PRINT("Failed to resolve DNS\n");
            growl_cb(GROWL_ERR_RESOLVE_DNS, clientID);
        }
    }
}

static void socket_cb(SOCKET sock, uint8_t message, void *pvMsg)
{
    uint8_t clientID;
    tstrNotification *pstrNotification;

    if (sock == gstrNotificationNMA.Socket)
    {
        clientID = NMA_CLIENT;
        pstrNotification = (tstrNotification *)(&gstrNotificationNMA);
    }
    else if (sock == gstrNotificationProwl.Socket)
    {
        clientID = PROWL_CLIENT;
        pstrNotification = (tstrNotification *)(&gstrNotificationProwl);
    }
    else
    {
        return;
    }

    if (message == SOCKET_MSG_CONNECT)
    {
        static uint8_t u8Retry = GROWL_CONNECT_RETRY;
        t_socketConnect *pstrConnect = (t_socketConnect *)pvMsg;
        if (pstrConnect->error == 0)
        {
            uint8_t acBuffer[GROWL_MSG_SIZE];
            uint16_t u16MsgSize;

            u16MsgSize = FormatMsg(clientID, acBuffer);
            send(sock, acBuffer, u16MsgSize, 0);
            recv(pstrNotification->Socket, (void *)msg, GROWL_DESCRIPTION_MAX_LENGTH, GROWL_RX_TIMEOUT);
            u8Retry = GROWL_CONNECT_RETRY;
        }
        else
        {
            if ((u8Retry--) > 0)
            {
                SYS_CONSOLE_PRINT("Retry %s\n", (clientID == NMA_CLIENT) ? "NMA" : "PROWL");
                if (clientID == NMA_CLIENT)
                {
                    resolve_cb((uint8_t *)NMA_DOMAIN_NAME, pstrNotification->serverIPAddress);
                }
                else if (clientID == PROWL_CLIENT)
                {
                    resolve_cb((uint8_t *)PROWL_DOMAIN_NAME, pstrNotification->serverIPAddress);
                }
            }
            else
            {
                SYS_CONSOLE_PRINT("%s connection failed\n", (clientID == NMA_CLIENT) ? "NMA" : "PROWL");
                close(pstrNotification->Socket);
                pstrNotification->Socket = -1/*0xFF*/;
                pstrNotification->state = GROWL_STATE_IDLE;
                growl_cb(GROWL_ERR_CONN_FAILED, clientID);
                u8Retry = GROWL_CONNECT_RETRY;
            }
        }
    }
    else if (message == SOCKET_MSG_RECV)
    {
        tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg *)pvMsg;
        static uint8_t u8Error = 0xFF;
        uint8_t u8Reset = 1;

        if ((pstrRecvMsg->pu8Buffer != NULL) && (pstrRecvMsg->s16BufferSize > 0))
        {
            if (u8Error == 0xFF)
            {
                u8Error = GetResponseCode(pstrRecvMsg->pu8Buffer, pstrRecvMsg->s16BufferSize);
            }
        }
        if ((pstrRecvMsg->s16BufferSize > 0) && (pstrRecvMsg->u16RemainingSize != 0))
        {
            u8Reset = 0;
        }
        if (u8Reset)
        {
            close(pstrNotification->Socket);
            pstrNotification->Socket = -1/*0xFF*/;
            pstrNotification->state = GROWL_STATE_IDLE;
            growl_cb(u8Error, clientID);
            u8Error = 0xFF;
        }
    }
    else if (message == SOCKET_MSG_SEND)
    {
        int16_t s16Sent = *((int16_t *)pvMsg);

        if (s16Sent <= 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_INFO, "Growl send error %d\n", s16Sent);
        }
    }
}

void NMI_GrowlInit(uint8_t *pu8NmaKey, uint8_t *pu8PrwKey)
{
	/* Initialize socket module. */
	socketInit();
	registerSocketCallback(socket_cb, resolve_cb);

    if (pu8NmaKey)
    {

        nmaKey = pu8NmaKey;
    }
    else
    {
        SYS_CONSOLE_PRINT("NMA key NOT valid\n");
    }
    if (pu8PrwKey)
    {
        prwKey = pu8PrwKey;
    }
    else
    {
        SYS_CONSOLE_PRINT("Prowl key NOT valid\n");
    }
    memset((uint8_t *)&gstrNotificationNMA, 0, sizeof(tstrNotification));
    memset((uint8_t *)&gstrNotificationProwl, 0, sizeof(tstrNotification));
    gstrNotificationNMA.Socket = -1/*0xFF*/;
    gstrNotificationProwl.Socket = -1/*0xFF*/;
}

void NMI_GrowldeInit(void)
{
    memset((uint8_t *)&gstrNotificationNMA, 0, sizeof(tstrNotification));
    memset((uint8_t *)&gstrNotificationProwl, 0, sizeof(tstrNotification));
    if (gstrNotificationNMA.Socket != -1/*0xFF*/)
    {
        close(gstrNotificationNMA.Socket);
    }
    if (gstrNotificationProwl.Socket != -1/*0xFF*/)
    {
        close(gstrNotificationProwl.Socket);
    }
    gstrNotificationNMA.Socket = -1/*0xFF*/;
    gstrNotificationProwl.Socket = -1/*0xFF*/;
}

// Note: it's required to keep the {pApp, pEvent, pu8Description} pointers const or global.
int8_t NMI_GrowlSendNotification(uint8_t clientName, uint8_t *pApp, uint8_t *pEvent, uint8_t *pu8Description, uint8_t bUseSSL)
{
    int8_t retVal = 0;
    uint8_t u8Flags = 0;

    if ((clientName > 0) && (pApp != NULL) && (pEvent != NULL) && (pu8Description != NULL))
    {
        tstrNotification *pstrNotification;
        if ((clientName == NMA_CLIENT))
        {
            if (nmaKey == NULL)
            {
                SYS_CONSOLE_PRINT("NMA key NOT valid\n");
                return -1;
            }
            pstrNotification = (tstrNotification *)(&gstrNotificationNMA);
        }
        else if ((clientName == PROWL_CLIENT))
        {
            if (prwKey == NULL)
            {
                SYS_CONSOLE_PRINT("Prowl key NOT valid\n");
                return -1;
            }
            pstrNotification = (tstrNotification *)(&gstrNotificationProwl);
        }
        else
        {
            return -1;
        }

        if (pstrNotification->state == GROWL_STATE_IDLE)
        {
            if ((strlen((const char *)pu8Description) < GROWL_DESCRIPTION_MAX_LENGTH) &&
                (strlen((const char *)pApp) < GROWL_APPNAME_MAX_LENGTH) &&
                (strlen((const char *)pEvent) < GROWL_EVENT_MAX_LENGTH))
            {
                pstrNotification->pApp = pApp;
                pstrNotification->pEvent = pEvent;
                pstrNotification->pMsg = pu8Description;
                pstrNotification->port = _htons(GROWL_HTTP_PORT);

                /* Create Connection to the NMA Server. */
                if (bUseSSL)
                {
                    u8Flags = SOCKET_FLAGS_SSL;
                    pstrNotification->port = _htons(GROWL_HTTPS_PORT);
                }

                pstrNotification->Socket = socket(AF_INET, SOCK_STREAM, u8Flags);
                if (pstrNotification->Socket >= 0)
                {
                    pstrNotification->state = GROWL_STATE_REQ_PENDING;
                    if (clientName == NMA_CLIENT)
                        gethostbyname((uint8_t *)NMA_DOMAIN_NAME);
                    else if (clientName == PROWL_CLIENT)
                        gethostbyname((uint8_t *)PROWL_DOMAIN_NAME);
                }
                else
                {
                    SYS_ERROR_PRINT(SYS_ERROR_INFO, "No socket available for the current request\n");
                    retVal = -1;
                }
            }
            else
            {
                SYS_ERROR_PRINT(SYS_ERROR_INFO, "Msg size is too long\n");
                retVal = -1;
            }
        }
        else
        {
            SYS_ERROR_PRINT(SYS_ERROR_INFO, "Another %s request is pending\n", (clientName == NMA_CLIENT) ? "NMA" : "PROWL");
            retVal = -1;
        }
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

#endif /* SIMPLE_GROWL_EXAMPLE */

//DOM-IGNORE-END
