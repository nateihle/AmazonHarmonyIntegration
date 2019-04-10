/*******************************************************************************
  File Name:
    mdns.c

  Summary:
    WINC1500 mDNS Example

  Description:
    This example demonstrates the use of mDNS with the WINC1500.
    Example starts mDNS server with DNS service discovery (DNS-SD) support.
    One can resolve the IP address and service of the device by the host
    name within small networks. In this example, the device's host name is
    "WINC1500_HOST" and the device offers a service with the dummy HTTP
    server for the demo purpose. Once the device gets connected to a
    network, pick up your phone, and have it hooked up with the same network.
    Then open ZeroConfig app and you can find "Dummy HTTP server" on the
    service tab. Click it then You should see "Microchip WINC1500 mDNS example"
    string on your web browser.

    The configuration defines for this demo are:
        WLAN_SSID           -- AP to connect to
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        MDNS_SERVER_PORT    -- MDNS port number, 5353
        MDNS_SERVER_ADDR    -- MDNS IP address, "224.0.0.251"
        DEVICE_HOST_NAME    -- Host name of this device
        DEVICE_SERVICE_NAME -- Service name this device offers
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

#include <stdarg.h>

#include "app.h"
#include "socket.h"
#include "m2m_wifi.h"
#include "mdns.h"

#if MDNS_EXAMPLE

#define WLAN_SSID                   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /**< Password for destination SSID */

//#define MDNS_MODULE_DEBUG
//#define MDNS_MODULE_TRACE

#ifdef MDNS_MODULE_DEBUG
#define MDNS_DEBUG(...) do {SYS_CONSOLE_PRINT(__VA_ARGS__);} while (0)
#else
#define MDNS_DEBUG(...)
#endif

#ifdef MDNS_MODULE_TRACE
#define MDNS_TRACE() do {SYS_CONSOLE_PRINT("%s, %d\r\n",__FUNCTION__,__LINE__);} while (0)
#define MDNS_TRACE_DIR(in) do {SYS_CONSOLE_PRINT("%s, %d : %s\r\n",__FUNCTION__,__LINE__,in ? "enter" : "exit");} while (0)
#define MDNS_TRACE_IN() MDNS_TRACE_DIR(1)
#define MDNS_TRACE_OUT() MDNS_TRACE_DIR(0)
#else /* !MDNS_MODULE_TRACE */
#define MDNS_TRACE()
#define MDNS_TRACE_IN()
#define MDNS_TRACE_OUT()
#endif /* MDNS_MODULE_TRACE */

#define EXAMPLE_HEADER \
"\r\n=====================\r\n"\
    "WINC1500 mDNS Example\r\n"\
    "=====================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

#define mdns_state_get() s_mdns_state
#define mdns_state_set(x) do {s_mdns_state = x;} while (0)

typedef enum
{
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_WIFI_CONNECT_WAIT,
	APP_NET_UP,
	APP_END,
	APP_PARK
} APP_STATE;

#define WAIT_FOR_SENDTO_COMPLETION() WDRV_SEM_TAKE(&s_sendto_sync, OSAL_WAIT_FOREVER)
#define WAIT_FOR_RECVFROM_COMPLETION() WDRV_SEM_TAKE(&s_recvfrom_sync, OSAL_WAIT_FOREVER)
#define WAIT_FOR_RESPONDER_EVENT() WDRV_SEM_TAKE(&s_responder_sync, OSAL_WAIT_FOREVER)

#define SENDTO_COMPLETE_NOTIFY() WDRV_SEM_GIVE(&s_sendto_sync)
#define RECVFROM_COMPLETE_NOTIFY() WDRV_SEM_GIVE(&s_recvfrom_sync)
#define RESPONDER_EVENT_NOTIFY() WDRV_SEM_GIVE(&s_responder_sync)

#define MDNS_DESC() &s_mdns_desc

static DNSDesc_t s_mdns_desc;
static MDNS_MODULE_STATE s_mdns_state;
static IPV4_ADDR s_my_ip;
static OSAL_SEM_HANDLE_TYPE s_sendto_sync;
static OSAL_SEM_HANDLE_TYPE s_recvfrom_sync;
static OSAL_SEM_HANDLE_TYPE s_responder_sync;
static TaskHandle_t s_responder_handle;
static uint16_t s_mdns_rxlen;
static MDNS_CONFIG s_mdns_config;

/** Demo state */
static APP_STATE s_app_state;

/** Connection flag */
static bool s_connected;

/** Socket for mDNS */
static SOCKET s_mdns_sock;

static volatile bool s_mdns_deinit_inprogress;

/** Socket for dummy http server */
static SOCKET s_http_server_sock;
static uint8 s_http_buf[1460];

static void _mDNSPutString(uint8_t* string, DNSDesc_t * pDNSdesc);
static bool _SendTo(DNSDesc_t *pDNSdesc);

static int32_t mDNSv4Initialize(void)
{
	uint32_t mdns_addr;
	struct sockaddr_in addr;

	socketInit();

	s_mdns_sock = socket(AF_INET, SOCK_DGRAM, 0);

	WDRV_ASSERT(s_mdns_sock >= 0, "Failed to create mDNS socket");

	SYS_CONSOLE_PRINT("Initializing mDNS...\r\n");

	mdns_addr = nmi_inet_addr(MDNS_SERVER_ADDR);
	setsockopt(s_mdns_sock, 1, IP_ADD_MEMBERSHIP, &mdns_addr, sizeof(mdns_addr));

	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MDNS_SERVER_PORT);
	addr.sin_addr.s_addr = mdns_addr;
	bind(s_mdns_sock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));

	return s_mdns_sock;
}

static void mDNSDeinitialize(void)
{
    MDNS_TRACE();
}

void mdns_initialize(void)
{
    mdns_state_set(MDNS_INIT);
}

size_t strncpy_m(char* destStr, size_t destSize, int nStrings, ...)
{
    va_list     args;
    const char* str;
    char*       end;
    size_t      len;

    destStr[0] = '\0';
    end = destStr + destSize - 1;
    *end = '\0';
    len = 0;

    va_start( args, nStrings );

    while (nStrings--)
    {
        if (*end)
        {   // if already full don't calculate strlen outside the string area
            len = destSize;
            break;
        }

        str = va_arg(args, const char *);
        strncpy(destStr + len, str, destSize - len);
        len += strlen(str);
    }

    va_end( args );

    return len;
}

static void _mDNSCountersReset(mDNSProcessCtx_common *pHeader, bool bResetProbeConflictCount)
{
    MDNS_TRACE();

    if (bResetProbeConflictCount)
    {
        pHeader->nProbeConflictCount = 0;
    }

    pHeader->nProbeCount = 0;
    pHeader->nClaimCount = 0;
    pHeader->bLateConflictSeen = false;
    pHeader->bProbeConflictSeen = false;
}

static void _mDNSFillHostRecord(DNSDesc_t *pDNSdesc)
{
   uint8_t i;

   MDNS_TRACE();

   // Fill the type A resource record
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name     = pDNSdesc->mHostCtx.szHostName;
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].type.Val = QTYPE_A; // Query Type is Answer
   // CLASS 1=INternet, 255=Any class etc
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ttl.Val  = RESOURCE_RECORD_TTL_VAL;

   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.Val = 4u; // 4-byte for IP address

   // Fill in the data for an A RR record (IP address)
   for (i=0; i<=3; i++)
      pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[i] = pDNSdesc->netIPAddr.v[i];

   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].valid    = 1;
   pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].pOwnerCtx = (mDNSProcessCtx_common *) &pDNSdesc->mHostCtx;
}

static MDNSD_ERR_CODE _mDNSHostRegister(char *host_name, DNSDesc_t *pDNSdesc)
{

   MDNS_TRACE();

   memcpy((char*)pDNSdesc->mHostCtx.szUserChosenHostName
          , host_name
          , sizeof(pDNSdesc->mHostCtx.szUserChosenHostName));

   strncpy_m((char*)pDNSdesc->mHostCtx.szHostName
            , sizeof(pDNSdesc->mHostCtx.szHostName)
            , 3
            , pDNSdesc->mHostCtx.szUserChosenHostName
            , "."
            , pDNSdesc->CONST_STR_local);

   pDNSdesc->mHostCtx.szUserChosenHostName[MAX_HOST_NAME_SIZE-1]=0;
   pDNSdesc->mHostCtx.szHostName[MAX_HOST_NAME_SIZE-1]=0;

  _mDNSCountersReset((mDNSProcessCtx_common *) &pDNSdesc->mHostCtx, true);
   pDNSdesc->mHostCtx.common.type   = MDNS_CTX_TYPE_HOST;
   pDNSdesc->mHostCtx.common.state  = MDNS_STATE_INIT;
   pDNSdesc->mHostCtx.common.nInstanceId = 0;

   // Now create a QTYPE_A record for later use when answering queries.
   _mDNSFillHostRecord(pDNSdesc);
   pDNSdesc->mResponderCtx.bLastMsgIsIncomplete = false;

   pDNSdesc->mHostCtx.common.state = MDNS_STATE_INIT;

   return MDNSD_SUCCESS;
}

static bool mDNSInitialize(void)
{
    DNSDesc_t *desc;

    MDNS_TRACE();

    desc = MDNS_DESC();
    desc->netIPAddr.Val = s_my_ip.Val;
    desc->mResponderCtx.query_id.Val = 0;
    desc->mResponderCtx.prev_ipaddr.Val = desc->netIPAddr.Val;
    strncpy(desc->CONST_STR_local,"local", 6);
    _mDNSHostRegister(s_mdns_config.host_name, desc);
    /* Initialize MDNS-Responder by opening up Multicast-UDP-Socket */
    desc->mHostCtx.common.state = MDNS_STATE_INIT;
    desc->mDNS_socket = INVALID_UDP_SOCKET;

    return true;
}

static uint16_t _ReadRecvBuffer(uint16_t wLen, uint8_t *pcString, DNSDesc_t *pDNSdesc, bool offsetReset)
{
   uint16_t wOffset;
   uint16_t readLen;

   if (offsetReset)
            pDNSdesc->bytesRemained = pDNSdesc->totalBytes;

   readLen = pDNSdesc->bytesRemained > wLen ? wLen : pDNSdesc->bytesRemained;

   if (pcString != NULL) {
        wOffset = pDNSdesc->totalBytes - pDNSdesc->bytesRemained;
        memcpy(pcString, pDNSdesc->rxBuf + wOffset, readLen);
   }

   pDNSdesc->bytesRemained -= readLen;

   return readLen;
}

static uint16_t _WriteTxBuffer(DNSDesc_t *pDNSdesc, uint8_t *data, uint16_t size)
{
    uint16_t offset = pDNSdesc->txBuffOffset;

    if (offset + size > MDNS_BUFFER_SIZE) {
        WDRV_ASSERT(false, "");
        return 0;
    }
    memcpy(pDNSdesc->txBuf + offset, data, size);
    pDNSdesc->txBuffOffset += size;

    return size;
}

static uint16_t _mDNSDeCompress(uint16_t wPos
                                   ,uint8_t *pcString
                                   ,bool bFollowPtr
                                   ,uint8_t cElement
                                   ,uint8_t cDepth
                                   ,DNSDesc_t *pDNSdesc)
{
   uint16_t rr_len = 0; // As is in the packet. Could be in compressed format.
   uint16_t startOffset, endOffset;
   uint8_t i, tmp;
   uint16_t offset_in_ptr;
   uint8_t substr_len;

   MDNS_TRACE();

   startOffset = wPos;

   while (1)
   {
      rr_len++;
      if (_ReadRecvBuffer(1, &substr_len, pDNSdesc, false) == 0)
         break;

      if (substr_len == 0u)
      {
         if (pcString)
         {
            *pcString++ = '\0';
         }
         break;
      }

      if ((substr_len & 0xC0) == 0xC0) // b'11 at MSb indicates compression ptr
      {
         offset_in_ptr = substr_len & 0x3F; // the rest of 6 bits is part of offset_in_ptr.
         offset_in_ptr = offset_in_ptr << 8;

         /* Remove label-ptr byte */
         rr_len++;
         _ReadRecvBuffer(1, &i, pDNSdesc, false);
         offset_in_ptr += i;

         if (bFollowPtr)
         {
            cDepth++;

            _ReadRecvBuffer(offset_in_ptr, NULL, pDNSdesc, true);
            _mDNSDeCompress(offset_in_ptr, pcString, bFollowPtr, cElement, cDepth,pDNSdesc);

            // compressed ptr is always the last element
            break;
         }

         break;
      }
      else
      {
         if (pcString)
         {
            if (cElement > 0)
            {
               // not the first element in name
               *pcString++ = '.';
            }

            _ReadRecvBuffer(substr_len, pcString, pDNSdesc, false);
            pcString += substr_len;
         }
         else
         {
            i = substr_len;
            _ReadRecvBuffer(i, &tmp, pDNSdesc, false);
            i = 0;
         }

         cElement++;
         rr_len += substr_len;
      }
   }

   endOffset = startOffset + rr_len;
   _ReadRecvBuffer(endOffset, NULL, pDNSdesc, true);

   return rr_len;
}

static bool
_mDNSTieBreaker(mDNSResourceRecord *their, mDNSResourceRecord *our)
{
   bool WeWonTheTieBreaker = true;
   uint8_t i;

   if (their->type.Val == QTYPE_A)
   {
      for (i = 0; i<= 3; i++)
      {
         if (their->ip.v[i] < our->ip.v[i])
         {
            WeWonTheTieBreaker = true;
            break;
         }
         else if (their->ip.v[i] > our->ip.v[i])
         {
            WeWonTheTieBreaker = false;
            break;
         }
      }
   }
   else if (their->type.Val == QTYPE_SRV)
   {
      if (their->srv.port.Val >= our->srv.port.Val)
      {
         WeWonTheTieBreaker = false;
      }
   }

   MDNS_DEBUG( (char *) (WeWonTheTieBreaker ? "   tie-breaker won\r\n" : "   tie-breaker lost\r\n") );

   return WeWonTheTieBreaker;
}

static uint8_t _strcmp_local_ignore_case(uint8_t *str_1, uint8_t *str_2)
{
    if (str_1 == NULL || str_2 == NULL)
    {
        MDNS_DEBUG("strmcmp_local_ignore_case: String is NULL \r\n");
        return -1;
    }

    while (*str_1 && *str_2){
        if (*str_1 == *str_2 || (*str_1-32) == *str_2 ||
         *str_1 == (*str_2-32))
      {
         str_1++;
         str_2++;
            continue;
      }
      else
         return 1;

    }
    if (*str_1 == '\0' && *str_2 == '\0')
        return 0;
    else
        return 1;

}

static uint8_t _mDNSProcessIncomingRR(MDNS_RR_GROUP     tag
                      ,MDNS_MSG_HEADER *pmDNSMsgHeader
                      ,uint16_t         idxGroup
                      ,uint16_t         idxRR
                      ,DNSDesc_t       *pDNSdesc)
{
   mDNSResourceRecord res_rec;
   uint8_t name[2 * MAX_RR_NAME_SIZE];
   uint8_t i,j;
   uint16_t len;
   mDNSProcessCtx_common *pOwnerCtx;
   mDNSResourceRecord      *pMyRR;
   bool WeWonTheTieBreaker = false;
   bool bMsgIsAQuery;         // QUERY or RESPONSE ?
   bool bSenderHasAuthority;   // Sender has the authority ?
   uint8_t rxBuffer[6];

   MDNS_TRACE();

   bMsgIsAQuery = (pmDNSMsgHeader->flags.bits.qr == 0);
   bSenderHasAuthority = (pmDNSMsgHeader->flags.bits.qr == 1);

   res_rec.name = name; // for temporary name storage.

   // NAME
   memset(name, 0, sizeof(name));
   len = _mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
   pDNSdesc->mDNS_offset += len;

   // TYPE & CLASS
   _ReadRecvBuffer(4, rxBuffer, pDNSdesc, false);
   res_rec.type.v[1] = rxBuffer[0];
   res_rec.type.v[0] = rxBuffer[1];
   res_rec.class.v[1] = rxBuffer[2];
   res_rec.class.v[0] = rxBuffer[3];

   pDNSdesc->mDNS_offset += 4;

   // Do the first round name check
   for (i = 0; i < MAX_RR_NUM; i++)
   {
      pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = false;

      if (
         !_strcmp_local_ignore_case((void *)name, pDNSdesc->mResponderCtx.rr_list[i].name)
         &&
         ((res_rec.type.Val == QTYPE_ANY) ||
          (res_rec.type.Val == pDNSdesc->mResponderCtx.rr_list[i].type.Val))
         )
      {
         pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = true;
      }
      else if (
         (tag == MDNS_RR_GROUP_QD)
         &&
         !_strcmp_local_ignore_case(name,(uint8_t *) "_services._dns-sd._udp.local")
         &&
         (res_rec.type.Val == QTYPE_PTR)
         )
      {
         pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = true;
      }
   }


   // Only AN, NS, AR records have extra fields
   if ( tag != MDNS_RR_GROUP_QD )
   {

       // Now retrieve those extra fields
       _ReadRecvBuffer(6, rxBuffer, pDNSdesc, false);
       res_rec.ttl.v[3] = rxBuffer[0];
       res_rec.ttl.v[2] = rxBuffer[1];
       res_rec.ttl.v[1] = rxBuffer[2];
       res_rec.ttl.v[0] = rxBuffer[3];
       res_rec.rdlength.v[1] = rxBuffer[4];
       res_rec.rdlength.v[0] = rxBuffer[5];

       pDNSdesc->mDNS_offset += 6;

       // The rest is record type dependent
       switch (res_rec.type.Val)
       {
       case QTYPE_A:
          _ReadRecvBuffer(4, &res_rec.ip.v[0], pDNSdesc, false);

          pDNSdesc->mDNS_offset += 4;

          break;

       case QTYPE_PTR:

          memset(name, 0 , sizeof(name));
          len = _mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
          pDNSdesc->mDNS_offset += len;

          break;

       case QTYPE_SRV:
          _ReadRecvBuffer(6, rxBuffer, pDNSdesc, false);

          res_rec.srv.priority.v[1] = rxBuffer[0];
          res_rec.srv.priority.v[0] = rxBuffer[1];
          res_rec.srv.weight.v[1] = rxBuffer[2];
          res_rec.srv.weight.v[0] = rxBuffer[3];
          res_rec.srv.port.v[1] = rxBuffer[4];
          res_rec.srv.port.v[0] = rxBuffer[5];

          pDNSdesc->mDNS_offset += 6;

          memset(name, 0 , sizeof(name));
          len = _mDNSDeCompress(pDNSdesc->mDNS_offset, name, true, 0, 0,pDNSdesc);
          pDNSdesc->mDNS_offset += len;

          break;

       case QTYPE_TXT:
       default:

          // Still needs to read it off
          _ReadRecvBuffer(res_rec.rdlength.Val, NULL, pDNSdesc, false);

          pDNSdesc->mDNS_offset += res_rec.rdlength.Val;
          break;
       }

       // We now have all info about this received RR.
   }

   // Do the second round
   for (i = 0; i < MAX_RR_NUM; i++)
   {
      pMyRR = &(pDNSdesc->mResponderCtx.rr_list[i]);
      pOwnerCtx = pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx;

      if ( (!pMyRR->bNameAndTypeMatched) || (pOwnerCtx == NULL) )
      {
         // do nothing
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_QD) &&
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // Simple reply to an incoming DNS query.
         // Mark all of our RRs for reply.

         for (j = 0; j < MAX_RR_NUM; j++)
         {
            pDNSdesc->mResponderCtx.rr_list[j].bResponseRequested = true;
         }
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_AN) &&
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // An answer in the incoming DNS query.
         // Look for possible duplicate (known) answers suppression.
         if ((((res_rec.type.Val == QTYPE_PTR) && (res_rec.ip.Val == pDNSdesc->mResponderCtx.rr_list[i].ip.Val))
             ||
            (!_strcmp_local_ignore_case(name, pDNSdesc->mResponderCtx.rr_list[i].rdata)))
            &&
            (res_rec.ttl.Val > (pDNSdesc->mResponderCtx.rr_list[i].ttl.Val/2))
            )
         {
            pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed = true;
            MDNS_DEBUG("     rr suppressed\r\n");
         }
      }
      else if (
         bMsgIsAQuery &&
         (tag == MDNS_RR_GROUP_NS) &&
         ((pOwnerCtx->state == MDNS_STATE_PROBE) ||
          (pOwnerCtx->state == MDNS_STATE_ANNOUNCE))
         )
      {
         // Simultaneous probes by us and sender of this DNS query.
         // Mark as a conflict ONLY IF we lose the Tie-Breaker.

         WeWonTheTieBreaker = _mDNSTieBreaker(&res_rec,
                                    &(pDNSdesc->mResponderCtx.rr_list[i]));

         if (!WeWonTheTieBreaker)
         {
            pOwnerCtx->bProbeConflictSeen = true;
            pOwnerCtx->nProbeConflictCount++;
         }

         return 0;
      }
      else if (
         !bMsgIsAQuery             &&
         bSenderHasAuthority       &&
         (tag == MDNS_RR_GROUP_AN) &&
         ((pOwnerCtx->state == MDNS_STATE_PROBE) ||
          (pOwnerCtx->state == MDNS_STATE_ANNOUNCE))
         )
      {
         // An authoritative DNS response to our probe/announcement.
         // Mark as a conflict. Effect a re-name, followed by a
         // re-probe.

         pOwnerCtx->bProbeConflictSeen = true;
         pOwnerCtx->nProbeConflictCount++;

         return 0;
      }
      else if (bMsgIsAQuery             &&
             (tag == MDNS_RR_GROUP_NS) &&
             (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // A probe by the sender conflicts with our established record.
         // Need to defend our record. Effect a DNS response.

         MDNS_DEBUG("Defending RR: \r\n");

         pMyRR->bResponseRequested = true;

         return 0;
      }
      else if (
         !bMsgIsAQuery &&
          bSenderHasAuthority &&
         (tag == MDNS_RR_GROUP_AN) &&
         (pMyRR->type.Val != QTYPE_PTR ) &&      // No one can claim authority on shared RR
         (pOwnerCtx->state == MDNS_STATE_DEFEND)
         )
      {
         // Sender claims that it also has the authority on
         // a unique (non-shared) record that we have already established authority.
         // Effect a re-probe.

         pOwnerCtx->bLateConflictSeen = true;

         return 0;
      }
   }
   return 0;
}

static uint16_t _mDNSStringLength(uint8_t* string)
{
   uint8_t *right_ptr,*label_ptr;
   uint8_t label[MAX_LABEL_SIZE];
   uint8_t i;
   uint8_t len;
   uint16_t retVal = 0;

   right_ptr = string;

   while (1)
   {
        label_ptr = label;
        len = 0;
        while (*right_ptr)
        {
            i = *right_ptr;

            if (i == '.' || i == '/' ||
               i == ',' || i == '>' || i == '\\')
            {
             /* Formatted Serv-Instance will have '\.'
              * instead of just '.' */
                if (i == '\\')
                {
                    right_ptr++;
                }
                else
                    break;
            }
            *label_ptr++ = *right_ptr;
            len++;
            right_ptr++;
        }
        i = *right_ptr++;

      // Put the length and data
      // Also, skip over the '.' in the input string
      retVal ++;
      retVal += len;
      string =  right_ptr;

      if (i == 0x00u || i == '/' || i == ',' || i == '>')
         break;
   }
   retVal ++;
   return retVal;
}

static uint16_t _mDNSSendRRSize(mDNSResourceRecord *pRecord
          ,bool bIsFirstRR
)
{

    uint8_t rec_length;
    uint8_t record_type;

    uint16_t retValue = 0;

   record_type = pRecord->type.Val;

   if (bIsFirstRR)
   {
      retValue += sizeof(MDNS_MSG_HEADER);
   }

   retValue += _mDNSStringLength(pRecord->name);

   // Resource Record Type
   retValue += 10;

   switch (record_type)
   {
   case QTYPE_A:
        retValue += 6;
      break;

   case QTYPE_PTR:
        retValue += 2;
        retValue += _mDNSStringLength(((mDNSProcessCtx_sd *) (pRecord->pOwnerCtx))->sd_qualified_name); //0x97
      break;

   case QTYPE_SRV:
        retValue += 8;
        retValue += _mDNSStringLength(pRecord->rdata); // 0x120
      break;

   case QTYPE_TXT:
        rec_length = strlen((char*)pRecord->rdata);
        retValue += 3 + rec_length;
      break;

   default:

        MDNS_DEBUG("RR Type not supported \r\n");
   }

    return retValue;
}


static bool
_mDNSSendRR(mDNSResourceRecord *pRecord
          ,uint16_t query_id
          ,uint8_t cFlush
          ,uint16_t nAnswersInMsg
          ,bool bIsFirstRR
          ,bool bIsLastRR
          ,DNSDesc_t *pDNSdesc)
{
    MDNS_MSG_HEADER mDNS_header;
    UINT32_VAL ttl;
    uint8_t rec_length;
    uint8_t record_type;

    MDNS_TRACE();

    record_type = pRecord->type.Val;

    if (pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
    {
        MDNS_DEBUG("_mDNSSendRR: Opening UDP Socket Failed \r\n");
      return false;
    }

   if (bIsFirstRR)
   {
      pDNSdesc->txBuffOffset = 0;

      memset(&mDNS_header, 0, sizeof(MDNS_MSG_HEADER));

      mDNS_header.query_id.Val = _htons(query_id);

      mDNS_header.flags.bits.qr = 1; // this is a Response,
      mDNS_header.flags.bits.aa = 1; // and we are authoritative
      mDNS_header.flags.Val = _htons(mDNS_header.flags.Val);

      mDNS_header.nAnswers.Val = _htons(nAnswersInMsg);

      // Put out the mDNS message header
      _WriteTxBuffer(pDNSdesc, (uint8_t *)&mDNS_header, sizeof(MDNS_MSG_HEADER));
   }

   ttl.Val = pRecord->ttl.Val;

   _mDNSPutString(pRecord->name,pDNSdesc);


   {

    uint8_t resourceRecord[] = {
        0x00, record_type, 0x00, 0x01,
        ttl.v[3], ttl.v[2], ttl.v[1], ttl.v[0]
    };

    resourceRecord[2] = cFlush;

    _WriteTxBuffer(pDNSdesc, (uint8_t *)resourceRecord, sizeof(resourceRecord));

   }
   switch (record_type)
   {
   case QTYPE_A:
   {
       uint8_t aRecord[] = {
           0x00, 0x04,
           pRecord->ip.v[0],
           pRecord->ip.v[1],
           pRecord->ip.v[2],
           pRecord->ip.v[3]
       };
        /*TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);   // 0x0004 Data length
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x04);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->ip.v[0]);   // Put out IP address
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->ip.v[1]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->ip.v[2]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->ip.v[3]);*/
       _WriteTxBuffer(pDNSdesc, (uint8_t *)aRecord, sizeof(aRecord));
   }
      break;

   case QTYPE_PTR:
   {
        /* 2 bytes extra. One for Prefix Length for first-label.
         * Other one for NULL terminator */
        pRecord->rdlength.Val = strlen((char*)pRecord->rdata) + 2 ;

        //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]);
        //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]); // Res-Data Length. 0x4f
        {
            uint8_t ptrRecord[] = {
               pRecord->rdlength.v[1],
               pRecord->rdlength.v[0]
            };
            _WriteTxBuffer(pDNSdesc, (uint8_t *)ptrRecord, sizeof(ptrRecord));
        }
        _mDNSPutString(((mDNSProcessCtx_sd *) (pRecord->pOwnerCtx))->sd_qualified_name,pDNSdesc); //0x97
   }
      break;

   case QTYPE_SRV:
   {
        /* 2 bytes extra. One for Prefix Length for first-label.
         * Other one for NULL terminator */
        pRecord->rdlength.Val = strlen((char*)pRecord->rdata) + 2;
        pRecord->rdlength.Val += 6;               // for priority, weight, and port

        /*        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]);  // 0xee
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]);      // Res-Data Length

        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.priority.v[1]);   // Put Priority
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.priority.v[0]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.weight.v[1]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.weight.v[0]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.port.v[1]);
        TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->srv.port.v[0]);
        */
        {
       uint8_t srvRecord[] = {
        pRecord->rdlength.v[1],  // 0xee
        pRecord->rdlength.v[0],      // Res-Data Length

        pRecord->srv.priority.v[1],   // Put Priority
        pRecord->srv.priority.v[0],
        pRecord->srv.weight.v[1],
        pRecord->srv.weight.v[0],
        pRecord->srv.port.v[1],
        pRecord->srv.port.v[0],
       };
            _WriteTxBuffer(pDNSdesc, (uint8_t *)srvRecord, sizeof(srvRecord));
        }
        _mDNSPutString(pRecord->rdata,pDNSdesc); // 0x120

   }
      break;

   case QTYPE_TXT:

        rec_length = strlen((char*)pRecord->rdata);

        pRecord->rdlength.Val = rec_length + 1;

        //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[1]); // 0x178
        //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.v[0]); // Res-Data Length
        //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, pRecord->rdlength.Val-1); // As of now only single TXT string supported!!

        {
          uint8_t txtRecord[] = {
              pRecord->rdlength.v[1],
              pRecord->rdlength.v[0],
              pRecord->rdlength.Val-1
          };
          _WriteTxBuffer(pDNSdesc, (uint8_t *)txtRecord, sizeof(txtRecord));
        }

        if (rec_length>0)
        {
           _WriteTxBuffer(pDNSdesc, (uint8_t *)pRecord->rdata, rec_length);
        }
      break;

   default:

        MDNS_DEBUG("RR Type not supported \r\n");
   }

   if (bIsLastRR)
   {
       return _SendTo(pDNSdesc);
   }

    return false;
}


static uint16_t _RecvFrom(DNSDesc_t *pDNSdesc)
{
    MDNS_TRACE();

    if (pDNSdesc->mDNS_socket != INVALID_UDP_SOCKET) {
        recvfrom(pDNSdesc->mDNS_socket, (char*)pDNSdesc->rxBuf, MDNS_BUFFER_SIZE, 0);
		WAIT_FOR_RECVFROM_COMPLETION();
        return s_mdns_rxlen;
    }

    return 0;
}

static void _mDNSResponder(DNSDesc_t *pDNSdesc)
{
   MDNS_MSG_HEADER mDNS_header;
   uint16_t len;
   uint16_t i,j,count;
   uint16_t rr_count[4];
   MDNS_RR_GROUP rr_group[4];
   bool bMsgIsComplete;
   uint16_t packetSize = 0;

   MDNS_TRACE();

   pDNSdesc->mDNS_offset = 0;

   if (pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
   {
        pDNSdesc->mDNS_responder_state = MDNS_RESPONDER_INIT;
   }

   switch(pDNSdesc->mDNS_responder_state)
   {
      case MDNS_RESPONDER_INIT:
         pDNSdesc->mDNS_socket = mDNSv4Initialize();
         WDRV_ASSERT(pDNSdesc->mDNS_socket != INVALID_SOCKET, "");
         if (pDNSdesc->mDNS_socket == INVALID_SOCKET) {
            MDNS_DEBUG("_mDNSResponder: Can't open Multicast-DNS UDP-Socket \r\n");
            return;
         } else {
            pDNSdesc->mDNS_responder_state = MDNS_RESPONDER_LISTEN ;
         }
            break;

      case MDNS_RESPONDER_LISTEN:
         pDNSdesc->totalBytes = _RecvFrom(pDNSdesc);
         /* TODO : examine if checking rc == 0 is alright for non-blockign socket */
         if (pDNSdesc->totalBytes == 0)
            return;
         pDNSdesc->bytesRemained = pDNSdesc->totalBytes;

         // Retrieve the mDNS header
         len = _ReadRecvBuffer(sizeof(mDNS_header), (uint8_t *) &mDNS_header, pDNSdesc, false);
         WDRV_ASSERT(len == sizeof(mDNS_header), "");

         mDNS_header.query_id.Val = ntohs(mDNS_header.query_id.Val);
         mDNS_header.flags.Val = ntohs(mDNS_header.flags.Val);
         mDNS_header.nQuestions.Val = ntohs(mDNS_header.nQuestions.Val);
         mDNS_header.nAnswers.Val = ntohs(mDNS_header.nAnswers.Val);
         mDNS_header.nAuthoritativeRecords.Val = ntohs(mDNS_header.nAuthoritativeRecords.Val);
         mDNS_header.nAdditionalRecords.Val = ntohs(mDNS_header.nAdditionalRecords.Val);

         pDNSdesc->mDNS_offset += len; // MUST BE 12

         if (mDNS_header.flags.bits.qr == 0)
         {
            MDNS_DEBUG("_mDNSResponder: rx QUERY \r\n");
         }
         else
         {
            MDNS_DEBUG("_mDNSResponder: rx RESPONSE \r\n");
         }

         bMsgIsComplete = (mDNS_header.flags.bits.tc == 0);  // Message is not truncated.

         rr_count[0] = mDNS_header.nQuestions.Val;
         rr_group[0] = MDNS_RR_GROUP_QD;

         rr_count[1] = mDNS_header.nAnswers.Val;
         rr_group[1] = MDNS_RR_GROUP_AN;

         rr_count[2] = mDNS_header.nAuthoritativeRecords.Val;
         rr_group[2] = MDNS_RR_GROUP_NS;

         rr_count[3] = mDNS_header.nAdditionalRecords.Val;
         rr_group[3] = MDNS_RR_GROUP_AR;

         for (i = 0; i < MAX_RR_NUM; i++)
         {
            // Reset flags
            pDNSdesc->mResponderCtx.rr_list[i].bNameAndTypeMatched = false;

            if (pDNSdesc->mResponderCtx.bLastMsgIsIncomplete)
            {
               // Do nothing.
               // Whether a reply is needed is determined only when all parts
               // of the message are received.

               // Ideally, we want to verify that the current message is the
               // continuation of the previous message.
               // Don't have a cost-effective way to do this yet.
            }
            else
            {
               // Start of a new message
               pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested = false;
               pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed = false;
               pDNSdesc->mResponderCtx.rr_list[i].srv.port.Val=pDNSdesc->mSDCtx.sd_port;
            }
         }

         for (i=0; i<4; i++) // for all 4 groups: QD, AN, NS, AR
         {
            for (j=0; j < rr_count[i]; j++) // RR_count = {#QD, #AN, #NS, #AR}
            {
               _mDNSProcessIncomingRR(rr_group[i]
                                    ,&mDNS_header
                                    ,i
                                    ,j
                                    ,pDNSdesc);
            }
         }

         // Record the fact, for the next incoming message.
         pDNSdesc->mResponderCtx.bLastMsgIsIncomplete = (bMsgIsComplete == false);

         // Do not reply any answer if the current message is not the last part of
         // the complete message.
         // Future parts of the message may request some answers be suppressed.

         if (!bMsgIsComplete)
         {
            MDNS_DEBUG("_mDNSResponder: truncated msg.\r\n");
            return;
         }

         // Count all RRs marked as "reply needed".
         count = 0;
         for (i = 0; i < MAX_RR_NUM; i++)
         {
            if ((pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx != NULL) &&
               (pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx->state == MDNS_STATE_DEFEND) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested == true) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed == false)
               )
            {
                if (count == 0)
                {
                    packetSize = _mDNSSendRRSize(&pDNSdesc->mResponderCtx.rr_list[i], true);
                    //SYS_CONSOLE_PRINT("Packet Size %d count = %d\n\r", packetSize, count);
                }
                else
                {
                    packetSize += _mDNSSendRRSize(&pDNSdesc->mResponderCtx.rr_list[i], false);
                    //SYS_CONSOLE_PRINT("Packet Size %d count = %d\n\r", packetSize, count);

                }
               count++;
            }
         }

         // Send all RRs marked as "reply needed".

         j = 1;
         for (i = 0; (count > 0) && (i < MAX_RR_NUM); i++)
         {
            if ((pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx != NULL) &&
               (pDNSdesc->mResponderCtx.rr_list[i].pOwnerCtx->state == MDNS_STATE_DEFEND) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseRequested == true) &&
               (pDNSdesc->mResponderCtx.rr_list[i].bResponseSuppressed == false) )
            {
               _mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[i]
                         ,mDNS_header.query_id.Val
                         ,(pDNSdesc->mResponderCtx.rr_list[i].type.Val == QTYPE_PTR)?(0x00):(0x80) // flush, except for PTR; for Conformance Test.
                         ,count                           // MAX_RR_NUM answers;
                         ,(j==1)?true:false               // Is this the first RR?
                         ,(j==count)?true:false
                         ,pDNSdesc);         // Is this the last RR?

               j++;
            }
         }

         // end of MDNS_RESPONDER_LISTEN
         break;

      default:
         break;
   }

   return;
}

static void _mDNSRename(uint8_t *strLabel, uint8_t nLabelId, uint8_t *strBase, uint8_t *strTarget, uint8_t nMaxLen)
{
    size_t  targetLen;
   uint8_t n = nLabelId;
#define mDNSRename_ID_LEN 6
   uint8_t str_n[mDNSRename_ID_LEN]; //enough for "-255." + '\0'.
   uint8_t i = mDNSRename_ID_LEN - 1 ;

   str_n[i--] = 0;
   str_n[i--] = '.';

   // construct str_n from n
   while (i != 0)
   {
      str_n[i--] = '0'+ n%10;
      if (n < 10) break;
      n = n/10;
   }
   str_n[i] = '-';

    targetLen = strncpy_m((char*)strTarget, nMaxLen, 3, strLabel, &(str_n[i]), strBase);


   if ( targetLen == nMaxLen )
   {
      MDNS_DEBUG("_mDNSRename: label too long - truncated\r\n");
   }


}

static void _mDNSPutString(uint8_t* string, DNSDesc_t * pDNSdesc)
{
   uint8_t *right_ptr,*label_ptr;
   uint8_t label[MAX_LABEL_SIZE];
   uint8_t i;
   uint8_t len;
   uint8_t null_mark;

   right_ptr = string;

   while (1)
   {
        label_ptr = label;
        len = 0;
        while (*right_ptr)
        {
            i = *right_ptr;

            if (i == '.' || i == '/' ||
               i == ',' || i == '>' || i == '\\')
            {
             /* Formatted Serv-Instance will have '\.'
              * instead of just '.' */
                if (i == '\\')
                    right_ptr++;
                else
                    break;
            }
            *label_ptr++ = *right_ptr;
            len++;
            right_ptr++;
        }
        i = *right_ptr++;

      // Put the length and data
      // Also, skip over the '.' in the input string
      _WriteTxBuffer(pDNSdesc, (uint8_t *)&len, 1);
      _WriteTxBuffer(pDNSdesc, (uint8_t *)label, len);
      string =  right_ptr;

      if (i == 0x00u || i == '/' || i == ',' || i == '>')
         break;
   }

   // Put the string null terminator character
   null_mark = 0;
   _WriteTxBuffer(pDNSdesc, (uint8_t *)&null_mark, 1);
}

static bool _SendTo(DNSDesc_t *pDNSdesc)
{
    uint16_t ret = false;

    MDNS_TRACE_IN();

    if (pDNSdesc->mDNS_socket != INVALID_UDP_SOCKET) {
          struct sockaddr_in addr;

          addr.sin_family = AF_INET;
          addr.sin_port = _htons(MDNS_SERVER_PORT);
          addr.sin_addr.s_addr = nmi_inet_addr(MDNS_SERVER_ADDR);

          ret = sendto(pDNSdesc->mDNS_socket, (char*)pDNSdesc->txBuf, pDNSdesc->txBuffOffset, 0, (struct sockaddr*) &addr, sizeof(addr));
          if (ret == 0) {
                WAIT_FOR_SENDTO_COMPLETION();
                ret = true;
          } else {
                MDNS_DEBUG("mdns: sendto failed\r\n");
                ret = false;
          }
    }

    MDNS_TRACE_OUT();

    return ret;
}

static bool _mDNSProbe(mDNSProcessCtx_common *pCtx, DNSDesc_t *pDNSdesc)
{
   MDNS_MSG_HEADER mDNS_header;

   MDNS_TRACE();

    // Abort operation if no UDP sockets are available

   if (pDNSdesc->mDNS_socket == INVALID_UDP_SOCKET)
    {
        MDNS_DEBUG("_mDNSProbe: Opening UDP Socket Failed \r\n");
        return false;
    }

   pDNSdesc->txBuffOffset = 0;

    // Put DNS query here
   pDNSdesc->mResponderCtx.query_id.Val++;

   mDNS_header.query_id.Val = _htons(pDNSdesc->mResponderCtx.query_id.Val);   // User chosen transaction ID
   mDNS_header.flags.Val = 0;                              // Standard query with recursion
   mDNS_header.nQuestions.Val = _htons(((uint16_t)1u));               // 1 entry in the question section
   mDNS_header.nAnswers.Val = 0;                           // 0 entry in the answer section
   mDNS_header.nAuthoritativeRecords.Val = _htons(((uint16_t)1u));      // 1 entry in name server section
   mDNS_header.nAdditionalRecords.Val = 0;                     // 0 entry in additional records section

   // Put out the mDNS message header
   _WriteTxBuffer(pDNSdesc, (uint8_t *)&mDNS_header, sizeof(MDNS_MSG_HEADER));

   // Start of the QD section
   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      _mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name,pDNSdesc);
      break;

   case MDNS_CTX_TYPE_SD:
      _mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].name,pDNSdesc);
      break;
   }

   {
   //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);         // Type: Always QTYPE_ANY
   //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, QTYPE_ANY);

   //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x80);         // Class: Cache-Flush
   //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x01);         //        IN (Internet)
     uint8_t mdnsinfo[] = {0x00, QTYPE_ANY, 0x80, 0x01};
     _WriteTxBuffer(pDNSdesc, (uint8_t *)mdnsinfo, sizeof(mdnsinfo));
   }

   // Start of the NS section
   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      _mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].name,pDNSdesc);
      break;

   case MDNS_CTX_TYPE_SD:
      _mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].name,pDNSdesc);
      break;
   }

   {
       uint8_t nsInfo[] = {
           0x00, 0x00, 0x00, 0x01,
           0x00, 0x00, 0x00, 0x78
       };
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);      // Type: A or SRV
       switch (pCtx->type)
       {
       case MDNS_CTX_TYPE_HOST:
           nsInfo[1] = QTYPE_A;
          //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, QTYPE_A);
          break;

       case MDNS_CTX_TYPE_SD:
           nsInfo[1] = QTYPE_SRV;
          //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, QTYPE_SRV);
          break;
       }
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);      // Class: Cache-Flush bit MUST NOT be set
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x01);      //IN (Internet)

       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);      // 0x00000078 Time To Live, 2 minutes
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x00);
       //TCPIP_UDP_Put(pDNSdesc->mDNS_socket, 0x78);
       _WriteTxBuffer(pDNSdesc, (uint8_t *)nsInfo, sizeof(nsInfo));

   }

   switch (pCtx->type)
   {
   case MDNS_CTX_TYPE_HOST:
      {
          uint8_t hostInfo[] = {
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].rdlength.v[0],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[0],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[2],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_A_INDEX].ip.v[3]
          };
         _WriteTxBuffer(pDNSdesc, (uint8_t *)hostInfo, sizeof(hostInfo));

         break;
      }

   case MDNS_CTX_TYPE_SD:
      {
          uint8_t sdInfo[] = {
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdlength.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdlength.v[0],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.priority.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.priority.v[0],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.weight.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.weight.v[0],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.v[1],
              pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].srv.port.v[0],
          };
         _WriteTxBuffer(pDNSdesc, (uint8_t *)sdInfo, sizeof(sdInfo));

         _mDNSPutString(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX].rdata,pDNSdesc);

         break;
      }
   }

    return _SendTo(pDNSdesc);
}

static void _mDNSAnnounce(mDNSResourceRecord *pRR, DNSDesc_t *pDNSdesc)
{
    MDNS_TRACE();

    if ( false ==
      _mDNSSendRR(pRR
                 ,0
                 ,0x80
                 ,1
                 ,true
                 ,true
                 ,pDNSdesc)
      )
   {
        MDNS_DEBUG("_mDNSAnnounce: Error in sending out Announce pkt \r\n");
   }
}

static void _mDNSProcessInternal(mDNSProcessCtx_common *pCtx, DNSDesc_t *pDNSdesc)
{
    bool bIsHost = (((void *) pCtx) == ((void *) &pDNSdesc->mHostCtx));

    switch (pCtx->state)
    {
        case MDNS_STATE_HOME:

            MDNS_DEBUG("MDNS_STATE_HOME: Wrong state \r\n");
            break;

        case MDNS_STATE_NOT_READY: // SD starts from here. SD only.

            if (pDNSdesc->mHostCtx.common.state != MDNS_STATE_DEFEND)
            {
                /* Multicast DNS is not ready */
                return;
            }
            else
            {
                /* Multicast DNS is ready now */
                pCtx->state = MDNS_STATE_INIT;
                pCtx->time_recorded = 0;
            }

            MDNS_DEBUG("\r\nMDNS_STATE_NOT_READY --> MDNS_STATE_INIT \r\n");
            break;

        case MDNS_STATE_INTF_NOT_CONNECTED: // HOST starts from here. HOST only.
            if (is_link_up()) {
                /* Interface is connected now */
                pCtx->state = MDNS_STATE_IPADDR_NOT_CONFIGURED;
                pCtx->time_recorded = 0;
            } else {
                return;
            }

            // No break. Fall through

        case MDNS_STATE_IPADDR_NOT_CONFIGURED: // HOST only.
        {
            // Wait until IP addr is configured ...
            if (pDNSdesc->netIPAddr.Val == 0)
                break;

            pCtx->state = MDNS_STATE_INIT;
            pCtx->time_recorded = 0;

            MDNS_DEBUG("MDNS_STATE_IPADDR_NOT_CONFIGURED --> MDNS_STATE_INIT \r\n");

            // No break. Fall through
        }

        case MDNS_STATE_INIT:
        {
            pCtx->bConflictSeenInLastProbe = false;

            /* WDRV_TIME_DELAY(pCtx->random_delay); */ /* TODO : It does not seem necessary. Need to double check */

            // Completed the delay required

            // Clear all counters
            _mDNSCountersReset(pCtx, true);

            pCtx->state = MDNS_STATE_PROBE;
            MDNS_DEBUG("MDNS_STATE_INIT --> MDNS_STATE_PROBE \r\n");

            // No break. Fall through
        }

        case MDNS_STATE_PROBE:
        case MDNS_STATE_ANNOUNCE:
        {
            if (pCtx->bProbeConflictSeen)
            {
                pCtx->bConflictSeenInLastProbe = true;

                MDNS_DEBUG("Conflict detected. Will rename\r\n");

                /* Conflict with selected name */
                pCtx->state = MDNS_STATE_PROBE;

                // Do not reset nProbeConflictCount if in PROBE state
                _mDNSCountersReset(
                        pCtx,
                        (pCtx->state == MDNS_STATE_PROBE) ? false : true
                        );

                if (bIsHost)
                {
                    // Rename host name
                    _mDNSRename(pDNSdesc->mHostCtx.szUserChosenHostName
                            , ++(pDNSdesc->mHostCtx.common.nInstanceId)
                            , (uint8_t *) pDNSdesc->CONST_STR_local
                            , pDNSdesc->mHostCtx.szHostName
                            , MAX_HOST_NAME_SIZE);

                }
                else
                {
                    // Rename service instance name
                    if (pDNSdesc->mSDCtx.sd_auto_rename)
                    {
                        _mDNSRename(pDNSdesc->mSDCtx.srv_name
                                , ++pDNSdesc->mSDCtx.common.nInstanceId
                                , pDNSdesc->mSDCtx.srv_type
                                , pDNSdesc->mSDCtx.sd_qualified_name
                                , MAX_LABEL_SIZE);

                        /* Reset Multicast-UDP socket */
                        close(pDNSdesc->mDNS_socket);
                        pDNSdesc->mDNS_socket = INVALID_UDP_SOCKET;
                        RESPONDER_EVENT_NOTIFY();
                    }
                    else
                    {
                        pDNSdesc->mSDCtx.service_registered = 0;

                        pDNSdesc->mSDCtx.used = 0;
                        if (pDNSdesc->mSDCtx.sd_call_back != NULL)
                        {
                            pDNSdesc->mSDCtx.sd_call_back((char *) pDNSdesc->mSDCtx.srv_name,
                                    MDNSD_ERR_CONFLICT,
                                    pDNSdesc->mSDCtx.sd_context);
                        }
                    }
                }
                break;
            }


            while (1)
            {
                if (pCtx->random_delay) {
                    /* TODO: this delay probably does not need */
                    uint32_t delay = pCtx->random_delay > 10 ? 10 : pCtx->random_delay;
                    WDRV_TIME_DELAY(delay); /* wakes up every 50 ms */
                    pCtx->random_delay -= delay;
                }

                do
                {
                        if (pCtx->state == MDNS_STATE_PROBE)
                        {
                            if (((pCtx->nProbeCount >= MDNS_PROBE_NUM) && !pCtx->bConflictSeenInLastProbe) ||
                                    (pCtx->nProbeConflictCount >= MDNS_MAX_PROBE_CONFLICT_NUM))
                            {
                                /* Move onto Announce Step */
                                pCtx->state = MDNS_STATE_ANNOUNCE;
                                pCtx->bConflictSeenInLastProbe = false;

                                MDNS_DEBUG("MDNS_STATE_PROBE --> MDNS_STATE_ANNOUNCE \r\n");

                                return;
                            }
                        }
                        else
                        {
                            // We are in MDNS_STATE_ANNOUNCE

                            if (pCtx->nClaimCount >= MDNS_ANNOUNCE_NUM)
                            {
                                /* Finalize mDNS Host-name, Announced */
                                pCtx->state = MDNS_STATE_DEFEND;

                                if (bIsHost)
                                {
                                    MDNS_DEBUG("MDNS_STATE_ANNOUNCE --> MDNS_STATE_DEFEND \r\n");
                                }
                                else
                                {
                                    MDNS_DEBUG("\r\nZeroConf: Service = ");
                                    MDNS_DEBUG((char*) pDNSdesc->mSDCtx.sd_qualified_name);
                                    MDNS_DEBUG("\r\n");

                                    MDNS_DEBUG("MDNS_STATE_ANNOUNCE --> MDNS_STATE_DEFEND \r\n");

                                    _mDNSSendRR(&pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX]
                                            , 0
                                            , 0x00
                                            , 1
                                            , true
                                            , true
                                            , pDNSdesc); // This produces a bad PTR rec for MCHPDEMO.local

                                    pDNSdesc->mSDCtx.sd_service_advertised = 1;
                                    if (pDNSdesc->mSDCtx.sd_call_back != NULL)
                                    {
                                        pDNSdesc->mSDCtx.sd_call_back((char *) pDNSdesc->mSDCtx.srv_name
                                                , MDNSD_SUCCESS
                                                , pDNSdesc->mSDCtx.sd_context);
                                    }
                                }
                                _mDNSCountersReset(pCtx, true);

                                return;
                            }
                        }

                        if (pCtx->state == MDNS_STATE_PROBE)
                        {
                            // Send out Probe packet
                            _mDNSProbe(pCtx, pDNSdesc);

                            pCtx->nProbeCount++;
                            pCtx->bConflictSeenInLastProbe = false;

                            /* Need to set timeout for MDNS_PROBE_INTERVAL msec */
                            if (pCtx->nProbeConflictCount < 9) // less-than-10 is required to pass Bonjour Conformance test.
                            {
                                pCtx->random_delay = MDNS_PROBE_INTERVAL;
                            }
                            else
                            {
                                pCtx->random_delay = 1000; /* TODO : need to check the validity of this value */
                            }

                            return;
                        }

                        // We are in MDNS_STATE_ANNOUNCE

                        /* Announce Name chosen on Local Network */

                        _mDNSAnnounce(&pDNSdesc->mResponderCtx.rr_list[(bIsHost ? QTYPE_A_INDEX : QTYPE_SRV_INDEX)], pDNSdesc);

                        pCtx->nClaimCount++;

                        // Need to set timeout: ANNOUNCE_WAIT or INTERVAL

                        if (pCtx->nClaimCount == 1)
                        {
                            /* Setup a delay of MDNS_ANNOUNCE_WAIT before announcing */

                            /* Need to wait for time MDNS_ANNOUNCE_WAIT msec */
                            pCtx->random_delay = MDNS_ANNOUNCE_WAIT;
                        }
                        else
                        {
                            pCtx->random_delay = MDNS_ANNOUNCE_INTERVAL;
                        }

                        // Not Completed the delay proposed
                        return;
                }while (0);

                // Completed the delay required
                /* Set the timer for next announce */
            }
        }

        case MDNS_STATE_DEFEND:
        {
            /* On detection of Conflict Move back to PROBE step */

            if (pCtx->bLateConflictSeen)
            {
                /* Clear the Flag */
                pCtx->bLateConflictSeen = false;
                MDNS_DEBUG("CONFLICT DETECTED !!! \r\n");
                MDNS_DEBUG("Re-probing the Host-Name because of Conflict \r\n");
                pCtx->state = MDNS_STATE_INIT;
                pCtx->time_recorded = 0;

                MDNS_DEBUG("MDNS_STATE_DEFEND --> MDNS_STATE_INIT \r\n");
            }
            else
            {
                return;
            }
        }

        default:
            break;
    }
}


static void mDNSTask(void)
{
    DNSDesc_t *pDNSdesc;

    MDNS_TRACE();

    do
    {
        pDNSdesc = MDNS_DESC();

        if (!is_link_up())
        {
            pDNSdesc->mHostCtx.common.state = MDNS_STATE_INTF_NOT_CONNECTED;
        }

        if (pDNSdesc->netIPAddr.Val == 0x00)
        {
            continue;
        }

        if (pDNSdesc->netIPAddr.Val != pDNSdesc->mResponderCtx.prev_ipaddr.Val)
        {
            // IP address has been changed outside of Zeroconf.
            // Such change could be due to static IP assignment, or
            // a new dynamic IP lease.
            // Need to restart state-machine

            MDNS_DEBUG("IP-Address change is detected \r\n");
            pDNSdesc->mResponderCtx.prev_ipaddr.Val = pDNSdesc->netIPAddr.Val;
            pDNSdesc->mHostCtx.common.state = MDNS_STATE_IPADDR_NOT_CONFIGURED;

            // File in the host RR for the specified interface and mDNS desc
            _mDNSFillHostRecord(pDNSdesc);
        }

        /* Has _mDNSResponder check for incoming mDNS Quries/Responses */
		RESPONDER_EVENT_NOTIFY();

        if (pDNSdesc->mSDCtx.service_registered)
        {

            // Application has registered some services.
            // We now need to start the service probe/announce/defend process.
            if (pDNSdesc->mHostCtx.common.state != MDNS_STATE_DEFEND)
            {
                pDNSdesc->mSDCtx.common.state = MDNS_STATE_NOT_READY;
            }
            else
            {
               _mDNSProcessInternal((mDNSProcessCtx_common *) &pDNSdesc->mSDCtx,pDNSdesc);
            }
        }
        _mDNSProcessInternal((mDNSProcessCtx_common *) &pDNSdesc->mHostCtx,pDNSdesc);
    } while (0);

}

static size_t _mDNSSDFormatServiceInstance(uint8_t *string, size_t strSize )
{
   uint8_t *temp;
   uint8_t output[MAX_LABEL_SIZE];
   uint8_t i;
   uint8_t *right_ptr,*str_token;
   uint8_t len;

   temp = output;
   right_ptr = string;
   str_token = string;
   while (1)
   {
      do
      {
         i = *right_ptr++;
      } while ((i != 0x00u) && (i != '\\') && (i != '.') );


      /* Prefix '\' for every occurance of '.' & '\' */
      len = (uint8_t)(right_ptr-str_token-1);

      memcpy(temp,str_token,len);
      temp += len;
      str_token +=  len;
      if (i == '.' || i == '\\')
      {
         *temp = '\\';
         temp++;
         *temp++ = i;
         str_token += 1;

      }
      else if (i == 0x00u || i == '/' || i == ',' || i == '>')
      {
         break;
      }

   }
   *temp++ = '\0';
   return strncpy_m((char*)string, strSize, 1, output);
}

static void _mDNSSDFillResRecords(mDNSProcessCtx_sd *sd,DNSDesc_t *pDNSdesc)
{
    size_t srv_name_len,srv_type_len, qual_len;
    mDNSResourceRecord *rr_list;
    uint16_t serv_port;

    srv_name_len = strlen((char*)sd->srv_name);
    srv_type_len = strlen((char*)sd->srv_type);
    serv_port = pDNSdesc->mSDCtx.sd_port;

    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX]),0,(sizeof(mDNSResourceRecord)));
    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX]),0,(sizeof(mDNSResourceRecord)));
    memset(&(pDNSdesc->mResponderCtx.rr_list[QTYPE_TXT_INDEX]),0,(sizeof(mDNSResourceRecord)));


    /* Formatting Service-Instance name.
     * And preparing a fully qualified
     * Service-instance record . */


    strncpy((char*)sd->sd_qualified_name, (char*)sd->srv_name, sizeof(sd->sd_qualified_name));
    qual_len= _mDNSSDFormatServiceInstance(sd->sd_qualified_name, sizeof(sd->sd_qualified_name));
    strncpy_m((char*)&sd->sd_qualified_name[qual_len], sizeof(sd->sd_qualified_name) - qual_len, 2, ".", sd->srv_type);
    sd->sd_port = pDNSdesc->mSDCtx.sd_port = serv_port;

    /* Fill-up PTR Record */
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_PTR_INDEX];
    rr_list->type.Val = QTYPE_PTR;
    rr_list->name = (uint8_t *) (sd->srv_type);

    /* Res Record Name is
     * Service_Instance_name._srv-type._proto.domain */
   rr_list->rdata = (uint8_t *) (sd->sd_qualified_name);

    strncpy_m((char*)rr_list->rdata + srv_name_len, strlen((char*)sd->sd_qualified_name) - srv_name_len, 2, ".", sd->srv_type);

    /* 3 bytes extra. One for dot added between
     * Serv-Name and Serv-Type. One for length byte.
     * added for first-label in fully qualified name
     * Other one for NULL terminator */
    rr_list->rdlength.Val = srv_name_len+ srv_type_len + 3;
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL; /* Seconds. Not sure ! Need to check */
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */

      /* Fill-up SRV Record */
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_SRV_INDEX]; /* Move onto next entry */
    rr_list->name = (uint8_t *) (sd->sd_qualified_name);
    rr_list->type.Val = QTYPE_SRV;
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL;

    //rdlength is calculated/assigned last
    rr_list->srv.priority.Val = 0;
    rr_list->srv.weight.Val = 0;
    rr_list->srv.port.Val = pDNSdesc->mSDCtx.sd_port;

    /* Res Record Name is
     * Service_Instance_name._srv-type._proto.domain */
    rr_list->rdata = (uint8_t *) pDNSdesc->mHostCtx.szHostName;


    /* 2 bytes extra. One for Prefix Length for first-label.
     * Other one for NULL terminator */
   // then, add 6-byte extra: for priority, weight, and port

    rr_list->rdlength.Val = strlen((char*)rr_list->rdata)+2+6;
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */

    /* Fill-up TXT Record with NULL data*/
    rr_list = &pDNSdesc->mResponderCtx.rr_list[QTYPE_TXT_INDEX]; /* Move onto next entry */
    rr_list->type.Val = QTYPE_TXT;
    rr_list->name = (uint8_t *) (sd->sd_qualified_name);

    /* Res Record data is what defined by the user */
    rr_list->rdata = (uint8_t *) (sd->sd_txt_rec);

    /* Extra byte for Length-Byte of TXT string */
    rr_list->rdlength.Val = pDNSdesc->mSDCtx.sd_txt_rec_len+1;
    rr_list->ttl.Val = RESOURCE_RECORD_TTL_VAL;
    rr_list->pOwnerCtx = (mDNSProcessCtx_common *) sd; /* Save back ptr */
    rr_list->valid = 1; /* Mark as valid */
}

static MDNSD_ERR_CODE
mDnsServiceRegister( const char *srv_name
                    ,const char *srv_type
                    ,uint16_t port
                    ,const uint8_t *txt_record
                    ,uint8_t auto_rename
                    ,void (*call_back)(char *name, MDNSD_ERR_CODE err, void *context)
                    ,void *context)
{
   DNSDesc_t *desc;

   MDNS_TRACE();

   if ( (srv_name == NULL) || (srv_type == NULL) || (txt_record == NULL) )
   {
       return MDNSD_ERR_INVAL; // Invalid Parameter
   }

    do
    {
        desc = MDNS_DESC();

        if (desc->mSDCtx.used)
        {
            return MDNSD_ERR_BUSY;
        }

        /* Clear the State-Machine */
        memset(&desc->mSDCtx,0,sizeof(mDNSProcessCtx_sd));
        desc->mSDCtx.used = 1; /* Mark it as used */
        desc->mSDCtx.sd_auto_rename = auto_rename;
        desc->mSDCtx.sd_port = port;
        desc->mSDCtx.sd_service_advertised = 0;

        strncpy((char*)desc->mSDCtx.srv_name
                , (char*)srv_name
                , sizeof(desc->mSDCtx.srv_name));

        strncpy((char*)desc->mSDCtx.srv_type
                , (char*)srv_type
                , sizeof(desc->mSDCtx.srv_type));

        desc->mSDCtx.sd_call_back = call_back;
        desc->mSDCtx.sd_context   = context;

        desc->mSDCtx.sd_txt_rec_len = strncpy_m((char*)desc->mSDCtx.sd_txt_rec
                ,sizeof(desc->mSDCtx.sd_txt_rec)
                ,1
                ,(uint8_t *) txt_record);

        /* Fill up Resource-records for this
         * Service-instance, in MDNS-SD state-
         * -machine */
        _mDNSSDFillResRecords(&desc->mSDCtx,desc);

        desc->mSDCtx.common.type  = MDNS_CTX_TYPE_SD;
        desc->mSDCtx.common.state = MDNS_STATE_NOT_READY;
        desc->mSDCtx.common.nInstanceId = 0;

        /* Notify MDNS Stack about Service-Registration
         * to get a time-slot for its own processing */
        desc->mSDCtx.service_registered = 1;
        return MDNSD_SUCCESS;
    }while (0);

   return MDNSD_ERR_INVAL; // unknown interface
}

static int dummy_http_server_start(void)
{
	int ret = 0;

	SYS_CONSOLE_PRINT("Dummy Http server starts\r\n");

	s_http_server_sock = socket(AF_INET, SOCK_STREAM, 0);

	if (s_http_server_sock >= 0) {
		struct sockaddr_in	addr;

		addr.sin_family	= AF_INET;
		addr.sin_port = _htons(80);
		addr.sin_addr.s_addr = 0;
		bind(s_http_server_sock, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
	} else {
		WDRV_ASSERT(false, "");
		ret = -1;
	}

	return ret;
}

static void responder_task(uint32_t pContext)
{
	while (1) {
		WAIT_FOR_RESPONDER_EVENT();
		_mDNSResponder((DNSDesc_t *)pContext);
		if (s_mdns_deinit_inprogress)
			break;
	}
	s_mdns_deinit_inprogress = false;
}

static int mdns_run(bool terminate)
{
	MDNSD_ERR_CODE ret = 0;

	switch (mdns_state_get()) {
	case MDNS_INIT:
		strcpy(s_mdns_config.host_name, DEVICE_HOST_NAME);
		strcpy(s_mdns_config.service_name, DEVICE_SERVICE_NAME);
		MDNS_DEBUG("mDNS host name - %s\r\n", s_mdns_config.host_name);
		MDNS_DEBUG("mDNS service name - %s\r\n", s_mdns_config.service_name);
		WDRV_SEM_INIT(&s_sendto_sync);
		WDRV_SEM_INIT(&s_recvfrom_sync);
		WDRV_SEM_INIT(&s_responder_sync);
		mDNSInitialize();
		ret = WDRV_TASK_CREATE((void *const)responder_task, "Responder Task", 1024, MDNS_DESC(),
				configMAX_PRIORITIES - 1, &s_responder_handle);
		if (ret) {
			SYS_CONSOLE_PRINT("Failed to create responder task\r\n");
			mdns_state_set(MDNS_END);
			ret = 0;
		} else {
			mdns_state_set(MDNS_REGISTER_SERVICE);
		}
		break;
	case MDNS_REGISTER_SERVICE:
		ret = mDnsServiceRegister((char *)s_mdns_config.service_name, "_http._tcp.local", 80, ((const uint8_t *)"path=/index.htm")
				,1, NULL, NULL);
		if (ret) {
			SYS_CONSOLE_PRINT("Failed to register a service\r\n");
			mdns_state_set(MDNS_END);
			ret = 0;
		} else {
			dummy_http_server_start();
			mdns_state_set(MDNS_RUN);
		}
		break;
	case MDNS_RUN:
		if (terminate)
			mdns_state_set(MDNS_END);
		else
			mDNSTask();
		break;
	case MDNS_END:
		mDNSDeinitialize();
		if (s_http_server_sock >= 0)
			close(s_http_server_sock);
		if (s_responder_handle) {
			s_mdns_deinit_inprogress = true;
			RESPONDER_EVENT_NOTIFY();
			while (s_mdns_deinit_inprogress)
				WDRV_TIME_DELAY(10);
			WDRV_TASK_DELETE(s_responder_handle);
		}
		WDRV_SEM_DEINIT(&s_sendto_sync);
		WDRV_SEM_DEINIT(&s_recvfrom_sync);
		WDRV_SEM_DEINIT(&s_responder_sync);
		mdns_state_set(MDNS_PARK);
		break;
	case MDNS_PARK:
		ret = -1;
		break;
	default:
		WDRV_ASSERT(false, "");
		break;
	}

	return ret;
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            s_connected = false;
            s_my_ip.Val = 0;
            SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
            m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
        }
    }
        break;
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        s_connected = true;
        memcpy((char *)&s_my_ip, pu8IPAddress, sizeof(s_my_ip));
        SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
    }
        break;
    default:
        break;
    }
}

static void mdns_event_handle(uint8_t u8Msg, void *pvMsg)
{
	tstrSocketBindMsg *pstrBind;
	tstrSocketRecvMsg *pstrRx;

	switch(u8Msg) {
	case SOCKET_MSG_BIND:
		pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			/* Prepare next buffer reception. */
			SYS_CONSOLE_PRINT("socket_cb: bind success!\r\n");
		} else {
			SYS_CONSOLE_PRINT("socket_cb: bind error!\r\n");
		}
		break;
	case SOCKET_MSG_RECVFROM:
		pstrRx = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {
			s_mdns_rxlen = pstrRx->s16BufferSize;
			RECVFROM_COMPLETE_NOTIFY();
			SYS_CONSOLE_PRINT("socket_cb: received app message\r\n");
		} else {
			if (pstrRx->s16BufferSize == SOCK_ERR_TIMEOUT) {
				s_mdns_rxlen = 0;
				RECVFROM_COMPLETE_NOTIFY();
			} else {
				WDRV_ASSERT(false, "recvfrom failed");
			}
		}
		break;
	case SOCKET_MSG_SENDTO:
		SENDTO_COMPLETE_NOTIFY();
		break;
	default:
		break;
	}
}

static void http_event_handle(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	tstrSocketBindMsg *pstrBind;
	tstrSocketListenMsg	*pstrListen;
	tstrSocketAcceptMsg	*pstrAccept;
	tstrSocketRecvMsg	*pstrRx;

	switch (u8Msg) {
	case SOCKET_MSG_BIND:
		pstrBind = (tstrSocketBindMsg *)pvMsg;
        if (pstrBind && pstrBind->status == 0) {
            SYS_CONSOLE_PRINT("socket_cb: http server bind success!\r\n");
            listen(s_http_server_sock, 0);
        } else {
            SYS_CONSOLE_PRINT("socket_cb: http server bind error!\r\n");
            close(s_http_server_sock);
            s_http_server_sock = -1;
        }
		break;
	case SOCKET_MSG_LISTEN:
		pstrListen = (tstrSocketListenMsg*)pvMsg;
		SYS_CONSOLE_PRINT("http server listen %d\r\n", pstrListen->status);
		break;
	case SOCKET_MSG_ACCEPT:
		pstrAccept = (tstrSocketAcceptMsg*)pvMsg;
		if (pstrAccept->sock >= 0) {
			SYS_CONSOLE_PRINT("Accepted %d\r\n", pstrAccept->sock);
			recv(pstrAccept->sock, s_http_buf, sizeof(s_http_buf), 0);
		}
		break;
	case SOCKET_MSG_RECV:
		pstrRx = (tstrSocketRecvMsg*)pvMsg;
		if ((pstrRx->pu8Buffer != NULL) && (pstrRx->s16BufferSize > 0)) {
			uint8 txBuf[256];
			uint8 ret;
			uint32 u32BufSize;
			char httpResponseHdr[] = "HTTP/1.0 200 OK\r\n"\
				"Content-type: text/html\r\n"\
				"Content-length: %d\r\n"\
				"\r\n%s";

			u32BufSize = sprintf((char*)txBuf, httpResponseHdr, strlen("<html>Microchip WINC1500 mDNS example</html>"), "<html>Microchip WINC1500 mDNS example</html>");
			ret = send(sock, txBuf, u32BufSize, 0);
			if (ret == 0) {
				close(sock);
				SYS_CONSOLE_PRINT("Server sent default page successfully.\r\n");
			}
		}
		break;
	default:
		break;
	}
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	if (sock == s_mdns_sock)
		mdns_event_handle(u8Msg, pvMsg);
	else
		http_event_handle(sock, u8Msg, pvMsg);
}

static int8_t wifi_open(void)
{
    tstrWifiInitParam param;
    int8_t ret;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return -1;
    }

    /* Initialize socket module. */
    socketInit();
    registerSocketCallback(socket_cb, NULL);

    /* Connect to router. */
    ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

    return ret;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	s_app_state = APP_RADIO_INIT;
	s_connected = false;
	s_my_ip.Val = 0;
	s_mdns_rxlen = 0;
	s_mdns_sock = -1;
	s_mdns_deinit_inprogress = false;
	s_responder_handle = 0;
}

void app_task(void)
{
	int ret;

	switch (app_state_get()) {
	case APP_RADIO_INIT:
		radio_init(NULL);
		app_state_set(APP_WIFI_OPEN);
		break;
	case APP_WIFI_OPEN:
		ret = wifi_open();
		if (ret)
			app_state_set(APP_RADIO_INIT);
		else
			app_state_set(APP_WIFI_CONNECT_WAIT);
		break;
	case APP_WIFI_CONNECT_WAIT:
		if (is_link_up())
			app_state_set(APP_NET_UP);
		break;
	case APP_NET_UP:
		ret = mdns_run(false);
		if (ret)
			app_state_set(APP_END);
		break;
	case APP_END:
		radio_deinit();
		app_state_set(APP_PARK);
		break;
	case APP_PARK:
		/* Example has finished. Spinning wheels... */
		break;
	default:
		break;
	}
}

#endif /* MDNS_EXAMPLE */

//DOM-IGNORE-END
