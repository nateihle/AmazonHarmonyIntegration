/*******************************************************************************
  File Name:
    mdns.h

  Summary:
    WINC1500 mDns Example

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

#ifndef _MDNS_H
#define _MDNS_H

#define MDNS_SERVER_PORT	5353
#define MDNS_SERVER_ADDR	"224.0.0.251"

#define DEVICE_HOST_NAME	"WINC1500_HOST"
#define DEVICE_SERVICE_NAME	"Dummy HTTP server"

#define MDNS_BUFFER_SIZE	1460

#define INVALID_SOCKET		-1

#define MAX_HOST_NAME_SIZE  32  //31+'\0'  Max Host name size
#define MAX_LABEL_SIZE      64  //63+'\0'  Maximum size allowed for a label. RFC 1035 (2.3.4) == 63
#define MAX_RR_NAME_SIZE    256 //255+'\0' Max Resource Recd Name size. RFC 1035 (2.3.4) == 255
#define MAX_SRV_TYPE_SIZE   32  //31+'\0'  eg. "_http._tcp.local". Max could be 255, but is an overkill.
#define MAX_SRV_NAME_SIZE   64  //63+'\0'  eg. "My Web server". Max could be 255, but is an overkill.
#define MAX_TXT_DATA_SIZE   128 //127+'\0' eg. "path=/index.htm"
#define RESOURCE_RECORD_TTL_VAL     3600 // Time-To-Live for a Resource-Record in seconds.

#define MAX_RR_NUM 4            // for A, PTR, SRV, and TXT  Max No.of Resource-Records/Service

/* Constants from mdns.txt (IETF Draft)*/
#define MDNS_PROBE_WAIT             750 // msecs  (initial random delay)
#define MDNS_PROBE_INTERVAL         250 // msecs (maximum delay till repeated probe)
#define MDNS_PROBE_NUM                3 // (number of probe packets)
#define MDNS_MAX_PROBE_CONFLICT_NUM  30 // max num of conflicts before we insist and move on to announce ...
#define MDNS_ANNOUNCE_NUM             3 // (number of announcement packets)
#define MDNS_ANNOUNCE_INTERVAL      250 // msecs (time between announcement packets)
#define MDNS_ANNOUNCE_WAIT          250 // msecs (delay before announcing)

/* Resource-Record Types from RFC-1035 */
/*
All RRs have the same top level format shown below:

  0  1  2  3  4 5  6  7  8  9  10 11 12 13 14 15
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                                               |
/                                               /
/                    NAME                       /
|                                               |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    TYPE                       |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    CLASS                      |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                     TTL                       |
|                                               |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|                    RDLENGTH                   |
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--|
/                     RDATA                     /
/                                               /
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
typedef enum {
    QTYPE_A = 1,      //QUERY TYPE response = Address
    QTYPE_NS = 2,     //QUERY TYPE response = Authorative Name Server
    QTYPE_CNAME = 5,  //the canonical domain name for an alias
    QTYPE_PTR = 12,   // a domain name pointer
    QTYPE_TXT = 16,   // text strings
    QTYPE_SRV = 33,
    QTYPE_ANY = 255,
} MDNS_QTYPE;

/* Indexes in Resource-record list */
#define QTYPE_A_INDEX   0
#define QTYPE_PTR_INDEX 1
#define QTYPE_SRV_INDEX 2
#define QTYPE_TXT_INDEX 3

#define INVALID_UDP_SOCKET      (-1)		// Indicates a UDP socket that is not valid

typedef union
{
    uint8_t Val;
    struct __attribute__((packed))
    {
         uint8_t b0:1;
         uint8_t b1:1;
         uint8_t b2:1;
         uint8_t b3:1;
         uint8_t b4:1;
         uint8_t b5:1;
         uint8_t b6:1;
         uint8_t b7:1;
    } bits;
} UINT8_VAL, UINT8_BITS;

typedef union 
{
    uint16_t Val;
    uint8_t v[2];
    struct __attribute__((packed))
    {
        uint8_t LB;
        uint8_t HB;
    } byte;
    struct __attribute__((packed))
    {
         uint8_t b0:1;
         uint8_t b1:1;
         uint8_t b2:1;
         uint8_t b3:1;
         uint8_t b4:1;
         uint8_t b5:1;
         uint8_t b6:1;
         uint8_t b7:1;
         uint8_t b8:1;
         uint8_t b9:1;
         uint8_t b10:1;
         uint8_t b11:1;
         uint8_t b12:1;
         uint8_t b13:1;
         uint8_t b14:1;
         uint8_t b15:1;
    } bits;
} UINT16_VAL, UINT16_BITS;

typedef union
{
    uint32_t Val;
    uint16_t w[2] __attribute__((packed));
    uint8_t  v[4];
    struct __attribute__((packed))
    {
        uint16_t LW;
        uint16_t HW;
    } word;
    struct __attribute__((packed))
    {
        uint8_t LB;
        uint8_t HB;
        uint8_t UB;
        uint8_t MB;
    } byte;
    struct __attribute__((packed))
    {
        UINT16_VAL low;
        UINT16_VAL high;
    }wordUnion;
    struct __attribute__((packed))
    {
         uint8_t b0:1;
         uint8_t b1:1;
         uint8_t b2:1;
         uint8_t b3:1;
         uint8_t b4:1;
         uint8_t b5:1;
         uint8_t b6:1;
         uint8_t b7:1;
         uint8_t b8:1;
         uint8_t b9:1;
         uint8_t b10:1;
         uint8_t b11:1;
         uint8_t b12:1;
         uint8_t b13:1;
         uint8_t b14:1;
         uint8_t b15:1;
         uint8_t b16:1;
         uint8_t b17:1;
         uint8_t b18:1;
         uint8_t b19:1;
         uint8_t b20:1;
         uint8_t b21:1;
         uint8_t b22:1;
         uint8_t b23:1;
         uint8_t b24:1;
         uint8_t b25:1;
         uint8_t b26:1;
         uint8_t b27:1;
         uint8_t b28:1;
         uint8_t b29:1;
         uint8_t b30:1;
         uint8_t b31:1;
    } bits;
} UINT32_VAL;

typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t  v[4];
} IPV4_ADDR;

// Provides a handle to a UDP Socket
typedef int32_t UDP_SOCKET;

typedef enum {
    MDNSD_SUCCESS =  0,
    MDNSD_ERR_BUSY =     1, /* Already Being used for another Service */
    MDNSD_ERR_CONFLICT = 2, /* Name Conflict */
    MDNSD_ERR_INVAL =    3, /* Invalid Parameter */
} MDNSD_ERR_CODE;

/* MDNS Message Fomrat, which is common
 * for Queries and Resource-Records. Taken
 * from RFC 1035
 */
/* MDNS Message Header Flags */
typedef union _MDNS_MSG_HEADER_FLAGS {

    struct {
      uint8_t        rcode:4;
      uint8_t        z:3;
        uint8_t        ra:1;
      uint8_t        rd:1;
      uint8_t        tc:1;
      uint8_t        aa:1;
      uint8_t        opcode:4;
        uint8_t        qr:1;
    } bits;
    uint16_t Val;
   uint8_t v[2];
} MDNS_MSG_HEADER_FLAGS;

/* MDNS Message-Header Format */
typedef struct _MDNS_MSG_HEADER
{
   UINT16_VAL query_id;
   MDNS_MSG_HEADER_FLAGS flags;
   UINT16_VAL nQuestions;
   UINT16_VAL nAnswers;
   UINT16_VAL nAuthoritativeRecords;
   UINT16_VAL nAdditionalRecords;
} MDNS_MSG_HEADER;

/* DNS-Query Format, which is prepended by
 * DNS-MESSAGE Header defined above */
struct question
{
    unsigned char *name;
    unsigned short int type, class;
};

/* DNS-Resource Record Format, which is
 * prepended by DNS-MESSAGE Header
 * defined above. This definition includes
 * all resource-record data formats, to have
 * small-memory foot print */

struct _mDNSProcessCtx_sd;// mdnsd_struct
struct _mDNSProcessCtx_common;

typedef struct _mDNSResourceRecord
{
    uint8_t            *name;
    UINT16_VAL   type;
    UINT16_VAL   class;
    UINT32_VAL   ttl;
    UINT16_VAL   rdlength;

   union {
      IPV4_ADDR ip;      // for A record

      struct {
         UINT16_VAL priority;
         UINT16_VAL weight;
         UINT16_VAL port;
      } srv;         // for SRV record
   };

   // DO NOT merge this into the union.
   uint8_t *rdata;      // for PTR, SRV and TXT records.

    /* House-Keeping Stuff */

   // pointer to the header Ctx of the process that "owns" this resource record.
   struct _mDNSProcessCtx_common *pOwnerCtx;

    uint8_t valid; /* indicates whether rr is valid */
   bool bNameAndTypeMatched;
   bool bResponseRequested;
   bool bResponseSuppressed;
} mDNSResourceRecord;

/* DNS-SD Specific Data-Structures */

typedef enum _MDNS_STATE
{
   MDNS_STATE_HOME = 0,
    MDNS_STATE_INTF_NOT_CONNECTED,
    MDNS_STATE_IPADDR_NOT_CONFIGURED,
   MDNS_STATE_NOT_READY,
   MDNS_STATE_INIT,
   MDNS_STATE_PROBE,
   MDNS_STATE_ANNOUNCE,
   MDNS_STATE_DEFEND,
} MDNS_STATE;

typedef enum _MDNS_RR_GROUP
{
   MDNS_RR_GROUP_QD, // Quuery count
   MDNS_RR_GROUP_AN, // Answer count
   MDNS_RR_GROUP_NS, // Authority record count
   MDNS_RR_GROUP_AR  // Addition Record Count
} MDNS_RR_GROUP;

typedef struct _mDNSResponderCtx
{
   mDNSResourceRecord   rr_list[MAX_RR_NUM];   // Our resource records.

   bool                 bLastMsgIsIncomplete;   // Last DNS msg was truncated
   UINT16_VAL     query_id;            // mDNS Query transaction ID
   IPV4_ADDR            prev_ipaddr;         // To keep track of changes in IP-addr
} mDNSResponderCtx;

typedef enum _MDNS_CTX_TYPE
{
   MDNS_CTX_TYPE_HOST = 0,
   MDNS_CTX_TYPE_SD
} MDNS_CTX_TYPE;

typedef struct _mDNSProcessCtx_common
{
   MDNS_CTX_TYPE   type;      // Is owner mDNS ("HOST") or mDNS-SD ("SD")?
   MDNS_STATE      state;      // PROBE, ANNOUNCE, DEFEND, ...

   uint8_t nProbeCount;
   uint8_t nProbeConflictCount;
    uint8_t nClaimCount;
    bool bProbeConflictSeen;
    bool bLateConflictSeen;

   bool bConflictSeenInLastProbe;
   uint8_t nInstanceId;

   uint32_t event_time;   // Internal Timer, to keep track of events
   uint8_t time_recorded; // Flag to indicate event_time is loaded
   uint32_t random_delay;


} mDNSProcessCtx_common;

typedef struct _mDNSProcessCtx_host
{
   mDNSProcessCtx_common common;

   mDNSResponderCtx *pResponderCtx;

   // other host name related info

   uint8_t szUserChosenHostName[MAX_HOST_NAME_SIZE];   // user chosen host name
   uint8_t szHostName[MAX_HOST_NAME_SIZE];               // mDNS chosen Host-Name

} mDNSProcessCtx_host;

typedef struct _mDNSProcessCtx_sd
{
   mDNSProcessCtx_common common;

   mDNSResponderCtx *pResponderCtx;

   // info specific to SD
    uint8_t srv_name[MAX_SRV_NAME_SIZE];
    uint8_t srv_type[MAX_SRV_TYPE_SIZE];
    uint8_t sd_qualified_name[MAX_RR_NAME_SIZE];
    uint8_t used; /* Spinlock to protect Race-cond. */

    uint8_t sd_auto_rename: 1,        /* Flag to show auto-Rename is enabled */
         sd_service_advertised: 1, /* Flag to show whether service is advertised */
       service_registered: 1;    /* Flag to indicate that user has registered this service */

    uint16_t sd_port; /* Port number in Local-sys where Service is being offered */
    uint8_t sd_txt_rec[MAX_TXT_DATA_SIZE];
    uint8_t sd_txt_rec_len;

    void (*sd_call_back)(char *, MDNSD_ERR_CODE , void *);
    void *sd_context;

} mDNSProcessCtx_sd;


/* DNS-SD State-Machine */


/* Multicast-DNS States defintion */

/************** Global Declarations ***************/
/* Remote Node info, which is Multicast-Node
 * whose IP-address is 224.0.0.251 & MAC-Address
 * is 01:00:5E:00:00:FB. Multicast-IP address for
 * mDNS is specified by mdns.txt (IETF). IP is
 * translated into Multicast-MAC address according
 * rules specified in Std.
 */


                         // mDNS Server/Client (Responder/Qurier)


/* Global declaration to support Message-Compression
 * defined in RFC-1035, Section 4.1.4 */



////////////////////////////////////
typedef enum
{
      MDNS_RESPONDER_INIT,
      MDNS_RESPONDER_LISTEN
} MDNS_RESPONDER_TYPE;

typedef union
{
    uint16_t    val;
    struct
    {
        uint16_t    mcastFilterSet      :  1;       // Multi cast filter enabled on this interface
        uint16_t    reserved            : 15;       // future use
    };
} MDNS_DESC_FLAGS;

typedef struct
{
    IPV4_ADDR		netIPAddr;
    mDNSProcessCtx_host    mHostCtx;
    mDNSProcessCtx_sd      mSDCtx;
    mDNSResponderCtx       mResponderCtx;
    char                   CONST_STR_local[9];
    UDP_SOCKET             mDNS_socket;
    uint16_t               mDNS_offset;
    uint16_t               mDNS_responder_state;    // MDNS_RESPONDER_TYPE type
    MDNS_DESC_FLAGS        flags;
    uint8_t                rxBuf[MDNS_BUFFER_SIZE];
    uint16_t               totalBytes;
    uint16_t               bytesRemained;
    uint8_t                txBuf[MDNS_BUFFER_SIZE];
    uint16_t               txBuffOffset;
} DNSDesc_t;

typedef enum _MDNS_MODULE_STATE
{
	MDNS_INIT,
    MDNS_REGISTER_SERVICE,
    MDNS_RUN,
    MDNS_END,
    MDNS_PARK,
} MDNS_MODULE_STATE;

typedef struct _MDNS_CONFIG {
    char host_name[32];
    char service_name[32];
} MDNS_CONFIG;

// DOM-IGNORE-END
#endif /* !_MDNS_H */
