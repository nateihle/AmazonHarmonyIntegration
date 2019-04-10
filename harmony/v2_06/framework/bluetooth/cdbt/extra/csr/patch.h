/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __BTX_CSR_PATCH_H_INCLUDED__
#define __BTX_CSR_PATCH_H_INCLUDED__

#ifdef W
#undef W
#endif

#ifdef _W
#undef _W
#endif

#ifdef H
#undef H
#endif

#define W(w)   _W((bt_uint)(w))
#define _W(w)  (bt_byte)((w) & 0xFF), (bt_byte)((w) >> 8)

#define H(ps_key, val_len, seq) \
	0x01,         /* HCI command packet indicator.*/ \
	0x00, 0xfc,   /* HCI opcode { OGF=0x3f OCF=0x00 } */ \
	              /*     OGF=0x3f implies a manufacturer-sepcific command. */ \
				  /* Parameter length. */ \
	1 + (5 + 3 + val_len) * 2, \
	0xc2,         /* CSR payload descriptor: */ \
                  /*    bit 7: Last fragment = 1 */ \
				  /*    bit 6: First fragment = 1 */ \
				  /*    bits 5-0 Channel ID = 2 (BCCMD) */ \
	W(0x0002),    /* SETREQ message */ \
	W(5+3+val_len), /* length*/ \
	W(seq),       /* sequence number */ \
	W(0x7003),    /* Variable ID of PS command */ \
	W(0x0000),    /* Status (must be 0 in SETREQ) */ \
	\
	W(ps_key),    /* PS key */ \
	W(val_len),   /* Value length in 16-bit words */ \
	W(0)          /* PS key store (0 = use default) */

#define DCRH(block_count, val_len, seq) \
	0x01,         /* HCI command packet indicator.*/ \
	0x00, 0xfc,   /* HCI opcode { OGF=0x3f OCF=0x00 } */ \
	/*     OGF=0x3f implies a manufacturer-sepcific command. */ \
	/* Parameter length. */ \
	1 + (5 + 1 + val_len) * 2, \
	0xc3,         /* CSR payload descriptor: */ \
	/*    bit 7: Last fragment = 1 */ \
	/*    bit 6: First fragment = 1 */ \
	/*    bits 5-0 Channel ID = 2 (BCCMD) */ \
	W(0x0001),    /* GETRESP message */ \
	W(5+1+val_len), /* length*/ \
	W(seq),       /* sequence number */ \
	W(0x101c),    /* Variable ID of DSP manager config request */ \
	W(0x0000),    /* Status (must be 0 in SETREQ) */ \
	\
	W(block_count)     /* Number of configuration blocks */

#endif // __BTX_CSR_PATCH_H_INCLUDED__
