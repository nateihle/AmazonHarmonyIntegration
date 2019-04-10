#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/extra/ti/ti.h"

//--------------------------------------------------------------------------------
// Description : Orca L PG 2.0 ROM Initialization Script
//
// Compatibility: Orca, 7.0.16 ROM
//
// Last Updated: 13-Jun-2013  12:36:41.53
//
// Version     : TI_P7_16.1
//
//
//
// Trio LMP Subversion: 1f01
// Orca LMP Subversion: 1b01
//
// Notes       : Use this script on Orca L PG 2.0, 7.0.16 ROM device only (FW v7.0.16)
//--------------------------------------------------------------------------------
//################################################################
//# START of CC256x Add-On
//################################################################
//# Change UART baudrate
//static const unsigned char packet01[] = {0x01,0x36,0xff,0x04,0x00,0xc2,0x01,0x00};
//################################################################
//# END of CC256x Add-On
//################################################################
static const unsigned char packet02[] = {0x01,0x37,0xfe,0x02,0x07,0x10};
static const unsigned char packet03[] = {0x01,0x05,0xff,0xff,0xd0,0x62,0x08,0x00,0xfa,0x07,0x10,0x47,0x01,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x70,0xb5,0x16,0x4d,0xae,0x7f,0x01,0x24,0xa6,0x46,0x12,
                               0x48,0xfe,0x44,0x00,0x47,0xb0,0x42,0xf8,0xd1,0x03,0x20,0x17,0x21,0x89,0x01,0xa6,
                               0x46,0x0e,0x4a,0xfe,0x44,0x10,0x47,0xad,0x7f,0xa6,0x46,0x0b,0x48,0xfe,0x44,0x00,
                               0x47,0xa8,0x42,0xf9,0xd1,0xfe,0xe7,0x0b,0x48,0x01,0x1f,0x0b,0x4a,0x11,0x60,0x04,
                               0x21,0x0d,0x4a,0x11,0x70,0x09,0x49,0x01,0x22,0x0a,0x70,0x09,0x4a,0x02,0x60,0x02,
                               0x22,0x4a,0x70,0x08,0x49,0x41,0x60,0xf7,0x46,0xc5,0x83,0x04,0x00,0x65,0x88,0x04,
                               0x00,0x3c,0x4e,0x08,0x00,0xd4,0x62,0x08,0x00,0x70,0x52,0x08,0x00,0xe4,0x52,0x08,
                               0x00,0x6d,0x64,0x08,0x00,0x77,0x64,0x08,0x00,0xf2,0x4f,0x08,0x00,0x70,0xb5,0x05,
                               0x1c,0x2c,0x69,0xa2,0x8e,0x23,0x8f,0x93,0x42,0x32,0xdd,0x2a,0x48,0x01,0x78,0x60,
                               0x8d,0x01,0x30,0x41,0x43,0x08,0x04,0x00,0x0c,0x82,0x42,0x29,0xda,0xc6,0x26,0x31,
                               0x5d,0xc9,0x09,0x25,0xd1,0x10,0x1a,0x24,0x49,0x08,0x18,0x83,0x42,0x20,0xdd,0xc5,
                               0x20,0x00,0x5d,0x01,0x28,0x01,0xd0,0x03,0x28,0x1a,0xd1,0x30,0x5d,0x01,0x28,0x05,
                               0xd1,0xce,0x21,0x09,0x5d,0x10,0x29,0x04,0xd0,0x11,0x29,0x02,0xd0,0x02,0x28,0x0a,
                               0xd0,0x70,0xbd,0xb5,0x20,0x00,0x5d,0x00,0x21,0x01,0x22,0x96,0x46,0x17,0x4a,0xfe,
                               0x44,0x10,0x47,0x02,0x20,0x30,0x55,0x16,0x49,0x97,0x20,0xc0,0x00,0x40,0x18,0x28,
                               0x62,0x70,0xbd};
static const unsigned char packet04[] = {0x01,0x05,0xff,0xbb,0xca,0x63,0x08,0x00,0xb6,0x70,0xb5,0x02,0x69,0xd4,0x8e,0x15,
                               0x8f,0xa5,0x42,0x19,0xdd,0xc8,0x21,0x8b,0x5c,0x40,0x21,0x19,0x40,0xde,0x11,0xf6,
                               0x01,0x31,0x43,0x11,0xd1,0x2c,0x1b,0x0c,0x49,0x09,0x78,0x09,0x02,0x8c,0x42,0x0b,
                               0xdd,0x02,0x2b,0x09,0xd1,0xc7,0x21,0x89,0x5c,0x01,0x29,0x01,0xd0,0x03,0x29,0x03,
                               0xd1,0x08,0x49,0x06,0x4a,0x51,0x18,0x01,0x62,0x70,0xbd,0x2e,0x2e,0x08,0x00,0xff,
                               0xff,0x00,0x00,0xcb,0xa3,0x00,0x00,0xed,0x92,0x00,0x00,0xf2,0x4f,0x08,0x00,0x82,
                               0x04,0x00,0x00,0x45,0x9f,0x00,0x00,0x40,0x1e,0x80,0x00,0x0b,0x4b,0x19,0x50,0x09,
                               0x49,0x0a,0x50,0xf7,0x46,0x00,0xb5,0x09,0x4a,0x01,0x8b,0x91,0x42,0x07,0xd0,0x40,
                               0x69,0x40,0x30,0x2c,0x21,0x01,0x22,0x96,0x46,0x06,0x4a,0xfe,0x44,0x10,0x47,0x05,
                               0x48,0x2b,0x30,0x00,0xbd,0xc0,0x46,0x04,0xf3,0x1a,0x00,0x80,0x7b,0x08,0x00,0xd9,
                               0xfc,0x00,0x00,0x99,0x94,0x03,0x00,0xf9,0x8c,0x00,0x00,0xff,0xb5,0x68,0x46,0xff,
                               0xf7,0x70,0xff,0xff,0xbd,0xff,0xb5,0x68,0x46,0xff,0xf7,0xa6,0xff,0xff,0xbd};
static const unsigned char packet05[] = {0x01,0x05,0xff,0x8d,0x78,0x7b,0x08,0x00,0x88,0x00,0xb5,0xf8,0xf0,0x41,0xfa,0x00,
                               0xbd,0x37,0x64,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00};
static const unsigned char packet06[] = {0x01,0x05,0xff,0x85,0x04,0xf3,0x1a,0x00,0x80,0x1e,0x8d,0x00,0x00,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
static const unsigned char packet07[] = {0x01,0x05,0xff,0xa1,0x00,0x00,0x18,0x00,0x9c,0xf0,0xb5,0x1f,0x4e,0x07,0x22,0x32,
                               0x70,0x10,0x23,0x73,0x70,0x47,0x20,0xb0,0x70,0x01,0x24,0xf4,0x70,0x03,0x20,0x1c,
                               0x4d,0x29,0x1c,0x01,0x39,0xa6,0x46,0x19,0x4f,0xfe,0x44,0x38,0x47,0xb2,0x78,0xf3,
                               0x78,0x03,0x20,0x29,0x1c,0xa6,0x46,0x15,0x4e,0xfe,0x44,0x30,0x47,0x03,0x20,0x29,
                               0x1c,0x01,0x31,0xa6,0x46,0x13,0x4a,0xfe,0x44,0x10,0x47,0xa6,0x46,0x12,0x48,0xfe,
                               0x44,0x00,0x47,0x12,0x4b,0x00,0x21,0x08,0x1c,0x1a,0x68,0x00,0x2a,0x04,0xd0,0x02,
                               0x07,0x15,0x0f,0x22,0x1c,0xaa,0x40,0x11,0x43,0x02,0x07,0x12,0x0f,0x0f,0x2a,0x05,
                               0xd1,0xc5,0x08,0x06,0x22,0x2a,0x40,0x0a,0x4d,0xa9,0x52,0x00,0x21,0x04,0x33,0x01,
                               0x30,0x20,0x28,0xe9,0xd3,0xf0,0xbd,0xc0,0x46,0xf8,0x4f,0x08,0x00,0x7b,0x88,0x04,
                               0x00,0xc6,0x05,0x00,0x00,0x65,0x88,0x04,0x00,0x0f,0x63,0x08,0x00,0x80,0x7b,0x08,
                               0x00,0x84,0xf3,0x1a,0x00};
static const unsigned char packet08[] = {0x01,0x83,0xff,0x14,0x79,0x7b,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const unsigned char packet09[] = {0x01,0x0c,0xfd,0x09,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0x64,0x00};
static const unsigned char packet10[] = {0x01,0x09,0xfd,0x08,0x58,0x60,0x1a,0x00,0x00,0x10,0x00,0x10};
static const unsigned char packet11[] = {0x01,0x09,0xfd,0x08,0x10,0x60,0x1a,0x00,0x10,0x00,0x10,0x00};
static const unsigned char packet12[] = {0x01,0x82,0xfd,0x14,0x00,0x9c,0x18,0xd2,0xd2,0xd2,0xd2,0xd2,0xd2,0xd2,0xdc,0xe6,
                               0xf0,0xfa,0x04,0x0e,0x18,0xff,0x00,0x00};
static const unsigned char packet13[] = {0x01,0x82,0xfd,0x14,0x01,0x9c,0xce,0xce,0xce,0xce,0xce,0xce,0xce,0xce,0xd8,0xe2,
                               0xec,0xf6,0x00,0x0a,0x14,0xff,0x00,0x00};
static const unsigned char packet14[] = {0x01,0x82,0xfd,0x14,0x02,0x9c,0xce,0xce,0xce,0xce,0xce,0xce,0xce,0xce,0xd8,0xe2,
                               0xec,0xf6,0x00,0x0a,0x14,0xff,0x00,0x00};
static const unsigned char packet15[] = {0x01,0x87,0xfd,0x03,0x0f,0x0f,0x0f};
static const unsigned char packet16[] = {0x01,0x76,0xfd,0x31,0x01,0x21,0x54,0x00,0x00,0x61,0x57,0x00,0x00,0x14,0x05,0x0a,
                               0x05,0x00,0x07,0x06,0x0a,0x04,0x05,0x08,0x09,0x0b,0x0c,0x0d,0x0e,0x10,0x10,0x10,
                               0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
                               0x10,0x00,0x00,0x00,0x00};
static const unsigned char packet17[] = {0x01,0x80,0xfd,0x06,0x00,0x01,0x00,0x00,0x00,0x01};
static const unsigned char packet18[] = {0x01,0x80,0xfd,0x06,0x3c,0xf0,0x5f,0x00,0x00,0x00};
static const unsigned char packet19[] = {0x01,0x1c,0xfd,0x14,0xff,0x88,0x13,0x00,0x00,0xff,0x00,0x00,0x00,0xff,0xff,0xff,
                               0xff,0xfa,0x00,0x00,0x00,0xff,0xff,0x00};
static const unsigned char packet20[] = {0x01,0x38,0xfe,0x00};
//################################################################
//# START of CC256x Add-On
//################################################################
//LE Enable
static const unsigned char packet21[] = {0x01,0x5b,0xfd,0x02,0x01,0x01};
//# Enable fast clock XTAL support
//static const unsigned char packet22[] = {0x01,0x1c,0xfd,0x14,0x01,0x88,0x13,0x00,0x00,0xd0,0x07,0x00,0x00,0xff,0xff,0x04,
//                               0xff,0xff,0xff,0xfa,0x00,0x00,0x00,0x00};
//# Enable eHCILL
//static const unsigned char packet23[] = {0x01,0x2b,0xfd,0x05,0x10,0x00,0x50,0x00,0x96};
//static const unsigned char packet24[] = {0x01,0x0c,0xfd,0x09,0x01,0x01,0x00,0xff,0xff,0xff,0xff,0x64,0x00};
//################################################################
//# END of CC256x Add-On
//################################################################

static const unsigned char* const packets[] = {/*packet01,*/packet02,packet03,packet04,packet05,packet06,packet07,packet08,packet09,packet10,packet11,packet12,packet13,packet14,packet15,
                               packet16,packet17,packet18,packet19,packet20,packet21/*,packet22,packet23,packet24*/};

#define PACKET_COUNT (sizeof(packets) / sizeof(packets[0]))

static const btx_ti_script_t script =
{
	packets,
	PACKET_COUNT,
	7, 16         // compatible FW version 7016
};

const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_0_1(void)
{
	return &script;
}
