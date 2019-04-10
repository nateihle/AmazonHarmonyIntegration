/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap Converter for emWin V5.34c.                          *
*        Compiled Jun 24 2016, 09:39:51                              *
*                                                                    *
*        (c) 1998 - 2016 Segger Microcontroller GmbH & Co. KG        *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: un_muteButton_30_30                                          *
* Dimensions:  30 * 30                                               *
* NumColors:   256                                                   *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmun_muteButton_30_30;

/*********************************************************************
*
*       Palette
*
*  Description
*    The following are the entries of the palette table.
*    The entries are stored as a 32-bit values of which 24 bits are
*    actually used according to the following bit mask: 0xBBGGRR
*
*    The lower   8 bits represent the Red   component.
*    The middle  8 bits represent the Green component.
*    The highest 8 bits represent the Blue  component.
*/
static GUI_CONST_STORAGE GUI_COLOR _Colorsun_muteButton_30_30[] = {
  0x000000, 0xBDBDBD, 0xB9B9B9, 0xC1C1C1,
  0xC5C5C5, 0xC9C9C9, 0x513D3A, 0xDCF43D,
  0xAFACC0, 0xB5B5B5, 0xB0ADC1, 0xCDCDCD,
  0xD1D1D1, 0x46302D, 0xDEF540, 0x111112,
  0xDCF43A, 0xD2E64D, 0xD1E451, 0xC7D372,
  0xB0AEBD, 0xC2CB82, 0x402926, 0xD7EC45,
  0xB1B0B9, 0xB7BAA5, 0xB2B1B6, 0xB5B4BA,
  0x0A0A0B, 0x222124, 0x252529, 0x353538,
  0x422C28, 0x6C5C5A, 0xDBF23A, 0xDBF33C,
  0xC0D057, 0xD8ED45, 0xB9BD9F, 0xB9BDA0,
  0xBCBAB9, 0xC1CA84, 0xD5D5D5, 0xD8D9DA,
  0xFFFFFF, 0x080809, 0x48322F, 0x493431,
  0x605E6A, 0x6B6A72, 0xDCF33C, 0xBDCD51,
  0xD7EC48, 0xDAF046, 0xA59E9C, 0xB8BAA5,
  0xB6B4BD, 0xB9B9BC, 0xD6D8D8, 0xDDDEDF,
  0xE4E6E6, 0xE6E8E9, 0xECEEEE, 0xFAFDFE,
  0x252526, 0x3D2622, 0x3F2925, 0x2C2C30,
  0x3A3A3B, 0x39383C, 0x412B28, 0x442D2A,
  0x483330, 0x4D3936, 0x3D3B45, 0x464552,
  0x504E5C, 0x645350, 0x665553, 0x595862,
  0x5C5A66, 0x62606D, 0x786966, 0x686671,
  0x686776, 0x6C6A76, 0x6E6D7A, 0x706F75,
  0x706E79, 0x867A78, 0x8A7E7C, 0xA2AD5C,
  0xA7B359, 0xA1AA66, 0xA9B367, 0xB4C25A,
  0xB4C15C, 0xD1E54A, 0xDCF246, 0x807E8C,
  0x82818D, 0x9E9594, 0xA19998, 0xB5AEAD,
  0xB6B0AF, 0xB4B3B5, 0xCACBCC, 0xD0CFD0,
  0xD2D4D4, 0xD6D7D8, 0xDFE1E1, 0xEEF1F2,
  0xF1F2F2, 0xF2F6F6, 0xF5F6F6, 0x08080A,
  0x38201D, 0x29292D, 0x2A292E, 0x2B2A2E,
  0x2C2C2F, 0x3C3B3E, 0x3C3C3D, 0x4C3734,
  0x4F3B38, 0x3F3E40, 0x3E3D45, 0x414143,
  0x414047, 0x424149, 0x44434C, 0x49484F,
  0x574441, 0x5A4744, 0x494753, 0x494754,
  0x4A4950, 0x4B4955, 0x4B4A56, 0x4E4C57,
  0x504E58, 0x51505C, 0x54525E, 0x6E5E5C,
  0x595763, 0x595864, 0x61606B, 0x726361,
  0x736462, 0x766866, 0x7B6D6B, 0x70706F,
  0x7E716F, 0x77786E, 0x6C6C72, 0x6B6976,
  0x6D6C76, 0x717071, 0x7B7C75, 0x75747B,
  0x73717F, 0x74727D, 0x79787F, 0x887C7A,
  0xDCF33B, 0x9DA75B, 0x9AA35E, 0x9FA95C,
  0xAEBB57, 0xA7B25D, 0xABB65D, 0xAFBC5C,
  0xB1BF56, 0x828767, 0x888D66, 0x80836D,
  0x8F936F, 0x949A6B, 0x838575, 0x8B807E,
  0x8D807F, 0x929776, 0x9CA463, 0x9AA16B,
  0xA0A76C, 0xA6AE6C, 0xA8B360, 0xA9B369,
  0xADB769, 0xB1BD61, 0xB2BD66, 0xB0BB68,
  0xA1A775, 0xA4AC71, 0xB3C156, 0xBBCB53,
  0xB9C75B, 0xB8C55D, 0xBECD5A, 0xBAC760,
  0xBBC860, 0xCADC4E, 0xCCDF4E, 0xC3D451,
  0xC5D750, 0xC8D953, 0xCFE34B, 0xD0E34C,
  0xD3E849, 0xD6EB48, 0x787686, 0x7A7886,
  0x7D7C85, 0x7B7A88, 0x7D7B8B, 0x827F90,
  0x858682, 0x828187, 0x82818B, 0x878789,
  0x978D8B, 0x8F9180, 0x929481, 0x9A908F,
  0xA19796, 0xACA3A2, 0xAAA4A3, 0xACA5A3,
  0xACA6A5, 0xB1A8A7, 0xB5ADAB, 0xB8B0AE,
  0xBAB4B3, 0xBCB6B5, 0xB7B8B7, 0xBAB8B7,
  0xB4B3B9, 0xB1B0BE, 0xB4B3BC, 0xB8B7B8,
  0xBCBBBC, 0xBBBCBC, 0xC1BEBE, 0xC2CC82,
  0xC4C2C1, 0xC3C4C3, 0xC3C4C4, 0xCAC7C7,
  0xCBCCCC, 0xCFD0D0, 0xD3D3D4, 0xDADBDC,
  0xDBDCDC, 0xE9EBEB, 0xF6F7F8, 0xF5F8F9
};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palun_muteButton_30_30 = {
  256,  // Number of entries
  1,    // Has transparency
  &_Colorsun_muteButton_30_30[0]
};

static GUI_CONST_STORAGE unsigned char _acun_muteButton_30_30[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x77, 0x80, 0x8C, 0x8A, 0x89, 0x8B, 0x7E, 0x75, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x53, 0xA0, 0xB2, 0xB1, 0xBA, 0x5F, 0x5F, 0xAA, 0xB6, 0xAF, 0x9B, 0x30, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x31, 0xA2, 0x5D, 0xCE, 0x0E, 0x07, 0x10, 0x25, 0x25, 0xA4, 0x07, 0x0E, 0x61, 0x5B, 0x9D, 0x92, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0xD2, 0xB7, 0x34, 0x10, 0x12, 0x15, 0x19, 0xED, 0x0A, 0x0A, 0xED, 0x37, 0x15, 0x12, 0x07, 0x35, 0xA6, 0x54, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7D, 0xD3, 0x60, 0x0E, 0x11, 0x26, 0x0A, 0x38, 0x39, 0x39, 0xF0, 0xF0, 0xF0, 0x39, 0x38, 0x0A, 0x26, 0x11, 0x0E, 0x33, 0x31, 0x45, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x44, 0xD5, 0x24, 0x10, 0x13, 0x0A, 0x1B, 0xF0, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0xF0, 0x38, 0x0A, 0x13, 0x07, 0xC9, 0x31, 0x1F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xD6, 0xC4, 0x10, 0x29, 0x0A, 0x02, 0x01, 0x03, 0xF6, 0x04, 0x05, 0x05, 0x05, 0x05, 0xF7, 0x04, 0x03, 0x01, 0x02, 0x0A, 0x29, 0x07, 0x33, 0x54, 0x0F, 0x00, 0x00,
  0x00, 0x00, 0x58, 0xB8, 0x07, 0x13, 0x0A, 0x02, 0x01, 0xF5, 0x04, 0x05, 0x0B, 0x0B, 0x0C, 0xFA, 0x6B, 0x0B, 0x6A, 0xF7, 0xF5, 0xF2, 0x02, 0x0A, 0x13, 0x0E, 0xA5, 0x30, 0x00, 0x00,
  0x00, 0x40, 0xD9, 0x34, 0x11, 0x08, 0xEF, 0x01, 0xF5, 0xF7, 0xF8, 0x6B, 0xFA, 0xFC, 0x0C, 0x68, 0x2B, 0xFB, 0x3A, 0x0B, 0x05, 0xF6, 0xF2, 0x02, 0x0A, 0x11, 0x35, 0x97, 0x1D, 0x00,
  0x00, 0x9C, 0xBB, 0x10, 0x26, 0xEC, 0xF0, 0x03, 0xF7, 0xF8, 0x0C, 0x2A, 0x3C, 0xF7, 0x41, 0xE4, 0xFD, 0xDC, 0x65, 0x3B, 0x6C, 0x05, 0xF5, 0x01, 0x1B, 0x26, 0x07, 0x5C, 0x50, 0x00,
  0x2D, 0x63, 0xCF, 0x12, 0x08, 0x02, 0xF2, 0x04, 0x6A, 0x6B, 0x2A, 0x3E, 0xE3, 0x42, 0x47, 0xE6, 0xFF, 0x4E, 0x41, 0x2F, 0xF4, 0x6C, 0x04, 0x03, 0x02, 0x0A, 0x12, 0xD0, 0x53, 0x2D,
  0x78, 0xDD, 0x0E, 0x15, 0xEE, 0xF0, 0x03, 0x6A, 0x2B, 0x6E, 0x6F, 0x59, 0x16, 0x06, 0x0D, 0x68, 0xFE, 0xE9, 0x85, 0x0D, 0x47, 0x0C, 0x6A, 0xF5, 0x01, 0xEE, 0x15, 0x0E, 0xAD, 0x1E,
  0x83, 0xC1, 0x07, 0x19, 0x1B, 0x01, 0x04, 0xEB, 0x93, 0x98, 0x4D, 0x0D, 0x06, 0x06, 0x0D, 0x68, 0xFE, 0x72, 0x3F, 0x8F, 0x0D, 0x52, 0x3A, 0x04, 0x01, 0xEF, 0x19, 0x07, 0xA7, 0x4A,
  0x4F, 0xBE, 0x22, 0x14, 0xEA, 0x01, 0x6A, 0x36, 0x20, 0x7B, 0x49, 0x06, 0x06, 0x06, 0x0D, 0xE7, 0x2C, 0xE8, 0x70, 0x3D, 0x46, 0x47, 0x2B, 0xF7, 0x03, 0x02, 0x14, 0x23, 0xAC, 0x86,
  0x90, 0xC7, 0x17, 0x08, 0xEA, 0x01, 0xF8, 0x65, 0x48, 0x06, 0x06, 0x06, 0x06, 0x06, 0x0D, 0xE6, 0x2C, 0x21, 0xB4, 0x3F, 0x4D, 0x46, 0x01, 0x05, 0xF2, 0x02, 0x08, 0x17, 0xC3, 0x4B,
  0x91, 0xC8, 0x17, 0x08, 0xEA, 0x01, 0x6A, 0x66, 0x2E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x0D, 0xE5, 0x2C, 0x52, 0x96, 0x3F, 0x21, 0x46, 0x28, 0x05, 0x03, 0x02, 0x08, 0x17, 0xC3, 0x4B,
  0x4F, 0xBF, 0x22, 0x14, 0x09, 0x01, 0x05, 0xE2, 0x20, 0x49, 0x7C, 0x06, 0x06, 0x06, 0x0D, 0xE1, 0x2C, 0x5A, 0x2A, 0x71, 0x2F, 0x16, 0x6C, 0x04, 0x01, 0xEF, 0x14, 0x23, 0xC2, 0x87,
  0x88, 0xC0, 0x07, 0x19, 0x69, 0xF0, 0xF6, 0xF2, 0x21, 0x94, 0x84, 0x48, 0x06, 0x06, 0x2E, 0x36, 0x70, 0x3E, 0x71, 0x36, 0x46, 0x21, 0x3A, 0xF5, 0x01, 0x1B, 0x19, 0x07, 0x5B, 0x4A,
  0x43, 0xDE, 0x07, 0xF3, 0x18, 0x02, 0xF2, 0xF7, 0x6D, 0x3B, 0x3D, 0x59, 0x42, 0x06, 0x2E, 0xE0, 0x3E, 0x3C, 0x66, 0x16, 0x20, 0x05, 0x05, 0x03, 0x28, 0x18, 0x15, 0x0E, 0xAE, 0x1E,
  0x2D, 0x64, 0x61, 0x12, 0x08, 0x09, 0xF0, 0x03, 0xF7, 0xF8, 0x0C, 0x3C, 0x28, 0x2F, 0x20, 0xDF, 0x6F, 0x95, 0x74, 0x2E, 0x28, 0xF9, 0xF5, 0x01, 0xEA, 0x08, 0x12, 0x34, 0x9A, 0x73,
  0x00, 0x58, 0xBC, 0x10, 0x27, 0x1A, 0xEF, 0x01, 0xF5, 0x05, 0xF8, 0x6B, 0x2B, 0x6E, 0x4E, 0xA3, 0x3D, 0xB3, 0x5A, 0x6D, 0xF9, 0xF6, 0xF2, 0x02, 0xEC, 0x27, 0x07, 0xA8, 0x50, 0x00,
  0x00, 0x1E, 0xDB, 0x25, 0x11, 0x08, 0x69, 0x02, 0x01, 0xF5, 0x04, 0x05, 0x0B, 0x6B, 0x3B, 0xF2, 0x0C, 0x2A, 0x6C, 0x05, 0xF6, 0xF2, 0x39, 0x09, 0x08, 0x11, 0x62, 0x99, 0x1D, 0x00,
  0x00, 0x00, 0xA1, 0xB9, 0x07, 0x13, 0x08, 0x69, 0x02, 0x01, 0x03, 0xF6, 0x04, 0x05, 0x05, 0x6A, 0x05, 0xF7, 0xF6, 0x03, 0x01, 0x02, 0x09, 0x08, 0x13, 0x0E, 0x5C, 0x30, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xD7, 0xC6, 0x10, 0x29, 0x0A, 0x69, 0xEA, 0xF0, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0xF0, 0xEF, 0x69, 0x08, 0x29, 0x32, 0xCC, 0x55, 0x0F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7A, 0x64, 0xCD, 0x10, 0x13, 0x0A, 0x1A, 0x09, 0xEA, 0x02, 0xF0, 0xF0, 0xF1, 0xF0, 0x02, 0xEF, 0x09, 0x1A, 0x0A, 0x13, 0x07, 0xCA, 0x57, 0x1F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xDA, 0x24, 0x07, 0x11, 0x27, 0x0A, 0x18, 0x1A, 0x69, 0x09, 0x09, 0x69, 0x1A, 0x18, 0x08, 0x27, 0x11, 0x0E, 0xCB, 0x57, 0x45, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0x63, 0x5E, 0x35, 0x23, 0x12, 0x15, 0x37, 0x14, 0x0A, 0x0A, 0x14, 0x37, 0x15, 0x12, 0x07, 0x62, 0xA9, 0x56, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x56, 0xD8, 0xBD, 0xD1, 0x0E, 0x23, 0x22, 0x17, 0x17, 0x23, 0x07, 0x0E, 0x34, 0xAB, 0x9E, 0x51, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x55, 0xD4, 0xB5, 0x5E, 0xC5, 0x24, 0x24, 0x60, 0x5D, 0xB0, 0x9F, 0x51, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x43, 0x82, 0x8E, 0x4C, 0x4C, 0x8D, 0x81, 0x76, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

GUI_CONST_STORAGE GUI_BITMAP bmun_muteButton_30_30 = {
  30, // xSize
  30, // ySize
  30, // BytesPerLine
  8, // BitsPerPixel
  _acun_muteButton_30_30,  // Pointer to picture data (indices)
  &_Palun_muteButton_30_30   // Pointer to palette
};

/*************************** End of file ****************************/
