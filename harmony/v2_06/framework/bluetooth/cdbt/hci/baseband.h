/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*
* SEARAN LLC is the exclusive licensee and developer of dotstack with
* all its modifications and enhancements.
*
* Contains proprietary and confidential information of CandleDragon and
* may not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2009, 2010, 2011 CandleDragon. All Rights Reserved.
*******************************************************************************/

#ifndef __BASEBAND_H
#define __BASEBAND_H

#define COS_LIMITED_DISCOVERABLE_MODE	0x002000

#define COS_POSITIONING		0x010000
#define COS_NETWORKING		0x020000
#define COS_RENDERING		0x040000
#define COS_CAPTURING		0x080000
#define COS_OBJECTTRANSFER	0x100000
#define COS_AUDIO			0x200000
#define COS_TELEPHONY		0x400000
#define COS_INFORMATION		0x800000

#define COD_MAJOR_MISC					0x0000
#define COD_MAJOR_COMPUTER				0x0100
#define COD_MAJOR_PHONE					0x0200
#define COD_MAJOR_NET_ACCESS_POINT		0x0300
#define COD_MAJOR_AUDIO					0x0400
#define COD_MAJOR_PERIPHERAL			0x0500
#define COD_MAJOR_IMAGING				0x0600
#define COD_MAJOR_WEARABLE				0x0700
#define COD_MAJOR_TOY					0x0800
#define COD_MAJOR_HEALTH				0x0900
#define COD_MAJOR_UNCATEGORIZED			0x1f00

#define COD_MINOR_COMPUTER_UNCATEGORIZED	0x00
#define COD_MINOR_COMPUTER_DESKTOP			(0x01 << 2)
#define COD_MINOR_COMPUTER_SERVER			(0x02 << 2)
#define COD_MINOR_COMPUTER_LAPTOP			(0x03 << 2)
#define COD_MINOR_COMPUTER_HANDHELD			(0x04 << 2)
#define COD_MINOR_COMPUTER_PALMSIZED		(0x05 << 2)
#define COD_MINOR_COMPUTER_WEARABLE			(0x06 << 2)

#define COD_MINOR_PHONE_UNCATEGORIZED		0x00
#define COD_MINOR_PHONE_CELLULAR			(0x01 << 2)
#define COD_MINOR_PHONE_CORDLESS			(0x02 << 2)
#define COD_MINOR_PHONE_SMART				(0x03 << 2)
#define COD_MINOR_PHONE_WIREDMODEM			(0x04 << 2)
#define COD_MINOR_PHONE_ISDN				(0x05 << 2)

#define COD_MINOR_LAN_UNCATEGORIZED			0x00
#define COD_MINOR_LAN_FULLY_AVAILABLE		0x00
#define COD_MINOR_LAN_01_17					(0x01 << 5)
#define COD_MINOR_LAN_17_33					(0x02 << 5)
#define COD_MINOR_LAN_33_50					(0x03 << 5)
#define COD_MINOR_LAN_50_67					(0x04 << 5)
#define COD_MINOR_LAN_67_83					(0x05 << 5)
#define COD_MINOR_LAN_83_99					(0x06 << 5)
#define COD_MINOR_LAN_NO_SERVICE			(0x07 << 5)

#define COD_MINOR_AV_UNCATEGORIZED			0x00
#define COD_MINOR_AV_HEADSET				(0x01 << 2)
#define COD_MINOR_AV_HANDSFREE				(0x02 << 2)
#define COD_MINOR_AV_RESERVED				(0x03 << 2)
#define COD_MINOR_AV_MICROPHONE				(0x04 << 2)
#define COD_MINOR_AV_LOUDSPEAKER			(0x05 << 2)
#define COD_MINOR_AV_HEADPHONES				(0x06 << 2)
#define COD_MINOR_AV_PORTABLE_AUDIO			(0x07 << 2)
#define COD_MINOR_AV_CAR_AUDIO				(0x08 << 2)
#define COD_MINOR_AV_SET_TOP_BOX			(0x09 << 2)
#define COD_MINOR_AV_HIFI_AUDIO				(0x0a << 2)
#define COD_MINOR_AV_VCR					(0x0b << 2)
#define COD_MINOR_AV_VIDEO_CAMERA			(0x0c << 2)
#define COD_MINOR_AV_CAMCORDER				(0x0d << 2)
#define COD_MINOR_AV_VIDEO_MONITOR			(0x0e << 2)
#define COD_MINOR_AV_VIDE_DISPLAY_AND_LOUDSPEAKER	(0x0f << 2)
#define COD_MINOR_AV_VIDEO_CONFERENCING		(0x10 << 2)
#define COD_MINOR_AV_RESERVERD2				(0x11 << 2)
#define COD_MINOR_AV_GAMING					(0x12 << 2)

#define COD_MINOR_PERIPHERAL_OTHER			0x00
#define COD_MINOR_PERIPHERAL_KEYBOARD		0x40
#define COD_MINOR_PERIPHERAL_MOUSE			0x80
#define COD_MINOR_PERIPHERAL_COMBO			0xC0

#define COD_MINOR_PERIPHERAL_UNCATEGORIZED	0x00
#define COD_MINOR_PERIPHERAL_JOYSTICK		(0x01 << 2)
#define COD_MINOR_PERIPHERAL_GAMEPAD		(0x02 << 2)
#define COD_MINOR_PERIPHERAL_REMOTE			(0x03 << 2)
#define COD_MINOR_PERIPHERAL_SENSING		(0x04 << 2)
#define COD_MINOR_PERIPHERAL_DIGITIZER		(0x05 << 2)
#define COD_MINOR_PERIPHERAL_CARD_READER	(0x06 << 2)

#define COD_MINOR_IMAGING_UNCATEGORIZED		0x00
#define COD_MINOR_IMAGING_DISPLAY			(0x01 << 4)
#define COD_MINOR_IMAGING_CAMERA			(0x01 << 5)
#define COD_MINOR_IMAGING_SCANNER			(0x01 << 6)
#define COD_MINOR_IMAGING_PRINTER			(0x01 << 7)

#define COD_MINOR_WEARABLE_UNCATEGORIZED	0x00
#define COD_MINOR_WEARABLE_WATCH			(0x01 << 2)
#define COD_MINOR_WEARABLE_PAGER			(0x02 << 2)
#define COD_MINOR_WEARABLE_JACKET			(0x03 << 2)
#define COD_MINOR_WEARABLE_HELMET			(0x04 << 2)
#define COD_MINOR_WEARABLE_GLASSES			(0x05 << 2)

#define COD_MINOR_TOY_UNCATEGORIZED			0x00
#define COD_MINOR_TOY_ROBOT					(0x01 << 2)
#define COD_MINOR_TOY_VEHICLE				(0x02 << 2)
#define COD_MINOR_TOY_DOLL					(0x03 << 2)
#define COD_MINOR_TOY_CONTROLLER			(0x04 << 2)
#define COD_MINOR_TOY_GAME					(0x05 << 2)

#define COD_MINOR_HEALTH_UNCATEGORIZED		0x00
#define COD_MINOR_HEALTH_BPM				(0x01 << 2)
#define COD_MINOR_HEALTH_THERMOMETER		(0x02 << 2)
#define COD_MINOR_HEALTH_WEIGHING_SCALE		(0x03 << 2)
#define COD_MINOR_HEALTH_GLUCOSE_METER		(0x04 << 2)
#define COD_MINOR_HEALTH_PULSE_OXIMETER		(0x05 << 2)
#define COD_MINOR_HEALTH_HEART_MONITOR		(0x06 << 2)
#define COD_MINOR_HEALTH_DATA_DISPLAY		(0x07 << 2)

#endif // __BASEBAND_H

