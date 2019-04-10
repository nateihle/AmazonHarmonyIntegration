/*******************************************************************************
    BT APP SDP data base

  Company:
    Microchip Technology Inc.

  File Name:
    btapp_spp_sdpdb.h

  Summary:
    Contains device specific declarations for BT service discovery protocol
    database.

  Description:
    This file contains device specific declarations for BT service discovery protocol
    database.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifdef __cplusplus
extern "C" {
#endif

//
// XML source
// -------------------------------------------------------------------
//	<?xml version="1.0" encoding="UTF-8" ?>
//
//	<sdp xmlns="http://www.searanllc.com/sdp" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.searanllc.com/sdp sdp.xsd">
//		<!-- Server -->
//		<record>
//			<attribute id="SDP_ATTRID_ServiceRecordHandle">
//				<uint32 value="0" />
//			</attribute>
//			<attribute id="SDP_ATTRID_VersionNumberList">
//				<sequence>
//					<uint16 value="0x0100" />
//					<uint16 value="0x0101" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceClassIDList">
//				<sequence>
//					<uuid16 value="SDP_CLSID_ServiceDiscoveryServerServiceClassID" />
//				</sequence>
//			</attribute>
//		</record>
//
//		<!--  PNP Information -->
//		<record>
//			<attribute id="SDP_ATTRID_ServiceRecordHandle">
//				<uint32 value="0x10002" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceClassIDList">
//				<sequence>
//					<uuid16 value="SDP_CLSID_PNPInformation" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_BrowseGroupList">
//				<sequence>
//					<uuid16 value="SDP_CLSID_PublicBrowseGroup" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_DocumentationURL">
//				<url value="http://www.searanllc.com" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ClientExecutableURL">
//				<url value="http://www.searanllc.com" />
//			</attribute>
//			<!-- SDP_ATTRID_PrimaryLanguageBaseId + SDP_ATTRID_OFFSET_ServiceDescription -->
//			<attribute id="0x0101">
//				<string value="Serial Port on DotStack Demo" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DISpecificationId">
//				<uint16 value="0x0102" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DIVendorId">
//				<uint16 value="0x0111" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DIProductId">
//				<uint16 value="0x0001" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DIVersion">
//				<uint16 value="0x0001" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DIPrimaryRecord">
//				<bool value="true" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DIVendorIdSource">
//				<uint16 value="0x0001" />
//			</attribute>
//		</record>
//
//		<!-- RFCOMM -->
//		<record>
//			<attribute id="SDP_ATTRID_ServiceRecordHandle">
//				<uint32 value="0x10000" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceRecordState">
//				<uint32 value="0" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceClassIDList">
//				<sequence>
//					<uuid16 value="SDP_CLSID_SerialPort" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceID">
//				<uuid16 value="0x1234" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ProtocolDescriptorList">
//				<sequence>
//					<sequence>
//						<uuid16 value="SDP_CLSID_L2CAP" />
//						<uint16 value="3" />
//					</sequence>
//					<sequence>
//						<uuid16 value="SDP_CLSID_RFCOMM" />
//						<uint8 value="1" />
//					</sequence>
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_BluetoothProfileDescriptorList">
//				<sequence>
//					<sequence>
//						<uuid16 value="SDP_CLSID_RFCOMM" />
//						<uint16 value="0x0101" />
//					</sequence>
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_BrowseGroupList">
//				<sequence>
//					<uuid16 value="SDP_CLSID_PublicBrowseGroup" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_LanguageBaseAttributeIDList">
//				<sequence>
//					<uint16 value="0x656E" /> <!-- English  -->
//					<uint16 value="0x006A" /> <!-- UTF-8 encoding -->
//					<uint16 value="0x0100" /> <!--  Primary langugae base ID (SDP_ATTRID_PrimaryLanguageBaseId) -->
//				</sequence>
//			</attribute>
//			<!-- SDP_ATTRID_PrimaryLanguageBaseId + SDP_ATTRID_OFFSET_ServiceName -->
//			<attribute id="0x0100">
//				<string value="Serial Port" />
//			</attribute>
//			<!-- SDP_ATTRID_PrimaryLanguageBaseId + SDP_ATTRID_OFFSET_ServiceDescription -->
//			<attribute id="0x0101">
//				<string value="Serial Port" />
//			</attribute>
//			<!-- SDP_ATTRID_PrimaryLanguageBaseId + SDP_ATTRID_OFFSET_ProviderName -->
//			<attribute id="0x0102">
//				<string value="SEARAN LLC" />
//			</attribute>
//			<attribute id="SDP_ATTRID_DocumentationURL">
//				<url value="http://www.searanllc.com" />
//			</attribute>
//			<attribute id="SDP_ATTRID_ClientExecutableURL">
//				<url value="http://www.searanllc.com" />
//			</attribute>
//			<attribute id="SDP_ATTRID_IconURL">
//				<url value="http://www.searanllc.com" />
//			</attribute>
//			<attribute id="SDP_ATTRID_VersionNumberList">
//				<sequence>
//					<uint16 value="0x0100" />
//					<uint16 value="0x0101" />
//				</sequence>
//			</attribute>
//			<attribute id="SDP_ATTRID_ServiceDatabaseState">
//				<uint32 value="0" />
//			</attribute>
//		</record>
//
//	</sdp>
// -------------------------------------------------------------------
//


#ifndef BT_DEFINE_SDP_DATABASE

extern const bt_byte sdp_db_spp[466];
extern const bt_uint sdp_db_spp_len;


#else

const bt_byte sdp_db_spp[466] = {
    // Number of records
    '\x00','\x03',

    // Record started, handle = 0x0
    '\x00','\x00','\x00','\x00','\x00','\x1E',
    // Attribute started, id = 0x0
    '\x00','\x00','\x00','\x05',
    // Data element started, type = uint32, value = 0
    '\x0A','\x00','\x00','\x00','\x00',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x200
    '\x02','\x00','\x00','\x08',
    // Data element started, type = sequence
    '\x35','\x06',
    // Data element started, type = uint16, value = 0x0100
    '\x09','\x01','\x00',
    // Data element ended
    // Data element started, type = uint16, value = 0x0101
    '\x09','\x01','\x01',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x1
    '\x00','\x01','\x00','\x05',
    // Data element started, type = sequence
    '\x35','\x03',
    // Data element started, type = uuid16, value = SDP_CLSID_ServiceDiscoveryServerServiceClassID
    '\x19','\x10','\x00',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Record ended

    // Record started, handle = 0x10002
    '\x00','\x01','\x00','\x02','\x00','\xA2',
    // Attribute started, id = 0x0
    '\x00','\x00','\x00','\x05',
    // Data element started, type = uint32, value = 0x10002
    '\x0A','\x00','\x01','\x00','\x02',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x1
    '\x00','\x01','\x00','\x05',
    // Data element started, type = sequence
    '\x35','\x03',
    // Data element started, type = uuid16, value = SDP_CLSID_PNPInformation
    '\x19','\x12','\x00',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x5
    '\x00','\x05','\x00','\x05',
    // Data element started, type = sequence
    '\x35','\x03',
    // Data element started, type = uuid16, value = SDP_CLSID_PublicBrowseGroup
    '\x19','\x10','\x02',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0xa
    '\x00','\x0A','\x00','\x1A',
    // Data element started, type = url, value = http://www.searanllc.com
    '\x45','\x18','\x68','\x74','\x74','\x70','\x3A','\x2F','\x2F','\x77','\x77','\x77','\x2E','\x73','\x65','\x61','\x72','\x61','\x6E','\x6C','\x6C','\x63','\x2E','\x63','\x6F','\x6D',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0xb
    '\x00','\x0B','\x00','\x1A',
    // Data element started, type = url, value = http://www.searanllc.com
    '\x45','\x18','\x68','\x74','\x74','\x70','\x3A','\x2F','\x2F','\x77','\x77','\x77','\x2E','\x73','\x65','\x61','\x72','\x61','\x6E','\x6C','\x6C','\x63','\x2E','\x63','\x6F','\x6D',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x101
    '\x01','\x01','\x00','\x1E',
    // Data element started, type = string, value = Serial Port on DotStack Demo
    '\x25','\x1C','\x53','\x65','\x72','\x69','\x61','\x6C','\x20','\x50','\x6F','\x72','\x74','\x20','\x6F','\x6E','\x20','\x44','\x6F','\x74','\x53','\x74','\x61','\x63','\x6B','\x20','\x44','\x65','\x6D','\x6F',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x200
    '\x02','\x00','\x00','\x03',
    // Data element started, type = uint16, value = 0x0102
    '\x09','\x01','\x02',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x201
    '\x02','\x01','\x00','\x03',
    // Data element started, type = uint16, value = 0x0111
    '\x09','\x01','\x11',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x202
    '\x02','\x02','\x00','\x03',
    // Data element started, type = uint16, value = 0x0001
    '\x09','\x00','\x01',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x203
    '\x02','\x03','\x00','\x03',
    // Data element started, type = uint16, value = 0x0001
    '\x09','\x00','\x01',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x204
    '\x02','\x04','\x00','\x02',
    // Data element started, type = bool, value = true
    '\x28','\x01',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x205
    '\x02','\x05','\x00','\x03',
    // Data element started, type = uint16, value = 0x0001
    '\x09','\x00','\x01',
    // Data element ended
    // Attribute ended
    // Record ended

    // Record started, handle = 0x10000
    '\x00','\x01','\x00','\x00','\x00','\xFE',
    // Attribute started, id = 0x0
    '\x00','\x00','\x00','\x05',
    // Data element started, type = uint32, value = 0x10000
    '\x0A','\x00','\x01','\x00','\x00',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x2
    '\x00','\x02','\x00','\x05',
    // Data element started, type = uint32, value = 0
    '\x0A','\x00','\x00','\x00','\x00',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x1
    '\x00','\x01','\x00','\x05',
    // Data element started, type = sequence
    '\x35','\x03',
    // Data element started, type = uuid16, value = SDP_CLSID_SerialPort
    '\x19','\x11','\x01',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x3
    '\x00','\x03','\x00','\x03',
    // Data element started, type = uuid16, value = 0x1234
    '\x19','\x12','\x34',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x4
    '\x00','\x04','\x00','\x11',
    // Data element started, type = sequence
    '\x35','\x0F',
    // Data element started, type = sequence
    '\x35','\x06',
    // Data element started, type = uuid16, value = SDP_CLSID_L2CAP
    '\x19','\x01','\x00',
    // Data element ended
    // Data element started, type = uint16, value = 3
    '\x09','\x00','\x03',
    // Data element ended
    // Data element ended
    // Data element started, type = sequence
    '\x35','\x05',
    // Data element started, type = uuid16, value = SDP_CLSID_RFCOMM
    '\x19','\x00','\x03',
    // Data element ended
    // Data element started, type = uint8, value = 1
    '\x08','\x01',
    // Data element ended
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x9
    '\x00','\x09','\x00','\x0A',
    // Data element started, type = sequence
    '\x35','\x08',
    // Data element started, type = sequence
    '\x35','\x06',
    // Data element started, type = uuid16, value = SDP_CLSID_RFCOMM
    '\x19','\x00','\x03',
    // Data element ended
    // Data element started, type = uint16, value = 0x0101
    '\x09','\x01','\x01',
    // Data element ended
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x5
    '\x00','\x05','\x00','\x05',
    // Data element started, type = sequence
    '\x35','\x03',
    // Data element started, type = uuid16, value = SDP_CLSID_PublicBrowseGroup
    '\x19','\x10','\x02',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x6
    '\x00','\x06','\x00','\x0B',
    // Data element started, type = sequence
    '\x35','\x09',
    // Data element started, type = uint16, value = 0x656E
    '\x09','\x65','\x6E',
    // Data element ended
    // Data element started, type = uint16, value = 0x006A
    '\x09','\x00','\x6A',
    // Data element ended
    // Data element started, type = uint16, value = 0x0100
    '\x09','\x01','\x00',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x100
    '\x01','\x00','\x00','\x0D',
    // Data element started, type = string, value = Serial Port
    '\x25','\x0B','\x53','\x65','\x72','\x69','\x61','\x6C','\x20','\x50','\x6F','\x72','\x74',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x101
    '\x01','\x01','\x00','\x0D',
    // Data element started, type = string, value = Serial Port
    '\x25','\x0B','\x53','\x65','\x72','\x69','\x61','\x6C','\x20','\x50','\x6F','\x72','\x74',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x102
    '\x01','\x02','\x00','\x0C',
    // Data element started, type = string, value = SEARAN LLC
    '\x25','\x0A','\x53','\x45','\x41','\x52','\x41','\x4E','\x20','\x4C','\x4C','\x43',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0xa
    '\x00','\x0A','\x00','\x1A',
    // Data element started, type = url, value = http://www.searanllc.com
    '\x45','\x18','\x68','\x74','\x74','\x70','\x3A','\x2F','\x2F','\x77','\x77','\x77','\x2E','\x73','\x65','\x61','\x72','\x61','\x6E','\x6C','\x6C','\x63','\x2E','\x63','\x6F','\x6D',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0xb
    '\x00','\x0B','\x00','\x1A',
    // Data element started, type = url, value = http://www.searanllc.com
    '\x45','\x18','\x68','\x74','\x74','\x70','\x3A','\x2F','\x2F','\x77','\x77','\x77','\x2E','\x73','\x65','\x61','\x72','\x61','\x6E','\x6C','\x6C','\x63','\x2E','\x63','\x6F','\x6D',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0xc
    '\x00','\x0C','\x00','\x1A',
    // Data element started, type = url, value = http://www.searanllc.com
    '\x45','\x18','\x68','\x74','\x74','\x70','\x3A','\x2F','\x2F','\x77','\x77','\x77','\x2E','\x73','\x65','\x61','\x72','\x61','\x6E','\x6C','\x6C','\x63','\x2E','\x63','\x6F','\x6D',
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x200
    '\x02','\x00','\x00','\x08',
    // Data element started, type = sequence
    '\x35','\x06',
    // Data element started, type = uint16, value = 0x0100
    '\x09','\x01','\x00',
    // Data element ended
    // Data element started, type = uint16, value = 0x0101
    '\x09','\x01','\x01',
    // Data element ended
    // Data element ended
    // Attribute ended
    // Attribute started, id = 0x201
    '\x02','\x01','\x00','\x05',
    // Data element started, type = uint32, value = 0
    '\x0A','\x00','\x00','\x00','\x00',
    // Data element ended
    // Attribute ended
    // Record ended

};

const bt_uint sdp_db_spp_len = 466;


#endif // BT_DEFINE_SDP_DATABASE


#ifdef __cplusplus
}
#endif
/*******************************************************************************
 End of File
*/
