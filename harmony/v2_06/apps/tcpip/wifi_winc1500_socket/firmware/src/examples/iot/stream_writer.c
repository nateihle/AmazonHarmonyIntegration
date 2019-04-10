/*******************************************************************************
  File Name:
    stream_writer.c

  Summary:
    Stream utility for the IoT (Internet of Things) service
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
	 
#if IOT_SUPPORT

#include <string.h>
#include "stream_writer.h"

void stream_writer_init(struct stream_writer * writer, char *buffer, size_t max_length, stream_writer_write_func_t func, void *priv_data)
{
	writer->max_size = max_length;
	writer->buffer = buffer;
	writer->written = 0;
	writer->write_func = func;
	writer->priv_data = priv_data;
}

void stream_writer_send_8(struct stream_writer * writer, int8_t value)
{
	int remain = writer->max_size - writer->written;

	if (remain < 1) {
		stream_writer_send_remain(writer);
	}

	writer->buffer[writer->written++] = (char)value;
}

void stream_writer_send_16BE(struct stream_writer * writer, int16_t value)
{
	stream_writer_send_8(writer, (value >> 8) & 0xFF);
	stream_writer_send_8(writer, value & 0xFF);
}

void stream_writer_send_16LE(struct stream_writer * writer, int16_t value)
{
	stream_writer_send_8(writer, value & 0xFF);
	stream_writer_send_8(writer, (value >> 8) & 0xFF);
}

void stream_writer_send_32BE(struct stream_writer * writer, int32_t value)
{
	stream_writer_send_8(writer, (value >> 24) & 0xFF);
	stream_writer_send_8(writer, (value >> 16) & 0xFF);
	stream_writer_send_8(writer, (value >> 8) & 0xFF);
	stream_writer_send_8(writer, value & 0xFF);
}

void stream_writer_send_32LE(struct stream_writer * writer, int32_t value)
{
	stream_writer_send_8(writer, value & 0xFF);
	stream_writer_send_8(writer, (value >> 8) & 0xFF);
	stream_writer_send_8(writer, (value >> 16) & 0xFF);
	stream_writer_send_8(writer, (value >> 24) & 0xFF);
}

void stream_writer_send_buffer(struct stream_writer * writer, const char *buffer, size_t length)
{
	for (; length > 0; length--, buffer++) {
		stream_writer_send_8(writer, *buffer);
	}
}

void stream_writer_send_remain(struct stream_writer * writer)
{
	if(writer->written > 0) {
		writer->write_func(writer->priv_data, writer->buffer, writer->written);
		writer->written = 0;
	}
}

#endif /* #if IOT_SUPPORT */

// DOM-IGNORE-END
