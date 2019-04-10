/*******************************************************************************
  File Name:
    stream_writer.h

  Summary:
    Stream utility for the IoT service
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
// DOM-IGNORE-END

#ifndef _STREAM_WRITTER_H
#define _STREAM_WRITTER_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*stream_writer_write_func_t)(void *module, char *buffer, size_t buffer_len);

struct stream_writer {
	size_t max_size;
	size_t written;
	stream_writer_write_func_t write_func;
	void *priv_data;
	char *buffer;
};

/**
 * \brief Initialize the Stream writer module.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  buffer          Buffer which will be used for the storing the data.
 * \param[in]  max_length      Maximum size of buffer.
 * \param[in]  func            Function to be called when the buffer is full.
 * \param[in]  priv_data       Private data. It is passed along when callback was called.
 */
void stream_writer_init(struct stream_writer * writer, char *buffer, size_t max_length, stream_writer_write_func_t func, void *priv_data);

/**
 * \brief Write 8bit to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  value           Value will be written.
 */
void stream_writer_send_8(struct stream_writer * writer, int8_t value);

/**
 * \brief Write 16bit big endian value to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  value           Value will be written.
 */
void stream_writer_send_16BE(struct stream_writer * writer, int16_t value);

/**
 * \brief Write 16bit little endian value to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  value           Value will be written.
 */
void stream_writer_send_16LE(struct stream_writer * writer, int16_t value);

/**
 * \brief Write 32bit big endian value to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  value           Value will be written.
 */
void stream_writer_send_32BE(struct stream_writer * writer, int32_t value);

/**
 * \brief Write 32bit little endian value to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  value           Value will be written.
 */
void stream_writer_send_32LE(struct stream_writer * writer, int32_t value);

/**
 * \brief Write buffer to the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 * \param[in]  buffer          Buffer will be written.
 * \param[in]  length          Size of the buffer.
 */
void stream_writer_send_buffer(struct stream_writer * writer, const char *buffer, size_t length);

/**
 * \brief Process remain data in the writer.
 *
 * \param[in]  writer          Pointer of stream writer.
 */
void stream_writer_send_remain(struct stream_writer * writer);


#ifdef __cplusplus
}
#endif

#endif /* _STREAM_WRITTER_H */