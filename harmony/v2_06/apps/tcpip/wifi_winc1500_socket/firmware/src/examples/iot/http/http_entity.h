/*******************************************************************************
  File Name:
    http_entity.h

  Summary:
    HTTP base entity
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

#ifndef _HTTP_ENTITY_H
#define _HTTP_ENTITY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief A structure that the implementation of HTTP entity.
 */
struct http_entity {
	/** A flag for the using the chunked encoding transfer or not. */
	uint8_t is_chunked;
	/**
	 * \brief Get content mime type.
	 *
	 * \param[in]  priv_data       Private data of this entity.
	 *
	 * \return     Content type of entity.
	 */
	const char* (*get_contents_type)(void *priv_data);
	/**
	 * \brief Get content length.
	 * If using the chunked encoding, This function does not needed.
	 *
	 * \param[in]  priv_data       Private data of this entity.
	 *
	 * \return     Content length of entity.
	 */
	int (*get_contents_length)(void *priv_data);
	/**
	 * \brief Read the content.
	 *
	 * \param[in]  priv_data       Private data of this entity.
	 * \param[in]  buffer          A buffer that stored read data.
	 * \param[in]  size            Maximum size of the buffer.
	 * \param[in]  written         total size of ever read.
	 *
	 * \return     Read size.
	 */
	int (*read)(void *priv_data, char *buffer, uint32_t size, uint32_t written);
	/**
	 * \brief Close the entity.
	 * Completed to send request. So release the resource.
	 *
	 * \param[in]  priv_data       Private data of this entity.
	 */
	void (*close)(void *priv_data);
	/** Private data of this entity. Stored various data necessary for the operation of the entity. */
	void *priv_data;

};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _HTTP_ENTITY_H */
