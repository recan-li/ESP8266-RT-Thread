/**
  *********************************************************************************
  *
  * @file    ald_crypt.c
  * @brief   CRYPT module driver.
  *	     This is the common part of the CRYPT initialization
  *
  * @version V1.0
  * @date    19 Jun 2019
  * @author  AE Team
  * @note
  *          Change Logs:
  *          Date            Author          Notes
  *          19 Jun 2019     AE Team         The first version
  *
  * Copyright (C) Shanghai Eastsoft Microelectronics Co. Ltd. All rights reserved.
  *
  * SPDX-License-Identifier: Apache-2.0
  *
  * Licensed under the Apache License, Version 2.0 (the License); you may
  * not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an AS IS BASIS, WITHOUT
  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  **********************************************************************************
  */


#include "ald_conf.h"


/** @addtogroup ES32FXXX_ALD
  * @{
  */

/** @defgroup CRYPT CRYPT
  * @brief CRYPT module driver
  * @{
  */
#ifdef ALD_CRYPT

/** @addtogroup CRYPT_Private_Functions CRYPT Private Functions
  * @{
  */
void crypt_reset(crypt_handle_t *hperh);
#ifdef ALD_DMA
static void crypt_dma_crypt_cplt(void *arg);
static void crypt_dma_error(void *arg);
#endif
/**
  * @}
  */


/** @defgroup CRYPT_Public_Functions CRYPT Public Functions
  * @{
  */

/** @defgroup CRYPT_Public_Functions_Group1 Initialization functions
  * @brief Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Initializes the CRYPT mode according to the specified parameters in
  *         the crypt_init_t and create the associated handle.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_init(crypt_handle_t *hperh)
{
	uint32_t tmp = 0;

	if (hperh == NULL)
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_MODE(hperh->init.mode));
	assert_param(IS_CRYPT_KS(hperh->init.key));

	if ((hperh->init.key == CRYPT_DES_KEYS_1 || hperh->init.key == CRYPT_DES_KEYS_2 || hperh->init.key == CRYPT_DES_KEYS_3)
		&& (hperh->init.mode == CRYPT_MODE_CTR)) {
		return ERROR;
	}

	__LOCK(hperh);
	crypt_reset(hperh);

	if (hperh->state == CRYPT_STATE_RESET)
		__UNLOCK(hperh);

	tmp = hperh->perh->CON;
	switch (hperh->init.key) {
	case CRYPT_AES_BITS_128:
		tmp |= ((CRYPT_CRYSEL_AES << CRYPT_CON_CRYSEL_POS) | \
		        (CRYPT_BITS_128 << CRYPT_CON_AESKS_POSS));
		hperh->step = 4;
		break;

	case CRYPT_AES_BITS_192:
		tmp |= ((CRYPT_CRYSEL_AES << CRYPT_CON_CRYSEL_POS) | \
		        (CRYPT_BITS_192 << CRYPT_CON_AESKS_POSS));
		hperh->step = 4;
		break;

	case CRYPT_AES_BITS_256:
		tmp |= ((CRYPT_CRYSEL_AES << CRYPT_CON_CRYSEL_POS) | \
		        (CRYPT_BITS_256 << CRYPT_CON_AESKS_POSS));
		hperh->step = 4;
		break;

	case CRYPT_DES_KEYS_1:
		SET_BIT(tmp, CRYPT_CON_CRYSEL_MSK);
		hperh->step = 2;
		break;

	case CRYPT_DES_KEYS_2:
		SET_BIT(tmp, CRYPT_CON_CRYSEL_MSK);
		tmp |= ((1 << CRYPT_CON_TDES_POS) | (CRYPT_KEYS_2 << CRYPT_CON_DESKS_POS));
		hperh->step = 2;
		break;

	case CRYPT_DES_KEYS_3:
		SET_BIT(tmp, CRYPT_CON_CRYSEL_MSK);
		tmp |= ((1 << CRYPT_CON_TDES_POS) | (CRYPT_KEYS_3 << CRYPT_CON_DESKS_POS));
		hperh->step = 2;
		break;

	default:
		hperh->state = CRYPT_STATE_ERROR;
		__UNLOCK(hperh);
		return ERROR;
	}

	tmp |= ((1 << CRYPT_CON_FIFOODR_POS) | (hperh->init.mode << CRYPT_CON_MODE_POSS) | \
		(hperh->init.type << CRYPT_CON_TYPE_POSS) | (1 << CRYPT_CON_FIFOEN_POS));
	hperh->perh->CON = tmp;

	hperh->state = CRYPT_STATE_READY;
	__UNLOCK(hperh);
	return OK;
}

/**
  * @brief  Write the Content of KEY.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  key: Pointer to key data buffer
  * @param  len: The length of key(32 bits)
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_write_key(crypt_handle_t *hperh, uint32_t * key, crypt_key_len_t len)
{
	uint32_t *temp   = key;
	uint32_t i;

	if ((hperh == NULL) || (key == NULL))
		return ERROR;
	if (hperh->state == CRYPT_STATE_BUSY)
		return BUSY;

	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_KEY_LEN(len));

	if ((hperh->init.key & 0xF) != len)
		return ERROR;

	switch (len) {
	case KEY_8_LEN:
		hperh->perh->KEY[7] = *temp++;
		hperh->perh->KEY[6] = *temp++;
		hperh->perh->KEY[5] = *temp++;
		hperh->perh->KEY[4] = *temp++;
		hperh->perh->KEY[3] = *temp++;
		hperh->perh->KEY[2] = *temp++;
		hperh->perh->KEY[1] = *temp++;
		hperh->perh->KEY[0] = *temp;
		break;

	case KEY_6_LEN:
		hperh->perh->KEY[5] = *temp++;
		hperh->perh->KEY[4] = *temp++;
		hperh->perh->KEY[3] = *temp++;
		hperh->perh->KEY[2] = *temp++;
		hperh->perh->KEY[1] = *temp++;
		hperh->perh->KEY[0] = *temp;
		break;

	case KEY_4_LEN:
		hperh->perh->KEY[3] = *temp++;
		hperh->perh->KEY[2] = *temp++;
		hperh->perh->KEY[1] = *temp++;
		hperh->perh->KEY[0] = *temp;
		break;

	case KEY_2_LEN:
		hperh->perh->KEY[1] = *temp++;
		hperh->perh->KEY[0] = *temp;
		break;

	default:
		break;
	}

	for (i = 0; i < len; i++)
		hperh->key[i] = *key++;

	hperh->key_size = len;

	return OK;
}

/**
  * @brief  Read the Content of KEY.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  key: Pointer to key data buffer
  * @param  len: The length of key(32 bits)
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_read_key(crypt_handle_t *hperh, uint32_t * key, crypt_key_len_t len)
{
	uint32_t *temp   = key;

	if ((hperh == NULL) || (key == NULL))
		return ERROR;
	if (hperh->state == CRYPT_STATE_BUSY)
		return BUSY;

	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_KEY_LEN(len));

	switch (len) {
	case KEY_8_LEN:
		*temp++ = hperh->perh->KEY[7];
		*temp++ = hperh->perh->KEY[6];
		*temp++ = hperh->perh->KEY[5];
		*temp++ = hperh->perh->KEY[4];
		*temp++ = hperh->perh->KEY[3];
		*temp++ = hperh->perh->KEY[2];
		*temp++ = hperh->perh->KEY[1];
		*temp   = hperh->perh->KEY[0];
		break;

	case KEY_6_LEN:
		*temp++ = hperh->perh->KEY[5];
		*temp++ = hperh->perh->KEY[4];
		*temp++ = hperh->perh->KEY[3];
		*temp++ = hperh->perh->KEY[2];
		*temp++ = hperh->perh->KEY[1];
		*temp   = hperh->perh->KEY[0];
		break;

	case KEY_4_LEN:
		*temp++ = hperh->perh->KEY[3];
		*temp++ = hperh->perh->KEY[2];
		*temp++ = hperh->perh->KEY[1];
		*temp   = hperh->perh->KEY[0];
		break;

	case KEY_2_LEN:
		*temp++ = hperh->perh->KEY[1];
		*temp   = hperh->perh->KEY[0];
		break;

	default:
		break;
	}

	return OK;
}

/**
  * @brief  Write the Content of IV if you use CBC mode
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  iv: Pointer to iv data buffer
  * @param  len: the length of iv(32 bits)
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_write_ivr(crypt_handle_t *hperh, uint32_t * iv, crypt_ivr_len_t len)
{
	uint32_t *temp = iv;
	uint32_t i;

	if ((hperh == NULL) || (iv == NULL))
		return ERROR;
	if (hperh->state == CRYPT_STATE_BUSY)
		return BUSY;

	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_IV_LEN(len));

	switch (len) {
	case IV_4_LEN:
		hperh->perh->IV[3] = *temp++;
		hperh->perh->IV[2] = *temp++;
		hperh->perh->IV[1] = *temp++;
		hperh->perh->IV[0] = *temp;
		break;

	case IV_2_LEN:
		hperh->perh->IV[1] = *temp++;
		hperh->perh->IV[0] = *temp;
		break;

	default:
		break;
	}

	for (i = 0; i < len; i++)
		hperh->iv[i] = *iv++;

	hperh->iv_size = len;

	CRYPT_IVEN_ENABLE(hperh);
	return OK;
}

/**
  * @brief  Read the Content of IV.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  iv: Pointer to iv data buffer
  * @param  len: the length of iv(32 bits)
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_read_ivr(crypt_handle_t *hperh, uint32_t *iv, crypt_ivr_len_t len)
{
	uint32_t *temp   = iv;

	if ((hperh == NULL) || (iv == NULL))
		return ERROR;
	if (hperh->state == CRYPT_STATE_BUSY)
		return BUSY;

	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_IV_LEN(len));

	switch (len) {
	case IV_4_LEN:
		*temp++ = hperh->perh->IV[3];
		*temp++ = hperh->perh->IV[2];
		*temp++ = hperh->perh->IV[1];
		*temp   = hperh->perh->IV[0];
		break;

	case IV_2_LEN:
		*temp++ = hperh->perh->IV[1];
		*temp   = hperh->perh->IV[0];
		break;

	default:
		break;
	}

	return OK;
}

/**
  * @}
  */

/** @defgroup CRYPT_Public_Functions_Group2 Encrypt or Decrypt functions
  * @brief Encrypt or Decrypt functions
  * @{
  */

/**
  * @brief  Encrypt an amount of data in blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  plain_text: Pointer to plain data buffer
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of plain data
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_encrypt(crypt_handle_t *hperh, uint8_t * plain_text, uint8_t * cipher_text, uint32_t size)
{
	uint32_t count = 0;
	uint32_t i;
	uint32_t *plain_buf  = (uint32_t *)plain_text;
	uint32_t *cipher_buf = (uint32_t *)cipher_text;

	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;

	if ((plain_buf == NULL) || (cipher_buf == NULL) || (size == 0))
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;
	CRYPT_SETDIR(hperh, CRYPT_ENCRYPT);
	count = size / (4 * hperh->step);

	while (count--) {
		for (i = 0; i < hperh->step; i++) {
			CRYPT_WRITE_FIFO(hperh, *plain_buf);
			plain_buf++;
		}

		while (ald_crypt_get_flag_status(hperh, CRYPT_FLAG_DONE) == SET);

		for (i = 0; i < hperh->step; i++)
			*cipher_buf++ = CRYPT_READ_FIFO(hperh);
	}

	hperh->state = CRYPT_STATE_READY;
	__UNLOCK(hperh);

	return OK;
}

/**
  * @brief  Decrypt an amount of data in blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  plain_text: Pointer to plain data buffer
  * @param  size: Amount of cipher data
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_decrypt(crypt_handle_t *hperh, uint8_t *cipher_text, uint8_t *plain_text, uint32_t size)
{
	uint32_t count = 0;
	uint32_t i;
	uint32_t *plain_buf  = (uint32_t*)plain_text;
	uint32_t *cipher_buf = (uint32_t*)cipher_text;

	if (hperh->init.mode == CRYPT_MODE_CTR) {
		return ald_crypt_encrypt(hperh, cipher_text, plain_text, size);
	}

	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;

	if ((plain_buf == NULL) || (cipher_buf == NULL) || (size == 0))
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;
	CRYPT_SETDIR(hperh, CRYPT_DECRYPT);
	count = size / (4 * hperh->step);

	while (count--) {
		for (i = 0; i < hperh->step; i++) {
			CRYPT_WRITE_FIFO(hperh, *cipher_buf);
			cipher_buf++;
		}

		while (ald_crypt_get_flag_status(hperh, CRYPT_FLAG_DONE) == SET);

		for (i = 0; i < hperh->step; i++)
			*plain_buf++ = CRYPT_READ_FIFO(hperh);
	}

	hperh->state = CRYPT_STATE_READY;
	__UNLOCK(hperh);

	return OK;
}

void gcm_mul(uint32_t *res, uint32_t *data, uint32_t *iv)
{
	CRYPT->CON = 0;
	CRYPT->DATA[0] = data[3];
	CRYPT->DATA[1] = data[2];
	CRYPT->DATA[2] = data[1];
	CRYPT->DATA[3] = data[0];
	CRYPT->IV[0]   = iv[3];
	CRYPT->IV[1]   = iv[2];
	CRYPT->IV[2]   = iv[1];
	CRYPT->IV[3]   = iv[0];
	CRYPT->CON |= ((1 << CRYPT_CON_RESCLR_POS) | (3 << CRYPT_CON_MODE_POSS) | 	\
		       (1 << CRYPT_CON_GO_POS));

	while (READ_BIT(CRYPT->IF, CRYPT_IF_MULTHIF_MSK) == 0);

	res[3] = CRYPT->RES[0];
	res[2] = CRYPT->RES[1];
	res[1] = CRYPT->RES[2];
	res[0] = CRYPT->RES[3];

	SET_BIT(CRYPT->IFC, CRYPT_IFC_MULTHIFC_MSK);
	return;
}

/**
  * @brief  verify an amount of data in gcm mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of cipher data
  * @param  aadata: Pointer to additional authenticated data buffer
  * @param  alen: Amount of additional authenticated data
  * @param  tag: Pointer to authentication tag buffer
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_gcm_verify(crypt_handle_t *hperh, uint8_t *cipher_text, uint32_t size, uint8_t *aadata, uint32_t alen, uint8_t *tag)
{
	uint8_t GCM_HASH_in[0x60] = {0} ;
	uint8_t ecb[16] = {0} ;
	uint32_t x_temp[4];
	uint64_t u, v;
	uint32_t len = 0;
	uint32_t j, i, k;
	uint32_t *tag_temp, *cipher_text_temp;

	/* calculate u and v */
	u = 128 * ((size % 16) ? (size / 16 + 1) : size / 16) - size * 8;
	v = 128 * ((alen % 16) ? (alen / 16 + 1): alen / 16) - alen * 8;

	/* get the input of GHASH algorithm,the input:A||0^v||C||0^u||[len(A)]_64||[len(C)]_64 */
	for (i = 0; i < alen; i++) {
		GCM_HASH_in [i] = * (aadata + i);
	}
	len += alen;
	for (i = 0; i < v / 8; i++) {
		GCM_HASH_in[i + len] = 0;
	}
	len += v / 8;
	for (i = 0; i < size; i++) {
		GCM_HASH_in[i + len] = * (cipher_text + i);
	}
	len += size;
	for (i = 0; i < u / 8; i++) {
		GCM_HASH_in[i + len] = 0;
	}
	len += u / 8;

	for (i = 0; i < 4; i++) {
		GCM_HASH_in[i + len] = 0;
	}
	len += 4;

	for (i = 0; i < 4; i++) {
		GCM_HASH_in[i + len] = ((alen * 8) >> (8 * i)) & 0xFF;
	}
	len += 4;

	for (i = 0; i < 4; i++) {
		GCM_HASH_in[i + len] = 0;
	}
	len += 4;

	for (i = 0; i < 4; i++) {
		GCM_HASH_in[i + len] = ((size * 8) >> (8 * i)) & 0xFF;
	}
	len += 4;

	CRYPT->CON &= ~(3U << CRYPT_CON_MODE_POSS);
	CRYPT->CON |= (CRYPT_MODE_ECB << CRYPT_CON_MODE_POSS);

	ald_crypt_encrypt(hperh, ecb, ecb, 16);

	k = len / 16;
	for (i = 0; i < 16; i++) {
		tag[i] = 0;
	}

	cipher_text_temp = (uint32_t *)GCM_HASH_in;
	tag_temp         = (uint32_t *)tag;
	for (i = 0; i < k; i++) {
		for (j = 0; j < 4; j++) {
			x_temp[j] = (*cipher_text_temp) ^ tag_temp[j];
			++cipher_text_temp;
		}

		gcm_mul((uint32_t *)tag_temp, x_temp, (uint32_t *)ecb);
	}

	/* calculate the authentication tag T,
	 * T = CIPH_K(J0)^S,J0=IV||0^31||1,CIPH_K is the algorithm of AES in ECB mode
	 */
	tag_temp = (uint32_t *)tag;
	ald_crypt_init(hperh);
	CRYPT->CON &= ~(3U << CRYPT_CON_MODE_POSS);
	CRYPT->CON |= (CRYPT_MODE_CTR << CRYPT_CON_MODE_POSS);
	ald_crypt_write_key(hperh, hperh->key, KEY_4_LEN);
	hperh->iv[3] = 1;
	ald_crypt_write_ivr(hperh, hperh->iv, IV_4_LEN);
	ald_crypt_encrypt(hperh, tag, tag, 16);

	return OK;
}

/**
  * @brief  Encrypt an amount of data in non-blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  plain_text: Pointer to plain data buffer
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of plain data
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_encrypt_by_it(crypt_handle_t *hperh, uint8_t * plain_text, uint8_t *cipher_text, uint32_t size)
{
	uint32_t i;
	uint32_t *plain_buf = (uint32_t *)plain_text;

	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;

	if ((plain_text == NULL) || (cipher_text == NULL) || (size == 0))
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;
	CRYPT_SETDIR(hperh, CRYPT_ENCRYPT);
	hperh->count       = hperh->step;
	hperh->plain_text  = plain_text;
	hperh->cipher_text = cipher_text;
	hperh->size        = size;
	ald_crypt_interrupt_config(hperh, CRYPT_IT_IT, ENABLE);

	for (i = 0; i < hperh->step; i++) {
		CRYPT_WRITE_FIFO(hperh, *plain_buf);
		++plain_buf;
	}

	__UNLOCK(hperh);
	return OK;
}

/**
  * @brief  Decrypt an amount of data in non-blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  plain_text: Pointer to plain data buffer
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of cipher data
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_decrypt_by_it(crypt_handle_t *hperh, uint8_t *cipher_text, uint8_t *plain_text, uint32_t size)
{
	uint32_t i;
	uint32_t *cipher_buf = (uint32_t*)cipher_text;

	if (hperh->init.mode == CRYPT_MODE_CTR) {
		return ald_crypt_decrypt_by_it(hperh, cipher_text, plain_text, size);
	}

	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;

	if ((plain_text == NULL) || (cipher_text == NULL) || (size == 0))
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;
	CRYPT_SETDIR(hperh, CRYPT_DECRYPT);
	hperh->count       = hperh->step;
	hperh->plain_text  = plain_text;
	hperh->cipher_text = cipher_text;
	hperh->size        = size;
	ald_crypt_interrupt_config(hperh, CRYPT_IT_IT, ENABLE);

	for (i = 0; i < hperh->step; i++) {
		CRYPT_WRITE_FIFO(hperh, *cipher_buf);
		cipher_buf ++;
	}

	__UNLOCK(hperh);
	return OK;
}

#ifdef ALD_DMA
/**
  * @brief  Encrypt an amount of data in non-blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  plain_text: Pointer to plain data buffer
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of plain data
  * @param  channel_m2p: Memory to Crypt module DMA channel
  * @param  channel_p2m: Crypt module to Memory DMA channel
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_encrypt_by_dma(crypt_handle_t *hperh, uint8_t * plain_text,
             uint8_t *cipher_text, uint32_t size, uint8_t channel_m2p, uint8_t channel_p2m)
{
	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;

	if (plain_text == NULL || cipher_text == NULL || size == 0)
		return ERROR;

	assert_param(IS_CRYPT(hperh->perh));

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;

	hperh->plain_text  = plain_text;
	hperh->cipher_text = cipher_text;
	hperh->size        = size;
	hperh->count       = size;

	if (hperh->hdma_m2p.perh == NULL)
		hperh->hdma_m2p.perh = DMA0;
	if (hperh->hdma_p2m.perh == NULL)
		hperh->hdma_p2m.perh = DMA0;

	hperh->hdma_m2p.cplt_arg = NULL;
	hperh->hdma_m2p.cplt_cbk = NULL;
	hperh->hdma_m2p.err_arg  = NULL;
	hperh->hdma_m2p.err_cbk  = NULL;

	hperh->hdma_p2m.cplt_arg = (void *)hperh;
	hperh->hdma_p2m.cplt_cbk = &crypt_dma_crypt_cplt;
	hperh->hdma_p2m.err_arg  = (void *)hperh;
	hperh->hdma_p2m.err_cbk  = &crypt_dma_error;

	CRYPT_SETDIR(hperh, CRYPT_ENCRYPT);

	ald_dma_config_struct(&hperh->hdma_m2p.config);
	hperh->hdma_m2p.config.data_width = DMA_DATA_SIZE_WORD;
	hperh->hdma_m2p.config.src        = (void *)hperh->plain_text;
	hperh->hdma_m2p.config.dst        = (void *)&hperh->perh->FIFO;
	hperh->hdma_m2p.config.size       = size / 4;
	hperh->hdma_m2p.config.src_inc    = DMA_DATA_INC_WORD;
	hperh->hdma_m2p.config.dst_inc    = DMA_DATA_INC_NONE;
	hperh->hdma_m2p.config.msel       = DMA_MSEL_CRYPT;
	hperh->hdma_m2p.config.msigsel    = DMA_MSIGSEL_CRYPT_WRITE;
	hperh->hdma_m2p.config.burst      = ENABLE;
	hperh->hdma_m2p.config.channel    = channel_m2p;
	ald_dma_config_basic(&(hperh->hdma_m2p));

	ald_dma_config_struct(&hperh->hdma_p2m.config);
	hperh->hdma_p2m.config.data_width = DMA_DATA_SIZE_WORD;
	hperh->hdma_p2m.config.src        = (void *)&hperh->perh->FIFO;
	hperh->hdma_p2m.config.dst        = (void *)hperh->cipher_text;
	hperh->hdma_p2m.config.size       = size / 4;
	hperh->hdma_p2m.config.src_inc    = DMA_DATA_INC_NONE;
	hperh->hdma_p2m.config.dst_inc    = DMA_DATA_INC_WORD;
	hperh->hdma_p2m.config.msel       = DMA_MSEL_CRYPT;
	hperh->hdma_p2m.config.msigsel    = DMA_MSIGSEL_CRYPT_READ;
	hperh->hdma_p2m.config.burst      = ENABLE;
	hperh->hdma_p2m.config.channel    = channel_p2m;
	ald_dma_config_basic(&(hperh->hdma_p2m));

	CRYPT_DMA_ENABLE(hperh);
	__UNLOCK(hperh);

	return OK;
}

/**
  * @brief  Decrypt an amount of data in non-blocking mode.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  plain_text: Pointer to plain data buffer
  * @param  cipher_text: Pointer to cipher data buffer
  * @param  size: Amount of cipher data
  * @param  channel_m2p: Memory to Crypt module DMA channel
  * @param  channel_p2m: Crypt module to Memory DMA channel
  * @retval Status, see @ref ald_status_t.
  * @note   the size is multiple of 8(des) or 16(ase)
  */
ald_status_t ald_crypt_decrypt_by_dma(crypt_handle_t *hperh, uint8_t * cipher_text,
               uint8_t *plain_text, uint32_t size, uint8_t channel_m2p, uint8_t channel_p2m)
{
	if (hperh->init.mode == CRYPT_MODE_CTR)
		return ald_crypt_decrypt_by_dma(hperh, cipher_text, plain_text, size, channel_m2p, channel_p2m);

	if (hperh->state != CRYPT_STATE_READY)
		return ERROR;
	if (plain_text == NULL || cipher_text == NULL || size == 0)
		return ERROR;

	__LOCK(hperh);
	hperh->state = CRYPT_STATE_BUSY;

	hperh->plain_text  = plain_text;
	hperh->cipher_text = cipher_text;
	hperh->size        = size;
	hperh->count       = size;

	if (hperh->hdma_m2p.perh == NULL)
		hperh->hdma_m2p.perh = DMA0;
	if (hperh->hdma_p2m.perh == NULL)
		hperh->hdma_p2m.perh = DMA0;


	hperh->hdma_m2p.cplt_arg = NULL;
	hperh->hdma_m2p.cplt_cbk = NULL;
	hperh->hdma_m2p.err_arg  = NULL;
	hperh->hdma_m2p.err_cbk  = NULL;

	hperh->hdma_p2m.cplt_arg = (void *)hperh;
	hperh->hdma_p2m.cplt_cbk = &crypt_dma_crypt_cplt;
	hperh->hdma_p2m.err_arg  = (void *)hperh;
	hperh->hdma_p2m.err_cbk  = &crypt_dma_error;

	CRYPT_SETDIR(hperh, CRYPT_DECRYPT);

	ald_dma_config_struct(&hperh->hdma_m2p.config);
	hperh->hdma_m2p.config.data_width = DMA_DATA_SIZE_WORD;
	hperh->hdma_m2p.config.src        = (void *)hperh->cipher_text;
	hperh->hdma_m2p.config.dst        = (void *)&hperh->perh->FIFO;
	hperh->hdma_m2p.config.size       = size / 4;
	hperh->hdma_m2p.config.src_inc    = DMA_DATA_INC_WORD;
	hperh->hdma_m2p.config.dst_inc    = DMA_DATA_INC_NONE;
	hperh->hdma_m2p.config.msel       = DMA_MSEL_CRYPT;
	hperh->hdma_m2p.config.msigsel    = DMA_MSIGSEL_CRYPT_WRITE;
	hperh->hdma_m2p.config.burst      = ENABLE;
	hperh->hdma_m2p.config.channel    = channel_m2p;
	ald_dma_config_basic(&(hperh->hdma_m2p));

	ald_dma_config_struct(&hperh->hdma_p2m.config);
	hperh->hdma_p2m.config.data_width = DMA_DATA_SIZE_WORD;
	hperh->hdma_p2m.config.src        = (void *)&hperh->perh->FIFO;
	hperh->hdma_p2m.config.dst        = (void *)hperh->plain_text;
	hperh->hdma_p2m.config.size       = size / 4;
	hperh->hdma_p2m.config.src_inc    = DMA_DATA_INC_NONE;
	hperh->hdma_p2m.config.dst_inc    = DMA_DATA_INC_WORD;
	hperh->hdma_p2m.config.msel       = DMA_MSEL_CRYPT;
	hperh->hdma_p2m.config.msigsel    = DMA_MSIGSEL_CRYPT_READ;
	hperh->hdma_m2p.config.burst      = ENABLE;
	hperh->hdma_p2m.config.channel    = channel_p2m;
	ald_dma_config_basic(&(hperh->hdma_p2m));

	CRYPT_DMA_ENABLE(hperh);
	__UNLOCK(hperh);

	return OK;
}

/**
  * @}
  */

/** @defgroup CRYPT_Public_Functions_Group3 DMA operation functions
  * @brief DMA operation functions
  * @{
  */

/**
  * @brief  Pauses the DMA Transfer.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_dma_pause(crypt_handle_t *hperh)
{
	__LOCK(hperh);
	CRYPT_DMA_DISABLE(hperh);
	__UNLOCK(hperh);

	return OK;

}

/**
  * @brief  Resumes the DMA Transfer.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_dma_resume(crypt_handle_t *hperh)
{
	__LOCK(hperh);
	CRYPT_DMA_ENABLE(hperh);
	__UNLOCK(hperh);

	return OK;
}

/**
  * @brief  Stops the DMA Transfer.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval Status, see @ref ald_status_t.
  */
ald_status_t ald_crypt_dma_stop(crypt_handle_t *hperh)
{
	__LOCK(hperh);
	CRYPT_DMA_DISABLE(hperh);
	__UNLOCK(hperh);

	hperh->state = CRYPT_STATE_READY;
	return OK;
}
#endif

/**
  * @brief  This function handles CRYPT interrupt request.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval None
  */
void ald_crypt_irq_handler(crypt_handle_t *hperh)
{
	uint32_t i;
	uint32_t *in_buf  = (uint32_t *)hperh->cipher_text;
	uint32_t *out_buf = (uint32_t *)hperh->plain_text;

	if (READ_BIT(hperh->perh->CON, CRYPT_CON_ENCS_MSK) == (CRYPT_DECRYPT << CRYPT_CON_ENCS_POS)) {
		in_buf  = (uint32_t *)hperh->cipher_text + hperh->count;
		out_buf = (uint32_t *)hperh->plain_text + hperh->count - hperh->step;
	}
	else {
		in_buf  = (uint32_t *)hperh->plain_text + hperh->count;
		out_buf = (uint32_t *)hperh->cipher_text + hperh->count - hperh->step;
	}

	if (ald_crypt_get_flag_status(hperh, CRYPT_FLAG_AESIF) == SET) {
		ald_crypt_clear_flag_status(hperh, CRYPT_FLAG_AESIF);
	}

	if (ald_crypt_get_flag_status(hperh, CRYPT_FLAG_DESIF) == SET) {
		ald_crypt_clear_flag_status(hperh, CRYPT_FLAG_DESIF);
	}

	for (i = 0; i < hperh->step; i++)
		*out_buf++ = CRYPT_READ_FIFO(hperh);

	hperh->count = hperh->count + hperh->step;
	if (hperh->count > (hperh->size / 4)) {
		hperh->count = 0;
		hperh->state = CRYPT_STATE_READY;
		hperh->crypt_cplt_cbk(hperh);
	}
	else {
		for (i = 0; i < hperh->step; i++) {
			CRYPT_WRITE_FIFO(hperh, *in_buf);
			++in_buf;
		}
	}
}
/**
  * @}
  */

/** @defgroup CRYPT_Public_Functions_Group4 Peripheral Control functions
  *  @brief   CRYPT control functions
  * @{
  */

/**
  * @brief  Enables or disables the specified CRYPT interrupts.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  it: Specifies the CRYPT interrupt sources to be enabled or disabled.
  *           This parameter can be one of the following values:
  *           @arg crypt_it_t:  CRYPT interrupt
  * @param  state: New status
  *           - ENABLE
  *           - DISABLE
  * @retval None
  */
void ald_crypt_interrupt_config(crypt_handle_t *hperh, crypt_it_t it, type_func_t state)
{
	assert_param(IS_CRYPT(hperh->perh));

	if (it == CRYPT_IT_IT) {
		CLEAR_BIT(CRYPT->CON, CRYPT_CON_IE_MSK);
		CRYPT->CON |= (state << CRYPT_CON_IE_POS);
	}

	return;
}

/** @brief  Check whether the specified CRYPT flag is set or not.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  flag: specifies the flag to check.
  *         This parameter can be one of the @ref crypt_flag_t.
  * @retval Status
  *           - SET
  *           - RESET
  */
flag_status_t ald_crypt_get_flag_status(crypt_handle_t *hperh, crypt_flag_t flag)
{
	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_FLAG(flag));

	if (CRYPT->IF & flag)
		return SET;

	return RESET;
}

/** @brief  Clear the specified CRYPT pending flags.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  flag: specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg CRYPT_FLAG_AESIF: AES encrypt or decrypt Complete flag.
  *            @arg CRYPT_FLAG_DESIF: AES encrypt or decrypt Complete flag.
  *            @arg CRYPT_FLAG_DONE: encrypt or decrypt Complete flag.
  * @retval None
  */
void ald_crypt_clear_flag_status(crypt_handle_t *hperh, crypt_flag_t flag)
{
	assert_param(IS_CRYPT(hperh->perh));
	assert_param(IS_CRYPT_FLAG(flag));

	CRYPT->IFC = (uint32_t)flag;
	return;
}

/**
  * @brief  Checks whether the specified CRYPT interrupt has occurred or not.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @param  it: Specifies the CRYPT interrupt source to check.
  *	       This parameter can be one of the following values:
  *	       @arg crypt_it_t:  CRYPT interrupt
  * @retval Status
  *           - SET
  *           - RESET
  */
it_status_t ald_crypt_get_it_status(crypt_handle_t *hperh, crypt_it_t it)
{
	assert_param(IS_CRYPT_IT(it));

	if (READ_BIT(CRYPT->CON, CRYPT_CON_IE_MSK))
		return SET;

	return RESET;
}


/**
  * @}
  */

/** @defgroup CRYPT_Public_Functions_Group5 Peripheral State and Errors functions
  * @brief    State and Errors functions
  * @{
  */

/**
  * @brief  Returns the CRYPT state.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval CRYPT state
  */
crypt_state_t ald_crypt_get_state(crypt_handle_t *hperh)
{
	assert_param(IS_CRYPT(hperh->perh));

	return hperh->state;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup CRYPT_Private_Functions   CRYPT Private Functions
  *  @brief   CRYPT Private functions
  * @{
  */

/**
  * @brief  Reset the CRYPT peripheral.
  * @param  hperh: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval None
  */
void crypt_reset(crypt_handle_t *hperh)
{
	hperh->perh->DATA[0] = 0x0;
	hperh->perh->DATA[1] = 0x0;
	hperh->perh->DATA[2] = 0x0;
	hperh->perh->DATA[3] = 0x0;
	hperh->perh->KEY[0]  = 0x0;
	hperh->perh->KEY[1]  = 0x0;
	hperh->perh->KEY[2]  = 0x0;
	hperh->perh->KEY[3]  = 0x0;
	hperh->perh->KEY[4]  = 0x0;
	hperh->perh->KEY[5]  = 0x0;
	hperh->perh->KEY[6]  = 0x0;
	hperh->perh->KEY[7]  = 0x0;
	hperh->perh->IV[0]   = 0x0;
	hperh->perh->IV[1]   = 0x0;
	hperh->perh->IV[2]   = 0x0;
	hperh->perh->IV[3]   = 0x0;
	hperh->perh->CON     = 0x0;

	hperh->state = CRYPT_STATE_READY;
	__UNLOCK(hperh);
}

#ifdef ALD_DMA
/**
  * @brief  DMA CRYPT encrypt or decrypt process complete callback.
  * @param  arg: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval None
  */
static void crypt_dma_crypt_cplt(void *arg)
{
	crypt_handle_t *hperh = (crypt_handle_t *)arg;

	CRYPT_DMA_DISABLE(hperh);
	hperh->count       = 0;
	hperh->plain_text  = NULL;
	hperh->cipher_text = NULL;
	hperh->size        = 0;

	hperh->state = CRYPT_STATE_READY;

	if (hperh->crypt_cplt_cbk)
		hperh->crypt_cplt_cbk(hperh);
}

/**
  * @brief  DMA CRYPT communication error callback.
  * @param  arg: Pointer to a crypt_handle_t structure that contains
  *         the configuration information for the specified CRYPT module.
  * @retval None
  */
static void crypt_dma_error(void *arg)
{
	crypt_handle_t *hperh = (crypt_handle_t *)arg;
	CRYPT_DMA_DISABLE(hperh);

	hperh->count       = 0;
	hperh->plain_text  = NULL;
	hperh->cipher_text = NULL;
	hperh->size        = 0;

	hperh->state = CRYPT_STATE_READY;

	if (hperh->err_cplt_cbk)
		hperh->err_cplt_cbk(hperh);
}
#endif
/**
  * @}
  */

#endif /* ALD_CRYPT */
/**
  * @}
  */

/**
  * @}
  */
