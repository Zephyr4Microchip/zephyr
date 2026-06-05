/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_crypto_g2

#include <string.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <cam_aes.h>
LOG_MODULE_REGISTER(crypto_mchp_g2, CONFIG_CRYPTO_LOG_LEVEL);

/* Capability flags advertised by this driver */
#define MCHP_G2_HW_CAPS \
	(CAP_RAW_KEY | CAP_INPLACE_OPS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX | CAP_SEPARATE_IO_BUFS)

/* Opaque Crypto context buffer size in bytes */
#define MCHP_G2_CTX_BUF_SIZE 512UL

/* Flag bit indicating AAD is present in the CCM dataset */
#define CCM_AAD_PRESENT_FLAG (1U << 6)

/* CCM authentication tag scratch buffer size in bytes */
#define AES_CCM_AUTHTAG_SIZE (16U)

struct mchp_g2_session {
	/* Crypto context buffer; must be word-aligned */
	uint32_t crypto_ctx[MCHP_G2_CTX_BUF_SIZE / sizeof(uint32_t)];

	/* AES key (up to 32 bytes for AES-256) */
	uint8_t key[32];

	/* Key length in bytes (16, 24, or 32) */
	uint16_t keylen;

	/* CCM authentication tag length in bytes */
	uint16_t ccm_tag_len;

	/* CCM nonce length in bytes */
	uint16_t ccm_nonce_len;

	/* True while this session slot is acquired by a caller */
	bool in_use;
};

static struct mchp_g2_session g_session;
static struct k_mutex g_session_lock;

/*
 * Returns the number of padding bytes needed to reach the next AES block
 * boundary. Returns zero when data_len is already block-aligned.
 */
static uint32_t crypto_mchp_g2_aes_get_num_invalid_bytes(uint32_t data_len)
{
	uint32_t bytes_over = data_len % (uint32_t)AES_BLOCK_SIZE;

	return (bytes_over != 0U) ? ((uint32_t)AES_BLOCK_SIZE - bytes_over) : 0U;
}

/*
 * Returns the number of pad bytes needed to complete an AES block.
 */
static uint32_t crypto_mchp_g2_aes_get_pad_bytes(uint32_t data_len)
{
	uint32_t mask = (AES_BLOCK_SIZE - 1UL);

	return ((data_len + mask) & ~mask) - data_len;
}

/*
 * Builds the CCM B0 header block per RFC 3610 section 2.2.
 * header must be at least 22 bytes. header_len is set to bytes written.
 */
static void crypto_mchp_g2_build_ccm_header(uint8_t *header, uint32_t *header_len,
					    uint8_t *nonce, uint32_t nonce_len,
					    uint32_t aad_len, uint32_t data_len,
					    uint32_t auth_tag_len)
{
	uint8_t *p = header;
	uint8_t tag_size = ((uint8_t)auth_tag_len - 2U) / 2U;
	uint8_t length_sz = (uint8_t)(0x0FU - (uint8_t)nonce_len);
	uint8_t flags = 0U;

	flags |= (tag_size & 0x07U) << 3U;
	flags |= (length_sz - 1U) & 0x07U;
	if (aad_len > 0U) {
		flags |= CCM_AAD_PRESENT_FLAG;
	}

	/* Byte 0: flags */
	*p++ = flags;

	/* Bytes 1..nonce_len: nonce */
	for (uint32_t i = 0U; i < nonce_len; i++) {
		*p++ = nonce[i];
	}

	/* Bytes nonce_len+1..15: big-endian payload length field */
	for (int8_t i = (int8_t)((length_sz - 1U) * 8U); i >= 0; i -= 8) {
		*p++ = (uint8_t)((data_len >> (uint32_t)(unsigned int)i) & 0xFFU);
	}

	/* Bytes 16+: AAD length encoding */
	p = &header[16];
	if (aad_len > 0U) {
		if (aad_len < 0xFF00UL) {
			*p++ = (uint8_t)((aad_len >> 8U) & 0xFFU);
			*p++ = (uint8_t)(aad_len & 0xFFU);
		} else {
			*p++ = 0xFFU;
			*p++ = 0xFEU;
			*p++ = (uint8_t)((aad_len >> 24U) & 0xFFU);
			*p++ = (uint8_t)((aad_len >> 16U) & 0xFFU);
			*p++ = (uint8_t)((aad_len >> 8U) & 0xFFU);
			*p++ = (uint8_t)(aad_len & 0xFFU);
		}
	}

	*header_len = (uint32_t)(p - header);
}

/*
 * Checks that the hardware CCM tag verification buffer is all-zeros.
 * The CAM in CCM decrypt mode outputs (computed_tag XOR received_tag) into
 * the output tag buffer; all-zeros indicates a successful authentication.
 * Returns true on auth pass, false on failure.
 */
static bool crypto_mchp_g2_ccm_tag_auth_ok(const uint8_t *tag_xor_buf, uint32_t len)
{
	for (uint32_t i = 0U; i < len; i++) {
		if (tag_xor_buf[i] != 0U) {
			return false;
		}
	}
	return true;
}

/*
 * Polls DRV_CRYPTO_AES_IsActive() until the CAM context is ready after
 * DRV_CRYPTO_AES_Initialize(). Must be called before AddInputData.
 */
static int mchp_g2_ctx_wait_active(void *ctx)
{
	AES_ERROR active;
	AES_ERROR err;

	err = DRV_CRYPTO_AES_IsActive(ctx, &active);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CAM AES IsActive error: %d", err);
		return -EIO;
	}

	return 0;
}

/* AES-ECB encrypt handler */
static int crypto_mchp_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	uint32_t pad, full_len;
	AES_ERROR err;

	if ((pkt->in_len == 0) || (pkt->in_len % AES_BLOCK_SIZE != 0)) {
		LOG_ERR("ECB enc: input length %d must be a non-zero multiple of %d",
			pkt->in_len, AES_BLOCK_SIZE);
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("ECB enc: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_ECB, OP_ENCRYPT, sess->key,
					(uint32_t)sess->keylen, NULL, 0U);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB enc: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	pad = crypto_mchp_g2_aes_get_num_invalid_bytes((uint32_t)pkt->in_len);
	full_len = (uint32_t)pkt->in_len + pad;

	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB enc: AddInputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB enc: AddOutputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB enc: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

/* AES-ECB decrypt handler */
static int crypto_mchp_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	AES_ERROR err;

	if ((pkt->in_len == 0) || (pkt->in_len % AES_BLOCK_SIZE != 0)) {
		LOG_ERR("ECB dec: input length %d must be a non-zero multiple of %d",
			pkt->in_len, AES_BLOCK_SIZE);
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("ECB dec: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_ECB, OP_DECRYPT, sess->key,
					(uint32_t)sess->keylen, NULL, 0U);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB dec: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf, (uint32_t)pkt->in_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB dec: AddInputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf,
					   (uint32_t)pkt->out_buf_max);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB dec: AddOutputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("ECB dec: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

/* AES-CBC encrypt handler */
static int crypto_mchp_cbc_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	uint32_t pad, full_len;
	AES_ERROR err;

	if ((pkt->in_len == 0) || (pkt->in_len % AES_BLOCK_SIZE != 0)) {
		LOG_ERR("CBC enc: input length %d must be a non-zero multiple of %d",
			pkt->in_len, AES_BLOCK_SIZE);
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("CBC enc: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_CBC, OP_ENCRYPT, sess->key,
					(uint32_t)sess->keylen, iv, AES_BLOCK_SIZE);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC enc: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	pad = crypto_mchp_g2_aes_get_num_invalid_bytes((uint32_t)pkt->in_len);
	full_len = (uint32_t)pkt->in_len + pad;

	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC enc: AddInputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC enc: AddOutputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC enc: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

/* AES-CBC decrypt handler */
static int crypto_mchp_cbc_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	AES_ERROR err;

	if ((pkt->in_len == 0) || (pkt->in_len % AES_BLOCK_SIZE != 0)) {
		LOG_ERR("CBC dec: input length %d must be a non-zero multiple of %d",
			pkt->in_len, AES_BLOCK_SIZE);
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("CBC dec: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_CBC, OP_DECRYPT, sess->key,
					(uint32_t)sess->keylen, iv, AES_BLOCK_SIZE);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC dec: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf, (uint32_t)pkt->in_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC dec: AddInputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf,
					   (uint32_t)pkt->out_buf_max);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC dec: AddOutputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CBC dec: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

/* AES-CTR encrypt/decrypt handler — CTR is symmetric, same op for both directions */
static int crypto_mchp_ctr_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *ctr)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	AES_ERROR err;
	uint32_t pad, full_len;

	if (pkt->in_len == 0) {
		LOG_ERR("CTR: input length must be non-zero");
		return -EINVAL;
	}

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("CTR: output buffer too small (%d < %d)", pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	/*
	 * The CAM CTR mode takes the full 16-byte counter block as the IV.
	 * ctr_info.ctr_len describes the counter field width within that block
	 * but the hardware manages the split internally.
	 */
	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_CTR, OP_ENCRYPT, sess->key,
					(uint32_t)sess->keylen, ctr, AES_BLOCK_SIZE);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CTR: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	pad = crypto_mchp_g2_aes_get_num_invalid_bytes((uint32_t)pkt->in_len);
	full_len = (uint32_t)pkt->in_len + pad;

	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CTR: AddInputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf, full_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CTR: AddOutputData failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CTR: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

/* AES-CCM encrypt handler */
static int crypto_mchp_ccm_encrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *aead_pkt,
				   uint8_t *nonce)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	struct cipher_pkt *pkt = aead_pkt->pkt;
	uint8_t header_buf[22];
	uint32_t header_len;
	uint32_t pad;
	AES_ERROR err;

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("CCM enc: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_CCM, OP_ENCRYPT, sess->key,
					(uint32_t)sess->keylen, NULL, 0U);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	/* Build and submit the CCM B0 header (RFC 3610 section 2.2) */
	crypto_mchp_g2_build_ccm_header(header_buf, &header_len, nonce,
					(uint32_t)sess->ccm_nonce_len, aead_pkt->ad_len,
					(uint32_t)pkt->in_len, (uint32_t)sess->ccm_tag_len);

	err = DRV_CRYPTO_AES_AddRawHeader(sess->crypto_ctx, header_buf, header_len,
					  AES_HEADER_DO_NOT_ALIGN);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: AddRawHeader (B0) failed (%d)", err);
		return -EIO;
	}

	if (aead_pkt->ad_len > 0U) {
		err = DRV_CRYPTO_AES_AddRawHeader(sess->crypto_ctx, aead_pkt->ad,
						  aead_pkt->ad_len, AES_HEADER_ALIGN);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM enc: AddRawHeader (AAD) failed (%d)", err);
			return -EIO;
		}
	}

	/*
	 * Mark trailing pad bytes of the header+AAD region as 'ignore' so the
	 * hardware skips them in the MAC computation, then discard them from
	 * the output stream so only ciphertext follows.
	 */
	pad = crypto_mchp_g2_aes_get_pad_bytes(header_len + aead_pkt->ad_len);

	err = DRV_CRYPTO_AES_IgnoreData(sess->crypto_ctx, pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: IgnoreData (header pad) failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, header_len + aead_pkt->ad_len + pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: DiscardData (header+AAD) failed (%d)", err);
		return -EIO;
	}

	if (pkt->in_len > 0) {
		err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf,
						  (uint32_t)pkt->in_len);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM enc: AddInputData failed (%d)", err);
			return -EIO;
		}

		err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf,
						   (uint32_t)pkt->in_len);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM enc: AddOutputData (ciphertext) failed (%d)", err);
			return -EIO;
		}

		/* Discard hardware-appended padding after the ciphertext */
		pad = crypto_mchp_g2_aes_get_pad_bytes((uint32_t)pkt->in_len);

		err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, pad);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM enc: DiscardData (ciphertext pad) failed (%d)", err);
			return -EIO;
		}
	}

	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, aead_pkt->tag, sess->ccm_tag_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: AddOutputData (tag) failed (%d)", err);
		return -EIO;
	}

	/* Discard hardware-appended padding after the tag */
	pad = crypto_mchp_g2_aes_get_pad_bytes(sess->ccm_tag_len);

	err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: DiscardData (tag pad) failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM enc: Execute failed (%d)", err);
		return -EIO;
	}

	pkt->out_len = pkt->in_len + (int)sess->ccm_tag_len;
	return 0;
}

/* AES-CCM decrypt handler */
static int crypto_mchp_ccm_decrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *aead_pkt,
				   uint8_t *nonce)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;
	struct cipher_pkt *pkt = aead_pkt->pkt;
	/*
	 * The CAM in CCM decrypt mode outputs (computed_tag XOR received_tag)
	 * here. All-zeros indicates successful authentication.
	 */
	uint8_t gen_tag[AES_CCM_AUTHTAG_SIZE] = {0};
	uint8_t header_buf[22];
	uint32_t header_len;
	uint32_t pad;
	AES_ERROR err;

	if (pkt->out_buf_max < pkt->in_len) {
		LOG_ERR("CCM dec: output buffer too small (%d < %d)",
			pkt->out_buf_max, pkt->in_len);
		return -EINVAL;
	}

	/*
	 * CCM decrypt requires OP_ENCRYPT at Initialize time followed by
	 * a SetOperation(OP_DECRYPT) call.
	 */
	err = DRV_CRYPTO_AES_Initialize(sess->crypto_ctx, MODE_CCM, OP_ENCRYPT, sess->key,
					(uint32_t)sess->keylen, NULL, 0U);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: Initialize failed (%d)", err);
		return -EIO;
	}

	if (mchp_g2_ctx_wait_active(sess->crypto_ctx) != 0) {
		return -EIO;
	}

	err = DRV_CRYPTO_AES_SetOperation(sess->crypto_ctx, OP_DECRYPT);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: SetOperation failed (%d)", err);
		return -EIO;
	}

	/* Build and submit the CCM B0 header (RFC 3610 section 2.2) */
	crypto_mchp_g2_build_ccm_header(header_buf, &header_len, nonce,
					(uint32_t)sess->ccm_nonce_len, aead_pkt->ad_len,
					(uint32_t)pkt->in_len, (uint32_t)sess->ccm_tag_len);

	err = DRV_CRYPTO_AES_AddRawHeader(sess->crypto_ctx, header_buf, header_len,
					  AES_HEADER_DO_NOT_ALIGN);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: AddRawHeader (B0) failed (%d)", err);
		return -EIO;
	}

	if (aead_pkt->ad_len > 0U) {
		err = DRV_CRYPTO_AES_AddRawHeader(sess->crypto_ctx, aead_pkt->ad,
						  aead_pkt->ad_len, AES_HEADER_ALIGN);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM dec: AddRawHeader (AAD) failed (%d)", err);
			return -EIO;
		}
	}

	pad = crypto_mchp_g2_aes_get_pad_bytes(header_len + aead_pkt->ad_len);

	err = DRV_CRYPTO_AES_IgnoreData(sess->crypto_ctx, pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: IgnoreData (header pad) failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, header_len + aead_pkt->ad_len + pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: DiscardData (header+AAD) failed (%d)", err);
		return -EIO;
	}

	if (pkt->in_len > 0) {
		err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, pkt->in_buf,
						  (uint32_t)pkt->in_len);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM dec: AddInputData failed (%d)", err);
			return -EIO;
		}

		err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, pkt->out_buf,
						   (uint32_t)pkt->in_len);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM dec: AddOutputData (plaintext) failed (%d)", err);
			return -EIO;
		}

		/* Discard hardware-appended padding after the plaintext */
		pad = crypto_mchp_g2_aes_get_pad_bytes((uint32_t)pkt->in_len);

		err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, pad);
		if (err != AES_NO_ERROR) {
			LOG_ERR("CCM dec: DiscardData (plaintext pad) failed (%d)", err);
			return -EIO;
		}
	}

	/* Supply the received tag as input; hardware XORs it with computed tag */
	err = DRV_CRYPTO_AES_AddInputData(sess->crypto_ctx, aead_pkt->tag, sess->ccm_tag_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: AddInputData (tag) failed (%d)", err);
		return -EIO;
	}

	/* Capture the XOR result; all-zeros means authentication passed */
	err = DRV_CRYPTO_AES_AddOutputData(sess->crypto_ctx, gen_tag, sess->ccm_tag_len);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: AddOutputData (tag) failed (%d)", err);
		return -EIO;
	}

	/* Discard hardware-appended padding after the tag */
	pad = crypto_mchp_g2_aes_get_pad_bytes(sess->ccm_tag_len);

	err = DRV_CRYPTO_AES_DiscardData(sess->crypto_ctx, pad);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: DiscardData (tag pad) failed (%d)", err);
		return -EIO;
	}

	err = DRV_CRYPTO_AES_Execute(sess->crypto_ctx);
	if (err != AES_NO_ERROR) {
		LOG_ERR("CCM dec: Execute failed (%d)", err);
		return -EBADMSG;
	}

	if (!crypto_mchp_g2_ccm_tag_auth_ok(gen_tag, sess->ccm_tag_len)) {
		LOG_ERR("CCM dec: authentication tag mismatch");
		return -EBADMSG;
	}

	pkt->out_len = pkt->in_len;
	return 0;
}

static int mchp_crypto_cipher_begin(const struct device *dev, struct cipher_ctx *ctx,
				    enum cipher_algo algo, enum cipher_mode mode,
				    enum cipher_op op_type)
{
	struct mchp_g2_session *sess;

	ARG_UNUSED(dev);

	if (ctx->flags & ~(MCHP_G2_HW_CAPS)) {
		LOG_ERR("Unsupported capability flags 0x%x", ctx->flags);
		return -EINVAL;
	}

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		LOG_ERR("Unsupported algo %d", algo);
		return -EINVAL;
	}

	if ((mode != CRYPTO_CIPHER_MODE_ECB) && (mode != CRYPTO_CIPHER_MODE_CBC) &&
	    (mode != CRYPTO_CIPHER_MODE_CTR) && (mode != CRYPTO_CIPHER_MODE_CCM)) {
		LOG_ERR("Unsupported mode %d", mode);
		return -EINVAL;
	}

	if ((ctx->keylen != 16U) && (ctx->keylen != 24U) && (ctx->keylen != 32U)) {
		LOG_ERR("Unsupported key length %u", ctx->keylen);
		return -EINVAL;
	}

	if (ctx->key.bit_stream == NULL) {
		LOG_ERR("No key provided");
		return -EINVAL;
	}

	k_mutex_lock(&g_session_lock, K_FOREVER);

	if (g_session.in_use) {
		k_mutex_unlock(&g_session_lock);
		LOG_ERR("Session already in use");
		return -EBUSY;
	}

	sess = &g_session;
	memset(sess, 0, sizeof(*sess));
	sess->in_use = true;

	k_mutex_unlock(&g_session_lock);

	memcpy(sess->key, ctx->key.bit_stream, ctx->keylen);
	sess->keylen = ctx->keylen;

	if (mode == CRYPTO_CIPHER_MODE_CCM) {
		sess->ccm_tag_len = ctx->mode_params.ccm_info.tag_len;
		sess->ccm_nonce_len = ctx->mode_params.ccm_info.nonce_len;

		if (sess->ccm_tag_len == 0U || sess->ccm_nonce_len == 0U) {
			LOG_ERR("CCM: tag_len and nonce_len must be non-zero");
			sess->in_use = false;
			return -EINVAL;
		}
	}

	ctx->drv_sessn_state = sess;
	ctx->ops.cipher_mode = mode;

	if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = crypto_mchp_ecb_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = crypto_mchp_cbc_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CTR:
			ctx->ops.ctr_crypt_hndlr = crypto_mchp_ctr_op;
			break;
		case CRYPTO_CIPHER_MODE_CCM:
			ctx->ops.ccm_crypt_hndlr = crypto_mchp_ccm_encrypt;
			break;
		default:
			break;
		}
	} else {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = crypto_mchp_ecb_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = crypto_mchp_cbc_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CTR:
			ctx->ops.ctr_crypt_hndlr = crypto_mchp_ctr_op;
			break;
		case CRYPTO_CIPHER_MODE_CCM:
			ctx->ops.ccm_crypt_hndlr = crypto_mchp_ccm_decrypt;
			break;
		default:
			break;
		}
	}

	return 0;
}

static int mchp_crypto_cipher_free(const struct device *dev, struct cipher_ctx *ctx)
{
	struct mchp_g2_session *sess = ctx->drv_sessn_state;

	ARG_UNUSED(dev);

	if (sess == NULL) {
		return 0;
	}

	k_mutex_lock(&g_session_lock, K_FOREVER);
	memset(sess->key, 0, sizeof(sess->key));
	sess->in_use = false;
	k_mutex_unlock(&g_session_lock);

	ctx->drv_sessn_state = NULL;
	return 0;
}

static int mchp_crypto_hw_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MCHP_G2_HW_CAPS;
}

static int mchp_crypto_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	k_mutex_init(&g_session_lock);
	memset(&g_session, 0, sizeof(g_session));

	LOG_DBG("Microchip G2 AES crypto driver initialised");
	return 0;
}

static DEVICE_API(crypto, mchp_crypto_api) = {
	.query_hw_caps = mchp_crypto_hw_query_caps,
	.cipher_begin_session = mchp_crypto_cipher_begin,
	.cipher_free_session = mchp_crypto_cipher_free,
	.cipher_async_callback_set = NULL,
};

DEVICE_DT_INST_DEFINE(0, mchp_crypto_driver_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_CRYPTO_MCHP_G2_INIT_PRIORITY, &mchp_crypto_api);
