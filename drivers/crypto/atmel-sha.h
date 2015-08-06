#ifndef __ATMEL_SHA_H__
#define __ATMEL_SHA_H__

int atmel_hmac_write_key(const u8 *key, size_t keylen,
			unsigned long hmac_type);

int atmel_hmac_write_esp(const u8 *esp_header, size_t head_len,
			const u8 *iv, size_t iv_len,
			size_t payload_len, u32 checkcnt,
			unsigned long hmac_type);
int atmel_hmac_check_icv(u32 *hash, u32 cnt);
int atmel_hmac_read_icv(u32 *hash, u32 cnt);

#endif /* __ATMEL_SHA_H__ */
