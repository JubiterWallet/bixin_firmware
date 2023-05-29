#ifndef __SE_CHIP_H__
#define __SE_CHIP_H__

#include <stdbool.h>
#include <stdint.h>

#define IN
#define OUT

#define SESSION_KEYLEN (16)
#define DEFAULT_SECKEYADDR (0x0800F000UL)

#define PUBLIC_REGION_SIZE (0x600)   // 1.5KB
#define PRIVATE_REGION_SIZE (0x200)  // 0.5KB
#define SE_PRIVATE_REGION_BASE PUBLIC_REGION_SIZE

#define SE_WRFLG_RESET (0x00)                     // se reset on device
#define SE_WRFLG_SETPIN (0x00)                    // se set pin
#define SE_WRFLG_CHGPIN (0x01)                    // se change pin
#define SE_WRFLG_GENSEED (0x00)                   // se generate seed
#define SE_WRFLG_GENMINISECRET (0x01)             // se generate minisecret
#define SE_WRFLG_MNEMONIC SE_WRFLG_GENMINISECRET  // se set mnemonic
#define SE_WRFLG_ENTROPY (0x02)                   // se set entropy
#define SE_VERIFYPIN_FIRST (0xff)                 // for first verify se pin
#define SE_VERIFYPIN_OTHER (0x01)                 // for others
#define SE_GENSEDMNISEC_FIRST SE_VERIFYPIN_FIRST  // for first generate
#define SE_GENSEDMNISEC_OTHER SE_VERIFYPIN_OTHER  // for others

#define SE_GENERATE_SEED_MAX_STEPS 100  // [1, 100] // total 100 steps

typedef enum {
  TYPE_SEED = 0x00,               /* BIP32 seed */
  TYPE_MINI_SECRET = 0x01,        /* polkadot mini secret */
  TYPE_ICARUS_MAIN_SECRET = 0x02, /* cardano icarus main secret */
} se_generate_type_t;

typedef struct {
  bool se_seed_status;
  bool se_minisecret_status;
  bool se_icarus_status;
} se_session_cached_status;

typedef enum {
  PROCESS_BEGIN = 0xFF,
  PROCESS_GENERATING = 0x01,
} se_generate_process_t;

typedef enum {
  STATE_FAILD,
  STATE_GENERATING,
  STATE_COMPLETE,
} se_generate_state_t;

typedef struct {
  se_generate_type_t type;
  se_generate_process_t processing;
} se_generate_session_t;

bool se_sync_session_key(void);
bool se_device_init(uint8_t mode, const char *passphrase);
bool se_ecdsa_get_pubkey(uint32_t *address, uint8_t count, uint8_t *pubkey);
bool se_set_value(uint16_t key, const void *val_dest, uint16_t len);
bool se_get_value(uint16_t key, void *val_dest, uint16_t max_len,
                  uint16_t *len);
bool se_delete_key(uint16_t key);
void se_reset_storage(void);
bool se_get_sn(char **serial, uint16_t len);
char *se_get_version(void);
bool se_verify(void *message, uint16_t message_len, uint16_t max_len,
               void *cert_val, uint16_t *cert_len, void *signature_val,
               uint16_t *signature_len);
bool se_isInitialized(void);
bool se_hasPin(void);
bool se_setPin(const char *pin);
bool se_verifyPin(const char *pin, uint8_t mode);
bool se_changePin(const char *oldpin, const char *newpin);
uint32_t se_pinFailedCounter(void);
bool se_getRetryTimes(uint8_t *pcnts);
bool se_clearSecsta(void);
bool se_setPinValidtime(uint8_t data);
bool se_getPinValidtime(uint8_t *data_buf);
bool se_applyPinValidtime(void);
bool se_getSecsta(void);
bool se_isFactoryMode(void);
bool se_isLifecyComSta(void);
bool se_set_u2f_counter(uint32_t u2fcounter);
bool se_get_u2f_counter(uint32_t *u2fcounter);
bool se_get_entropy(uint8_t entropy[32]);
bool se_set_entropy(const void *entropy, uint16_t len);
bool se_set_mnemonic(const void *mnemonic, uint16_t len);
bool se_sessionStart(OUT uint8_t *session_id_bytes);
bool se_sessionOpen(IN uint8_t *session_id_bytes);

// generateing secret when create/recover wallet
se_generate_state_t se_beginGenerate(se_generate_type_t type,
                                     se_generate_session_t *session);
se_generate_state_t se_generating(se_generate_session_t *session);

// generateing secret when use passprase session
se_generate_state_t se_sessionBeginGenerate(const uint8_t *passphase,
                                            uint16_t len,
                                            se_generate_type_t type,
                                            se_generate_session_t *session);
se_generate_state_t se_sessionGenerating(se_generate_session_t *session);

bool se_getSessionCachedState(se_session_cached_status *status);
bool se_sessionClose(void);
bool se_sessionClear(void);
bool se_set_public_region(uint16_t offset, const void *val_dest, uint16_t len);
bool se_get_public_region(uint16_t offset, void *val_dest, uint16_t len);
bool se_set_private_region(uint16_t offset, const void *val_dest, uint16_t len);
bool se_get_private_region(uint16_t offset, void *val_dest, uint16_t len);
bool se_aes_128_encrypt(uint8_t mode, uint8_t *key, uint8_t *iv, uint8_t *send,
                        uint16_t send_len, uint8_t *recv, uint16_t *recv_len);
bool se_aes_128_decrypt(uint8_t mode, uint8_t *key, uint8_t *iv, uint8_t *send,
                        uint16_t send_len, uint8_t *recv, uint16_t *recv_len);
bool se_containsMnemonic(const char *mnemonic);
bool se_hasWipeCode(void);
bool se_changeWipeCode(const char *wipe_code);
uint16_t se_lasterror(void);

bool se_random_buffer(uint8_t *random, uint16_t count);

#endif
