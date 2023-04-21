/*
 * This file is part of the Trezor project, https://trezor.io/
 *
 * Copyright (C) 2014 Pavol Rusnak <stick@satoshilabs.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "bip32.h"
#include "messages-bitcoin.pb.h"
#include "messages-common.pb.h"
#include "messages-management.pb.h"
#include "messages.pb.h"

typedef enum {
  COIN_SWITCH_ETH_EIP712 = 0x01,
  COIN_SWITCH_SOLANA = 0x02
} CoinSwitch;

#define MIN_PIN_LEN 1
#define MAX_PIN_LEN 9
#define DEFAULT_PIN_LEN 4
#define MAX_LABEL_LEN 32
#define MAX_LANGUAGE_LEN 16
#define MAX_MNEMONIC_LEN 240
#define HOMESCREEN_SIZE 1024
#define UUID_SIZE 12
#define SE_SESSION_KEY 16
#define SE_SESSION_SEED 0x5a
#define SE_SESSION_MINISECRET 0xfe
#define BUILD_ID_MAX_LEN 64

#if DEBUG_LINK
#define MIN_AUTOLOCK_DELAY_MS (10 * 1000U)  // 10 seconds
#else
#define MIN_AUTOLOCK_DELAY_MS (60 * 1000U)  // 1 minute
#endif
#define MAX_AUTOLOCK_DELAY_MS 0x20000000U  // ~6 days

void config_init(void);
void session_clear(bool lock);
void session_endCurrentSession(void);
void config_lockDevice(void);

void config_loadDevice(const LoadDevice *msg);
void config_loadDevice_ex(const BixinLoadDevice *msg);

const uint8_t *config_getSeed(void);

bool config_setCoinJoinAuthorization(const AuthorizeCoinJoin *authorization);
MessageType config_getAuthorizationType(void);
const AuthorizeCoinJoin *config_getCoinJoinAuthorization(void);

bool config_getU2FRoot(HDNode *node);
// TODO
bool config_getRootNode(HDNode *node, const char *curve);

bool config_getLabel(char *dest, uint16_t dest_size);
void config_setLabel(const char *label);

bool config_getLanguage(char *dest, uint16_t dest_size);
void config_setLanguage(const char *lang);

void config_setPassphraseProtection(bool passphrase_protection);
bool config_getPassphraseProtection(bool *passphrase_protection);

bool config_getHomescreen(uint8_t *dest, uint16_t dest_size);
void config_setHomescreen(const uint8_t *data, uint32_t size);

uint8_t *session_startSession(const uint8_t *received_session_id);

bool config_genSessionSeed(void);
bool config_setMnemonic(const char *mnemonic, bool import);
// bool config_containsMnemonic(const char *mnemonic);
bool config_hasMnemonic(void);

bool config_getMnemonic(char *dest, uint16_t dest_size);
// bool config_getMnemonicBytes(uint8_t *dest, uint16_t dest_size,
//  uint16_t *real_size);

bool config_genSeed(void);
bool config_setSeedsBytes(const uint8_t *seeds, uint8_t len);

// bool config_SeedsEncExportBytes(BixinOutMessageSE_outmessage_t *get_msg);
// bool config_stBackUpEntoryToSe(uint8_t *seed, uint8_t seed_len);
// bool config_stRestoreEntoryFromSe(uint8_t *seed, uint8_t *seed_len);

bool config_setPin(const char *pin);
bool config_firstVerifyPin(const char *pin);
bool config_verifyPin(const char *pin);
bool config_hasPin(void);
bool config_changePin(const char *old_pin, const char *new_pin);
#define config_unlock(pin) config_verifyPin(pin)

bool session_isUnlocked(void);
bool session_isProtectUnlocked(void);
bool config_hasWipeCode(void);
bool config_changeWipeCode(const char *pin, const char *wipe_code);

uint32_t config_nextU2FCounter(void);
void config_setU2FCounter(uint32_t u2fcounter);

bool config_isInitialized(void);

bool config_getImported(bool *imported);
void config_setImported(bool imported);

bool config_getMnemonicsImported(void);

bool config_getNeedsBackup(bool *needs_backup);
void config_setNeedsBackup(bool needs_backup);

bool config_getUnfinishedBackup(bool *unfinished_backup);
void config_setUnfinishedBackup(bool unfinished_backup);

bool config_getNoBackup(bool *no_backup);
// void config_setNoBackup(void);

void config_applyFlags(uint32_t flags);
bool config_getFlags(uint32_t *flags);

uint32_t config_getAutoLockDelayMs(void);
void config_setAutoLockDelayMs(uint32_t auto_lock_delay_ms);

uint32_t config_getSleepDelayMs(void);
void config_setSleepDelayMs(uint32_t auto_sleep_ms);

SafetyCheckLevel config_getSafetyCheckLevel(void);
void config_setSafetyCheckLevel(SafetyCheckLevel safety_check_level);

void config_wipe(void);
void config_setFastPayPinFlag(bool flag);
bool config_getFastPayPinFlag(void);
void config_setFastPayConfirmFlag(bool flag);
bool config_getFastPayConfirmFlag(void);
void config_setFastPayMoneyLimt(uint64_t MoneyLimt);
uint64_t config_getFastPayMoneyLimt(void);
void config_setFastPayTimes(uint32_t times);
uint32_t config_getFastPayTimes(void);

void config_setBleTrans(bool mode);

void config_setWhetherUseSE(bool flag);
bool config_getWhetherUseSE(void);
ExportType config_setSeedsExportFlag(ExportType flag);
bool config_getMessageSE(BixinMessageSE_inputmessage_t *input_msg,
                         BixinOutMessageSE_outmessage_t *get_msg);
void config_setIsBixinAPP(void);

void config_setSeSessionKey(uint8_t *data, uint32_t size);
bool config_getSeSessionKey(uint8_t *dest, uint16_t dest_size);

bool config_setSeedPin(const char *pin);
uint32_t config_getPinFails(void);

bool config_getCoinSwitch(CoinSwitch loc);
void config_setCoinSwitch(CoinSwitch loc, bool flag);

bool config_hasTrezorCompMode(void);
void config_setTrezorCompMode(bool trezor_comp_mode);
bool config_getTrezorCompMode(bool *trezor_comp_mode);

extern char config_uuid_str[2 * UUID_SIZE + 1];

#endif
