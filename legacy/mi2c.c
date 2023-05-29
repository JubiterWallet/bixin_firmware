#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <stdint.h>
#include <string.h>

#include "mi2c.h"
#include "timer.h"
#include "compatible.h"

uint8_t g_ucMI2cRevBuf[MI2C_BUF_MAX_LEN];
uint8_t g_ucMI2cSendBuf[MI2C_BUF_MAX_LEN];
uint16_t g_usMI2cRevLen;
uint16_t g_lasterror;  // TODO:will change in encrypt+MAC
uint16_t i2c_retry_cnts = 0;

static uint8_t ucXorCheck(uint8_t ucInputXor, uint8_t *pucSrc, uint16_t usLen) {
  uint16_t i;
  uint8_t ucXor;

  ucXor = ucInputXor;
  for (i = 0; i < usLen; i++) {
    ucXor ^= pucSrc[i];
  }
  return ucXor;
}

static bool bMI2CDRV_ReadBytes(uint32_t i2c, uint8_t *res,
                               uint16_t *pusOutLen) {
  uint8_t ucLenBuf[2], ucSW[2], ucXor = 0, ucXor1 = 0;
  uint16_t i, usRevLen, usRealLen = 0, usTimeout = 0;

  i2c_retry_cnts = 0;
  while (1) {
    if (i2c_retry_cnts > MI2C_RETRYCNTS) {
      return false;
    }

    // send start
    i2c_send_start(i2c);
    i2c_enable_ack(i2c);
    usTimeout = 0;
    while (!(I2C_SR1(i2c) & I2C_SR1_SB)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {  // setup timeout is 5ms once
        break;
      }
    }
    // send read address
    i2c_send_7bit_address(i2c, MI2C_ADDR, MI2C_READ);
    usTimeout = 0;
    // Waiting for address is transferred.
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {  // setup timeout is 5ms once
        break;
      }
    }
    if (usTimeout > MI2C_TIMEOUT) {
      usTimeout = 0;
      i2c_retry_cnts++;
      i2c_send_stop(i2c);  // it will release i2c bus
      continue;
    }
    /* Clearing ADDR condition sequence. */
    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);
    break;
  }
  // rev len
  for (i = 0; i < 2; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    ucLenBuf[i] = i2c_get_data(i2c);
  }
  // cal len xor
  ucXor = ucXorCheck(ucXor, ucLenBuf, sizeof(ucLenBuf));

  // len-SW1SW2
  usRevLen = (ucLenBuf[0] << 8) + (ucLenBuf[1] & 0xFF) - 2;

  if (usRevLen > 0 && (res == NULL)) {
    i2c_send_stop(i2c);
    return false;
  }

  // rev data
  for (i = 0; i < usRevLen; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    if (i < *pusOutLen) {
      res[i] = i2c_get_data(i2c);
      // cal data xor
      ucXor = ucXorCheck(ucXor, res + i, 1);
      usRealLen++;
    } else {
      ucLenBuf[0] = i2c_get_data(i2c);
      ucXor = ucXorCheck(ucXor, ucLenBuf, 1);
    }
  }

  // sw1 sw2 len
  for (i = 0; i < 2; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    ucSW[i] = i2c_get_data(i2c);
    usRealLen++;
  }
  // cal sw1sw2 xor
  ucXor = ucXorCheck(ucXor, ucSW, sizeof(ucSW));

  // xor len
  i2c_disable_ack(i2c);
  for (i = 0; i < MI2C_XOR_LEN; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    ucXor1 = i2c_get_data(i2c);
    usRealLen++;
  }

  i2c_send_stop(i2c);
  if (0x00 == usRealLen) {
    return false;
  }

  if (ucXor != ucXor1) {
    return false;
  }
  usRealLen -= MI2C_XOR_LEN;
  g_lasterror = (ucSW[0] << 8) + ucSW[1];
  if ((0x90 != ucSW[0]) || (0x00 != ucSW[1])) {
    if (ucSW[0] == 0x6c || ucSW[0] == 0x90) {  // for se generate seed not first
                                               // generate will return 0x90xx
      res[0] = ucSW[1];
      *pusOutLen = 1;
    } else {
      *pusOutLen = usRealLen - 2;
    }
    return false;
  }
  *pusOutLen = usRealLen - 2;
  return true;
}

static bool bMI2CDRV_WriteBytes(uint32_t i2c, uint8_t *data,
                                uint16_t ucSendLen) {
  uint8_t ucLenBuf[2], ucXor = 0;
  uint16_t i, usTimeout = 0;

  i2c_retry_cnts = 0;
  while (1) {
    if (i2c_retry_cnts > MI2C_RETRYCNTS) {
      return false;
    }

    i2c_send_start(i2c);
    usTimeout = 0;
    while (!(I2C_SR1(i2c) & I2C_SR1_SB)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }

    i2c_send_7bit_address(i2c, MI2C_ADDR, MI2C_WRITE);
    usTimeout = 0;
    // Waiting for address is transferred.
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }
    if (usTimeout > MI2C_TIMEOUT) {
      i2c_retry_cnts++;
      usTimeout = 0;
      continue;
    }
    /* Clearing ADDR condition sequence. */
    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);
    break;
  }
  // send L + V + xor
  ucLenBuf[0] = ((ucSendLen >> 8) & 0xFF);
  ucLenBuf[1] = ucSendLen & 0xFF;
  // len xor
  ucXor = ucXorCheck(ucXor, ucLenBuf, sizeof(ucLenBuf));
  // send len
  for (i = 0; i < 2; i++) {
    i2c_send_data(i2c, ucLenBuf[i]);
    usTimeout = 0;
    while (!(I2C_SR1(i2c) & (I2C_SR1_TxE))) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        return false;
      }
    }
  }
  // cal xor
  ucXor = ucXorCheck(ucXor, data, ucSendLen);
  // send data
  for (i = 0; i < ucSendLen; i++) {
    i2c_send_data(i2c, data[i]);
    usTimeout = 0;
    while (!(I2C_SR1(i2c) & (I2C_SR1_TxE))) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        return false;
      }
    }
  }
  // send Xor
  i2c_send_data(i2c, ucXor);
  usTimeout = 0;
  while (!(I2C_SR1(i2c) & (I2C_SR1_TxE))) {
    usTimeout++;
    if (usTimeout > MI2C_TIMEOUT) {
      return false;
    }
  }

  i2c_send_stop(i2c);
  return true;
}

void vMI2CDRV_Init(void) {
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_set_output_options(GPIO_MI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_set_af(GPIO_MI2C_PORT, GPIO_AF4, GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_mode_setup(GPIO_MI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  i2c_reset(MI2CX);
  delay_ms(100);
  i2c_peripheral_disable(MI2CX);

  // 100k
  i2c_set_speed(MI2CX, i2c_speed_sm_100k, 30);
  i2c_peripheral_enable(MI2CX);
  delay_ms(100);
}

/*
 *master i2c rev
 */
bool bMI2CDRV_ReceiveData(uint8_t *pucStr, uint16_t *pusRevLen) {
  if (false == bMI2CDRV_ReadBytes(MI2CX, pucStr, pusRevLen)) {
    return false;
  }

  return true;
}
/*
 *master i2c send
 */
bool bMI2CDRV_SendData(uint8_t *pucStr, uint16_t usStrLen) {
  if (usStrLen > (MI2C_BUF_MAX_LEN - 3)) {
    usStrLen = MI2C_BUF_MAX_LEN - 3;
  }

  return bMI2CDRV_WriteBytes(MI2CX, pucStr, usStrLen);
}

uint16_t get_lasterror(void) { return g_lasterror; }

/// re-implementation
static inline uint8_t xor_cal(uint8_t ref, const uint8_t *inbuf,
                              uint16_t inlen) {
  while (inlen-- > 0) {
    ref ^= *inbuf++;
  }
  return ref;
}

static inline bool bmi2c_bus_prepare(uint32_t i2c, uint8_t mode,
                                     uint32_t timeout) {
  uint32_t timeout_cnt;
  /* static uint16_t i2c_retry_cnts; */

  i2c_retry_cnts = 0;
  do {
    if (i2c_retry_cnts > MI2C_RETRYCNTS) {
      return false;
    }
    // send start
    i2c_send_start(i2c);
    i2c_enable_ack(i2c);
    timeout_cnt = 0;
    while (!(I2C_SR1(i2c) & I2C_SR1_SB)) {
      timeout_cnt++;
      if (timeout_cnt > timeout) {  // setup timeout is 5ms once
        break;
      }
    }
    // send read address
    i2c_send_7bit_address(i2c, MI2C_ADDR, mode);
    timeout_cnt = 0;
    // Waiting for address is transferred.
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
      timeout_cnt++;
      if (timeout_cnt > timeout) {  // setup timeout is 5ms once
        break;
      }
    }
    if (timeout_cnt > timeout) {
      timeout_cnt = 0;
      i2c_retry_cnts++;
      i2c_send_stop(i2c);  // it will release i2c bus
      continue;
    }
    //  i2c bus is ready
    break;
  } while (1);
  /* Clearing ADDR condition sequence. */
  (void)I2C_SR1(i2c);
  (void)I2C_SR2(i2c);
  return true;
}

static bool bmi2c_readbytes(uint32_t i2c, uint8_t *recv, uint16_t *recv_len,
                            uint32_t timeout) {
  uint8_t len_buf[2], xor = 0, xor_ref = 0;
  uint16_t i, read_len;

  // i2c bus prepare
  if (!bmi2c_bus_prepare(i2c, MI2C_READ, timeout)) return false;
  // read len
  for (i = 0; i < 2; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    len_buf[i] = i2c_get_data(i2c);
  }
  // cal len xor
  xor = xor_cal(xor, len_buf, sizeof(len_buf));
  // len
  read_len = (len_buf[0] << 8) + (len_buf[1] & 0xFF);
  if (read_len > *recv_len) {
  }
  if (!recv || read_len > *recv_len) {
    i2c_send_stop(i2c);
    return false;
  }
  *recv_len = read_len;

  // read data sw and xor
  while (read_len-- > 0) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    *recv = i2c_get_data(i2c);
    xor = xor_cal(xor, recv, 1);
    recv++;
  }

  // xor len
  i2c_disable_ack(i2c);
  for (i = 0; i < MI2C_XOR_LEN; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    xor_ref = i2c_get_data(i2c);
  }

  i2c_send_stop(i2c);

  if (xor != xor_ref) {
    return false;
  }
  return true;
}

static bool bmi2c_writebytes(uint32_t i2c, const uint8_t *data,
                             uint16_t datalen, uint32_t timeout) {
  uint8_t len_buf[2], xor = 0;
  uint16_t i;

  // i2c bus prepare
  if (!bmi2c_bus_prepare(i2c, MI2C_WRITE, timeout)) return false;

  // send L + V + xor
  len_buf[0] = ((datalen >> 8) & 0xFF);
  len_buf[1] = datalen & 0xFF;
  // len xor
  xor = xor_cal(xor, len_buf, sizeof(len_buf));
  // send len
  for (i = 0; i < 2; i++) {
    i2c_send_data(i2c, len_buf[i]);
    while (!(I2C_SR1(i2c) & (I2C_SR1_TxE)))
      ;
  }
  // cal xor
  xor = xor_cal(xor, data, datalen);
  // send data
  while (datalen-- > 0) {
    i2c_send_data(i2c, *data++);
    while (!(I2C_SR1(i2c) & (I2C_SR1_TxE)))
      ;
  }
  // send Xor
  i2c_send_data(i2c, xor);
  while (!(I2C_SR1(i2c) & (I2C_SR1_TxE)))
    ;
  i2c_send_stop(i2c);
  return true;
}

void mi2c_init(void) {
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_set_output_options(GPIO_MI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_set_af(GPIO_MI2C_PORT, GPIO_AF4, GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_mode_setup(GPIO_MI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  i2c_reset(MI2CX);
  delay_ms(100);
  i2c_peripheral_disable(MI2CX);

  // 100k
  i2c_set_speed(MI2CX, i2c_speed_sm_100k, 30);
  i2c_peripheral_enable(MI2CX);
  delay_ms(100);
}

bool mi2c_send(const uint8_t *data, uint16_t data_len) {
  if (data_len > MI2C_SEND_MAX_LEN) return false;
  return bmi2c_writebytes(MI2CX, data, data_len, MI2C_TIMEOUT);
}

bool mi2c_receive(uint8_t *resp, uint16_t *resp_len) {
  return bmi2c_readbytes(MI2CX, resp, resp_len, MI2C_TIMEOUT);
}
