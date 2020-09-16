#ifndef _NUCLEO_I2C_H
#define _NUCLEO_I2C_H

/**
 * @brief   Funtions for I2C communication
 * @file    nucleo_o2c.h
 * @author  PW
 *
 * [OPTIONAL] Only for I2C1 on PB8/PB9 pins - Nucleo Board
 */

#include "stm_core.h"

#define MAX_TIMEOUT   10000

typedef enum
{
  i2cSpeed100k = 100000,
  i2cSpeed400k = 400000
} i2cSpeed;
bool InitI2C1(i2cSpeed spd);

bool I2C1_WriteByte(uint8_t devAdr, uint8_t regAdr, uint8_t val);
// replaced ... uint8_t I2C1_ReadByte(uint8_t devAdr, uint8_t regAdr);
bool I2C1_ReadByte(uint8_t devAdr, uint8_t regAdr, uint8_t *pResult);
bool I2C1_ReadBytes(uint8_t devAdr, uint8_t regAdr, uint8_t *pbuf, uint32_t len);

#endif // _NUCLEO_I2C_H
