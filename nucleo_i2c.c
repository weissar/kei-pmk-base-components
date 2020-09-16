#include "nucleo_i2c.h"

#ifndef __CC_ARM                // for compatibility between GCC and ARM (Keil)
#define __nop()  asm("nop")
#endif

// local function definitions hidden from external use (static attribute)
static bool I2C_Start(void);
static bool I2C_Stop(void);
static bool I2C_Addr(uint8_t adr);
static void I2C_Reset(void);

// Nucleo - PB8 SCL, PB9 SDA
bool InitI2C1(i2cSpeed spd)
{
  if ((spd != i2cSpeed100k) && (spd != i2cSpeed400k))
    return false;

  if (!(RCC->APB1ENR & RCC_APB1ENR_I2C1EN))
  {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  }

  I2C_Reset();
  
  Nucleo_SetPinGPIO(GPIOB, 8, ioPortAlternatePP);   // I2C CLK
  Nucleo_SetAFGPIO(GPIOB, 8, 4);                    // AF04 = I2C1_SCL

  Nucleo_SetPinGPIO(GPIOB, 9, ioPortAlternateOC);   // I2C data
  Nucleo_SetAFGPIO(GPIOB, 9, 4);                    // AF04 = I2C1_SDA

  // configuration
  I2C1->CR1 = I2C_CR1_PE;         // enable peripheral, remainnig bits = 0
  I2C1->CR2 = 0;                  // clear all cfg. bits
  I2C1->CR2 &= ~ I2C_CR2_FREQ;    // clear bits FREQ[5:0]
  
  int apbClk = GetBusClock(busClockAPB1);
  int apbClkMhz = apbClk / 1000000;          // clock in MHz

  I2C1->CR2 = apbClkMhz;
  I2C1->CR1 = 0;                  // disable preipheral

  // inspired by Cube generated code
  I2C1->TRISE = (spd <= 100000U)
      ? (apbClkMhz + 1U) : (((apbClkMhz * 300U) / 1000U) + 1U);

#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            (((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CCR_CCR) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? ((__PCLK__) / ((__SPEED__) * 3U)) : (((__PCLK__) / ((__SPEED__) * 25U)) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))
#define I2C_DUTYCYCLE_2                 ((uint32_t)0x00000000U)
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY

  I2C1->CCR = I2C_SPEED(apbClk, spd, I2C_DUTYCYCLE_2);
  
  // cannot write when PE=0 = ambigous ...  I2C1->CR1 |= I2C_CR1_ACK;       // enable ACK

  #define I2C_ADDRESSINGMODE_7BIT         ((uint32_t)0x00004000)
  I2C1->OAR1 = I2C_ADDRESSINGMODE_7BIT;   // by Cube
  
  #define I2C_DUALADDRESS_DISABLE         ((uint32_t)0x00000000)
  #define I2C_DUALADDRESS_DISABLED                I2C_DUALADDRESS_DISABLE
  I2C1->OAR2 = I2C_DUALADDRESS_DISABLED;  // by Cube
  // end Wizard settings
  
  I2C1->CR1 |= I2C_CR1_PE;        // enable peripheral
  return true;
}

/**
 * Perform short RESET pulse on SWRST bit in CR1 register
 */
static void I2C_Reset(void)
{
  uint16_t tout;

  I2C1->CR1 |= I2C_CR1_SWRST;   // reset peripheral signal
  for (tout = 1000; tout; tout--)  // short delay
    __nop();
  I2C1->CR1 = 0;
}


static const uint16_t _timeoutI2C = MAX_TIMEOUT;

/**
 * Read 16-bit status from SR1 and SR2 register
 * @return
 */
static __inline uint16_t I2C_sr(void) 
{
  uint16_t sr;

  sr  = I2C1->SR1;
  sr |= I2C1->SR2 << 16;
  return (sr);
}

/**
 * Perform I2C Start-Condition
 * @return
 */
static bool I2C_Start(void)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C_sr() & I2C_SR1_SB))      // wait for start condition generated
  {
    if (w)
      w--;
    else
      break;
  }
  
  return w;
}

/**
 * Perform I2C Stop-Condition
 * @return
 */
static bool I2C_Stop(void)
{
  uint16_t w = _timeoutI2C;

  I2C1->CR1 |= I2C_CR1_STOP;
  while (I2C_sr() & (I2C_SR2_MSL << 16))         // Wait until BUSY bit reset          
  {
    if (w)
      w--;
    else
      break;
  }
  
  return w;
}

/**
 * Send Address
 * @param adr
 * @return
 */
static bool I2C_Addr(uint8_t adr)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->DR = adr;
  while(!(I2C_sr() & I2C_SR1_ADDR))  // wait for sending completion
  {
    if (w)
      w--;
    else
      break;
  }
  
  return true;
}

/**
 * Send data
 * @param val
 * @return
 */
static bool I2C_Write(uint8_t val)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->DR = val;
  while (!(I2C_sr() & I2C_SR1_BTF))
  {
    if (w)
      w--;
    else
      break;
  }
  
  return true;
}

/**
 * Read data - ACK = 1
 * @param ack
 * @return
 */
static uint8_t I2C_Read(bool ack)
{
  uint16_t w = _timeoutI2C;

  // Enable/disable Master acknowledge
  if (ack) I2C1->CR1 |= I2C_CR1_ACK;
  else     I2C1->CR1 &= ~I2C_CR1_ACK;

  while (!(I2C_sr() & I2C_SR1_RXNE))
  {
    if (w)
      w--;
    else
      break;
  }
  
  return (I2C1->DR);
}

// public functions:

/**
 *
 * @param devAdr - 7-bit address, last bit R = 1, W = 0
 * @param regAdr
 * @param val
 * @return
 */
bool I2C1_WriteByte(uint8_t devAdr, uint8_t regAdr, uint8_t val)
{
  uint8_t result = 0;
  result |= I2C_Start() ? 0x01 : 0;
  result |= I2C_Addr(devAdr) ? 0x02 : 0;        // write
  result |= I2C_Write(regAdr) ? 0x04 : 0;       // address
  result |= I2C_Write(val) ? 0x08 : 0;          // data
  result |= I2C_Stop() ? 0x10 : 0;

  return result == 0x1f;                        // all partial functions OK ?
}

/**
 *
 * @param devAdr - 7-bit address, last bit R = 1, W = 0
 * @param regAdr
 * @param pResult
 * @return
 */
bool I2C1_ReadByte(uint8_t devAdr, uint8_t regAdr, uint8_t *pResult)
{
  uint8_t result = 0;
  result |= I2C_Start() ? 0x01 : 0;
  result |= I2C_Addr(devAdr & 0xFE) ? 0x02 : 0; // write
  result |= I2C_Write(regAdr) ? 0x04 : 0;       // address of first register
  result |= I2C_Start() ? 0x08 : 0;
  result |= I2C_Addr(devAdr | 1) ? 0x10 : 0;    // read
  *pResult = I2C_Read(0);                       // single read - generate nack
  result |= I2C_Stop() ? 0x20 : 0;

  return result == 0x3f;                        // all partial functions OK ?
}

/**
 *
 * @param devAdr - 7-bit address, last bit R = 1, W = 0
 * @param regAdr
 * @param pbuf
 * @param len
 * @return
 */
bool I2C1_ReadBytes(uint8_t devAdr, uint8_t regAdr, uint8_t *pbuf, uint32_t len)
{
  uint8_t result = 0;
  result |= I2C_Start() ? 0x01 : 0;
  result |= I2C_Addr(devAdr & 0xFE) ? 0x02 : 0; // write
  result |= I2C_Write(regAdr) ? 0x04 : 0;       // address of first register
  result |= I2C_Start() ? 0x08 : 0;
  result |= I2C_Addr(devAdr | 1) ? 0x10 : 0;    // read
  for(; len; len--, pbuf++)
    *pbuf = I2C_Read(len > 1);                  // for last read is false

  result |= I2C_Stop() ? 0x20 : 0;

  return result == 0x3f;                        // all partial functions OK ?
}
