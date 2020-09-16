#include "nucleo_usart.h"

// addon for CubeIDE with new structure of syscalls.c
int __io_putchar(int ch) { return Usart2Send(ch); }
int __io_getchar(void) { return Usart2Recv(); }
// end of addon

int Usart2Send(char c)
{
  while (!(USART2->SR & USART_SR_TXE))            // wait while not empty TDR
    ;
  USART2->DR = c;                                 // write data to TDR
  return c;
}

void Usart2String(char *txt)
{
  while (*txt)
  {
    Usart2Send(*txt);
    txt++;
  }
}

int Usart2Recv(void)
{
  while (!(USART2->SR & USART_SR_RXNE))           // wait for something received
    ;

  return USART2->DR;                              // read and return as value
}

bool IsUsart2Recv(void)                           // check if something in recv. register
{
  return (USART2->SR & USART_SR_RXNE) != 0;       // condition makes true/false result
}

void Usart2Init(int baudSpeed)
{
  Nucleo_SetPinGPIO(GPIOA, 2, ioPortAlternatePP);
  Nucleo_SetAFGPIO(GPIOA, 2, 7);                  // AF7 is USART2
  Nucleo_SetPinGPIO(GPIOA, 3, ioPortAlternatePP);
  Nucleo_SetAFGPIO(GPIOA, 3, 7);                  // AF7 is USART2

  if (!(RCC->APB1ENR & RCC_APB1ENR_USART2EN))     // USART2 not enabled yet ?
  {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
  }

  USART2->CR1 = USART_CR1_RE | USART_CR1_TE;      // enable only Recv and Tansmit
  USART2->CR2 = 0;                                // nothing special
  USART2->CR3 = 0;                                // nothing special

  // USART2->BRR = 0x1A1; // speed 38400 by 16MHz - pre-calculated
  {
    uint sampling = (USART2->CR1 & USART_CR1_OVER8) ? 8 : 16;
    uint32_t apb1, mant, tmp, frac;
    apb1 = GetBusClock(busClockAPB1);

    mant = apb1 * 16 / (sampling * baudSpeed);    // part of 16th
    tmp = mant / 16;

    frac = mant - (tmp * 16);                     // remain after 16 division
    USART2->BRR = (tmp << 4) | (frac & 0x0f);
  }

  USART2->CR1 |= USART_CR1_UE;                    // last step - enable USART

  setvbuf(stdout, NULL, _IONBF, 0);               // disable buffering of output and input
  setvbuf(stdin, NULL, _IONBF, 0);
}
