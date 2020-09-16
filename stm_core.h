#ifndef STM_CORE_H_
#define STM_CORE_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef enum
{
	ioPortOutputPP,       // output type Push-Pull
	ioPortOutputOC,       // output type Open Collector
	ioPortAnalog,         // analog input - for A/D
	ioPortInputFloat,     // input without pull-up/down
	ioPortInputPU,        // input with pull-up
	ioPortInputPD,        // input with pull-down
	ioPortAlternatePP,    // alternate output - push/pull
	ioPortAlternateOC     // alternate output - open drain
} eIoPortModes;

bool Nucleo_SetPinGPIO(GPIO_TypeDef *gpio, uint32_t bitnum, eIoPortModes mode);
bool Nucleo_SetAFGPIO(GPIO_TypeDef *gpio, uint32_t bitnum, uint32_t afValue);

void GPIOToggle(GPIO_TypeDef *gpio, uint32_t bitnum);
bool GPIORead(GPIO_TypeDef *gpio, uint32_t bitnum);
void GPIOWrite(GPIO_TypeDef *gpio, uint32_t bitnum, bool state);

#define BOARD_BTN_BLUE GPIOC,13         // Nucleo LED - !! same as SPI SCK !!
#define BOARD_LED GPIOA,5               // Nucleo BLue Button

typedef enum { clockSourceHSI, clockSourceHSE } eClockSources;
bool SetClock100MHz(eClockSources clkSrc);
bool SetClockHSI(void);

typedef enum { busClockAHB,
  busClockAPB1, busClockAPB2,
  timersClockAPB1, timersClockAPB2 } eBusClocks;
uint32_t GetTimerClock(int timerNum);
uint32_t GetBusClock(eBusClocks clk);

#endif /* STM_CORE_H_ */
