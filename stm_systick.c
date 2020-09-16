#include <stm_systick.h>

volatile uint32_t _ticks = 0;
volatile uint32_t _interval = 1;

void SysTick_Handler(void)
{
  _ticks += _interval;
}

/**
 *
 * @param ms
 */
void WaitMs(uint32_t ms)
{
  ms += _ticks;
  while(_ticks < ms)
    ;
}

void InitSystickDefault(void)
{
  InitSystick(1);
}

void InitSystick(uint32_t intervalMs)
{
  SystemCoreClockUpdate();               // for safe
  
  _interval = intervalMs;                // save for waiting calculation 
  SysTick_Config(SystemCoreClock / 1000 / _interval);  // Generate interrupt every X ms
}
