#include "func_watchdog.h"

#ifdef ENABLE_WATCH_DAWG

#include "stm32f0xx_iwdg.h"

const uint32_t LsiFreq = 40000;

void watchdogSetup(void)
{
  /* Check if the system has resumed from IWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
  {
    /* Clear reset flags */
    RCC_ClearFlag();
  }

  /* IWDG timeout equal to 250 ms. */
  
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_64);

  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
  IWDG_SetReload(4000);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}

int watchdogPet(void)
{
    IWDG_ReloadCounter();
}

#else

void watchdogSetup(void){}
int watchdogPet(void){}

#endif // ENABLE_WATCH_DAWG