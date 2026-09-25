/**
  ******************************************************************************
  * @file    watchdog.c
  * @brief   STM32F1 IWDG, LSI ~40 kHz (tolerans 30..60 kHz), prescaler /32
  ******************************************************************************
  */
#include "watchdog.h"
#include "main.h"

#define IWDG_KEY_START    0xCCCCU
#define IWDG_KEY_ACCESS   0x5555U
#define IWDG_KEY_RELOAD   0xAAAAU
#define IWDG_PR_DIV32     3U
#define LSI_NOMINAL_HZ    40000U

void Watchdog_Init(uint32_t timeout_ms)
{
  /* Nominal: 40 kHz / 32 = 1250 Hz -> 1 tick = 0.8 ms */
  uint32_t reload = (timeout_ms * (LSI_NOMINAL_HZ / 1000U)) / 32U;
  if (reload < 1U)
  {
    reload = 1U;
  }
  if (reload > 0x0FFFU)
  {
    reload = 0x0FFFU;
  }

  /* Debugger'da breakpoint'te dururken reset atmasin */
  DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

  IWDG->KR  = IWDG_KEY_START;
  IWDG->KR  = IWDG_KEY_ACCESS;
  IWDG->PR  = IWDG_PR_DIV32;
  IWDG->RLR = reload;

  /* Register guncellemesinin bitmesini bekle (sinirli) */
  for (uint32_t i = 0U; (IWDG->SR != 0U) && (i < 1000000U); i++)
  {
  }
  IWDG->KR = IWDG_KEY_RELOAD;
}

void Watchdog_Kick(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}
