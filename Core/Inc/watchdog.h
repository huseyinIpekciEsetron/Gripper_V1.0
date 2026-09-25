/**
  ******************************************************************************
  * @file    watchdog.h
  * @brief   Bagimsiz watchdog (IWDG). Yazilim takilirsa MCU resetlenir.
  *          CubeMX'te IWDG acmaya gerek yok, dogrudan register kullanir.
  *          Bir kez baslatilinca durdurulamaz.
  ******************************************************************************
  */
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

void Watchdog_Init(uint32_t timeout_ms);
void Watchdog_Kick(void);

#endif /* WATCHDOG_H */
