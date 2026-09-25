/**
  ******************************************************************************
  * @file    vnh7100.h
  * @brief   VNH7100BAS H-kopru surucusu - dusuk seviye surucu
  *          Yon/SEL0, PWM, akim olcumu (ADC+DMA), ariza latch sifirlama.
  *          Uygulama mantigi (kiskac vb.) burada YOK.
  ******************************************************************************
  */
#ifndef VNH7100_H
#define VNH7100_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  VNH7100_DIR_FORWARD = 0,   /* INA=1 INB=0 SEL0=1 -> CS = HSA akimi */
  VNH7100_DIR_REVERSE        /* INA=0 INB=1 SEL0=0 -> CS = HSB akimi */
} VNH7100_Dir_t;

typedef struct
{
  ADC_HandleTypeDef *hadc;          /* CS pinine bagli ADC (tek kanal, surekli, DMA circular) */
  TIM_HandleTypeDef *htim;          /* PWM timer'i */
  uint32_t           tim_channel;
  GPIO_TypeDef      *port;          /* INA, INB, SEL0 AYNI portta olmali (atomik yazma) */
  uint16_t           pin_ina;
  uint16_t           pin_inb;
  uint16_t           pin_sel0;
  uint32_t           rsense_ohm;    /* R2 */
  uint32_t           k_factor;      /* datasheet K */
  uint32_t           vref_mv;
} VNH7100_Config_t;

HAL_StatusTypeDef VNH7100_Init(const VNH7100_Config_t *cfg);

void     VNH7100_SetDirection(VNH7100_Dir_t dir);
void     VNH7100_SetDuty(uint8_t percent);
uint8_t  VNH7100_GetDuty(void);
void     VNH7100_Off(void);            /* PWM=0, INA=INB=SEL0=0 (standby) */
void     VNH7100_ResetLatch(void);     /* ariza latch'ini acar, sonunda Off (~3-6 ms bloklar) */

uint16_t VNH7100_ReadRaw(void);                /* son ~1.5 ms ADC ortalamasi */
uint32_t VNH7100_RawToMilliAmp(uint16_t raw);
bool     VNH7100_IsSaturated(uint16_t raw);

#endif /* VNH7100_H */
