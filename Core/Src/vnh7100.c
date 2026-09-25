/**
  ******************************************************************************
  * @file    vnh7100.c
  * @brief   VNH7100BAS dusuk seviye surucu
  ******************************************************************************
  */
#include "vnh7100.h"
#include <stddef.h>

#define ADC_BUF_SIZE     64U     /* ~1.5 ms'lik pencere, DMA kesmesi ~0.75 ms'de bir */
#define ADC_FULL_SCALE   4095U
#define ADC_SAT_LEVEL    4000U   /* ~3.22 V */

static VNH7100_Config_t  s_cfg;
static volatile uint16_t s_adc_buf[ADC_BUF_SIZE];
static uint8_t           s_duty = 0U;

HAL_StatusTypeDef VNH7100_Init(const VNH7100_Config_t *cfg)
{
  if ((cfg == NULL) || (cfg->hadc == NULL) || (cfg->htim == NULL) ||
      (cfg->port == NULL) || (cfg->rsense_ohm == 0U))
  {
    return HAL_ERROR;
  }
  s_cfg = *cfg;

  /* Guvenli baslangic: her sey kapali */
  VNH7100_Off();

  /* F1'de ADC kalibrasyonu sart */
  if (HAL_ADCEx_Calibration_Start(s_cfg.hadc) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_ADC_Start_DMA(s_cfg.hadc, (uint32_t *)s_adc_buf, ADC_BUF_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* PWM duty=0 ile baslar; INx ayarlanmadan motor donmez */
  if (HAL_TIM_PWM_Start(s_cfg.htim, s_cfg.tim_channel) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* Datasheet Table 11: SEL0 yone gore secilmezse CS Hi-Z olur.
 * BSRR tek yazmada set+reset yapar -> arada INA=INB=1 durumu olusmaz. */
void VNH7100_SetDirection(VNH7100_Dir_t dir)
{
  if (dir == VNH7100_DIR_FORWARD)
  {
    s_cfg.port->BSRR = (uint32_t)s_cfg.pin_ina | (uint32_t)s_cfg.pin_sel0 |
                       ((uint32_t)s_cfg.pin_inb << 16);
  }
  else
  {
    s_cfg.port->BSRR = (uint32_t)s_cfg.pin_inb |
                       (((uint32_t)s_cfg.pin_ina | (uint32_t)s_cfg.pin_sel0) << 16);
  }
}

/* %100'de CCR = ARR+1 -> PWM1 modunda tam %100 */
void VNH7100_SetDuty(uint8_t percent)
{
  if (percent > 100U)
  {
    percent = 100U;
  }
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(s_cfg.htim);
  __HAL_TIM_SET_COMPARE(s_cfg.htim, s_cfg.tim_channel, ((arr + 1U) * percent) / 100U);
  s_duty = percent;
}

uint8_t VNH7100_GetDuty(void)
{
  return s_duty;
}

void VNH7100_Off(void)
{
  VNH7100_SetDuty(0U);
  s_cfg.port->BSRR = ((uint32_t)s_cfg.pin_ina | (uint32_t)s_cfg.pin_inb |
                      (uint32_t)s_cfg.pin_sel0) << 16;
}

/* Table 12 / Fig. 9-10: latch INx toggle ile acilir (HS: 1->0, LS: 0->1).
 * PWM=0 iken iki girisi 0->1->0 yaparak iki durumu da kapsiyoruz. */
void VNH7100_ResetLatch(void)
{
  uint32_t in_mask = (uint32_t)s_cfg.pin_ina | (uint32_t)s_cfg.pin_inb;

  VNH7100_SetDuty(0U);
  s_cfg.port->BSRR = in_mask << 16;
  HAL_Delay(1);
  s_cfg.port->BSRR = in_mask;
  HAL_Delay(1);
  s_cfg.port->BSRR = in_mask << 16;
  HAL_Delay(1);
  VNH7100_Off();
}

uint16_t VNH7100_ReadRaw(void)
{
  uint32_t sum = 0U;
  for (uint32_t i = 0U; i < ADC_BUF_SIZE; i++)
  {
    sum += s_adc_buf[i];
  }
  return (uint16_t)(sum / ADC_BUF_SIZE);
}

/* I[mA] = raw * Vref[mV] * K / (4095 * Rsense) */
uint32_t VNH7100_RawToMilliAmp(uint16_t raw)
{
  uint64_t num = (uint64_t)raw * s_cfg.vref_mv * s_cfg.k_factor;
  uint64_t den = (uint64_t)ADC_FULL_SCALE * s_cfg.rsense_ohm;
  return (uint32_t)(num / den);
}

bool VNH7100_IsSaturated(uint16_t raw)
{
  return (raw >= ADC_SAT_LEVEL);
}
