/**
  ******************************************************************************
  * @file    platform.c
  * @brief   VL53L8CX ULD platform katmani - STM32F1 HAL
  *
  *  F4'te calisan orijinale gore degisenler:
  *   - I2C handle platform yapisindan geliyor (extern yok)
  *   - Timeout transfer boyutuna gore hesaplaniyor. HAL timeout'u TUM transfere
  *     uygular; firmware yuklemedeki 32 KB'lik yazma 400 kHz'de ~0.8 s surer,
  *     sabit 1000 ms sinirda kaliyordu.
  *   - Buyuk yazmalar 4 KB'lik parcalara bolunuyor, aralarda IdleHook cagriliyor
  *   - I2C hat kurtarma (F1 errata)
  ******************************************************************************
  */
#include "platform.h"
#include "app_config.h"

#define I2C_WRITE_CHUNK      4096U
#define I2C_TIMEOUT_BASE_MS  10U

/* 400 kHz'de ~23 us/byte. size/10 ms ~4 kat pay birakir. */
static uint32_t I2C_Timeout(uint32_t size)
{
  return I2C_TIMEOUT_BASE_MS + (size / 10U);
}

__weak void VL53L8CX_IdleHook(void)
{
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
  return (uint8_t)HAL_I2C_Mem_Read(p_platform->hi2c, p_platform->address, RegisterAdress,
                                   I2C_MEMADD_SIZE_16BIT, p_value, 1U, I2C_Timeout(1U));
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
  return (uint8_t)HAL_I2C_Mem_Write(p_platform->hi2c, p_platform->address, RegisterAdress,
                                    I2C_MEMADD_SIZE_16BIT, &value, 1U, I2C_Timeout(1U));
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
  return (uint8_t)HAL_I2C_Mem_Read(p_platform->hi2c, p_platform->address, RegisterAdress,
                                   I2C_MEMADD_SIZE_16BIT, p_values, (uint16_t)size,
                                   I2C_Timeout(size));
}

/* Ardisik adreslere parca parca yazar (sensor adresi otomatik artirir) */
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
  uint8_t status = 0U;

  while (size > 0U)
  {
    uint16_t n = (size > I2C_WRITE_CHUNK) ? (uint16_t)I2C_WRITE_CHUNK : (uint16_t)size;

    status = (uint8_t)HAL_I2C_Mem_Write(p_platform->hi2c, p_platform->address, RegisterAdress,
                                        I2C_MEMADD_SIZE_16BIT, p_values, n, I2C_Timeout(n));
    if (status != 0U)
    {
      break;
    }
    RegisterAdress = (uint16_t)(RegisterAdress + n);
    p_values += n;
    size -= n;

    VL53L8CX_IdleHook();
  }
  return status;
}

/* LPn / I2C_RST pinleri karta bagli degil (sensor kartinda pull-up ile aktif) */
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
  (void)p_platform;
  return 0U;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
  uint32_t i, tmp;
  for (i = 0U; i < size; i = i + 4U)
  {
    tmp = ((uint32_t)buffer[i] << 24) | ((uint32_t)buffer[i + 1U] << 16) |
          ((uint32_t)buffer[i + 2U] << 8) | (uint32_t)buffer[i + 3U];
    memcpy(&(buffer[i]), &tmp, 4U);
  }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
  (void)p_platform;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < TimeMs)
  {
    VL53L8CX_IdleHook();
  }
  return 0U;
}

/* ============================ HAT KURTARMA =============================== */

static void BusDelay(void)
{
  for (volatile uint32_t i = 0U; i < 40U; i++)
  {
  }
}

void Platform_I2C_BusRecover(void)
{
  GPIO_InitTypeDef g = {0};

  TOF_I2C_GPIO_CLK_ENABLE();

  /* Pinleri gecici olarak GPIO open-drain yap */
  HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN | TOF_I2C_SDA_PIN, GPIO_PIN_SET);
  g.Pin   = TOF_I2C_SCL_PIN | TOF_I2C_SDA_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOF_I2C_GPIO_PORT, &g);
  BusDelay();

  /* Yarim kalmis bir transfer SDA'yi low tutuyorsa en fazla 9 saat darbesi */
  for (uint32_t i = 0U; (i < 9U) &&
       (HAL_GPIO_ReadPin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN) == GPIO_PIN_RESET); i++)
  {
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_RESET);
    BusDelay();
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_SET);
    BusDelay();
  }

  /* STOP kosulu: SCL low -> SDA low -> SCL high -> SDA high */
  HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_RESET);
  BusDelay();
  HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN, GPIO_PIN_RESET);
  BusDelay();
  HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_SET);
  BusDelay();
  HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN, GPIO_PIN_SET);
  BusDelay();

  /* F1 errata: BUSY bayragi takili kalabilir -> cevre birimini resetle.
   * Pinler HAL_I2C_MspInit'te tekrar AF open-drain yapilir. */
  TOF_I2C_CLK_ENABLE();
  TOF_I2C_FORCE_RESET();
  TOF_I2C_RELEASE_RESET();
}
