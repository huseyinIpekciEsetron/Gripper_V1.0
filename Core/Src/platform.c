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
  *   - Buyuk yazmalar 256 byte'lik parcalara bolunuyor, aralarda IdleHook cagriliyor
  *   - Her islemde hata olursa hat kurtarilip en fazla 3 kez tekrar deneniyor
  *     (uzun kablo / slip ring parazitine karsi). Sayaclar: platform_i2c_stats
  *   - I2C hat kurtarma (F1 errata)
  ******************************************************************************
  */
#include "platform.h"
#include "app_config.h"

#define I2C_WRITE_CHUNK      256U   /* kucuk parca: parazit olursa sadece bu parca tekrarlanir */
#define I2C_TIMEOUT_BASE_MS  10U
#define I2C_MAX_RETRY        3U     /* hata olursa: hat kurtarma + tekrar deneme sayisi */

volatile PlatformI2CStats_t platform_i2c_stats;

typedef enum { XFER_READ = 0, XFER_WRITE } XferDir_t;

/* Timeout I2C hizina ve boyuta gore: 1 byte = 9 bit, 3 kat pay.
 * 100 kHz'de 1 byte ~90 us -> 256 byte icin ~80 ms. */
static uint32_t I2C_Timeout(const I2C_HandleTypeDef *h, uint32_t size)
{
  uint32_t clk = (h->Init.ClockSpeed != 0U) ? h->Init.ClockSpeed : 100000U;
  uint32_t byte_us = 9000000U / clk;
  return I2C_TIMEOUT_BASE_MS + ((size * byte_us * 3U) / 1000U);
}

/* Takilmis hatti toparla: HAL'i kapat, SCL'ye 9 darbe + STOP, I2C birimini
 * resetle, HAL'i yeniden ac. Slip ringdeki tek bir parazit F1'in I2C birimini
 * "BUSY" durumunda kilitleyebiliyor; bundan yalnizca boyle cikilir. */
static void I2C_Recover(I2C_HandleTypeDef *h)
{
  platform_i2c_stats.recoveries++;
  (void)HAL_I2C_DeInit(h);
  Platform_I2C_BusRecover();
  (void)HAL_I2C_Init(h);
}

/* Tek bir I2C islemi: hata olursa hatti kurtarip ayni islemi tekrar dener.
 * VL53L8CX'e ayni adrese ayni veriyi tekrar yazmak / okumak zararsizdir. */
static uint8_t I2C_Xfer(VL53L8CX_Platform *p, XferDir_t dir, uint16_t reg,
                        uint8_t *buf, uint16_t n)
{
  HAL_StatusTypeDef st = HAL_ERROR;

  for (uint32_t attempt = 0U; attempt <= I2C_MAX_RETRY; attempt++)
  {
    if (attempt > 0U)
    {
      platform_i2c_stats.retries++;
      I2C_Recover(p->hi2c);
    }

    if (dir == XFER_READ)
    {
      st = HAL_I2C_Mem_Read(p->hi2c, p->address, reg, I2C_MEMADD_SIZE_16BIT,
                            buf, n, I2C_Timeout(p->hi2c, n));
    }
    else
    {
      st = HAL_I2C_Mem_Write(p->hi2c, p->address, reg, I2C_MEMADD_SIZE_16BIT,
                             buf, n, I2C_Timeout(p->hi2c, n));
    }

    if (st == HAL_OK)
    {
      return 0U;
    }
    platform_i2c_stats.last_hal_status = (uint8_t)st;
    platform_i2c_stats.last_error      = HAL_I2C_GetError(p->hi2c);
  }

  platform_i2c_stats.failures++;
  return (uint8_t)st;
}

__weak void VL53L8CX_IdleHook(void)
{
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
  return I2C_Xfer(p_platform, XFER_READ, RegisterAdress, p_value, 1U);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
  return I2C_Xfer(p_platform, XFER_WRITE, RegisterAdress, &value, 1U);
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
  return I2C_Xfer(p_platform, XFER_READ, RegisterAdress, p_values, (uint16_t)size);
}

/* Ardisik adreslere parca parca yazar (sensor adresi otomatik artirir).
 * Bir parca bozulursa I2C_Xfer sadece o parcayi tekrarlar. */
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
  uint8_t status = 0U;

  while (size > 0U)
  {
    uint16_t n = (size > I2C_WRITE_CHUNK) ? (uint16_t)I2C_WRITE_CHUNK : (uint16_t)size;

    status = I2C_Xfer(p_platform, XFER_WRITE, RegisterAdress, p_values, n);
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
