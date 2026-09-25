/**
  ******************************************************************************
  * @file    tof_sensor.c
  * @brief   VL53L8CX mesafe sensoru - uygulama katmani
  ******************************************************************************
  */
#include "tof_sensor.h"
#include "vl53l8cx_api.h"
#include "platform.h"
#include "app_config.h"
#include <string.h>

/* ST: 5 = %100 gecerli, 9 = gecerli (buyuk darbe). Digerleri guvenilmez. */
#define TOF_STATUS_IS_VALID(s)   (((s) == 5U) || ((s) == 9U))

static VL53L8CX_Configuration s_dev;     /* ~2.3 KB */
static VL53L8CX_ResultsData   s_res;
static ToF_Frame_t            s_frame;
static ToF_Status_t           s_stat;
static I2C_HandleTypeDef     *s_hi2c = NULL;
static bool                   s_new_frame = false;
static uint32_t               s_last_frame_tick = 0U;
static uint8_t                s_consec_errors = 0U;

static bool Sensor_Start(void);
static bool Fail(ToF_Error_t err, uint8_t api_status);
static void On_I2C_Error(uint8_t api_status);
static void Frame_Update(void);

/* ============================ API ======================================== */

bool ToF_Init(I2C_HandleTypeDef *hi2c)
{
  memset(&s_stat, 0, sizeof(s_stat));
  memset(&s_frame, 0, sizeof(s_frame));
  s_frame.min_distance_mm = -1;
  s_frame.min_zone        = 0xFFU;
  s_hi2c = hi2c;

  if (s_hi2c == NULL)
  {
    return Fail(TOF_ERR_NOT_FOUND, VL53L8CX_STATUS_INVALID_PARAM);
  }
  return Sensor_Start();
}

bool ToF_Reinit(void)
{
  if (s_hi2c == NULL)
  {
    return false;
  }
  s_stat.reinit_count++;

  (void)HAL_I2C_DeInit(s_hi2c);
  Platform_I2C_BusRecover();
  if (HAL_I2C_Init(s_hi2c) != HAL_OK)
  {
    return Fail(TOF_ERR_I2C, 0U);
  }
  return Sensor_Start();
}

void ToF_Task(void)
{
  uint8_t ready = 0U;
  uint8_t st;

  if (s_stat.state != TOF_STATE_RUNNING)
  {
    return;
  }

  st = vl53l8cx_check_data_ready(&s_dev, &ready);
  if (st != VL53L8CX_STATUS_OK)
  {
    On_I2C_Error(st);
    return;
  }

  if (ready == 0U)
  {
    if ((HAL_GetTick() - s_last_frame_tick) > TOF_STALE_MS)
    {
      (void)Fail(TOF_ERR_STALE, 0U);
    }
    return;
  }

  st = vl53l8cx_get_ranging_data(&s_dev, &s_res);
  if (st != VL53L8CX_STATUS_OK)
  {
    On_I2C_Error(st);
    return;
  }

  s_consec_errors   = 0U;
  s_last_frame_tick = HAL_GetTick();
  Frame_Update();
}

bool ToF_HasNewFrame(void)
{
  return s_new_frame;
}

bool ToF_GetFrame(ToF_Frame_t *out)
{
  bool was_new = s_new_frame;
  if (out != NULL)
  {
    *out = s_frame;
    s_new_frame = false;
  }
  return was_new;
}

void ToF_GetStatus(ToF_Status_t *out)
{
  if (out != NULL)
  {
    *out = s_stat;
  }
}

/* ============================ DAHILI ===================================== */

static bool Sensor_Start(void)
{
  uint8_t alive = 0U;
  uint8_t st;

  s_stat.state    = TOF_STATE_UNINIT;
  s_new_frame     = false;
  s_consec_errors = 0U;

  memset(&s_dev, 0, sizeof(s_dev));
  s_dev.platform.address = TOF_I2C_ADDR;
  s_dev.platform.hi2c    = s_hi2c;

  st = vl53l8cx_is_alive(&s_dev, &alive);
  if ((st != VL53L8CX_STATUS_OK) || (alive == 0U))
  {
    return Fail(TOF_ERR_NOT_FOUND, st);
  }

  st = vl53l8cx_init(&s_dev);                     /* firmware yukleme, ~1-2 s */
  if (st != VL53L8CX_STATUS_OK)
  {
    return Fail(TOF_ERR_INIT, st);
  }

  /* Once cozunurluk: frekans siniri cozunurluge bagli (8x8: 15 Hz, 4x4: 60 Hz) */
  st  = vl53l8cx_set_resolution(&s_dev, TOF_RESOLUTION);
  st |= vl53l8cx_set_ranging_frequency_hz(&s_dev, TOF_FREQ_HZ);
  st |= vl53l8cx_set_ranging_mode(&s_dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
  if (st != VL53L8CX_STATUS_OK)
  {
    return Fail(TOF_ERR_CONFIG, st);
  }

  st = vl53l8cx_start_ranging(&s_dev);
  if (st != VL53L8CX_STATUS_OK)
  {
    return Fail(TOF_ERR_START, st);
  }

  s_stat.state      = TOF_STATE_RUNNING;
  s_stat.last_error = TOF_ERR_NONE;
  s_last_frame_tick = HAL_GetTick();
  return true;
}

static bool Fail(ToF_Error_t err, uint8_t api_status)
{
  s_stat.state           = TOF_STATE_ERROR;
  s_stat.last_error      = err;
  s_stat.last_api_status = api_status;
  s_stat.error_count++;
  return false;
}

/* Tek seferlik I2C hatasi tolere edilir, ust uste olursa ERROR */
static void On_I2C_Error(uint8_t api_status)
{
  s_stat.i2c_error_count++;
  s_consec_errors++;
  if (s_consec_errors >= TOF_MAX_CONSEC_ERRORS)
  {
    (void)Fail(TOF_ERR_I2C, api_status);
  }
}

static void Frame_Update(void)
{
  uint8_t zones = TOF_RESOLUTION;

  s_frame.frame_id++;
  s_frame.timestamp_ms    = s_last_frame_tick;
  s_frame.zone_count      = zones;
  s_frame.valid_mask      = 0U;
  s_frame.min_distance_mm = -1;
  s_frame.min_zone        = 0xFFU;

  for (uint8_t z = 0U; z < zones; z++)
  {
    uint32_t idx = (uint32_t)z * VL53L8CX_NB_TARGET_PER_ZONE;   /* 1. hedef */
    int16_t  d   = s_res.distance_mm[idx];
    uint8_t  ts  = s_res.target_status[idx];

    s_frame.distance_mm[z]   = d;
    s_frame.target_status[z] = ts;

    if ((s_res.nb_target_detected[z] > 0U) && TOF_STATUS_IS_VALID(ts) && (d > 0))
    {
      s_frame.valid_mask |= (1ULL << z);
      if ((s_frame.min_distance_mm < 0) || (d < s_frame.min_distance_mm))
      {
        s_frame.min_distance_mm = d;
        s_frame.min_zone        = z;
      }
    }
  }

  s_stat.frame_count++;
  s_new_frame = true;
}
