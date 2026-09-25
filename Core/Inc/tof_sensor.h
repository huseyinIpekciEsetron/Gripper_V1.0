/**
  ******************************************************************************
  * @file    tof_sensor.h
  * @brief   VL53L8CX mesafe sensoru - uygulama katmani
  *
  *  - Init bloklar (~1-2 s firmware yukleme): motor dururken cagrilmali
  *  - ToF_Task periyodik cagrilir, veri hazirsa okur (bloklama ~5 ms)
  *  - Sensor hatasi karti durdurmaz: kiskac calismaya devam eder
  *  - Tum fonksiyonlar ana donguden cagrilmali (kesmeden degil)
  *
  *  Bolge sirasi: zone 0 bir kosede baslar ve satir satir ilerler.
  *  Sensorun lensi goruntuyu ters cevirir; yonu montaja gore test ederek belirle.
  ******************************************************************************
  */
#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define TOF_MAX_ZONES   64U

typedef enum
{
  TOF_STATE_UNINIT = 0,
  TOF_STATE_RUNNING,
  TOF_STATE_ERROR
} ToF_State_t;

typedef enum
{
  TOF_ERR_NONE = 0,
  TOF_ERR_NOT_FOUND,   /* I2C'de cevap yok / yanlis ID: kablo, besleme, pull-up */
  TOF_ERR_INIT,        /* firmware yukleme basarisiz */
  TOF_ERR_CONFIG,      /* cozunurluk / frekans / mod ayari */
  TOF_ERR_START,       /* olcum baslatilamadi */
  TOF_ERR_I2C,         /* calisirken ust uste I2C hatasi */
  TOF_ERR_STALE        /* sensor veri uretmeyi birakti */
} ToF_Error_t;

typedef struct
{
  uint32_t frame_id;                     /* her yeni olcumde artar           */
  uint32_t timestamp_ms;                 /* HAL_GetTick() degeri             */
  uint8_t  zone_count;                   /* 16 (4x4) veya 64 (8x8)           */
  int16_t  distance_mm[TOF_MAX_ZONES];
  uint8_t  target_status[TOF_MAX_ZONES]; /* 5 ve 9 = gecerli                 */
  uint64_t valid_mask;                   /* bit i = 1 -> zone i gecerli      */
  int16_t  min_distance_mm;              /* gecerli bolgelerin en yakini, yoksa -1 */
  uint8_t  min_zone;                     /* en yakin bolge, yoksa 0xFF       */
} ToF_Frame_t;

typedef struct
{
  ToF_State_t state;
  ToF_Error_t last_error;
  uint8_t     last_api_status;   /* ST API donus kodu (0 = OK) */
  uint32_t    frame_count;
  uint32_t    error_count;
  uint32_t    i2c_error_count;
  uint32_t    reinit_count;
} ToF_Status_t;

bool ToF_Init(I2C_HandleTypeDef *hi2c);   /* bloklar; false: sensor calismiyor */
bool ToF_Reinit(void);                    /* I2C + sensoru bastan kurar, bloklar */
void ToF_Task(void);                      /* her kontrol periyodunda cagir */

bool ToF_HasNewFrame(void);
bool ToF_GetFrame(ToF_Frame_t *out);      /* son olcumu kopyalar; true: yeniydi */
void ToF_GetStatus(ToF_Status_t *out);

#endif /* TOF_SENSOR_H */
