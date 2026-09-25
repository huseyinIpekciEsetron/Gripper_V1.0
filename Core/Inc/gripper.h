/**
  ******************************************************************************
  * @file    gripper.h
  * @brief   Kiskac kontrolu: ac / kapa / dur, akimla zorlanma algilama
  *
  *  Koruma ozeti:
  *   - Zorlanma (nesne kistirildi / uc noktaya dayandi) -> dur, o yon kilitlenir
  *   - Kilitli yone komut reddedilir, ters yone hareket kilidi acar
  *   - Hareket zaman asimi, akim yok (kopuk kablo) -> dur
  *   - Sert asiri akim -> hizli durus, o yon kilitlenir
  *   - CS doygun (surucu arizasi) -> FAULT, 500 ms sonra otomatik kurtarma;
  *     ust uste GRIP_FAULT_MAX_RETRIES arizadan sonra kalici, ClearFault gerekir
  *   - FAULT sirasinda ters yon komutu kabul edilir, kurtarma sonrasi calisir
  *   - Yumusak kalkis, her durustan sonra bekleme, yon degisiminde once durma
  *
  *  NOT: Gripper_Command / Gripper_ClearFault ana donguden cagrilmali.
  *       Ileride CAN kesmesinden gelecek komutlar bir bayrakla donguye aktarilmali.
  ******************************************************************************
  */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  GRIP_CMD_NONE = 0,
  GRIP_CMD_OPEN,
  GRIP_CMD_CLOSE,
  GRIP_CMD_STOP
} GripperCmd_t;

typedef enum
{
  GRIP_STATE_IDLE = 0,     /* durgun, komut bekliyor              */
  GRIP_STATE_STOPPING,     /* durustan sonra bekleme              */
  GRIP_STATE_STARTING,     /* yon pinleri ayarlandi, PWM bekliyor */
  GRIP_STATE_MOVING,       /* hareket + koruma                    */
  GRIP_STATE_FAULT         /* ariza: otomatik kurtarma veya ClearFault */
} GripperState_t;

typedef enum
{
  GRIP_MOTION_NONE = 0,
  GRIP_MOTION_OPEN,
  GRIP_MOTION_CLOSE
} GripperMotion_t;

typedef enum
{
  GRIP_STOP_NONE = 0,
  GRIP_STOP_USER,          /* STOP veya ters yon komutu                   */
  GRIP_STOP_STALL,         /* zorlanma: nesne kistirildi / uc noktaya dayandi */
  GRIP_STOP_TIMEOUT,       /* sure doldu, zorlanma gorulmedi              */
  GRIP_STOP_NO_LOAD,       /* akim yok: motor bagli degil                 */
  GRIP_STOP_OVERCURRENT,   /* sert akim limiti -> dur, yon kilitlenir     */
  GRIP_STOP_DRIVER_FAULT   /* CS doygun / surucu latch  -> FAULT          */
} GripperStopReason_t;

typedef struct
{
  GripperState_t      state;
  GripperMotion_t     motion;           /* su anki veya son hareket          */
  GripperStopReason_t stop_reason;      /* son durma sebebi                  */
  GripperMotion_t     blocked;          /* bu yone hareket kilitli           */
  uint8_t             duty;             /* uygulanan PWM %                   */
  uint32_t            current_mA;       /* anlik akim (~1.5 ms ortalama)     */
  uint32_t            current_filt_mA;  /* filtreli akim (korumada kullanilan) */
  uint32_t            peak_mA;          /* son hareketteki tepe (filtreli)   */
  uint32_t            move_time_ms;     /* son hareketin suresi              */
  uint32_t            fault_count;      /* toplam ariza sayisi               */
  bool                fault_latched;    /* true: kalici FAULT, ClearFault gerekir */
} GripperStatus_t;

void Gripper_Init(void);
bool Gripper_Command(GripperCmd_t cmd);   /* false: reddedildi (kalici FAULT / kilitli yon) */
bool Gripper_ClearFault(void);            /* false: FAULT'ta degil */
void Gripper_Task(void);                  /* her CTRL_PERIOD_MS'de bir cagir */
void Gripper_GetStatus(GripperStatus_t *out);

#endif /* GRIPPER_H */
