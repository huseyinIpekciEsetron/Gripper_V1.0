/**
  ******************************************************************************
  * @file    gripper.c
  * @brief   Kiskac durum makinesi ve akim tabanli korumalar
  *
  *   IDLE --komut--> STARTING --1 periyot--> MOVING --dur--> STOPPING --> IDLE
  *                                            |
  *                                            +--surucu arizasi--> FAULT --500 ms / ClearFault--> STOPPING
  ******************************************************************************
  */
#include "gripper.h"
#include "vnh7100.h"
#include "app_config.h"
#include <stddef.h>

#define GRIP_OPEN_DIR  ((GRIP_CLOSE_DIR == VNH7100_DIR_FORWARD) ? \
                        VNH7100_DIR_REVERSE : VNH7100_DIR_FORWARD)

static GripperStatus_t s;
static GripperMotion_t s_pending;      /* bekleyen hareket komutu */
static uint32_t        s_timer_ms;     /* STOPPING bekleme sayaci */
static uint32_t        s_stall_ms;
static uint32_t        s_noload_ms;
static uint32_t        s_oc_ms;
static uint32_t        s_sat_ms;
static uint32_t        s_filt_acc;     /* filtre akumulatoru (akim << SHIFT) */
static uint32_t        s_consec_faults;/* ust uste ariza sayisi */

static void StopMotion(GripperStopReason_t reason);
static void BeginStart(void);
static void Moving_Task(bool saturated);

/* ============================ API ======================================== */

void Gripper_Init(void)
{
  s.state           = GRIP_STATE_IDLE;
  s.motion          = GRIP_MOTION_NONE;
  s.stop_reason     = GRIP_STOP_NONE;
  s.blocked         = GRIP_MOTION_NONE;
  s.duty            = 0U;
  s.current_mA      = 0U;
  s.current_filt_mA = 0U;
  s.peak_mA         = 0U;
  s.move_time_ms    = 0U;
  s.fault_count     = 0U;
  s.fault_latched   = false;

  s_consec_faults = 0U;
  s_pending  = GRIP_MOTION_NONE;
  s_timer_ms = 0U;
  s_filt_acc = 0U;

  VNH7100_Off();
}

bool Gripper_Command(GripperCmd_t cmd)
{
  bool moving = (s.state == GRIP_STATE_MOVING) || (s.state == GRIP_STATE_STARTING);

  switch (cmd)
  {
    case GRIP_CMD_STOP:
      s_pending = GRIP_MOTION_NONE;
      if (moving)
      {
        StopMotion(GRIP_STOP_USER);
      }
      return true;

    case GRIP_CMD_OPEN:
    case GRIP_CMD_CLOSE:
    {
      GripperMotion_t m = (cmd == GRIP_CMD_OPEN) ? GRIP_MOTION_OPEN : GRIP_MOTION_CLOSE;

      if (s.fault_latched)
      {
        return false;                    /* kalici ariza: once ClearFault */
      }
      if (m == s.blocked)
      {
        return false;                    /* zaten dayanmis, zorlamaya devam etme */
      }
      if (moving && (s.motion == m))
      {
        return true;                     /* zaten o yone gidiyor */
      }
      if (moving)
      {
        StopMotion(GRIP_STOP_USER);      /* ters yon: once dur, bekle */
      }
      s_pending = m;                     /* IDLE'da, bekleme veya kurtarma bitince baslar */
      return true;
    }

    default:
      return false;
  }
}

bool Gripper_ClearFault(void)
{
  if (s.state != GRIP_STATE_FAULT)
  {
    return false;
  }
  VNH7100_ResetLatch();
  s_consec_faults = 0U;
  s.fault_latched = false;
  s_pending   = GRIP_MOTION_NONE;
  s_timer_ms  = 0U;
  s.state     = GRIP_STATE_STOPPING;
  /* s.blocked korunur: arizaya giren yone tekrar zorlama yapilmasin */
  return true;
}

void Gripper_GetStatus(GripperStatus_t *out)
{
  if (out != NULL)
  {
    *out = s;
  }
}

void Gripper_Task(void)
{
  /* 1) Olcum (her durumda) */
  uint16_t raw = VNH7100_ReadRaw();
  bool     sat = VNH7100_IsSaturated(raw);

  s.current_mA      = VNH7100_RawToMilliAmp(raw);
  s_filt_acc        = s_filt_acc - (s_filt_acc >> CURRENT_FILTER_SHIFT) + s.current_mA;
  s.current_filt_mA = s_filt_acc >> CURRENT_FILTER_SHIFT;

  /* 2) Durum makinesi */
  switch (s.state)
  {
    case GRIP_STATE_IDLE:
      if (s_pending != GRIP_MOTION_NONE)
      {
        BeginStart();
      }
      break;

    case GRIP_STATE_STOPPING:
      s_timer_ms += CTRL_PERIOD_MS;
      if (s_timer_ms >= GRIP_STOP_DWELL_MS)
      {
        s.state = GRIP_STATE_IDLE;
        if (s_pending != GRIP_MOTION_NONE)
        {
          BeginStart();
        }
      }
      break;

    case GRIP_STATE_STARTING:
      /* Yon pinleri bir periyot once ayarlandi (datasheet: PWM'den >=20 us once) */
      s.duty         = 0U;
      s.peak_mA      = 0U;
      s.move_time_ms = 0U;
      s_stall_ms = s_noload_ms = s_oc_ms = s_sat_ms = 0U;
      s_filt_acc = 0U;
      s.state = GRIP_STATE_MOVING;
      break;

    case GRIP_STATE_MOVING:
      Moving_Task(sat);
      break;

    case GRIP_STATE_FAULT:
      VNH7100_Off();   /* emin olmak icin her periyot kapali tut */
      if (!s.fault_latched)
      {
        s_timer_ms += CTRL_PERIOD_MS;
        if (s_timer_ms >= GRIP_FAULT_RECOVERY_MS)
        {
          /* Otomatik kurtarma: latch'i ac, bekle, bekleyen komut varsa calistir.
           * Arizaya giren yon kilitli kalir, ters yon serbest. */
          VNH7100_ResetLatch();
          s_timer_ms = 0U;
          s.state    = GRIP_STATE_STOPPING;
        }
      }
      break;

    default:
      StopMotion(GRIP_STOP_DRIVER_FAULT);
      break;
  }
}

/* ============================ DAHILI ===================================== */

static void BeginStart(void)
{
  s.motion    = s_pending;
  s_pending   = GRIP_MOTION_NONE;

  if (s.motion == s.blocked)
  {
    /* Beklerken bu yon kilitlendiyse baslatma */
    s.motion = GRIP_MOTION_NONE;
    return;
  }

  s.blocked     = GRIP_MOTION_NONE;   /* diger yone hareket kilidi acar */
  s.stop_reason = GRIP_STOP_NONE;

  VNH7100_SetDirection((s.motion == GRIP_MOTION_CLOSE) ? GRIP_CLOSE_DIR : GRIP_OPEN_DIR);
  s.state = GRIP_STATE_STARTING;
}

static void StopMotion(GripperStopReason_t reason)
{
  VNH7100_Off();
  s.duty        = 0U;
  s.stop_reason = reason;

  /* Zorlanmayla durulan yon kilitlenir */
  if ((reason == GRIP_STOP_STALL) || (reason == GRIP_STOP_OVERCURRENT) ||
      (reason == GRIP_STOP_DRIVER_FAULT))
  {
    s.blocked = s.motion;
  }

  s_timer_ms = 0U;

  if (reason == GRIP_STOP_DRIVER_FAULT)
  {
    s_pending = GRIP_MOTION_NONE;
    s.fault_count++;
    s_consec_faults++;
    s.fault_latched = (s_consec_faults >= GRIP_FAULT_MAX_RETRIES);
    s.state = GRIP_STATE_FAULT;
  }
  else
  {
    if (reason != GRIP_STOP_USER)
    {
      s_consec_faults = 0U;   /* hareket arizasiz bitti */
    }
    s.state = GRIP_STATE_STOPPING;
  }
}

static void Moving_Task(bool saturated)
{
  s.move_time_ms += CTRL_PERIOD_MS;

  /* --- Sert korumalar: korluk suresinde de aktif --- */
  if (saturated)
  {
    s_oc_ms = 0U;
    s_sat_ms += CTRL_PERIOD_MS;
    if (s_sat_ms >= CS_SAT_CONFIRM_MS)
    {
      StopMotion(GRIP_STOP_DRIVER_FAULT);
      return;
    }
  }
  else
  {
    s_sat_ms = 0U;
    if (s.current_mA >= HARD_OVERCURRENT_MA)
    {
      s_oc_ms += CTRL_PERIOD_MS;
      if (s_oc_ms >= HARD_OC_CONFIRM_MS)
      {
        StopMotion(GRIP_STOP_OVERCURRENT);
        return;
      }
    }
    else
    {
      s_oc_ms = 0U;
    }
  }

  /* --- Yumusak kalkis --- */
  uint8_t target = (s.motion == GRIP_MOTION_CLOSE) ? GRIP_CLOSE_DUTY : GRIP_OPEN_DUTY;
  if (target > 100U)
  {
    target = 100U;
  }
  if (s.duty < target)
  {
    uint8_t step = (uint8_t)(target - s.duty);
    s.duty = (step > GRIP_RAMP_STEP) ? (uint8_t)(s.duty + GRIP_RAMP_STEP) : target;
  }
  else if (s.duty > target)
  {
    s.duty = target;
  }
  VNH7100_SetDuty(s.duty);

  /* --- Kalkis akimi korlugu --- */
  if (s.move_time_ms <= GRIP_INRUSH_BLANK_MS)
  {
    return;
  }

  if (s.current_filt_mA > s.peak_mA)
  {
    s.peak_mA = s.current_filt_mA;
  }

  /* --- Zorlanma: nesne kistirildi veya uc noktaya dayandi --- */
  uint32_t stall_limit = (s.motion == GRIP_MOTION_CLOSE) ? GRIP_STALL_CLOSE_MA
                                                         : GRIP_STALL_OPEN_MA;
  if (s.current_filt_mA >= stall_limit)
  {
    s_stall_ms += CTRL_PERIOD_MS;
    if (s_stall_ms >= GRIP_STALL_CONFIRM_MS)
    {
      StopMotion(GRIP_STOP_STALL);
      return;
    }
  }
  else
  {
    s_stall_ms = 0U;
  }

  /* --- Akim yok: motor bagli degil / kablo kopuk --- */
  if (s.current_filt_mA < GRIP_NO_LOAD_MA)
  {
    s_noload_ms += CTRL_PERIOD_MS;
    if (s_noload_ms >= GRIP_NO_LOAD_CONFIRM_MS)
    {
      StopMotion(GRIP_STOP_NO_LOAD);
      return;
    }
  }
  else
  {
    s_noload_ms = 0U;
  }

  /* --- Zaman asimi --- */
  if (s.move_time_ms >= GRIP_TRAVEL_TIMEOUT_MS)
  {
    StopMotion(GRIP_STOP_TIMEOUT);
  }
}
