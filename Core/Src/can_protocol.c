/**
  ******************************************************************************
  * @file    can_protocol.c
  * @brief   Kiskac CAN protokolu - kiskac karti tarafi
  ******************************************************************************
  */
#include "can_protocol.h"
#include "gripper_can_defs.h"
#include "can_comm.h"
#include "gripper.h"
#include "app_config.h"

/* gripper.h ile protokol kodlari ayni kalmali (master bu sayilari yorumluyor) */
_Static_assert(GRIP_STATE_IDLE == GCAN_STATE_IDLE && GRIP_STATE_STOPPING == GCAN_STATE_STOPPING &&
               GRIP_STATE_STARTING == GCAN_STATE_STARTING && GRIP_STATE_MOVING == GCAN_STATE_MOVING &&
               GRIP_STATE_FAULT == GCAN_STATE_FAULT, "state kodlari uyusmuyor");
_Static_assert(GRIP_MOTION_OPEN == GCAN_MOTION_OPEN && GRIP_MOTION_CLOSE == GCAN_MOTION_CLOSE,
               "motion kodlari uyusmuyor");
_Static_assert(GRIP_STOP_USER == GCAN_STOP_USER && GRIP_STOP_STALL == GCAN_STOP_STALL &&
               GRIP_STOP_TIMEOUT == GCAN_STOP_TIMEOUT && GRIP_STOP_NO_LOAD == GCAN_STOP_NO_LOAD &&
               GRIP_STOP_OVERCURRENT == GCAN_STOP_OVERCURRENT &&
               GRIP_STOP_DRIVER_FAULT == GCAN_STOP_DRIVER_FAULT, "stop kodlari uyusmuyor");

static bool     s_can_ok = false;
static bool     s_force_status = true;      /* acilista hemen durum yolla */
static bool     s_cmd_rejected = false;
static bool     s_comm_lost = false;
static bool     s_comm_timeout_flag = false;
static bool     s_tof_reinit_req = false;
static bool     s_seq_synced = false;       /* false: sonraki cercevede sadece senkronize ol */
static uint8_t  s_last_seq = 0U;
static uint8_t  s_ack_seq = 0U;
static uint8_t  s_last_key[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint32_t s_status_timer = 0U;
static uint32_t s_diag_timer = 0U;

static void    Handle_Command(const CanFrame_t *f);
static bool    Execute_Command(uint8_t cmd);
static void    Check_CommTimeout(void);
static void    Build_Status(uint8_t d[8]);
static void    Build_Diag(uint8_t d[8]);
static uint8_t ToF_StateByte(void);

/* ============================ API ======================================== */

void CanProto_Init(bool can_ok)
{
  s_can_ok       = can_ok;
  s_force_status = true;
  s_seq_synced   = false;
}

void CanProto_ProcessRx(void)
{
  CanFrame_t f;

  if (!s_can_ok)
  {
    return;
  }
  while (CanComm_Receive(&f))
  {
    if (f.id == GCAN_ID_CMD)
    {
      Handle_Command(&f);
    }
  }
  Check_CommTimeout();
}

void CanProto_Tx(void)
{
  uint8_t d[8];

  if (!s_can_ok)
  {
    return;
  }

  /* DURUM: periyodik + onemli alanlar degisince hemen */
  Build_Status(d);
  bool changed = (d[0] != s_last_key[0]) || (d[1] != s_last_key[1]) ||
                 (d[2] != s_last_key[2]) || (d[3] != s_last_key[3]);

  s_status_timer += CTRL_PERIOD_MS;
  if (s_force_status || changed || (s_status_timer >= CAN_STATUS_PERIOD_MS))
  {
    if (CanComm_Send(GCAN_ID_STATUS, d, 8U, CAN_PRIO_HIGH))
    {
      for (uint8_t i = 0U; i < 4U; i++)
      {
        s_last_key[i] = d[i];
      }
      s_force_status = false;
      s_status_timer = 0U;
    }
  }

  /* TANI */
  s_diag_timer += CTRL_PERIOD_MS;
  if (s_diag_timer >= CAN_DIAG_PERIOD_MS)
  {
    s_diag_timer = 0U;
    Build_Diag(d);
    (void)CanComm_Send(GCAN_ID_DIAG, d, 8U, CAN_PRIO_LOW);
  }
}

void CanProto_SendToF(const ToF_Frame_t *fr)
{
#if TOF_ENABLE && CAN_TOF_SEND_ENABLE
  uint8_t d[8];
  uint8_t segs;
  uint8_t valid = 0U;

  if (!s_can_ok || (fr == NULL) || (fr->zone_count == 0U))
  {
    return;
  }
  segs = (uint8_t)(fr->zone_count / GCAN_TOF_ZONES_PER_SEG);
  if (segs > GCAN_TOF_MAX_SEGS)
  {
    segs = GCAN_TOF_MAX_SEGS;
  }

  /* Olcumu yarim gondermemek icin once kuyrukta yer var mi bak */
  if (CanComm_FreeSlots(CAN_PRIO_LOW) < (uint8_t)(segs + 1U))
  {
    return;
  }

  for (uint8_t z = 0U; z < fr->zone_count; z++)
  {
    if ((fr->valid_mask & (1ULL << z)) != 0U)
    {
      valid++;
    }
  }

  d[0] = (uint8_t)(fr->frame_id & 0xFFU);
  d[1] = fr->zone_count;
  d[2] = segs;
  d[3] = valid;
  GCan_PutU16(&d[4], (fr->min_distance_mm > 0) ? (uint16_t)fr->min_distance_mm : 0U);
  d[6] = fr->min_zone;
  d[7] = ToF_StateByte();
  (void)CanComm_Send(GCAN_ID_TOF_HDR, d, 8U, CAN_PRIO_LOW);

  for (uint8_t s = 0U; s < segs; s++)
  {
    for (uint8_t k = 0U; k < GCAN_TOF_ZONES_PER_SEG; k++)
    {
      uint8_t  z = (uint8_t)(s * GCAN_TOF_ZONES_PER_SEG + k);
      uint16_t v = 0U;
      if (((fr->valid_mask & (1ULL << z)) != 0U) && (fr->distance_mm[z] > 0))
      {
        v = (uint16_t)fr->distance_mm[z];
      }
      GCan_PutU16(&d[k * 2U], v);
    }
    (void)CanComm_Send(GCAN_ID_TOF_DATA(s), d, 8U, CAN_PRIO_LOW);
  }
#else
  (void)fr;
#endif
}

bool CanProto_TakeTofReinitRequest(void)
{
  bool r = s_tof_reinit_req;
  s_tof_reinit_req = false;
  return r;
}

/* ============================ DAHILI ===================================== */

/* Komut sadece seq degisince calisir; tekrar eden cerceve = canlilik */
static void Handle_Command(const CanFrame_t *f)
{
  if (f->dlc < 2U)
  {
    return;
  }
  uint8_t cmd = f->data[0];
  uint8_t seq = f->data[1];

  s_comm_timeout_flag = false;

  if (!s_seq_synced)
  {
    /* Acilis / haberlesme donusu: eski komutu calistirma, sadece senkronize ol */
    s_seq_synced   = true;
    s_last_seq     = seq;
    s_ack_seq      = seq;
    s_force_status = true;
    return;
  }
  if (seq == s_last_seq)
  {
    return;
  }

  s_last_seq     = seq;
  s_ack_seq      = seq;
  s_cmd_rejected = !Execute_Command(cmd);
  s_force_status = true;                          /* ack hemen gitsin */
}

static bool Execute_Command(uint8_t cmd)
{
  switch (cmd)
  {
    case GCAN_CMD_NOP:         return true;
    case GCAN_CMD_OPEN:        return Gripper_Command(GRIP_CMD_OPEN);
    case GCAN_CMD_CLOSE:       return Gripper_Command(GRIP_CMD_CLOSE);
    case GCAN_CMD_STOP:        return Gripper_Command(GRIP_CMD_STOP);
    case GCAN_CMD_CLEAR_FAULT: return Gripper_ClearFault();
    case GCAN_CMD_TOF_REINIT:
      s_tof_reinit_req = true;                    /* main motor dururken yapar */
      return true;
    default:                   return false;
  }
}

/* Master CAN_COMM_TIMEOUT_MS susarsa hareketi durdur, geri gelince yeniden
 * senkronize ol (arada kalan komutu calistirma). */
static void Check_CommTimeout(void)
{
#if CAN_COMM_TIMEOUT_MS > 0
  if (!CanComm_HasReceived())
  {
    return;
  }
  uint32_t last = CanComm_LastRxTick();   /* once son RX, sonra simdiki zaman */
  uint32_t now  = HAL_GetTick();

  if ((now - last) > CAN_COMM_TIMEOUT_MS)
  {
    if (!s_comm_lost)
    {
      GripperStatus_t gs;
      s_comm_lost  = true;
      s_seq_synced = false;
      Gripper_GetStatus(&gs);
      if ((gs.state == GRIP_STATE_MOVING) || (gs.state == GRIP_STATE_STARTING))
      {
        (void)Gripper_Command(GRIP_CMD_STOP);
        s_comm_timeout_flag = true;
        s_force_status = true;
      }
    }
  }
  else
  {
    s_comm_lost = false;
  }
#endif
}

static uint8_t ToF_StateByte(void)
{
#if TOF_ENABLE
  ToF_Status_t ts;
  ToF_GetStatus(&ts);
  return (uint8_t)(((uint8_t)ts.state & 0x03U) | (((uint8_t)ts.last_error & 0x07U) << 2));
#else
  return 0U;
#endif
}

static void Build_Status(uint8_t d[8])
{
  GripperStatus_t gs;
  uint8_t flags = 0U;
  bool moving;

  Gripper_GetStatus(&gs);
  moving = (gs.state == GRIP_STATE_MOVING) || (gs.state == GRIP_STATE_STARTING);

  /* Akim limitiyle durdu ve o yon hala kilitli: nesneyi tutuyor / uca dayali */
  if (!moving && (gs.blocked != GRIP_MOTION_NONE) &&
      ((gs.stop_reason == GRIP_STOP_STALL) || (gs.stop_reason == GRIP_STOP_OVERCURRENT)))
  {
    flags |= GCAN_FLAG_CURRENT_LIMIT;
  }
  if (gs.blocked == GRIP_MOTION_OPEN)  flags |= GCAN_FLAG_BLOCKED_OPEN;
  if (gs.blocked == GRIP_MOTION_CLOSE) flags |= GCAN_FLAG_BLOCKED_CLOSE;
  if (gs.fault_latched)                flags |= GCAN_FLAG_FAULT_LATCHED;
  if (moving)                          flags |= GCAN_FLAG_MOVING;
#if TOF_ENABLE
  ToF_Status_t ts;
  ToF_GetStatus(&ts);
  if (ts.state == TOF_STATE_RUNNING)   flags |= GCAN_FLAG_TOF_OK;
#endif
  if (s_comm_timeout_flag)             flags |= GCAN_FLAG_COMM_TIMEOUT;
  if (s_cmd_rejected)                  flags |= GCAN_FLAG_CMD_REJECTED;

  d[0] = (uint8_t)gs.state;
  d[1] = (uint8_t)gs.motion;
  d[2] = (uint8_t)gs.stop_reason;
  d[3] = flags;
  GCan_PutU16(&d[4], GCan_SatU16(gs.current_filt_mA));
  d[6] = gs.duty;
  d[7] = s_ack_seq;
}

static void Build_Diag(uint8_t d[8])
{
  GripperStatus_t gs;
  CanStats_t cs;
  uint8_t f = 0U;

  Gripper_GetStatus(&gs);
  CanComm_GetStats(&cs);

  if (cs.bus_off_seen)     f |= GCAN_DIAG_BUS_OFF_SEEN;
  if (cs.error_passive)    f |= GCAN_DIAG_ERROR_PASSIVE;
  if (cs.tx_dropped > 0U)  f |= GCAN_DIAG_TX_DROPPED;
  if (cs.rx_dropped > 0U)  f |= GCAN_DIAG_RX_DROPPED;
  if (HAL_GPIO_ReadPin(CAN_XCVR_FAULT_PORT, CAN_XCVR_FAULT_PIN) == GPIO_PIN_SET)
  {
    f |= GCAN_DIAG_XCVR_FAULT_PIN;
  }

  GCan_PutU16(&d[0], GCan_SatU16(gs.peak_mA));
  GCan_PutU16(&d[2], GCan_SatU16(gs.move_time_ms));
  d[4] = GCan_SatU8(gs.fault_count);
  d[5] = ToF_StateByte();
  d[6] = f;
  d[7] = FW_VERSION;
}
