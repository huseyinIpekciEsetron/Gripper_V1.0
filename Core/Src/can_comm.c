/**
  ******************************************************************************
  * @file    can_comm.c
  * @brief   bxCAN dusuk seviye katman
  ******************************************************************************
  */
#include "can_comm.h"
#include "app_config.h"
#include <string.h>

#define RX_RING_SIZE   16U    /* 2'nin kuvveti olmali */
#define TXH_SIZE       8U
#define TXL_SIZE       32U    /* 8x8 ToF olcumu = 17 cerceve */

typedef struct
{
  CanFrame_t buf[TXL_SIZE];
  uint8_t    size;
  uint8_t    head;
  uint8_t    tail;
  uint8_t    count;
} TxQueue_t;

static CAN_HandleTypeDef *s_hcan = NULL;
static bool               s_started = false;

static CanFrame_t         s_rx[RX_RING_SIZE];
static volatile uint8_t   s_rx_head = 0U;     /* ISR yazar  */
static volatile uint8_t   s_rx_tail = 0U;     /* ana dongu yazar */
static volatile uint32_t  s_rx_dropped = 0U;
static volatile uint32_t  s_last_rx_tick = 0U;
static volatile bool      s_has_rx = false;

static TxQueue_t          s_txh = { .size = TXH_SIZE };
static TxQueue_t          s_txl = { .size = TXL_SIZE };

static uint32_t           s_tx_count = 0U;
static uint32_t           s_rx_count = 0U;
static uint32_t           s_tx_dropped = 0U;
static bool               s_bus_off_seen = false;

/* ============================ BIT ZAMANLAMASI ============================ */
/* 16 tq, ornekleme noktasi %87.5 (1 + 13 + 2) - master ile birebir ayni.
 *   PCLK1 32 MHz (HSI 64 MHz):  250k -> /8   500k -> /4
 *   PCLK1 36 MHz (HSE 72 MHz):  250k -> /9   500k -> /4.5 (yok, 18 tq ile /4)
 * SJW 2 tq: kiskac tarafinda saat toleransini artirir, master'la uyumludur. */
static HAL_StatusTypeDef ApplyBitTiming(CAN_HandleTypeDef *hcan)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_2TQ;

#if (CAN_BITRATE == 250000U)
  if      (pclk1 == 32000000U) { hcan->Init.Prescaler = 8U; }
  else if (pclk1 == 36000000U) { hcan->Init.Prescaler = 9U; }
  else                         { return HAL_ERROR; }
#elif (CAN_BITRATE == 500000U)
  if      (pclk1 == 32000000U) { hcan->Init.Prescaler = 4U; }
  else if (pclk1 == 36000000U) { hcan->Init.Prescaler = 4U; hcan->Init.TimeSeg1 = CAN_BS1_15TQ; }
  else                         { return HAL_ERROR; }
#else
#error "CAN_BITRATE 250000 veya 500000 olmali"
#endif

  hcan->Init.SyncJumpWidth        = CAN_SJW_2TQ;
  hcan->Init.Mode                 = CAN_MODE_NORMAL;
  hcan->Init.TimeTriggeredMode    = DISABLE;
  hcan->Init.AutoBusOff           = ENABLE;   /* bus-off'tan donanim kendisi doner */
  hcan->Init.AutoWakeUp           = DISABLE;
  hcan->Init.AutoRetransmission   = ENABLE;
  hcan->Init.ReceiveFifoLocked    = DISABLE;
  hcan->Init.TransmitFifoPriority = ENABLE;   /* posta kutulari sirayla gider */

  return HAL_CAN_Init(hcan);
}

/* 16-bit liste modu: en fazla 4 standart ID, sadece data cercevesi.
 * Motor trafigi donanim filtresinde elenir, kesmeye hic gelmez. */
static HAL_StatusTypeDef ConfigFilter(CAN_HandleTypeDef *hcan, const uint16_t *ids, uint8_t count)
{
  CAN_FilterTypeDef f = {0};
  uint32_t v[4];

  if ((ids == NULL) || (count == 0U) || (count > 4U))
  {
    return HAL_ERROR;
  }
  for (uint8_t i = 0U; i < 4U; i++)
  {
    uint16_t id = ids[(i < count) ? i : (uint8_t)(count - 1U)];   /* bos yerleri tekrarla */
    v[i] = ((uint32_t)id & 0x7FFU) << 5;
  }

  f.FilterBank           = 0U;
  f.FilterMode           = CAN_FILTERMODE_IDLIST;
  f.FilterScale          = CAN_FILTERSCALE_16BIT;
  f.FilterIdLow          = v[0];
  f.FilterIdHigh         = v[1];
  f.FilterMaskIdLow      = v[2];
  f.FilterMaskIdHigh     = v[3];
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(hcan, &f);
}

/* ============================ API ======================================== */

HAL_StatusTypeDef CanComm_Init(CAN_HandleTypeDef *hcan, const uint16_t *rx_ids, uint8_t count)
{
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }
  s_hcan = hcan;

  if (ApplyBitTiming(hcan) != HAL_OK)                               return HAL_ERROR;
  if (ConfigFilter(hcan, rx_ids, count) != HAL_OK)                  return HAL_ERROR;
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return HAL_ERROR;
  if (HAL_CAN_Start(hcan) != HAL_OK)                                return HAL_ERROR;

  s_started = true;
  return HAL_OK;
}

static bool Queue_Push(TxQueue_t *q, uint16_t id, const uint8_t *data, uint8_t dlc)
{
  if (q->count >= q->size)
  {
    return false;
  }
  CanFrame_t *f = &q->buf[q->head];
  f->id  = id & 0x7FFU;
  f->dlc = (dlc > 8U) ? 8U : dlc;
  memset(f->data, 0, sizeof(f->data));
  if ((data != NULL) && (f->dlc > 0U))
  {
    memcpy(f->data, data, f->dlc);
  }
  q->head = (uint8_t)((q->head + 1U) % q->size);
  q->count++;
  return true;
}

bool CanComm_Send(uint16_t id, const uint8_t *data, uint8_t dlc, CanPrio_t prio)
{
  TxQueue_t *q = (prio == CAN_PRIO_HIGH) ? &s_txh : &s_txl;

  if (!s_started || !Queue_Push(q, id, data, dlc))
  {
    s_tx_dropped++;
    return false;
  }
  CanComm_Task();   /* bos posta kutusu varsa hemen gonder */
  return true;
}

uint8_t CanComm_FreeSlots(CanPrio_t prio)
{
  const TxQueue_t *q = (prio == CAN_PRIO_HIGH) ? &s_txh : &s_txl;
  return (uint8_t)(q->size - q->count);
}

void CanComm_Task(void)
{
  if (!s_started)
  {
    return;
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) > 0U)
  {
    TxQueue_t *q = (s_txh.count > 0U) ? &s_txh : ((s_txl.count > 0U) ? &s_txl : NULL);
    if (q == NULL)
    {
      break;
    }

    CanFrame_t *f = &q->buf[q->tail];
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mailbox;

    h.StdId              = f->id;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = f->dlc;
    h.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(s_hcan, &h, f->data, &mailbox) != HAL_OK)
    {
      break;
    }
    q->tail = (uint8_t)((q->tail + 1U) % q->size);
    q->count--;
    s_tx_count++;
  }
}

bool CanComm_Receive(CanFrame_t *out)
{
  uint8_t tail = s_rx_tail;

  if ((out == NULL) || (tail == s_rx_head))
  {
    return false;
  }
  __DMB();
  *out = s_rx[tail];
  __DMB();
  s_rx_tail = (uint8_t)((tail + 1U) & (RX_RING_SIZE - 1U));
  s_rx_count++;
  return true;
}

bool CanComm_HasReceived(void)
{
  return s_has_rx;
}

uint32_t CanComm_LastRxTick(void)
{
  return s_last_rx_tick;
}

void CanComm_GetStats(CanStats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  uint32_t esr = (s_hcan != NULL) ? s_hcan->Instance->ESR : 0U;

  out->tx_count      = s_tx_count;
  out->rx_count      = s_rx_count;
  out->tx_dropped    = s_tx_dropped;
  out->rx_dropped    = s_rx_dropped;
  out->tec           = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
  out->rec           = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
  out->bus_off       = ((esr & CAN_ESR_BOFF) != 0U);
  out->error_passive = ((esr & CAN_ESR_EPVF) != 0U);
  if (out->bus_off)
  {
    s_bus_off_seen = true;
  }
  out->bus_off_seen  = s_bus_off_seen;
}

/* ============================ KESME ====================================== */
/* HAL_CAN_IRQHandler (stm32f1xx_it.c, USB_LP_CAN1_RX0_IRQHandler) cagirir */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, d) != HAL_OK)
    {
      break;
    }
    if ((h.IDE != CAN_ID_STD) || (h.RTR != CAN_RTR_DATA))
    {
      continue;
    }

    s_last_rx_tick = HAL_GetTick();
    s_has_rx       = true;

    uint8_t head = s_rx_head;
    uint8_t next = (uint8_t)((head + 1U) & (RX_RING_SIZE - 1U));
    if (next == s_rx_tail)
    {
      s_rx_dropped++;
      continue;
    }
    s_rx[head].id  = (uint16_t)h.StdId;
    s_rx[head].dlc = (h.DLC > 8U) ? 8U : (uint8_t)h.DLC;
    memcpy(s_rx[head].data, d, 8U);
    __DMB();
    s_rx_head = next;
  }
}
