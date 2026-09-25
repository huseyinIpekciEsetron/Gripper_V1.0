/**
  ******************************************************************************
  * @file    can_comm.h
  * @brief   bxCAN dusuk seviye katman: bit zamanlamasi, filtre, TX kuyrugu,
  *          kesme tabanli RX halkasi. Protokolden bagimsiz.
  *
  *  - TX: iki yazilim kuyrugu (HIGH: durum/ack, LOW: ToF/tani). HIGH once gider.
  *        CanComm_Task her ana dongu turunda cagrilmali.
  *  - RX: FIFO0 kesmesi halkaya yazar, ana dongu CanComm_Receive ile okur.
  ******************************************************************************
  */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
} CanFrame_t;

typedef enum
{
  CAN_PRIO_HIGH = 0,
  CAN_PRIO_LOW
} CanPrio_t;

typedef struct
{
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t tx_dropped;       /* kuyruk dolu (bus yok / ack yok / yuk fazla) */
  uint32_t rx_dropped;       /* RX halkasi dolu */
  uint8_t  tec;              /* transmit error counter */
  uint8_t  rec;              /* receive error counter  */
  bool     bus_off;
  bool     error_passive;
  bool     bus_off_seen;     /* acilistan beri en az bir kez bus-off oldu */
} CanStats_t;

/* Bit zamanlamasini PCLK1'e gore ayarlar (32 veya 36 MHz), filtreyi kurar,
 * RX kesmesini acar ve CAN'i baslatir. rx_ids: kabul edilecek en fazla 4 ID. */
HAL_StatusTypeDef CanComm_Init(CAN_HandleTypeDef *hcan, const uint16_t *rx_ids, uint8_t count);

bool     CanComm_Send(uint16_t id, const uint8_t *data, uint8_t dlc, CanPrio_t prio);
uint8_t  CanComm_FreeSlots(CanPrio_t prio);
bool     CanComm_Receive(CanFrame_t *out);
void     CanComm_Task(void);

bool     CanComm_HasReceived(void);      /* acilistan beri gecerli mesaj geldi mi */
uint32_t CanComm_LastRxTick(void);
void     CanComm_GetStats(CanStats_t *out);

#endif /* CAN_COMM_H */
