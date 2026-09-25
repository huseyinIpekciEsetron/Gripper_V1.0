/**
  ******************************************************************************
  * @file    can_protocol.h
  * @brief   Kiskac CAN protokolu - kiskac karti tarafi
  *          Mesaj formatlari: gripper_can_defs.h (master ile ortak)
  *
  *  10 ms kontrol dongusunde cagri sirasi:
  *    CanProto_ProcessRx()   -> komutlari isle   (Gripper_Task'tan ONCE)
  *    Gripper_Task(), ToF_Task()
  *    CanProto_SendToF()     -> yeni ToF olcumu varsa
  *    CanProto_Tx()          -> durum / tani     (Gripper_Task'tan SONRA)
  ******************************************************************************
  */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include "tof_sensor.h"

/* Debugger'dan izlemek icin (Live Expressions: can_proto_dbg) */
typedef struct
{
  uint32_t cmd_frames;     /* alinan komut cercevesi (tekrarlar dahil)     */
  uint32_t executed;       /* calistirilan komut (seq degisti)             */
  uint32_t rejected;       /* calistirilip reddedilen                      */
  uint32_t resyncs;        /* acilis / iletisim donusu senkronizasyonu     */
  uint32_t comm_losses;    /* master'in CAN_COMM_TIMEOUT_MS sustugu an     */
  uint8_t  last_cmd;
  uint8_t  last_seq;
} CanProtoDebug_t;

extern volatile CanProtoDebug_t can_proto_dbg;

void CanProto_Init(bool can_ok);
void CanProto_ProcessRx(void);
void CanProto_Tx(void);
void CanProto_SendToF(const ToF_Frame_t *frame);
bool CanProto_TakeTofReinitRequest(void);

#endif /* CAN_PROTOCOL_H */
