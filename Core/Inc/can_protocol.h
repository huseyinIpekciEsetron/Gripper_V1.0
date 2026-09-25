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

void CanProto_Init(bool can_ok);
void CanProto_ProcessRx(void);
void CanProto_Tx(void);
void CanProto_SendToF(const ToF_Frame_t *frame);
bool CanProto_TakeTofReinitRequest(void);

#endif /* CAN_PROTOCOL_H */
