/**
  ******************************************************************************
  * @file    gripper_can_defs.h
  * @brief   Kiskac <-> Master CAN protokolu - ORTAK TANIMLAR
  *          Bu dosya kiskac kartinda VE master kartta birebir ayni olmali.
  *
  *  Hat: robot kol motorlariyla ortak CAN hatti
  *       CAN 2.0A (11-bit ID), 250 kbit/s, little-endian
  *
  *  ID secimi:
  *   Motorlar CANopen (SDO 0x580+id / 0x600+id, id = 10/20/30/40). CANopen'in
  *   hicbir node icin kullanmadigi 0x680-0x6DF araligi secildi:
  *    - hicbir motor bu ID'lere cevap vermez, bu ID'lerle mesaj uretmez
  *    - motor SDO'larindan (0x58A..0x628) dusuk oncelikli: master'in motor
  *      mesajlari kiskac trafigine karsi ASLA arbitrasyon kaybetmez
  *
  *  ID      Yon              Periyot                 Icerik
  *  --------------------------------------------------------------------------
  *  0x680   master->kiskac   50 ms + yeni komutta    KOMUT (ayni zamanda canlilik)
  *  0x681   kiskac->master   50 ms + degisimde       DURUM
  *  0x682   kiskac->master   1 s                     TANI
  *  0x683   kiskac->master   her ToF olcumunde       TOF BASLIK
  *  0x684+s kiskac->master   her ToF olcumunde       TOF VERI, s = 0..15
  ******************************************************************************
  */
#ifndef GRIPPER_CAN_DEFS_H
#define GRIPPER_CAN_DEFS_H

#include <stdint.h>

/* ============================ ID'LER ===================================== */
#define GCAN_BASE_ID               0x680U
#define GCAN_ID_CMD                (GCAN_BASE_ID + 0U)
#define GCAN_ID_STATUS             (GCAN_BASE_ID + 1U)
#define GCAN_ID_DIAG               (GCAN_BASE_ID + 2U)
#define GCAN_ID_TOF_HDR            (GCAN_BASE_ID + 3U)
#define GCAN_ID_TOF_DATA(seg)      (GCAN_BASE_ID + 4U + (uint16_t)(seg))
#define GCAN_ID_LAST               GCAN_ID_TOF_DATA(15U)          /* 0x693 */

/* Master filtresi: 0x680-0x69F (maske 0x7E0) kiskac mesajlarini yakalar */
#define GCAN_FILTER_ID             0x680U
#define GCAN_FILTER_MASK           0x7E0U

/* ============================ KOMUT (0x680) ============================== */
/*  byte 0 : komut (GCAN_CMD_*)
 *  byte 1 : sira no (seq)
 *
 *  Master bu cerceveyi SUREKLI (50 ms) gonderir. Kiskac bir komutu sadece
 *  seq DEGISTIGINDE bir kez calistirir; ayni seq'in tekrari sadece canlilik.
 *  Yeni komut = seq + 1.
 *
 *  Kiskac acildiginda veya haberlesme kopup geri geldiginde ilk gelen
 *  cercevenin seq'ine sadece senkronize olur, komutu CALISTIRMAZ
 *  (reset sonrasi eski bir KAPA komutuyla kendiliginden hareket etmesin).
 *
 *  Kiskac 500 ms komut cercevesi alamazsa hareketi durdurur.            */
#define GCAN_CMD_NOP               0U
#define GCAN_CMD_OPEN              1U
#define GCAN_CMD_CLOSE             2U
#define GCAN_CMD_STOP              3U
#define GCAN_CMD_CLEAR_FAULT       4U
#define GCAN_CMD_TOF_REINIT        5U   /* sadece motor dururken, 1-2 s */

/* ============================ DURUM (0x681) ============================== */
/*  byte 0 : state        (GCAN_STATE_*)
 *  byte 1 : motion       (GCAN_MOTION_*)  su anki / son hareket
 *  byte 2 : stop_reason  (GCAN_STOP_*)
 *  byte 3 : flags        (GCAN_FLAG_*)
 *  byte 4-5 : filtreli motor akimi [mA]
 *  byte 6 : PWM duty [%]
 *  byte 7 : son CALISTIRILAN komutun seq degeri (ack)                      */
#define GCAN_STATE_IDLE            0U
#define GCAN_STATE_STOPPING        1U
#define GCAN_STATE_STARTING        2U
#define GCAN_STATE_MOVING          3U
#define GCAN_STATE_FAULT           4U

#define GCAN_MOTION_NONE           0U
#define GCAN_MOTION_OPEN           1U
#define GCAN_MOTION_CLOSE          2U

#define GCAN_STOP_NONE             0U
#define GCAN_STOP_USER             1U
#define GCAN_STOP_STALL            2U   /* akim limiti: nesne / uc nokta */
#define GCAN_STOP_TIMEOUT          3U
#define GCAN_STOP_NO_LOAD          4U
#define GCAN_STOP_OVERCURRENT      5U
#define GCAN_STOP_DRIVER_FAULT     6U

#define GCAN_FLAG_CURRENT_LIMIT    (1U << 0)  /* akim limitinde durdu, bekliyor */
#define GCAN_FLAG_BLOCKED_OPEN     (1U << 1)  /* acma komutu reddedilir  */
#define GCAN_FLAG_BLOCKED_CLOSE    (1U << 2)  /* kapama komutu reddedilir */
#define GCAN_FLAG_FAULT_LATCHED    (1U << 3)  /* CLEAR_FAULT gerekir      */
#define GCAN_FLAG_MOVING           (1U << 4)
#define GCAN_FLAG_TOF_OK           (1U << 5)
#define GCAN_FLAG_COMM_TIMEOUT     (1U << 6)  /* master sustugu icin durdu */
#define GCAN_FLAG_CMD_REJECTED     (1U << 7)  /* son komut reddedildi     */

/* ============================ TANI (0x682) =============================== */
/*  byte 0-1 : son hareketteki tepe akim [mA]
 *  byte 2-3 : son hareketin suresi [ms]
 *  byte 4   : toplam ariza sayisi
 *  byte 5   : ToF state (bit0-1) | ToF son hata (bit2-4)
 *  byte 6   : GCAN_DIAG_* bayraklari
 *  byte 7   : firmware surumu                                               */
#define GCAN_DIAG_BUS_OFF_SEEN     (1U << 0)
#define GCAN_DIAG_ERROR_PASSIVE    (1U << 1)
#define GCAN_DIAG_TX_DROPPED       (1U << 2)
#define GCAN_DIAG_RX_DROPPED       (1U << 3)
#define GCAN_DIAG_XCVR_FAULT_PIN   (1U << 4)  /* TCAN337 FAULT ham seviye */

/* ============================ TOF ======================================== */
/*  BASLIK (0x683):
 *   byte 0   : olcum sayaci (8-bit, her olcumde +1)
 *   byte 1   : bolge sayisi (16 veya 64)
 *   byte 2   : veri segment sayisi (= bolge / 4)
 *   byte 3   : gecerli bolge sayisi
 *   byte 4-5 : en yakin gecerli mesafe [mm], 0 = yok
 *   byte 6   : en yakin bolge, 0xFF = yok
 *   byte 7   : ToF state (bit0-1) | son hata (bit2-4)
 *
 *  VERI (0x684 + s): 4 bolge x uint16 [mm], bolge = s*4 + 0..3, 0 = gecersiz
 *
 *  Once baslik, sonra segmentler sirayla. Alici baslikta tamponu sifirlar,
 *  tum segmentler gelince olcum tamamlanir.                                */
#define GCAN_TOF_ZONES_PER_SEG     4U
#define GCAN_TOF_MAX_SEGS          16U
#define GCAN_TOF_MAX_ZONES         64U

/* ============================ YARDIMCILAR ================================ */
static inline void GCan_PutU16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t GCan_GetU16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t GCan_SatU16(uint32_t v)
{
  return (v > 0xFFFFU) ? (uint16_t)0xFFFFU : (uint16_t)v;
}

static inline uint8_t GCan_SatU8(uint32_t v)
{
  return (v > 0xFFU) ? (uint8_t)0xFFU : (uint8_t)v;
}

#endif /* GRIPPER_CAN_DEFS_H */
