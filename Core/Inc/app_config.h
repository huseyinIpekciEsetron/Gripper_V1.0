/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Tum ayarlanabilir parametreler tek yerde.
  *          Kiskaci test ederken sadece bu dosyadaki degerlerle oyna.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================ DONANIM ==================================== */
/* R2 degeri: karttaki fiziksel dirence esit olmali */
#define CS_RSENSE_OHM            4700U
/* VNH7100 akim orani (datasheet tipik 1120). Multimetre ile kalibre edilebilir */
#define CS_K_FACTOR              2200U
#define ADC_VREF_MV              3300U

/* ============================ ZAMANLAMA ================================== */
#define CTRL_PERIOD_MS           10U     /* kontrol dongusu periyodu */

/* ============================ HAREKET ==================================== */
/* Kiskac ters yone gidiyorsa sadece bunu VNH7100_DIR_REVERSE yap */
#define GRIP_CLOSE_DIR           VNH7100_DIR_FORWARD

#define GRIP_CLOSE_DUTY          60U     /* kapanma hizi  %0..100 */
#define GRIP_OPEN_DUTY           60U     /* acilma hizi   %0..100 */
#define GRIP_RAMP_STEP           2U      /* her periyotta duty artisi (%) */
#define GRIP_STOP_DWELL_MS       150U    /* her durustan sonra bekleme (yon degisimi dahil) */

/* ============================ KORUMA ===================================== */
/* Kalkis akimini yok saymak icin hareket basindaki koruma korlugu */
#define GRIP_INRUSH_BLANK_MS     350U

/* Zorlanma esikleri (filtreli akim). Kapanmadaki esik = kavrama kuvveti.
 * Ayar: serbest harekette status.peak_mA'ya bak, esigi bunun ustune koy.   */
#define GRIP_STALL_CLOSE_MA      250U
#define GRIP_STALL_OPEN_MA       250U
#define GRIP_STALL_CONFIRM_MS    50U     /* esik bu kadar asilirsa dur */

/* Bu sure icinde zorlanma gorulmezse dur (tam strok suresinin ~1.5 kati) */
#define GRIP_TRAVEL_TIMEOUT_MS   500U

/* Akim yok: motor bagli degil / kablo kopuk */
#define GRIP_NO_LOAD_MA          20U
#define GRIP_NO_LOAD_CONFIRM_MS  300U

/* Sert akim limiti (filtresiz): hizli durus, o yon kilitlenir (FAULT DEGIL) */
#define HARD_OVERCURRENT_MA      650U
#define HARD_OC_CONFIRM_MS       20U

/* CS doygun (surucu latch'i veya olcum araligi disi) -> FAULT */
#define CS_SAT_CONFIRM_MS        20U

/* FAULT'tan otomatik kurtarma: latch sifirlanir, ters yone gidilebilir.
 * Ust uste bu kadar arizadan sonra kalici FAULT -> dbg_clear_fault gerekir. */
#define GRIP_FAULT_RECOVERY_MS   600U
#define GRIP_FAULT_MAX_RETRIES   3U

/* Akim filtresi: zaman sabiti = CTRL_PERIOD_MS * 2^SHIFT  (3 -> ~80 ms) */
#define CURRENT_FILTER_SHIFT     3U


/* ============================ TOF SENSOR (VL53L8CX) ====================== */
/* 0 yaparsan sensor kodu ve 86 KB firmware derlemeden cikar
 * (motor kodu STM32F103C8'in 64 KB flash'ina yine sigar). */
#define TOF_ENABLE               1


#define TOF_I2C_ADDR             0x52U                    /* 8-bit (0x29 << 1) */
#define TOF_RESOLUTION           VL53L8CX_RESOLUTION_4X4  /* veya _8X8 */
#define TOF_FREQ_HZ              10U     /* 8x8: max 15 Hz, 4x4: max 60 Hz */
#define TOF_STALE_MS             500U    /* bu kadar veri gelmezse ERROR */
#define TOF_MAX_CONSEC_ERRORS    5U      /* ust uste I2C hatasi -> ERROR */

/* I2C1: PB6 = SCL (kartta TX1 test noktasi), PB7 = SDA (RX1 test noktasi) */
#define TOF_I2C_GPIO_PORT        GPIOB
#define TOF_I2C_SCL_PIN          GPIO_PIN_6
#define TOF_I2C_SDA_PIN          GPIO_PIN_7
#define TOF_I2C_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define TOF_I2C_CLK_ENABLE()      __HAL_RCC_I2C1_CLK_ENABLE()
#define TOF_I2C_FORCE_RESET()     __HAL_RCC_I2C1_FORCE_RESET()
#define TOF_I2C_RELEASE_RESET()   __HAL_RCC_I2C1_RELEASE_RESET()



/* ============================ CAN ======================================== */
/* Protokol: gripper_can_defs.h (master ile ORTAK dosya).
 * Robot kol motorlariyla ayni hat -> master ile ayni hiz olmali. */
#define CAN_BITRATE              250000U
#define CAN_STATUS_PERIOD_MS     50U     /* durum mesaji (+ her degisimde hemen) */
#define CAN_DIAG_PERIOD_MS       1000U   /* tani mesaji */
#define CAN_TOF_SEND_ENABLE      1       /* ToF olcumlerinin hepsini CAN'e yolla */
#define CAN_COMM_TIMEOUT_MS      500U    /* master susarsa hareketi durdur, 0 = kapali */
#define FW_VERSION               1U

/* TCAN337 FAULT cikisi (open-drain), PA10, dahili pull-up ile okunur */
#define CAN_XCVR_FAULT_PORT      GPIOA
#define CAN_XCVR_FAULT_PIN       GPIO_PIN_10

/* ============================ WATCHDOG =================================== */
/* Yazilim takilirsa MCU resetlenir, motor durur. Breakpoint'te durur. */
#define WATCHDOG_ENABLE          0
#define WATCHDOG_TIMEOUT_MS      200U    /* LSI toleransi yuzunden gercekte ~130..270 ms */

#endif /* APP_CONFIG_H */
