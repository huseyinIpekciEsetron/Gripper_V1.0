# Gripper_V1.0 — Kıskaç kontrol kartı

Bu dosya Claude Code için proje bağlamıdır. Kullanıcıyla Türkçe konuş.

## Sistem

- **Kart:** STM32F103CBT6 (128 KB flash), HSE 12 MHz × 6 = 72 MHz, APB1 36 MHz.
- **Motor sürücü:** VNH7100BASTR. PA1 = INA, PA2 = INB, PA4 = SEL0, PA3 = PWM (TIM2_CH4, 20 kHz → ARR 3599), PA0 = CS akım ölçümü (ADC1_IN0, DMA circular). R2 (Rsense) = 4.7 kΩ.
- **Mesafe sensörü:** Pololu VL53L8CX taşıyıcı kartı, I2C1 (PB6 = SCL, PB7 = SDA), 4×4 bölge, 10 Hz. Açılışta sensöre 86 KB firmware yükleniyor.
- **CAN:** 250 kbit/s, PA11/PA12, TCAN337. Aynı hatta robot kol motorları (CANopen, node 10/20/30/40) ve master kart (ayrı repo: RoboticArm_V2) var. Kıskaç ID'leri 0x680–0x693 (protokol: `Core/Inc/gripper_can_defs.h`, master ile birebir aynı olmalı).

## Kod yapısı (Core/)

| Dosya | Görev |
|---|---|
| `app_config.h` | Ayarlanabilir tüm parametreler |
| `vnh7100.c/h` | Düşük seviye motor sürücü (yön + SEL0 tek BSRR yazması, PWM, akım) |
| `gripper.c/h` | Kıskaç durum makinesi: IDLE → STARTING → MOVING → STOPPING; rampa, korluk süresi, zorlanma (stall), yön kilidi, aşırı akım, zaman aşımı, FAULT kurtarma |
| `platform.c/h` | ST VL53L8CX kütüphanesinin I2C katmanı + hat kurtarma + teşhis |
| `tof_sensor.c/h` | Sensör başlatma / okuma, geçerli bölge ayıklama |
| `can_comm.c/h` | bxCAN: bit zamanlaması (PCLK1'e göre otomatik), filtre, TX kuyrukları, RX halka tamponu |
| `can_protocol.c/h` | Komut (seq tabanlı, tek sefer çalıştırma), durum / tanı / mesafe mesajları |
| `vl53l8cx_api.*`, `vl53l8cx_buffers.h` | ST'nin kütüphanesi, DEĞİŞTİRME |

Ana döngü `main.c`'de: her turda `CanComm_Task()`; her 10 ms'de `CanProto_ProcessRx → ToF_Task → CanProto_SendToF → CanProto_Tx`.
Kıskaç kontrolü (`Gripper_Task`: durum makinesi + akım korumaları) ana döngüde değil, SysTick kesmesinde
her 10 ms'de çalışır (`Gripper_SysTick`, `stm32f1xx_it.c`). Böylece ana döngü I2C'de (ToF) beklerken de
korumalar zamanında çalışır. Kesme içinde `HAL_Delay` kullanılamaz (`VNH7100_ResetLatch` DWT ile bekler).
`Gripper_Command` / `ClearFault` / `GetStatus` kendi kritik bölgelerinde çalışır.

## Kurallar ve bilinen tuzaklar

- `main.c`, `stm32f1xx_hal_msp.c`, `stm32f1xx_it.c` CubeMX tarafından üretiliyor. Kendi kodunu SADECE `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` arasına yaz, yoksa kod üretiminde silinir.
- **TIM2 Clock Source = Internal Clock** CubeMX'te seçili olmalı. Seçili değilse MSP'de `HAL_TIM_Base_MspInit` yerine `HAL_TIM_PWM_MspInit` üretiliyor, TIM2 saatsiz kalıyor, PWM çıkmıyor ve motor dönmüyor. Koruma olarak `USER CODE BEGIN TIM2_Init 0` içinde `__HAL_RCC_TIM2_CLK_ENABLE()` var ve `VNH7100_Init` timer çalışmıyorsa hata döndürüyor.
- **TIM2 Period 3599 olmalı** (72 MHz'de 20 kHz). 3199 = 22.5 kHz, VNH7100'ün 20 kHz sınırını aşıyor. Güncel `.ioc`'u kontrol et.
- CAN RX0 kesmesi (USB_LP_CAN1_RX0) NVIC'te açık olmalı.
- Her değişiklikten sonra derlemenin uyarısız geçtiğinden emin ol.

## Şu anki iş: I2C slip ring sorunu

**Belirti:** Sensör 20 cm kabloyla çalışıyor. 40 cm kablo + slip ring ile I2C hiç çalışmıyor.

**Kullanıcının ölçümleri:** Sensöre 3.3 V geliyor. SCL ve SDA boştayken 3.3 V. GND sürekliliği sağlam. Pull-up'lar 2.2 kΩ (3.3 V'a). Kablolar doğru bağlı.

**`platform_i2c_stats` (100 kHz'de):** `retries = 12`, `recoveries = 12`, `failures = 4`, `last_error = 0x20` (HAL_I2C_ERROR_TIMEOUT), `last_hal_status = 1` (HAL_ERROR). Yani `is_alive`'ın 4 işleminin hiçbiri 3 tekrarda bile başarılı olmadı. TIMEOUT (NACK değil) → STM32, SCL'yi bıraktıktan sonra hattın yukarı çıktığını göremiyor. DC seviyeler doğru olduğu için şüpheliler: SCL–SDA karışması (crosstalk), yavaş kenarlar (kapasitans) ya da slip ring kontak gürültüsü.

**Repoda zaten olanlar:** I2C 100 kHz (Standard mode). `platform.c`'de her işlemde hat kurtarma + 3 tekrar, 256 byte'lık firmware parçaları, hıza göre hesaplanan timeout, `platform_i2c_stats` sayaçları.

**Uygulanmayı bekleyen iki patch** (kullanıcı repo kök klasörüne koyacak):
1. `0001-I2C-teshis-hat-kurtarmada-SCL-SDA-bosta-seviyelerini.patch`: `platform_i2c_stats`'a `scl_idle`, `sda_idle`, `sda_after_recover`, `scl_low_count`, `sda_low_count` ekler. BusDelay'i ~10 µs'ye uzatır.
2. `0002-I2C-teshis-STM32-I2C-donanimi-olmadan-6-kHz-bit-bang.patch`: `Platform_I2C_Probe()` ekler. Bu fonksiyon `MX_I2C1_Init` içinde (USER CODE I2C1_Init 0) çağrılıyor ve STM32'nin I2C donanımını kullanmadan, pinleri yazılımla ~6 kHz'de sürerek sensörün kimlik kayıtlarını okuyor. Sonuç `i2c_probe` değişkeninde: `device_id` 0xF0, `revision_id` 0x0C bekleniyor.

Uygulama: `git am 0001-*.patch` sonra `git am 0002-*.patch`.

**Sonraki adım:** Kullanıcı kartı yükleyip `i2c_probe` değerlerini söyleyecek.
- `ok = 1` → sensör yavaşta çalışıyor, sorun hız / karışma. Önce I2C'yi 50 kHz'e düşür (firmware yüklemesi ~15 s sürer). Kablo tarafında SCL ve SDA'yı ayrı ayrı GND ile bükmek ve slip ringde SCL–GND–SDA sırası önerildi.
- `ack_page0 = 0` → sinyal yavaşta bile geçmiyor. Aynı kabloyu slip ringi atlayarak dene, sorunun kabloda mı slip ringde mi olduğunu ayır.
- ACK'ler 1 ama `device_id` yanlış → bitler bozuluyor (karışma / kontak gürültüsü).
- `scl_stuck = 1` → bırakılan SCL yukarı çıkmıyor.

**Kalıcı çözüm seçenekleri** (kullanıcıya önerildi): (a) Kıskaç kartını sensörün yanına taşıyıp slip ringden sadece 12 V + GND + CAN_H + CAN_L geçirmek (kartın P1 konnektörü tam bunun için uygun). (b) Slip ringin iki tarafına PCA9615 (diferansiyel I2C) koymak.
