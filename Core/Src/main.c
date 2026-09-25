/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : VNH7100 kiskac kontrol karti
  ******************************************************************************
  * Donanim (STM32F103C8T6, 64 MHz HSI):
  *   PA0 -> CS (ADC1_IN0) | PA1 -> INA | PA2 -> INB | PA3 -> PWM (TIM2_CH4)
  *   PA4 -> SEL0
  *   PB6 -> I2C1 SCL (TX1 test noktasi) | PB7 -> I2C1 SDA (RX1 test noktasi)
  *   PA11 -> CAN RX | PA12 -> CAN TX | PA10 -> TCAN337 FAULT
  *
  * Moduller:
  *   app_config.h  : tum ayarlar (esikler, hizlar, sureler)
  *   vnh7100.c/h   : dusuk seviye surucu (PWM, yon, akim olcumu)
  *   gripper.c/h   : kiskac durum makinesi ve korumalar
  *   watchdog.c/h  : IWDG
  *   tof_sensor.c/h: VL53L8CX uygulama katmani (platform.c + ST ULD API)
  *   can_comm.c/h  : bxCAN dusuk seviye (kuyruklar, filtre, bit zamanlamasi)
  *   can_protocol.c/h + gripper_can_defs.h : kiskac CAN protokolu (master ile ortak)
  *
  * Debugger (Live Expressions) ile kullanim:
  *   dbg_cmd         : 1 = AC, 2 = KAPA, 3 = DUR  (islenince 0'a doner)
  *   dbg_cmd_result  : son komut kabul edildi mi
  *   dbg_clear_fault : true yap -> ariza temizlenir
  *   grip_status     : durum, akim, tepe akim, durma sebebi...
  *   tof_status      : sensor durumu / hata sebebi
  *   tof_frame       : son olcum (distance_mm[], valid_mask, min_distance_mm)
  *   dbg_tof_reinit  : true yap -> sensor bastan kurulur (sadece motor dururken)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "app_config.h"
#include "vnh7100.h"
#include "gripper.h"
#include "watchdog.h"
#include "platform.h"
#include "tof_sensor.h"
#include "can_comm.h"
#include "can_protocol.h"
#include "gripper_can_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSCLK_EXPECTED_HZ   64000000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static const VNH7100_Config_t vnh_cfg =
{
  .hadc        = &hadc1,
  .htim        = &htim2,
  .tim_channel = TIM_CHANNEL_4,
  .port        = GPIOA,
  .pin_ina     = INA_Pin,
  .pin_inb     = INB_Pin,
  .pin_sel0    = sel_Pin,
  .rsense_ohm  = CS_RSENSE_OHM,
  .k_factor    = CS_K_FACTOR,
  .vref_mv     = ADC_VREF_MV,
};

/* Debugger arayuzu (ileride CAN bunlarin yerini alacak) */
volatile GripperCmd_t dbg_cmd = GRIP_CMD_NONE;
volatile bool         dbg_cmd_result = false;
volatile bool         dbg_clear_fault = false;
GripperStatus_t       grip_status;

volatile bool         can_ok = false;
CanStats_t            can_stats;

#if TOF_ENABLE
volatile bool         dbg_tof_reinit = false;
ToF_Status_t          tof_status;
ToF_Frame_t           tof_frame;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t last_tick;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (HAL_RCC_GetSysClockFreq() != SYSCLK_EXPECTED_HZ)
  {
    Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* Once motor: guvenli kapali duruma gelsin */
  if (VNH7100_Init(&vnh_cfg) != HAL_OK)
  {
    Error_Handler();
  }
  Gripper_Init();

#if TOF_ENABLE
  /* Firmware yukleme ~1-2 s bloklar; motor kapaliyken yapiliyor.
   * Sensor bulunamazsa kart yine calisir, tof_status.last_error'a bak. */
  (void)ToF_Init(&hi2c1);
  ToF_GetStatus(&tof_status);
#endif

  /* CAN: sadece master komut ID'si kabul edilir, motor trafigi filtrede elenir.
   * Hata olursa kart yine calisir (can_ok = false). */
  {
    const uint16_t rx_ids[1] = { GCAN_ID_CMD };
    can_ok = (CanComm_Init(&hcan, rx_ids, 1U) == HAL_OK);
    CanProto_Init(can_ok);
  }

#if WATCHDOG_ENABLE
  Watchdog_Init(WATCHDOG_TIMEOUT_MS);
#endif

  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* CAN gonderim kuyrugunu her turda bosalt */
    CanComm_Task();

    /* Debugger komutlari (CAN'e ek olarak) */
    if (dbg_cmd != GRIP_CMD_NONE)
    {
      dbg_cmd_result = Gripper_Command(dbg_cmd);
      dbg_cmd = GRIP_CMD_NONE;
    }
    if (dbg_clear_fault)
    {
      dbg_clear_fault = false;
      (void)Gripper_ClearFault();
    }

#if TOF_ENABLE
    /* Yeniden kurulum 1-2 s bloklar: sadece motor dururken izin ver */
    if (dbg_tof_reinit || CanProto_TakeTofReinitRequest())
    {
      dbg_tof_reinit = false;
      if ((grip_status.state == GRIP_STATE_IDLE) || (grip_status.state == GRIP_STATE_FAULT))
      {
        (void)ToF_Reinit();
        ToF_GetStatus(&tof_status);
        last_tick = HAL_GetTick();   /* kacirilan periyotlari toplu calistirma */
      }
    }
#endif

    /* 10 ms kontrol dongusu */
    if ((HAL_GetTick() - last_tick) >= CTRL_PERIOD_MS)
    {
      last_tick += CTRL_PERIOD_MS;

      /* 1) CAN komutlari  2) kiskac (guvenlik onceligi)  3) sensor  4) CAN gonderim */
      CanProto_ProcessRx();

      Gripper_Task();
      Gripper_GetStatus(&grip_status);

#if TOF_ENABLE
      ToF_Task();
      if (ToF_HasNewFrame())
      {
        (void)ToF_GetFrame(&tof_frame);
        CanProto_SendToF(&tof_frame);
      }
      ToF_GetStatus(&tof_status);
#endif

      CanProto_Tx();
      CanComm_GetStats(&can_stats);

#if WATCHDOG_ENABLE
      /* Sadece kontrol dongusu calisiyorsa besle */
      Watchdog_Kick();
#endif
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSI 8 MHz / 2 x16 = 64 MHz SYSCLK
  *        APB1 = 32 MHz (TIM2 clock = 64 MHz), APB2 = 64 MHz, ADC = 10.67 MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  *        Tek kanal (IN0), surekli donusum, DMA circular
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* DMA "Normal" kalmissa buffer bir kez dolup donar -> circular'a zorla */
  if (hdma_adc1.Init.Mode != DMA_CIRCULAR)
  {
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  *        64 MHz / (0+1) / (3199+1) = 20 kHz
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief I2C1 Initialization Function
  *        400 kHz fast mode, PB6 = SCL, PB7 = SDA
  *        (F1'de analog/dijital filtre fonksiyonlari yok, F4 kodundan cikarildi)
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* F1 errata: takili hat / BUSY bayragi -> init oncesi hat kurtarma */
  Platform_I2C_BusRecover();
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  *        Degerler 64 MHz (PCLK1 32 MHz) icin 250 kbit/s (master ile ayni).
  *        CanComm_Init bit zamanlamasini CAN_BITRATE ve PCLK1'e gore yeniden
  *        ayarlar, HSE 72 MHz'e gecersen de dogru calisir.
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INA_Pin|INB_Pin|sel_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INA_Pin INB_Pin sel_Pin */
  GPIO_InitStruct.Pin = INA_Pin|INB_Pin|sel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_FAULT (PA10), TCAN337 open-drain FAULT cikisi */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Sensor firmware yuklerken (1-2 s) watchdog'un reset atmamasi icin */
void VL53L8CX_IdleHook(void)
{
#if WATCHDOG_ENABLE
  Watchdog_Kick();
#endif
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  /* Guvenli durum: motoru dogrudan register ile kapat */
  TIM2->CCR4 = 0U;
  GPIOA->BSRR = (uint32_t)(INA_Pin | INB_Pin | sel_Pin) << 16;
  /* Watchdog aciksa bir sure sonra MCU resetlenir ve yeniden baslar */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
