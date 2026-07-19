/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_THRESHOLD       100        // Wyślij CAN gdy ADC == 0
#define CAN_TX_ID_ADC       0x100    // ID ramki ADC (normalny pomiar)
#define CAN_TX_ID_ZERO      0x100    // ID ramki gdy ADC == 0 (te same, dane 0x0000)
#define CAN_HIGH_PRIO_MAX   0x0FF    // Ramki z ID <= tego są "wyższy priorytet"
#define LED_TOGGLE_MS       200      // Okres migania LED [ms]
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
static volatile uint32_t adcValue = 0;
static volatile uint8_t  rxFlag   = 0;  // ustawiana przez callback RX
static volatile uint8_t rxHighPrio = 0; // High priority message received flag

static struct CAN_scheduledMsgList canScheduler = {0};

typedef struct {
    uint16_t ErrorCode;
    uint8_t  Severity;
    uint8_t  Node_Execution_Halted;
    uint8_t  Reserved;
    uint64_t Error_Specific_Data;
} SafeState_t;

static SafeState_t safeStateData = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void ADC_getData(uint8_t *data, void *context)
{
    (void)context;
    uint16_t val = adcValue;
    data[0] = (val >> 8) & 0xFF;  // High byte
    data[1] =  val       & 0xFF;  // Low byte
}
static void SafeState_getData(uint8_t *data, void *context)
{
    (void)context;

    // Bajty 0-1: ErrorCode
    data[0] =  safeStateData.ErrorCode       & 0xFF;
    data[1] = (safeStateData.ErrorCode >> 8) & 0xFF;

    // Bajt 2: bits 16-23
    // [3:0] = Reserved, [6:4] = Severity, [7] = Node_Execution_Halted
    data[2] =  (safeStateData.Reserved            & 0x0F)
             | ((safeStateData.Severity            & 0x07) << 4)
             | ((safeStateData.Node_Execution_Halted & 0x01) << 7);

    // Bajty 3-7: Error_Specific_Data (40 bitów)
    data[3] =  safeStateData.Error_Specific_Data        & 0xFF;
    data[4] = (safeStateData.Error_Specific_Data >>  8) & 0xFF;
    data[5] = (safeStateData.Error_Specific_Data >> 16) & 0xFF;
    data[6] = (safeStateData.Error_Specific_Data >> 24) & 0xFF;
    data[7] = (safeStateData.Error_Specific_Data >> 32) & 0xFF;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
         Error_Handler();

     CAN_init(&hcan);

     if (HAL_ADC_Start(&hadc1) != HAL_OK)
         Error_Handler();

     // Ramka ADC 0x100 — wartość ADC co 2000ms
     struct CAN_scheduledMsg adcMsg = {
         .header = {
             .StdId              = 0x100,
             .ExtId              = 0,
             .IDE                = CAN_ID_STD,
             .RTR                = CAN_RTR_DATA,
             .DLC                = 2,
             .TransmitGlobalTime = DISABLE,
         },
         .periodMs = 2000,
         .getData  = ADC_getData,
         .context  = NULL,
     };
     if (CAN_addScheduledMessage(adcMsg, &canScheduler) != HAL_OK)
         Error_Handler();

     // Ramka SafeState ID=1 — stan systemu co 2000ms
     struct CAN_scheduledMsg safeStateMsg = {
         .header = {
             .StdId              = 0,
             .ExtId              = 0,
             .IDE                = CAN_ID_STD,
             .RTR                = CAN_RTR_DATA,
             .DLC                = 8,
             .TransmitGlobalTime = DISABLE,
         },
         .periodMs = 1000,
         .getData  = SafeState_getData,
         .context  = NULL,
     };
     if (CAN_addScheduledMessage(safeStateMsg, &canScheduler) != HAL_OK)
         Error_Handler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint32_t lastLedToggle = 0;
  HAL_GPIO_WritePin(GPIOA, control_status_Pin, GPIO_PIN_SET);
  while (1)
  {


	  if(HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	  {
		  adcValue = HAL_ADC_GetValue(&hadc1);
	  }
	  safeStateData.ErrorCode = (adcValue < ADC_THRESHOLD) ? 1 : 0;//zaleznie od tego ustawimy stan pracy
	  CAN_handleScheduled(&hcan, &canScheduler);
	  if(adcValue < ADC_THRESHOLD)
	  {
		  CAN_handleScheduled(&hcan, &canScheduler);
	  }
	  CAN_handleScheduled(&hcan, &canScheduler);

	  if(HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	  {
	      adcValue = HAL_ADC_GetValue(&hadc1);
	  }

	  safeStateData.ErrorCode = (adcValue < ADC_THRESHOLD) ? 1 : 0;
	  if(adcValue < ADC_THRESHOLD)
	  {
	      CAN_handleScheduled(&hcan, &canScheduler);
	  }
	  if(rxHighPrio)
	  {
		  rxHighPrio = 0;//kasujemy flage
          HAL_GPIO_WritePin(GPIOA, control_status_Pin, GPIO_PIN_RESET);
	  }
      uint32_t now = HAL_GetTick();
      if ((now - lastLedToggle) >= LED_TOGGLE_MS)
      {
          lastLedToggle = now;
          HAL_GPIO_TogglePin(GPIOA, LED_Pin);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV16;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, control_status_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : control_status_Pin LED_Pin */
  GPIO_InitStruct.Pin = control_status_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanPtr)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    HAL_CAN_GetRxMessage(hcanPtr, CAN_RX_FIFO0, &rxHeader, rxData);

    // Odczytaj ErrorCode z bajtów 0-1 (little-endian)
    uint16_t errorCode = (uint16_t)rxData[0] | ((uint16_t)rxData[1] << 8);

    if (errorCode != 0)
    {
        rxHighPrio = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
