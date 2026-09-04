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
#include "app_sync_tick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_THRESHOLD         100      // Ponizej tej wartosci ADC -> podnapiecie
#define ADC_HYSTERESIS        20       // Powrot do normy dopiero powyzej THRESHOLD+HYST
#define LED_TOGGLE_MS         200      // Okres migania LED [ms]

/* DIAGNOSTYKA RX - przelacznik do bisekcji problemu z callbackiem:
 *   1 = przyjmuj KAZDA ramke z magistrali (filtr wylaczony)
 *   0 = tryb docelowy, tylko SAFE_STATE_ACTIV_ID
 * Jesli przy 1 callback wchodzi, a przy 0 nie - CANtool wysyla ramke
 * w zlym formacie (Extended zamiast Standard albo Remote zamiast Data). */
#define CAN_RX_PROMISCUOUS    1

/* Ramka SafeState_NODE - heartbeat wezla. ID i okres wg CAN_DB.dbc
 * (BO_ 3, GenMsgCycleTime 5000). Leci bezwarunkowo, bez kodow bledow. */
#define CAN_TX_ID_SAFESTATE   SAFE_STATE_NODE_ID
#define CAN_PERIOD_SS_MS      5000

/* Ramka SafeState_Activ - dokladana do schedulera na czas podnapiecia i
 * usuwana po powrocie do normy. Wg CAN_DB.dbc (BO_ 1) ma 8 bajtow i nie ma
 * zdefiniowanych sygnalow, dlatego payload jest zerowy.
 * DBC podaje GenMsgCycleTime 0 (ramka zdarzeniowa) - okres ponizej jest
 * naszym wyborem i wymaga uzgodnienia z zespolem. */
#define CAN_TX_ID_SS_ACTIV      SAFE_STATE_ACTIV_ID
#define CAN_DLC_SS_ACTIV        8
#define CAN_PERIOD_SS_ACTIV_MS  5000

/* Watchdog komendy throttle od JETSONa (BO_ 550 / BO_ 551).
 * Ramka z THROTTLE_ARM_VALUE uzbraja i odswieza stoper. Gdy przez
 * THROTTLE_TIMEOUT_MS nie przyjdzie kolejna, wysylamy te sama ramke
 * z THROTTLE_SAFE_VALUE i rozbrajamy stoper do nastepnego uzbrojenia. */
#define THROTTLE_DLC            8
#define THROTTLE_ARM_VALUE      1      // wartosc, ktora uzbraja stoper
#define THROTTLE_SAFE_VALUE     (-1)   // wysylane po przekroczeniu czasu
#define THROTTLE_TIMEOUT_MS     150
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
static volatile uint8_t rxHighPrio = 0; // High priority message received flag

/* Liczniki diagnostyczne - do podgladu debuggerem */
static volatile uint32_t canBusOffCount = 0;
static volatile uint32_t canErrorCount  = 0;

/* Stan detekcji podnapiecia (z histereza) */
static uint8_t adcUnderVoltage = 0;             // 1 = jestesmy ponizej progu
static volatile uint32_t ssActivCount = 0;      // ile razy weszlismy w podnapiecie

/* Diagnostyka RX - co faktycznie przyszlo z magistrali */
static volatile uint32_t canRxCount   = 0;
static volatile uint32_t canRxLastId  = 0;
static volatile uint8_t  canRxLastIde = 0;  // 0 = Standard, 4 = Extended
static volatile uint8_t  canRxLastRtr = 0;  // 0 = Data,     2 = Remote
static volatile uint8_t  canRxLastDlc = 0;

/* Watchdog komendy throttle - jeden wpis na silnik.
 * Pola pisane sa z przerwania RX, czytane w petli glownej -> volatile. */
typedef struct {
    uint16_t          id;             // ID monitorowanej ramki
    volatile uint8_t  armed;          // 1 = stoper biegnie
    volatile uint32_t lastRxTick;     // moment ostatniej ramki uzbrajajacej
    volatile uint32_t timeoutCount;   // diagnostyka: ile razy zadzialal
} ThrottleWatchdog_t;

static ThrottleWatchdog_t throttleWd[] = {
    { .id = JETSON_ENGINE_LEFT_RPDO1_ID  },
    { .id = JETSON_ENGINE_RIGHT_RPDO1_ID },
};
#define THROTTLE_WD_COUNT ((uint8_t)(sizeof(throttleWd) / sizeof(throttleWd[0])))

#if !CAN_RX_PROMISCUOUS
/* ID przyjmowane przez ten wezel - pozostale ramki odrzuca sprzet,
 * wiec callback RX w ogole sie dla nich nie uruchamia */
static const uint16_t canAcceptedIds[] = {
    SAFE_STATE_ACTIV_ID,            // 1   - aktywacja stanu bezpiecznego
    JETSON_ENGINE_LEFT_RPDO1_ID,    // 550 - throttle lewy
    JETSON_ENGINE_RIGHT_RPDO1_ID,   // 551 - throttle prawy
};
#define CAN_ACCEPTED_ID_COUNT ((uint8_t)(sizeof(canAcceptedIds) / sizeof(canAcceptedIds[0])))
#endif

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


/**
  * @brief Pakuje ramke SafeState_NODE wg CAN_DB.dbc (BO_ 3)
  *
  * ErrorCode 0|16@1+ | Reserved 16|4@1+ | Severity 20|3@1+
  * Node_Execution_Halted 23|1@1+ | Error_Specific_Data 24|40@1+
  */
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

#if CAN_RX_PROMISCUOUS
  if (CAN_init(&hcan, NULL, 0) != HAL_OK)          // filtr wylaczony - patrz CAN_RX_PROMISCUOUS
#else
  if (CAN_init(&hcan, canAcceptedIds, CAN_ACCEPTED_ID_COUNT) != HAL_OK)
#endif
      Error_Handler();

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
      Error_Handler();

  // Ramka SafeState_NODE - heartbeat wezla, leci bezwarunkowo
  struct CAN_scheduledMsg safeStateMsg = {
      .header = {
          .StdId              = CAN_TX_ID_SAFESTATE,
          .ExtId              = 0,
          .IDE                = CAN_ID_STD,
          .RTR                = CAN_RTR_DATA,
          .DLC                = 8,
          .TransmitGlobalTime = DISABLE,
      },
      .periodMs = CAN_PERIOD_SS_MS,
      .getData  = SafeState_getData,
      .context  = NULL,
  };
  if (CAN_addScheduledMessage(safeStateMsg, &canScheduler) != HAL_OK)
      Error_Handler();


  SyncTickTest_Init(&canScheduler);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  /* Szablon ramki SafeState_Activ. Nie jest rejestrowana od razu - trafia do
   * schedulera dopiero na czas podnapiecia. getData = NULL, bo payload ma byc
   * zerowy, a CAN_handleScheduled zeruje bufor przed wyslaniem. */
  struct CAN_scheduledMsg ssActivMsg = {
      .header = {
          .StdId              = CAN_TX_ID_SS_ACTIV,
          .ExtId              = 0,
          .IDE                = CAN_ID_STD,
          .RTR                = CAN_RTR_DATA,
          .DLC                = CAN_DLC_SS_ACTIV,
          .TransmitGlobalTime = DISABLE,
      },
      .periodMs = CAN_PERIOD_SS_ACTIV_MS,
      .getData  = NULL,
      .context  = NULL,
  };

  uint32_t lastLedToggle = 0;
  HAL_GPIO_WritePin(GPIOA, control_status_Pin, GPIO_PIN_SET);
  while (1)
  {
	  // 1. Pomiar ADC (tryb continuous - czekamy max 1 ms na EOC)
	  if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	  {
		  adcValue = HAL_ADC_GetValue(&hadc1);
	  }

	  // 2. Detekcja zbocza podnapiecia. Na czas trwania bledu ramka
	  //    SafeState_Activ dolacza do schedulera i leci cyklicznie obok NODE.
	  //    Histereza chroni magistrale przed lawina, gdy pomiar drga wokol progu.
	  if (!adcUnderVoltage)
	  {
		  if (adcValue < ADC_THRESHOLD)
		  {
			  adcUnderVoltage = 1;
			  ssActivCount++;
			  (void)CAN_addScheduledMessage(ssActivMsg, &canScheduler);
		  }
	  }
	  else
	  {
		  if (adcValue > (ADC_THRESHOLD + ADC_HYSTERESIS))
		  {
			  adcUnderVoltage = 0;
			  (void)CAN_removeScheduledMessage(CAN_TX_ID_SS_ACTIV, &canScheduler);
		  }
	  }

	  uint32_t now = HAL_GetTick();

	  // 3. Watchdog komendy throttle. Stoper uzbraja przerwanie RX; jesli przez
	  //    THROTTLE_TIMEOUT_MS nie przyjdzie kolejna komenda, wysylamy raz
	  //    wartosc bezpieczna i rozbrajamy stoper.
	  for (uint8_t i = 0U; i < THROTTLE_WD_COUNT; i++)
	  {
		  if (!throttleWd[i].armed)
		  {
			  continue;
		  }
		  if ((uint32_t)(now - throttleWd[i].lastRxTick) < THROTTLE_TIMEOUT_MS)
		  {
			  continue;
		  }

		  uint8_t payload[THROTTLE_DLC] = {0};
		  payload[0] = (uint8_t)( (uint16_t)THROTTLE_SAFE_VALUE       & 0xFF);
		  payload[1] = (uint8_t)(((uint16_t)THROTTLE_SAFE_VALUE >> 8) & 0xFF);

		  if (CAN_sendStdFrame(&hcan, throttleWd[i].id,
		                       payload, THROTTLE_DLC) == HAL_OK)
		  {
			  throttleWd[i].armed = 0;
			  throttleWd[i].timeoutCount++;
		  }
	  }

	  // 4. Wysylka ramek okresowych (o tempie decyduje periodMs, nie liczba wywolan)
	  CAN_handleScheduled(&hcan, &canScheduler);

	  // 5. Reakcja na odebrana ramke wysokiego priorytetu
	  if (rxHighPrio)
	  {
		  rxHighPrio = 0;
		  HAL_GPIO_WritePin(GPIOA, control_status_Pin, GPIO_PIN_RESET);
	  }

	  // 6. Heartbeat LED
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
  hcan.Init.AutoBusOff = ENABLE;
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
/**
  * @brief Callback ramki odebranej w FIFO0
  *
  * W trybie docelowym (CAN_RX_PROMISCUOUS = 0) filtr sprzetowy przepuszcza
  * wylacznie SAFE_STATE_ACTIV_ID, wiec kazda ramka podnosi flage bez
  * sprawdzania ID. Przy CAN_RX_PROMISCUOUS = 1 wchodza tu wszystkie ramki -
  * sluzy to wylacznie diagnostyce.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanPtr)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[CAN_MAX_DLC];

    if (HAL_CAN_GetRxMessage(hcanPtr, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
        return;
    }

    /* Slad dla debuggera - pokazuje, co naprawde wyslal CANtool */
    canRxCount++;
    canRxLastId  = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
    canRxLastIde = (uint8_t)rxHeader.IDE;
    canRxLastRtr = (uint8_t)rxHeader.RTR;
    canRxLastDlc = (uint8_t)rxHeader.DLC;

    /* Interesuja nas wylacznie standardowe ramki danych */
    if (rxHeader.IDE != CAN_ID_STD || rxHeader.RTR != CAN_RTR_DATA)
    {
        return;
    }

    /* Aktywacja stanu bezpiecznego */
    if (rxHeader.StdId == SAFE_STATE_ACTIV_ID)
    {
        rxHighPrio = 1;
        return;
    }

    /* Komenda throttle - uzbrojenie / odswiezenie stopera */
    for (uint8_t i = 0U; i < THROTTLE_WD_COUNT; i++)
    {
        if (rxHeader.StdId != throttleWd[i].id)
        {
            continue;
        }

        if (rxHeader.DLC >= 2U)
        {
            /* Sygnal 0|16@1- : int16, little endian */
            int16_t throttle = (int16_t)((uint16_t)rxData[0] |
                                        ((uint16_t)rxData[1] << 8));

            if (throttle == THROTTLE_ARM_VALUE)
            {
                throttleWd[i].lastRxTick = HAL_GetTick();
                throttleWd[i].armed      = 1;
            }
        }
        return;
    }
}

/**
  * @brief Callback bledow magistrali CAN (warning / passive / bus-off)
  *
  * Przy bus-off sprzet odzyska magistrale sam (ABOM = ENABLE), ale ramki
  * wiszace w mailboxach TX moglyby byc juz nieaktualne - przerywamy je,
  * scheduler i tak wysle swieze dane w kolejnym obiegu petli.
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcanPtr)
{
    canErrorCount++;

    if (hcanPtr->ErrorCode & HAL_CAN_ERROR_BOF)
    {
        canBusOffCount++;
        HAL_CAN_AbortTxRequest(hcanPtr,
            CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }

    hcanPtr->ErrorCode = HAL_CAN_ERROR_NONE;
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
