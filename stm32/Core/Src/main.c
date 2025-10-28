/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// WAV header structure
typedef struct {
  // RIFF Header
  char riff_header[4]; // Contains "RIFF". Marks the file as a riff file.
  uint32_t wav_size;   // Size of the overall file
  char wave_header[4]; // Contains "WAVE". File Type Header

  // Format Header
  char fmt_header[4]; // Contains "fmt ". Format chunk marker. Includes trailing null
  uint32_t fmt_chunk_size;  // Should be 16 for PCM
  uint16_t audio_format;    // Should be 1 for PCM
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;       //  (Sample Rate * BitsPerSample * Channels) / 8
  uint16_t block_align;     // (BitsPerSample * Channels) / 8
  uint16_t bits_per_sample; // Bits per sample

  // Data Header
  char data_header[4]; // Contains "data". Data chunk marker
  uint32_t data_bytes; // Size of the data section
} WavHeader;

typedef enum {
  ROVER_DO_NOTHING,
  ROVER_FORWARD,
  ROVER_BACKWARD,
  ROVER_RIGHT,
  ROVER_LEFT,
  ROVER_SPIN,
  ROVER_OPEN,
  ROVER_CLOSE,
  ROVER_SPEAK
} RoverCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 4
#define UART_RX_BUFFER_SIZE 200
#define DATA_BUFFER_SIZE 200
#define BIT_0 ( 1 << 0 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId audioTaskHandle;
osThreadId uartParseTaskHandle;
osThreadId roverTaskHandle;
osMessageQId audioQueueHandle;
osMutexId commandMutexHandle;
/* USER CODE BEGIN PV */
EventGroupHandle_t buttonEventHandle;

uint16_t audio_adc[ADC_BUFFER_SIZE];
uint32_t samples_taken = 0;

char uart_rx[UART_RX_BUFFER_SIZE];
bool received_uart_rx = false;

RoverCommand command = ROVER_DO_NOTHING;

const char *BARK_FILE_NAME = "bark.wav";

const uint32_t SAMPLE_RATE = 10000;
const uint16_t BITS_PER_SAMPLE = 16; // Half Word
const float DURATION = 1.0f;
const uint32_t NUMBER_OF_SAMPLES = SAMPLE_RATE * DURATION;
const uint16_t NUMBER_OF_CHANNELS = 1;
const uint32_t DATA_SIZE = NUMBER_OF_SAMPLES * NUMBER_OF_CHANNELS
    * BITS_PER_SAMPLE / 8;
const uint32_t FILE_SIZE = sizeof(WavHeader) + DATA_SIZE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void vAudioTask(void const *argument);
void vUartParseTask(void const *argument);
void vRoverTask(void const *argument);

/* USER CODE BEGIN PFP */
void Set_GPIO_Tmp(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t time_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Log(char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

#define UART_Transmit_ESP32(x) _Generic((x), \
    WavHeader*: HAL_UART_Transmit(&huart1, (uint8_t*) (x), sizeof(WavHeader), HAL_MAX_DELAY), \
    char*: HAL_UART_Transmit(&huart1, (uint8_t*) x, strlen((char*) x), HAL_MAX_DELAY))

static void UART_Transmit_Buffer_ESP32(int16_t *buffer, uint16_t buffer_idx) {
  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, buffer_idx * sizeof(int16_t),
  HAL_MAX_DELAY);
}

static void UART_Receive_ESP32() {
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) uart_rx, UART_RX_BUFFER_SIZE);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_DMA_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  Set_GPIO_Tmp(Rover_Forward_GPIO_Port, Rover_Forward_Pin, 500);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of commandMutex */
  osMutexDef(commandMutex);
  commandMutexHandle = osMutexCreate(osMutex(commandMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of audioQueue */
  osMessageQDef(audioQueue, 30000, int16_t);
  audioQueueHandle = osMessageCreate(osMessageQ(audioQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  buttonEventHandle = xEventGroupCreate();
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of audioTask */
  osThreadDef(audioTask, vAudioTask, osPriorityNormal, 0, 2200);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* definition and creation of uartParseTask */
  osThreadDef(uartParseTask, vUartParseTask, osPriorityNormal, 0, 128);
  uartParseTaskHandle = osThreadCreate(osThread(uartParseTask), NULL);

  /* definition and creation of roverTask */
  osThreadDef(roverTask, vRoverTask, osPriorityNormal, 0, 128);
  roverTaskHandle = osThreadCreate(osThread(roverTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = { 0 };

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Recording_LED_Pin | Rover_Open_Pin | Rover_Close_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
      SD_CS_Pin | Rover_Forward_Pin | Rover_Backward_Pin | Rover_Right_Pin
          | Rover_Left_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Recording_LED_Pin Rover_Open_Pin Rover_Close_Pin */
  GPIO_InitStruct.Pin = Recording_LED_Pin | Rover_Open_Pin | Rover_Close_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Rover_Forward_Pin Rover_Backward_Pin Rover_Right_Pin Rover_Left_Pin */
  GPIO_InitStruct.Pin = Rover_Forward_Pin | Rover_Backward_Pin | Rover_Right_Pin
      | Rover_Left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // If button pressed, set Event flag and start ADC
  if (GPIO_Pin == GPIO_PIN_13) {
    EventBits_t uxBits = xEventGroupGetBitsFromISR(buttonEventHandle);
    if ((uxBits & BIT_0) == 0) { // If Event flag not set
      UART_Log("Recording\n");
      HAL_GPIO_WritePin(Recording_LED_GPIO_Port, Recording_LED_Pin,
          GPIO_PIN_SET); // Turn on recording LED

      HAL_ADC_Start_DMA(&hadc1, (uint32_t*) audio_adc, ADC_BUFFER_SIZE);

      BaseType_t xHigherPriorityTaskWoken = pdFALSE; // TODO: awaken audiotask
      xEventGroupSetBitsFromISR(buttonEventHandle, BIT_0,
          &xHigherPriorityTaskWoken);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  int16_t adc;
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    // Map ADC data to int16_t
    adc = (int16_t) (((uint64_t) audio_adc[i]) * INT16_MAX * 2 / 4095
        + INT16_MIN);

    // Add data to queue
    xQueueSendFromISR(audioQueueHandle, &adc, portMAX_DELAY);

    samples_taken++;
    if (samples_taken == NUMBER_OF_SAMPLES) {
      HAL_ADC_Stop_DMA(hadc);
      UART_Log("Finished recording\n");
      HAL_GPIO_WritePin(Recording_LED_GPIO_Port, Recording_LED_Pin,
          GPIO_PIN_RESET); // Turn off recording LED
      samples_taken = 0;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  received_uart_rx = true;
}

// Creates the WAV header and sends it via UART to ESP32
void Write_WAV_Header() {
  // Fill in WAV header struct
  WavHeader header;
  memcpy(header.riff_header, "RIFF", 4);
  header.wav_size = DATA_SIZE + sizeof(WavHeader) - 8;
  memcpy(header.wave_header, "WAVE", 4);
  memcpy(header.fmt_header, "fmt ", 4);
  header.fmt_chunk_size = 16;
  header.audio_format = 1; // PCM
  header.num_channels = NUMBER_OF_CHANNELS;
  header.sample_rate = SAMPLE_RATE;
  header.bits_per_sample = BITS_PER_SAMPLE;
  header.byte_rate = DATA_SIZE;
  header.block_align = NUMBER_OF_CHANNELS * BITS_PER_SAMPLE / 8;
  memcpy(header.data_header, "data", 4);
  header.data_bytes = DATA_SIZE;

  // Send header to ESP32
  UART_Transmit_ESP32(&header);
}

/**
 * @brief  Sets GPIO output temporarily for time_ms milliseconds.
 * @retval None
 */
void Set_GPIO_Tmp(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t time_ms) {
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
  osDelay(time_ms);
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_vAudioTask */
/**
 * @brief  Function implementing the audioTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_vAudioTask */
void vAudioTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  int16_t data_buffer[DATA_BUFFER_SIZE];
  uint16_t buffer_idx = 0;
  bool sending_file = false; // Resets after the file has been sent
  uint32_t data_sent = 0; // Number of bytes from data batch sent to ESP32

  /* Infinite loop */
  for (;;) {
    EventBits_t uxBits = xEventGroupGetBits(buttonEventHandle);
    if ((uxBits & BIT_0) != 0) {
      if (!sending_file) {
        // Transmit file size
        char uart_buffer[20];
        sprintf(uart_buffer, "%lu\n", FILE_SIZE);
        UART_Transmit_ESP32(uart_buffer);
        // Transmit header of WAV file
        Write_WAV_Header();
        sending_file = true;
      }
      int16_t data;
      while (xQueueReceive(audioQueueHandle, &data, 0)) {
        data_buffer[buffer_idx++] = data;
        UBaseType_t data_left = uxQueueMessagesWaitingFromISR(audioQueueHandle);

        if ((buffer_idx == DATA_BUFFER_SIZE) || (data_left == 0)) {
          // If buffer is full or there is no more data left in queue, then send buffer contents to ESP32
          UART_Transmit_Buffer_ESP32(data_buffer, buffer_idx);
          data_sent += buffer_idx;
          // Empty buffer
          memset(data_buffer, '\0', sizeof(data_buffer));
          buffer_idx = 0;
        }
      }
      if (data_sent == NUMBER_OF_SAMPLES) {
        UART_Log("WAV file sent successfully\n");
        UART_Receive_ESP32();
        // Return to initial state
        xEventGroupClearBits(buttonEventHandle, BIT_0);
        data_sent = 0;
        sending_file = false;
      }
    }
    osDelay(xDelay * 1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vUartParseTask */
/**
 * @brief Function implementing the uartParseTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vUartParseTask */
void vUartParseTask(void const *argument) {
  /* USER CODE BEGIN vUartParseTask */
  /* Infinite loop */
  for (;;) {
    if (received_uart_rx) {
      // If received error message
      if (strncmp(uart_rx, "Error:", strlen("Error:")) == 0) {
        UART_Log(uart_rx);
      } else {
        // Trim "Success: " from message
        size_t l = strlen("Success: ");
        char *transcription =
            strncmp(uart_rx, "Success: ", l) ? uart_rx : uart_rx + l;
        // Make message lowercase
        for (int i = 0; transcription[i]; i++) {
          transcription[i] = tolower(transcription[i]);
        }

        xSemaphoreTake(commandMutexHandle, portMAX_DELAY);
        if (strstr(transcription, "forward") != NULL) {
          command = ROVER_FORWARD;
        } else if (strstr(transcription, "backward") != NULL) {
          command = ROVER_BACKWARD;
        } else if (strstr(transcription, "right") != NULL) {
          command = ROVER_RIGHT;
        } else if (strstr(transcription, "left") != NULL) {
          command = ROVER_LEFT;
        } else if (strstr(transcription, "spin") != NULL) {
          command = ROVER_SPIN;
        } else if (strstr(transcription, "open") != NULL) {
          command = ROVER_OPEN;
        } else if (strstr(transcription, "close") != NULL) {
          command = ROVER_CLOSE;
        } else {
          UART_Log("No instructions detected. Transcription: ");
          UART_Log(transcription);
        }
        xSemaphoreGive(commandMutexHandle);
      }
      received_uart_rx = false;
    }
    osDelay(1);
  }
  /* USER CODE END vUartParseTask */
}

/* USER CODE BEGIN Header_vRoverTask */
/**
 * @brief Function implementing the roverTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vRoverTask */
void vRoverTask(void const *argument) {
  /* USER CODE BEGIN vRoverTask */
  RoverCommand cmd;
  /* Infinite loop */
  for (;;) {
    xSemaphoreTake(commandMutexHandle, portMAX_DELAY);
    cmd = command;
    command = ROVER_DO_NOTHING;
    xSemaphoreGive(commandMutexHandle);
    if (cmd != ROVER_DO_NOTHING) {
      switch (cmd) {
      case ROVER_FORWARD:
        UART_Log("Forward!\n");
        Set_GPIO_Tmp(Rover_Forward_GPIO_Port, Rover_Forward_Pin, 500); // Go forward for 500ms
        break;
      case ROVER_BACKWARD:
        UART_Log("Backward!\n");
        Set_GPIO_Tmp(Rover_Backward_GPIO_Port, Rover_Backward_Pin, 500); // Go backward for 500ms
        break;
      case ROVER_RIGHT:
        UART_Log("Right!\n");
        Set_GPIO_Tmp(Rover_Right_GPIO_Port, Rover_Right_Pin, 200); // Turn right 90deg
        break;
      case ROVER_LEFT:
        UART_Log("Left!\n");
        Set_GPIO_Tmp(Rover_Left_GPIO_Port, Rover_Left_Pin, 200); // Turn left 90deg
      case ROVER_SPIN:
        UART_Log("Spin!\n");
        Set_GPIO_Tmp(Rover_Right_GPIO_Port, Rover_Right_Pin, 850); // Do a 360deg spin
        break;
      case ROVER_OPEN:
        UART_Log("Open lid!\n");
        Set_GPIO_Tmp(Rover_Open_GPIO_Port, Rover_Open_Pin, 1500); // Open lid
        break;
      case ROVER_CLOSE:
        UART_Log("Close lid!\n");
        Set_GPIO_Tmp(Rover_Close_GPIO_Port, Rover_Close_Pin, 1500); // Close lid
        break;
      case ROVER_SPEAK:
        // TODO
        break;
      }
    }
    osDelay(1);
  }
  /* USER CODE END vRoverTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
