/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define SAMPLES 100 // Количество точек на период
#define M_PI 3.14159265358979323846
float sine_wave[SAMPLES];

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float amplitude_scale = 0.6667; // Начальное значение амплитуды
float frequency = 1000;         // Начальная частота в Гц
int buf_size = 16;

uint8_t rx_byte;
char rx_buffer[16];
uint8_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void ParseUARTMessage(char* message);
void UpdateTimerFrequency(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t sample_index = 0;
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Запуск таймера в режиме прерываний
  HAL_TIM_Base_Start_IT(&htim6);

  // Запуск ЦАП
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  // Инициализация массива sine_wave
  for(int i = 0; i < SAMPLES; i++)
  {
      sine_wave[i] = (sinf(2 * M_PI * i / SAMPLES) * amplitude_scale + 1) / 2;
  }

  // Начало приема данных по UART
  HAL_UART_Receive_IT(&huart2, &rx_byte, buf_size);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    // Основной цикл ничего не делает, вся обработка происходит в прерываниях
    /* USER CODE END WHILE */
  }
}

/* USER CODE BEGIN 4 */
// Обработчик прерывания по завершению приема данных UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if(rx_byte != '\n' && rx_index < sizeof(rx_buffer) - 1)
        {
            rx_buffer[rx_index++] = rx_byte;
        }
        else
        {
            rx_buffer[rx_index] = '\0';
            // Обработка полученного сообщения
            HAL_UART_Transmit(&huart2, (uint8_t*)rx_buffer, strlen(rx_buffer), HAL_MAX_DELAY);
            ParseUARTMessage(rx_buffer);
            // Сброс индекса
            rx_index = 0;
        }

        // Перезапуск приема данных по UART
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// Функция разбора входящего сообщения
void ParseUARTMessage(char* message)
{
    float new_amplitude_scale;
    float new_frequency;

    // Парсинг сообщения
    if(sscanf(message, "A=%f,F=%f", &new_amplitude_scale, &new_frequency) == 2)
    {
        // Обновление амплитуды и частоты
        amplitude_scale = new_amplitude_scale;
        frequency = new_frequency;

        // Обновление массива sine_wave
        __disable_irq(); // Отключение прерываний
        for(int i = 0; i < SAMPLES; i++)
        {
            sine_wave[i] = (sinf(2 * M_PI * i / SAMPLES) * amplitude_scale + 1) / 2;
        }
        __enable_irq(); // Включение прерываний

        // Обновление настроек таймера
        UpdateTimerFrequency();

        // Отправка подтверждения по UART
        char ack_message[50];
        sprintf(ack_message, "Amplitude: %.3f, Frequency: %.3f Hz\r\n", amplitude_scale, frequency);
        HAL_UART_Transmit(&huart2, (uint8_t*)ack_message, strlen(ack_message), HAL_MAX_DELAY);
    }
    else
    {
        // Сообщение об ошибке парсинга
        char error_message[] = "Invalid format. Use A=amp,F=freq\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)error_message, strlen(error_message), HAL_MAX_DELAY);
    }
    char debug_message[50];
    sprintf(debug_message, "Amplitude set to: %.3f, Frequency set to: %.3f\r\n", amplitude_scale, frequency);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_message, strlen(debug_message), HAL_MAX_DELAY);
}

// Функция обновления настроек таймера для изменения частоты
void UpdateTimerFrequency()
{
    // Желательная частота прерываний таймера = частота * количество_samples
    uint32_t desired_timer_freq = frequency * SAMPLES;

    // Получение частоты таймера
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2; // Умножаем на 2 для корректной частоты таймера (если используется APB1)

    // Вычисление предделителя и периода
    uint32_t prescaler = (timer_clock / desired_timer_freq / 65536) + 1;
    if(prescaler > 0xFFFF) prescaler = 0xFFFF;

    uint32_t period = (timer_clock / (prescaler * desired_timer_freq)) - 1;
    if(period > 0xFFFF) period = 0xFFFF;

    // Остановка таймера
    HAL_TIM_Base_Stop_IT(&htim6);

    // Обновление настроек таймера
    htim6.Init.Prescaler = prescaler - 1;
    htim6.Init.Period = period;

    if(HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    // Запуск таймера
    HAL_TIM_Base_Start_IT(&htim6);
}

// Обработчик прерывания таймера
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        // Преобразование значения из массива в значение для ЦАП
        uint32_t dac_value = (uint32_t)(sine_wave[sample_index] * 4095); // Диапазон 0-4095

        // Установка значения на ЦАП
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

        // Увеличение индекса и проверка на переполнение
        sample_index++;
        if(sample_index >= SAMPLES)
        {
            sample_index = 0;
        }
    }
}
/* USER CODE END 4 */

/* Функции инициализации периферии */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Инициализация напряжения основного внутреннего регулятора */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Инициализация RCC Осцилляторов */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Инициализация тактирования CPU, AHB и APB шин */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // Обратите внимание на делитель
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // Обратите внимание на делитель

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  /** Инициализация DAC */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** Настройка канала DAC OUT1 */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /** Инициализация TIM6 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 39;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  /** Инициализация UART2 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  /* Включение тактирования GPIO портов */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  Функция обработки ошибок.
  * @retval None
  */
void Error_Handler(void)
{
  /* Пользователь может добавить свою реализацию для обработки ошибок */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Сообщает имя файла и номер строки, где произошла ошибка assert_param.
  * @param  file: указатель на имя файла исходного кода
  * @param  line: номер строки, где произошла ошибка
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Пользователь может добавить свою реализацию для сообщения об ошибке */
}
#endif /* USE_FULL_ASSERT */
