/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
 /* USER CODE END Header */
 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
#define SAMPLES 100         // Number of points for sine wave
#define M_PI 3.14159265358979323846
#define MAX_FREQ 1000.0f    // Maximum frequency allowed
#define TIMER_FREQUENCY 2000.0f  // Timer interrupt frequency in Hz
#define UART_BUFFER_SIZE 10

/* USER CODE BEGIN PD */
float sine_wave[SAMPLES];
volatile float freq = 2.0f;          // Frequency of sine wave in Hz
volatile float m = 0.6667f;
volatile float amp = 1.0f;// Amplitude of sine wave (maximum 1.0)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile float phase = 0.0f;
volatile float phase_increment = 0.0f;

uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
volatile uint8_t command_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
void Process_UART_Command(char* command);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
    /* HAL Initialization and system clock setup */
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_DAC_Init();
    MX_TIM6_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    // Start UART reception in interrupt mode
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_data, 1);

    // Start timer in interrupt mode
    HAL_TIM_Base_Start_IT(&htim6);

    // Start DAC
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    // Calculate phase increment
    phase_increment = (freq * SAMPLES) / TIMER_FREQUENCY;

    // Generate sine_wave[] array
    for(int i = 0; i < SAMPLES; i++)
    {
        sine_wave[i] = (sinf(2 * M_PI * i / SAMPLES) * m * amp + 1) / 2;
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        /* USER CODE BEGIN WHILE */
        if (command_ready)
        {
            command_ready = 0;
            Process_UART_Command((char*)uart_rx_buffer);
        }
        /* USER CODE END WHILE */
    }
}
/* Timer interrupt handler for DAC output */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        phase += phase_increment;
        if(phase >= SAMPLES)
            phase -= SAMPLES;

        uint32_t sample_index = (uint32_t)phase;
        uint32_t dac_value = (uint32_t)(sine_wave[sample_index] * 4095);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    }
}


/* UART receive interrupt handler */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if(uart_rx_data != '\r' && uart_rx_data != '\n' && uart_rx_index < UART_BUFFER_SIZE - 1)
        {
            uart_rx_buffer[uart_rx_index++] = uart_rx_data;
        }
        else if (uart_rx_index > 0) // If command has content
        {
            uart_rx_buffer[uart_rx_index] = '\0'; // Null-terminate the string
            command_ready = 1; // Set flag to process command in main loop
            uart_rx_index = 0;
        }
        // If uart_rx_data is '\r' or '\n' and uart_rx_index == 0, ignore it

        // Restart UART reception
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_data, 1);
    }
}

void Process_UART_Command(char* command)
{
    if(strncmp(command, "FREQ ", 5) == 0)
    {
        float new_freq = atof(&command[5]);
        if(new_freq > 0 && new_freq <= MAX_FREQ)
        {
            freq = new_freq;
            phase_increment = (freq * SAMPLES) / TIMER_FREQUENCY;
            // Send back confirmation
            char msg[32];
            snprintf(msg, sizeof(msg), "Frequency set to %.2f Hz\r\n", freq);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        else
        {
            char msg[] = "Invalid frequency\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    else if(strncmp(command, "AMP ", 4) == 0)
    {
        float new_amp = atof(&command[4]);
        if(new_amp >= 0.0f && new_amp <= 6.5f)
        {
            amp = new_amp;
            // Regenerate sine_wave[]
            for(int i = 0; i < SAMPLES; i++)
            {
                sine_wave[i] = (sinf(2 * M_PI * i / SAMPLES) * amp * m + 1) / 2;
            }
            // Send back confirmation
            char msg[32];
            snprintf(msg, sizeof(msg), "Amplitude set to %.2f\r\n", amp);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        else
        {
            char msg[] = "Invalid amplitude\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    else
    {
        // Unknown command
        char msg[] = "Unknown command\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
