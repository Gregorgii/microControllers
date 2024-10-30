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

/* Private define ------------------------------------------------------------*/
#define BLUE_BUTTON_PIN GPIO_PIN_13
#define RED_BUTTON_PIN GPIO_PIN_2
#define DEBOUNCE_DELAY 200 // Задержка

/* Private variables ---------------------------------------------------------*/
uint8_t led_state = 0; // Состояние светодиодов

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void update_leds(void);

/* Define LED pins */
const uint16_t led_pins[] = {
  GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
  GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11
};

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  uint8_t red_button_previous = 0;
  uint8_t blue_button_previous = 0;

  while (1)
  {
    uint8_t red_button_state = HAL_GPIO_ReadPin(GPIOD, RED_BUTTON_PIN);
    uint8_t blue_button_state = HAL_GPIO_ReadPin(GPIOC, BLUE_BUTTON_PIN);

    // Проверка нажатия красной кнопки
    if (red_button_state && !red_button_previous) // Нажатие красной кнопки
    {
      if (led_state < 8)
      {
        led_state++;
      }
      update_leds();
      HAL_Delay(DEBOUNCE_DELAY); // Задержка
    }
    red_button_previous = red_button_state;

    // Проверка нажатия синей кнопки
    if (blue_button_state && !blue_button_previous) // Нажатие синей кнопки
    {
      if (led_state > 0)
      {
        led_state--;
      }
      update_leds();
      HAL_Delay(DEBOUNCE_DELAY); // Задержка
    }
    blue_button_previous = blue_button_state;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(GPIOC, led_pins[i], GPIO_PIN_RESET);
  }

  /* Configure LED pins */
  for (int i = 0; i < 8; i++) {
    GPIO_InitStruct.Pin = led_pins[i];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }

  /* Configure button pins */
  GPIO_InitStruct.Pin = RED_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BLUE_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief Update LEDs state
  * @retval None
  */
void update_leds(void)
{
  for (int i = 0; i < 8; i++) {
    if (i < led_state) {
      HAL_GPIO_WritePin(GPIOC, led_pins[i], GPIO_PIN_SET); // Включаем светодиод
    } else {
      HAL_GPIO_WritePin(GPIOC, led_pins[i], GPIO_PIN_RESET); // Выключаем светодиод
    }
  }
}
