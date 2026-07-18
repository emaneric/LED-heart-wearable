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


// Segger RTT host: localhost
// Segger RTT port: 19021




/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LIS2DU12.h"
#include "LIS2DW12.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_gpio.h"
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SLEEP_MODE_LIGHT,
    SLEEP_MODE_DEEP
} SleepMode_t;

typedef enum {
    BEAT_LUB_RAMP_UP,
    BEAT_LUB_RAMP_DOWN,
    BEAT_GAP,
    BEAT_DUB_RAMP_UP,
    BEAT_DUB_RAMP_DOWN,
    BEAT_REST
} BeatPhase_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLEEP_TIMEOUT_MS 30000
#define LED_PWM_MAX 249  //matches TIM2 Period (ARR)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t calculate_movement_score(int8_t acceleration_data[256][3]);
void go_to_sleep(SleepMode_t mode); 
void auto_sleep_tick(uint8_t movement_score);
void led_tick(uint8_t movement_score);

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  //If accelerometer init fails, flash light and wait forever
  if (LIS2DW12_init(&hspi1) != 0){
    uint32_t flash_start_time = HAL_GetTick();

    while (HAL_GetTick() < flash_start_time + 5000){
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, LED_PWM_MAX); // full on
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);           // off
      HAL_Delay(100);
    }

    while (1){
      __NOP();
    }
  }
  HAL_Delay(10000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    static int8_t acceleration_data[256][3];
    static uint8_t FIFO_unread_length = 0;
    static uint8_t movement_score = 0;

    if (HAL_GPIO_ReadPin(INT2_GPIO_Port, INT2_Pin) == 1){ //Need to check config of sensor, this pin is not beign set high
      __NOP();
      if (LIS2DW12_read_FIFO(&hspi1, &FIFO_unread_length, acceleration_data) == 0){
        movement_score = calculate_movement_score(acceleration_data);
      }
      else {
        //Error reading FIFO
        __NOP();
      }
    }

    //auto_sleep_tick(movement_score);  //when uncommented, hard fault occurs
    led_tick(movement_score);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA5 PA6
                           PA8 PA9 PA10 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_Pin */
  GPIO_InitStruct.Pin = INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


#define MOVEMENT_MAX_AVG_DELTA 15

uint8_t calculate_movement_score(int8_t acceleration_data[256][3]){

  uint32_t total_variation = 0;

  //Sum the sample-to-sample change on each axis. Using deltas rather than raw magnitude cancels
  //out the constant gravity offset baked into whichever axis is vertical, so the score reflects
  //actual motion rather than orientation.
  for (uint16_t i = 1; i < 256; i++){
    for (uint8_t axis = 0; axis < 3; axis++){
      int16_t delta = (int16_t)acceleration_data[i][axis] - (int16_t)acceleration_data[i - 1][axis];
      total_variation += (delta < 0) ? -delta : delta;
    }
  }

  uint32_t avg_delta = total_variation / (255 * 3);
  uint32_t score = (avg_delta * 100) / MOVEMENT_MAX_AVG_DELTA;
  if (score > 100) score = 100;

  return (uint8_t)score;
}


/* Bit mask covering all 16 I/O positions. The PWR pull-down helper masks this
   down to the pins that actually exist on the sparse ports (D, F) internally. */
#define ALL_GPIO_PINS (PWR_GPIO_BIT_0  | PWR_GPIO_BIT_1  | PWR_GPIO_BIT_2  | PWR_GPIO_BIT_3  | \
                       PWR_GPIO_BIT_4  | PWR_GPIO_BIT_5  | PWR_GPIO_BIT_6  | PWR_GPIO_BIT_7  | \
                       PWR_GPIO_BIT_8  | PWR_GPIO_BIT_9  | PWR_GPIO_BIT_10 | PWR_GPIO_BIT_11 | \
                       PWR_GPIO_BIT_12 | PWR_GPIO_BIT_13 | PWR_GPIO_BIT_14 | PWR_GPIO_BIT_15)

void go_to_sleep(SleepMode_t mode){
    switch (mode) {
        case SLEEP_MODE_DEEP:
            /* PA1 = WKUP3, wired to the accelerometer's INT1. Movement drives it high,
               so wake on a rising edge. Clear any stale wake flag before enabling,
               otherwise a flag left set from a previous event wakes us immediately. */

            if(!LIS2DW12_configure_sleep(&hspi1)){
              /* In Shutdown the GPIO/analog config set in MX_GPIO_Init is lost, so
                 every pin reverts to a floating input. Floating inputs settle to
                 mid-rail and burn current, which is why a bare board measures far
                 above the datasheet shutdown spec. Only the PWR block's pull config
                 survives into Shutdown, so drive every pin low through it. This also
                 gives WKUP3 (PA1) a defined low level, consistent with wake-on-high. */
              HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, ALL_GPIO_PINS);
              HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, ALL_GPIO_PINS);
              HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, ALL_GPIO_PINS);
              HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_D, ALL_GPIO_PINS);
              HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_F, ALL_GPIO_PINS);
              HAL_PWREx_EnablePullUpPullDownConfig();

              __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
              HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
              HAL_PWR_EnterSHUTDOWNMode();
            }
            else{
              /* Configuring the accelerometer for wake-up failed. Don't enter
                 Shutdown - with no armed wake source the board would never wake. */
              __NOP();
            }

            break;

        case SLEEP_MODE_LIGHT:

        default:
            /* light sleep config */
            break;
    }
}


void auto_sleep_tick(uint8_t movement_score){

  static uint32_t zero_score_start_tick = 0;
  static uint8_t zero_score_timer_active = 0;
  
  if (movement_score == 0){
    
    if (!zero_score_timer_active){     
      zero_score_timer_active = 1;
      zero_score_start_tick = HAL_GetTick();
    }
    
    else if ((HAL_GetTick() - zero_score_start_tick) >= SLEEP_TIMEOUT_MS){
      go_to_sleep(SLEEP_MODE_DEEP);
    }
  }
  else {
    zero_score_timer_active = 0;
  }
}

void led_tick(uint8_t movement_score){

    static BeatPhase_t phase = BEAT_LUB_RAMP_UP;
    static uint32_t phase_start_tick = 0;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - phase_start_tick;

    //"Lub" is the smaller first thump, "dub" the stronger second thump, separated by a
    //short gap, followed by a longer rest before the next heartbeat.
    const uint32_t lub_ramp_ms = 40;
    const uint32_t gap_ms      = 80;
    const uint32_t dub_ramp_ms = 80;

    //Rest period shortens as movement increases, mimicking a rising heart rate.
    //Score 0 -> 2000ms rest, score 100 -> 150ms rest.
    uint32_t rest_ms = 2000 - ((uint32_t)movement_score * 1850) / 100;

    uint32_t duty = 0;

    switch (phase){
        case BEAT_LUB_RAMP_UP:
            duty = (elapsed * (LED_PWM_MAX / 2)) / lub_ramp_ms;
            if (elapsed >= lub_ramp_ms){
                phase = BEAT_LUB_RAMP_DOWN;
                phase_start_tick = now;
                duty = LED_PWM_MAX / 2;
            }
            break;

        case BEAT_LUB_RAMP_DOWN:
            duty = (LED_PWM_MAX / 2) - (elapsed * (LED_PWM_MAX / 2)) / lub_ramp_ms;
            if (elapsed >= lub_ramp_ms){
                phase = BEAT_GAP;
                phase_start_tick = now;
                duty = 0;
            }
            break;

        case BEAT_GAP:
            duty = 0;
            if (elapsed >= gap_ms){
                phase = BEAT_DUB_RAMP_UP;
                phase_start_tick = now;
            }
            break;

        case BEAT_DUB_RAMP_UP:
            duty = (elapsed * LED_PWM_MAX) / dub_ramp_ms;
            if (elapsed >= dub_ramp_ms){
                phase = BEAT_DUB_RAMP_DOWN;
                phase_start_tick = now;
                duty = LED_PWM_MAX;
            }
            break;

        case BEAT_DUB_RAMP_DOWN:
            duty = LED_PWM_MAX - (elapsed * LED_PWM_MAX) / dub_ramp_ms;
            if (elapsed >= dub_ramp_ms){
                phase = BEAT_REST;
                phase_start_tick = now;
                duty = 0;
            }
            break;

        case BEAT_REST:
        default:
            duty = 0;
            if (elapsed >= rest_ms){
                phase = BEAT_LUB_RAMP_UP;
                phase_start_tick = now;
            }
            break;
    }

    if (duty > LED_PWM_MAX) duty = LED_PWM_MAX; //clamp against rounding overshoot at phase boundaries

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
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
