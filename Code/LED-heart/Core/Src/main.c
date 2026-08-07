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
#include <stdio.h>
#include "LIS2DW12.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SLEEP_MODE_LIGHT,
    SLEEP_MODE_DEEP,
    SLEEP_MODE_DEEP_FORCE
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

//Beat shape. "Lub" is the smaller first thump, "dub" the stronger second thump,
//separated by a short gap, followed by a longer rest before the next heartbeat.
#define LUB_RAMP_MS 40
#define GAP_MS      80
#define DUB_RAMP_MS 80

//RTC wakeup timer ticks per millisecond. RTCCLK is LSI (32 kHz, LSI_VALUE in
//stm32u0xx_hal_conf.h) and RTC_WAKEUPCLOCK_RTCCLK_DIV16 divides it by 16, giving
//2000 Hz - one tick per 0.5 ms. The 16-bit counter therefore spans 32.7 s, well
//over our 2000 ms longest sleep. This path bypasses the RTC prescalers entirely,
//so the calendar's 32768-vs-32000 mismatch does not apply.
#define RTC_WUT_TICKS_PER_MS 2u

//Shortest idle worth taking Stop 2 for. Below this the clock restart and the
//RTC arm/disarm cost more than the sleep saves, so plain WFI wins.
#define STOP_MIN_MS 20u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Beat state machine position. File scope rather than static-local to led_tick()
   so the idle scheduler can ask how long the current phase has left to run. */
static BeatPhase_t beat_phase = BEAT_LUB_RAMP_UP;
static uint32_t beat_phase_start_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t calculate_movement_score(int8_t acceleration_data[256][3]);
void go_to_sleep(SleepMode_t mode);
void auto_sleep_tick(uint8_t movement_score);
void led_tick(uint8_t movement_score);
static uint32_t beat_rest_ms(uint8_t movement_score);
static uint32_t led_ms_until_next_update(uint8_t movement_score);
static uint8_t led_phase_just_entered_rest(void);
static void sleep_stop2_ms(uint32_t ms);

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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //Wake from Stop straight onto HSI16. The AHB prescaler (/4) is kept in RCC_CFGR
  //across Stop, so this lands back on HCLK = 4 MHz with flash latency 0 still
  //correct - identical to the running config, and no clock reconfiguration is
  //needed on each wake. Waking onto MSI instead would change HCLK and silently
  //shift every timing constant.
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  //If accelerometer init fails, flash light and then go to sleep forever
  HAL_Delay(1);
  if (LIS2DW12_init(&hspi1) != 0){ 
    uint32_t flash_start_time = HAL_GetTick();

    while (HAL_GetTick() < flash_start_time + 10000){
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, LED_PWM_MAX); // full on
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);           // off
      HAL_Delay(100);
    }
    go_to_sleep(SLEEP_MODE_DEEP_FORCE);
    
    while (1){
      __NOP();
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    static int8_t acceleration_data[256][3];
    static uint8_t FIFO_unread_length = 0;
    static uint8_t movement_score = 0;

    led_tick(movement_score);

    //Service the accelerometer once per beat, at the moment the beat enters its
    //rest phase. The read costs roughly 10ms (three HAL_Delay(1) inside the driver,
    //a ~150 byte burst at 250 kbit/s, then the 256-sample score), which disappears
    //inside a 150-2000ms rest but would visibly stutter a 40ms ramp.
    //
    //Deferring the read like this is also what makes sleeping safe: INT2 is left as
    //a polled input rather than a wake source, so every sleep runs for exactly the
    //duration we asked for and the SysTick correction in sleep_stop2_ms() is exact.
    //The cost is that motion starting mid-rest is not noticed until the next beat,
    //up to ~2.2s at the slowest rate.
    if (led_phase_just_entered_rest()){

      if (HAL_GPIO_ReadPin(INT2_GPIO_Port, INT2_Pin) == 1){
        if (LIS2DW12_read_FIFO(&hspi1, &FIFO_unread_length, acceleration_data) == 0){
          movement_score = calculate_movement_score(acceleration_data);
        }
        else {
          //Error reading FIFO
          __NOP();
        }
      }

      //Only reacts to movement_score, which now only changes here.
      auto_sleep_tick(movement_score);
    }

    //Idle out whatever is left before the PWM must next move. The long static
    //stretches (gap and rest) are worth stopping the CPU for; during the ramps the
    //duty changes every tick, so just gate the core clock until the next SysTick.
    uint32_t idle_ms = led_ms_until_next_update(movement_score);

    if (idle_ms >= STOP_MIN_MS){
      sleep_stop2_ms(idle_ms);
    }
    else {
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* CubeMX arms the wakeup timer above with a reload of 0, which at RTCCLK/16
     retriggers every 0.5ms forever. We drive the timer per-sleep from
     sleep_stop2_ms() instead, so shut it down until the first sleep asks for it.
     Kept here rather than deleting the generated call, which would come back on
     the next regeneration. */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  /* USER CODE END RTC_Init 2 */

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


/* Bit mask covering all 16 I/O positions. The PWR pull helpers mask this
   down to the pins that actually exist on the sparse ports (D, F) internally. */
#define ALL_GPIO_PINS (PWR_GPIO_BIT_0  | PWR_GPIO_BIT_1  | PWR_GPIO_BIT_2  | PWR_GPIO_BIT_3  | \
                       PWR_GPIO_BIT_4  | PWR_GPIO_BIT_5  | PWR_GPIO_BIT_6  | PWR_GPIO_BIT_7  | \
                       PWR_GPIO_BIT_8  | PWR_GPIO_BIT_9  | PWR_GPIO_BIT_10 | PWR_GPIO_BIT_11 | \
                       PWR_GPIO_BIT_12 | PWR_GPIO_BIT_13 | PWR_GPIO_BIT_14 | PWR_GPIO_BIT_15)

/* Entering Shutdown releases the GPIO output drivers and only the PWR block's pull
   config survives, so it alone decides what every net sits at while asleep. The
   STM32's own I/O logic is unpowered - a floating pin costs the MCU nothing - so
   the pulls exist purely for the parts on the other end of the nets, and which
   direction each one needs depends on who drives it:

     - Sensor INPUTS (SCK PB3, SDI PA7, CS PA15) must be held. Left floating they
       drift to mid-rail and burn crowbar current in the LIS2DW12's input buffers.
       CS is held HIGH to keep the device deselected; the rest idle low (CPOL=0).
     - Sensor OUTPUTS (SDO PA11, INT1 PA1, INT2 PA2) must be left alone. These are
       push-pull drivers, so a pull cannot help and can only fight: a ~40k pull-down
       against a driver asserting high wastes ~70uA at 2.8V. INT2 is the trap - it
       is not a wake pin, so it can sit asserted and burn current indefinitely
       without ever waking the board to reveal the problem.
     - PA3 drives the LED and is held LOW so the LED (or its driver) stays off.
     - PA13/PA14 are SWDIO/SWCLK, left unpulled so we never fight a debug probe.
     - Everything else is unconnected; pulled low as harmless insurance. */
#define PORT_A_SENSOR_OUTPUTS (PWR_GPIO_BIT_1 | PWR_GPIO_BIT_2 | PWR_GPIO_BIT_11)
#define PORT_A_SWD_PINS       (PWR_GPIO_BIT_13 | PWR_GPIO_BIT_14)
#define PORT_A_PULLDOWN_PINS  (ALL_GPIO_PINS & ~(PORT_A_SENSOR_OUTPUTS | \
                                                 PORT_A_SWD_PINS | PWR_GPIO_BIT_15))

static void configure_shutdown_pin_pulls(void){
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PORT_A_PULLDOWN_PINS);
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_15);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, ALL_GPIO_PINS);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, ALL_GPIO_PINS);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_D, ALL_GPIO_PINS);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_F, ALL_GPIO_PINS);
  HAL_PWREx_EnablePullUpPullDownConfig();
}

/* Wait for the accelerometer's INT1 to be quiet before arming WKUP3.
   PWR_WAKEUP_PIN3_HIGH is level-triggered, so arming it while INT1 is still
   asserted sets WUF3 immediately and Shutdown exits the moment it is entered -
   clearing the flag first does not help, because the level re-sets it. That is
   exactly the ~650ms false-sleep: the wake engine was still chewing through the
   filter transient left by configure_sleep's ODR/FS change, INT1 was high, and the
   board woke and reset instead of sleeping.

   The settle delay in configure_sleep gets most of the way there; this closes the
   gap adaptively rather than relying on that delay being exactly long enough.
   INT1 is pulsed (CTRL3.LIR = 0), so re-reading the source registers each pass
   drops anything latched while we wait.

   Returns 1 if INT1 settled low, 0 if it was still asserted at the timeout. */
#define INT1_QUIET_TIMEOUT_MS 3000
#define INT1_QUIET_HOLD_MS    100

static uint8_t wait_for_int1_quiet(void){

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Read-only peek at the wake pin. No pull: INT1 is a push-pull output on the
       sensor side, so a pull could only fight it. */
    GPIO_InitStruct.Pin = INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

    uint32_t start = HAL_GetTick();
    uint32_t quiet_since = start;

    while ((HAL_GetTick() - start) < INT1_QUIET_TIMEOUT_MS){

        if (HAL_GPIO_ReadPin(INT1_GPIO_Port, INT1_Pin) == GPIO_PIN_SET){
            /* Still firing. Clear the latched source and restart the hold window. */
            LIS2DW12_clear_interrupt_sources(&hspi1);
            quiet_since = HAL_GetTick();
        }
        else if ((HAL_GetTick() - quiet_since) >= INT1_QUIET_HOLD_MS){
            /* Low, and stayed low long enough that it is not between pulses. */
            return 1;
        }
    }

    return 0;
}

void go_to_sleep(SleepMode_t mode){
    switch (mode) {
        case SLEEP_MODE_DEEP:
            /* PA1 = WKUP3, wired to the accelerometer's INT1. Movement drives it high,
               so wake on a high level. Clear any stale wake flag before enabling,
               otherwise a flag left set from a previous event wakes us immediately. */

            if(!LIS2DW12_configure_sleep(&hspi1) && wait_for_int1_quiet()){
              configure_shutdown_pin_pulls();
              __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
              HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
              HAL_PWR_EnterSHUTDOWNMode();
            }
            else{
              /* Either configuring the accelerometer for wake-up failed, or INT1 never
                 went quiet. Don't enter Shutdown: with no usable wake source the board
                 would never wake, and arming against a stuck-high INT1 just burns a
                 wake-reset cycle. The caller backs off and retries later.

                 configure_sleep has already left the sensor in its sleep config by this
                 point - 1.6 Hz, FIFO bypassed, INT2 unrouted - so movement_score would
                 be stuck at 0 and the board would be deaf to motion until it managed to
                 sleep. Re-init to put it back to work in the meantime. */
              LIS2DW12_init(&hspi1);
            }

            break;

        case SLEEP_MODE_DEEP_FORCE:
          /* Forced path: used when the sensor has already failed init, so there may be
             no working wake source at all. Sleep regardless - a board that never wakes
             is the intended outcome here, and is far better than one left flashing. */
          LIS2DW12_configure_sleep(&hspi1);
          wait_for_int1_quiet();
          configure_shutdown_pin_pulls();
          __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
          HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
          HAL_PWR_EnterSHUTDOWNMode();
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

      /* Still running, so Shutdown was not entered. Restart the timer to back off
         for another SLEEP_TIMEOUT_MS. Without this the elapsed check stays true and
         go_to_sleep is retried on every beat - and since a failed attempt leaves the
         sensor in its sleep config (FIFO bypassed, INT2 unrouted), movement_score can
         never climb back off zero to break the cycle. That would stall the beat for
         the full settle time, every beat, indefinitely. */
      zero_score_start_tick = HAL_GetTick();
    }
  }
  else {
    zero_score_timer_active = 0;
  }
}

//Rest period shortens as movement increases, mimicking a rising heart rate.
//Score 0 -> 2000ms rest, score 100 -> 150ms rest.
static uint32_t beat_rest_ms(uint8_t movement_score){
    return 2000 - ((uint32_t)movement_score * 1850) / 100;
}

void led_tick(uint8_t movement_score){

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - beat_phase_start_tick;
    uint32_t rest_ms = beat_rest_ms(movement_score);

    uint32_t duty = 0;

    switch (beat_phase){
        case BEAT_LUB_RAMP_UP:
            duty = (elapsed * (LED_PWM_MAX / 2)) / LUB_RAMP_MS;
            if (elapsed >= LUB_RAMP_MS){
                beat_phase = BEAT_LUB_RAMP_DOWN;
                beat_phase_start_tick = now;
                duty = LED_PWM_MAX / 2;
            }
            break;

        case BEAT_LUB_RAMP_DOWN:
            duty = (LED_PWM_MAX / 2) - (elapsed * (LED_PWM_MAX / 2)) / LUB_RAMP_MS;
            if (elapsed >= LUB_RAMP_MS){
                beat_phase = BEAT_GAP;
                beat_phase_start_tick = now;
                duty = 0;
            }
            break;

        case BEAT_GAP:
            duty = 0;
            if (elapsed >= GAP_MS){
                beat_phase = BEAT_DUB_RAMP_UP;
                beat_phase_start_tick = now;
            }
            break;

        case BEAT_DUB_RAMP_UP:
            duty = (elapsed * LED_PWM_MAX) / DUB_RAMP_MS;
            if (elapsed >= DUB_RAMP_MS){
                beat_phase = BEAT_DUB_RAMP_DOWN;
                beat_phase_start_tick = now;
                duty = LED_PWM_MAX;
            }
            break;

        case BEAT_DUB_RAMP_DOWN:
            duty = LED_PWM_MAX - (elapsed * LED_PWM_MAX) / DUB_RAMP_MS;
            if (elapsed >= DUB_RAMP_MS){
                beat_phase = BEAT_REST;
                beat_phase_start_tick = now;
                duty = 0;
            }
            break;

        case BEAT_REST:
        default:
            duty = 0;
            if (elapsed >= rest_ms){
                beat_phase = BEAT_LUB_RAMP_UP;
                beat_phase_start_tick = now;
            }
            break;
    }

    if (duty > LED_PWM_MAX) duty = LED_PWM_MAX; //clamp against rounding overshoot at phase boundaries

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
}


//How long until led_tick() next has to change the PWM duty. During the ramps the
//duty moves every millisecond, so there is nothing to skip; during the gap and
//rest the output is static and the CPU can be stopped for the remainder.
//Call only after led_tick(), so the phase and elapsed time are current.
static uint32_t led_ms_until_next_update(uint8_t movement_score){

    uint32_t elapsed = HAL_GetTick() - beat_phase_start_tick;
    uint32_t phase_ms;

    switch (beat_phase){
        case BEAT_GAP:
            phase_ms = GAP_MS;
            break;

        case BEAT_REST:
            phase_ms = beat_rest_ms(movement_score);
            break;

        default:
            //Ramping: duty changes on every tick.
            return 1;
    }

    //led_tick() only advances the phase once elapsed has reached the phase length,
    //so sleeping the whole remainder lands us on the transition rather than past it.
    return (elapsed >= phase_ms) ? 0 : (phase_ms - elapsed);
}


//True on the single iteration that led_tick() moved the beat into its rest phase.
//That transition is the one point in the cycle with a long stretch of static
//output ahead, which is where the accelerometer read is cheapest to hide.
static uint8_t led_phase_just_entered_rest(void){

    static BeatPhase_t previous_phase = BEAT_LUB_RAMP_UP;

    uint8_t entered = (beat_phase == BEAT_REST) && (previous_phase != BEAT_REST);
    previous_phase = beat_phase;

    return entered;
}


//Stop the CPU for ms milliseconds, woken by the RTC wakeup timer.
//
//SysTick is not clocked in Stop 2, so HAL_GetTick() would otherwise lose the whole
//sleep and every timebase built on it (the beat state machine, the 30s inactivity
//timer, the driver's SPI timeouts) would stall. Because the caller chooses the
//duration and the accelerometer is deliberately not a wake source, the elapsed
//time is known exactly and uwTick can simply be advanced by it.
static void sleep_stop2_ms(uint32_t ms){

    uint32_t ticks = ms * RTC_WUT_TICKS_PER_MS;

    if (ticks == 0u){
        return;
    }

    /* The counter reload is zero-based: a value of n wakes after n+1 ticks. */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks - 1u,
                                    RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0u) != HAL_OK){
        /* Without an armed wake source Stop would never end, so stay awake and
           let the caller's normal timing carry on. */
        return;
    }

    HAL_SuspendTick();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    HAL_ResumeTick();

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    uwTick += ms;
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
