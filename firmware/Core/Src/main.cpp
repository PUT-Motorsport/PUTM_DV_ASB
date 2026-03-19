/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes
 * ------------------------------------------------------------------*/
#include "main.h"

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PUTM_CAN_M.h"
#include "PUTM_EV_CAN_LIBRARY/include/can_driver.hpp"
#include "gpioElements.hpp"
#include "stm32l4xx_hal_can.h"
#include "stm32l4xx_hal_def.h"
#include "timer.hpp"
#include <cstdint>
#include <functional>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef
 * -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define
 * ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_TEST 1

#define USE_TEST_POINTS 1

/* USER CODE END PD */

/* Private macro
 * -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum Ebs_supervisor_state {
  EBS_INITIAL_CHECKUP,
  EBS_CONTINOUS_MONITORING,
  EBS_STOP_MONITORING,
};

static struct Config {
  constexpr static float brake_lower_bound = 6.5f;
  constexpr static float brake_upper_bound = 8.f;
  constexpr static float brake_press_deviation = 0.01f;
  constexpr static float can_brake_offset = -0.01f;
  constexpr static float press_tf_coef = 1.f;
  constexpr static float can_brake_lower_bound = 1.f;
  constexpr static float can_engaged_brakes_upper_bound = 90.f;
  constexpr static float can_engaged_brakes_lower_bound = 60.f;

  constexpr static uint32_t sdc_settle_timeout = 100;
  constexpr static uint32_t valve_settle_delay = 200;
  constexpr static uint32_t valve_settle_timeout = 200;

  constexpr static uint16_t can_timeout = 100;

  constexpr static uint32_t adc_count = 3;
} volatile config;

struct TransferFcn {
  float a;
  float b;
  float solve(float x) { return a * x + b; }
};

struct Brake_data_can {
  uint16_t brake_pressure_front;
  uint16_t brake_pressure_rear;
  bool status;
};

class Brakes_data {
private:
  // driver_input.break_pressure* returns in kP and we use Bars, so times 0.01
  float convert_raw_data(uint16_t raw_data) {
    return static_cast<float>(raw_data) * 0.01f + config.can_brake_offset;
  }

public:
  float front;
  float rear;

  void update(Brake_data_can &brake_data) {
    this->front = convert_raw_data(brake_data.brake_pressure_front);
    this->rear = convert_raw_data(brake_data.rear);
  }

  HAL_StatusTypeDef check_buildup(float air_ebs) {
    if (this->front < config.press_tf_coef * air_ebs)
      return HAL_ERROR;
    if (this->rear < config.press_tf_coef * air_ebs)
      return HAL_ERROR;
    return HAL_OK;
  }

  HAL_StatusTypeDef check_holdup(float air_ebs) {
    if (this->front > config.press_tf_coef * air_ebs &&
        this->rear < config.can_brake_lower_bound)
      return HAL_OK;
    else
      return HAL_ERROR;
  }
  HAL_StatusTypeDef check_engaged() {
    if (config.can_engaged_brakes_lower_bound < this->front &&
        config.can_engaged_brakes_upper_bound > this->rear &&
        config.can_engaged_brakes_lower_bound < this->front &&
        config.can_engaged_brakes_upper_bound > this->rear)
      return HAL_OK;
    else
      return HAL_ERROR;
  }
};

class Air_pressure_data {
private:
  // Transfer Function for Air pressure ADC readings:
  TransferFcn air_transferFcn = {3.227f, 620.0f};

public:
  float ebs;
  float redundant;
  float main;

  void update(uint16_t adc_dma_data[config.adc_count]) {
    this->ebs = air_transferFcn.solve(static_cast<float>(adc_dma_data[0]));
    this->redundant =
        air_transferFcn.solve(static_cast<float>(adc_dma_data[1]));
    this->main = air_transferFcn.solve(static_cast<float>(adc_dma_data[2]));
  }
  HAL_StatusTypeDef check() {
    if (std::abs(this->ebs - this->redundant) > config.brake_press_deviation)
      return HAL_ERROR;
    if (config.brake_lower_bound >= this->ebs ||
        this->ebs >= config.brake_upper_bound)
      return HAL_ERROR;
    if (config.brake_lower_bound >= this->redundant ||
        this->redundant >= config.brake_upper_bound)
      return HAL_ERROR;
    return HAL_OK;
  }
};

/* USER CODE END PM */

/* Private variables
 * ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

Brake_data_can brake_data_can;

uint16_t adc_dma_buffer[3];

/* GPIO Section */
GpioOutElement led_ok(LD_OK_GPIO_Port, LD_OK_Pin);
GpioOutElement led_warn(LD_WARN_GPIO_Port, LD_WARN_Pin);
GpioOutElement led_err(LD_ERR_GPIO_Port, LD_ERR_Pin);
GpioOutElement led_chk(LD_CHK_GPIO_Port, LD_CHK_Pin);

GpioOutElement as_close_sdc(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin);
GpioOutElement as_mode(AS_MODE_GPIO_Port, AS_MODE_Pin);

GpioOutElement valve1(VALVE1_GPIO_Port, VALVE1_Pin);
GpioOutElement valve2(VALVE2_GPIO_Port, VALVE2_Pin);

GpioInElement sdc_ready(SDC_RDY_GPIO_Port, SDC_RDY_Pin);

/* Tests */
/* USER CODE END PV */

/* Private function prototypes
 * -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void startTogglingWatchdog();
void stopTogglingWatchdog();
void can_filter_config(CAN_HandleTypeDef *hcan);
void can_driver_input_cb(const PUTM_CAN_M_driver_input_t &driver_input);
Ebs_supervisor_state ebs_error();

/* USER CODE END PFP */

/* Private user code
 * ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
   */
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
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Ebs_supervisor_state ebs_state = EBS_INITIAL_CHECKUP;

  // Initialize CAN
  using namespace putm_ev_can;

  CanDriver can_m;
  can_filter_config(&hcan1);
  if (!can_m.Init(&hcan1)) {
    ebs_state = ebs_error();
  }
  can_m.RegisterCallback<PUTM_CAN_M_driver_input_t>(
      PUTM_CAN_M_DRIVER_INPUT_FRAME_ID, can_driver_input_cb);

  Brakes_data brakes;
  Air_pressure_data air_pressure;

  HAL_ADCEx_Calibration_Start(&hadc1, 10);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, config.adc_count);

  Timer sdc_timeout_timer(config.sdc_settle_timeout);
  Timer valve_timeout_timer(config.valve_settle_timeout);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while (1) {
  //     SDC_STATE = HAL_GPIO_ReadPin(SDC_RDY_GPIO_Port, SDC_RDY_Pin);
  //     starTogglingWatchdog();
  //     as_close_sdc.activate();
  //     if (SDC_STATE == GPIO_PIN_RESET) {
  //         HAL_GPIO_TogglePin(LD_WARN_GPIO_Port, LD_WARN_Pin);
  //     }

  // PUTM_CAN::ASB_main asb_data{
  //     .airpressure_bank1 = static_cast<uint16_t>(air_ebs),
  //     .airpressure_bank2 = static_cast<uint16_t>(air_main),
  //     .SDC_Ready = 0,
  //     .valve1_active = 0,
  //     .valve2_active = 0,
  //     .device_state = PUTM_CAN::ASB_states::ASB_OK};

  // auto test1 =
  //     PUTM_CAN::Can_tx_message<ASB_main>(asb_data,
  //     can_tx_header_ASB_MAIN);
  // auto status = test1.send(hcan1);
  // // HAL_Delay(20);

  // if (status == HAL_ERROR) {
  //   HAL_GPIO_TogglePin(LD_ERR_GPIO_Port, LD_ERR_Pin);
  // }

  while (true) {
    switch (ebs_state) {
    case (EBS_INITIAL_CHECKUP): {

      valve1.activate();
      valve2.activate();

      // check if sdc is working
      sdc_timeout_timer.restart();
      startTogglingWatchdog();
      while (true) {
        if (sdc_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_state = ebs_error();
        }
        if (sdc_ready.isActive())
          break;
      }

      sdc_timeout_timer.restart();
      stopTogglingWatchdog();
      while (true) {
        if (sdc_timeout_timer.checkIfTimedOutThenReset())
          ebs_state = ebs_error();
        if (!sdc_ready.isActive())
          break;
      }

      startTogglingWatchdog();

      // Check that the EBS energy storage is filled
      air_pressure.update(adc_dma_buffer);
      if (air_pressure.check() == HAL_ERROR)
        ebs_state = ebs_error();

      // Check that the brake pressure is built up correctly
      while (brake_data_can.status == false) {
      }
      brake_data_can.status = false;
      if (brakes.check_buildup(air_pressure.ebs) == HAL_ERROR)
        ebs_state = ebs_error();

      // wait for tc enabled
      //   bool tc_activated = false;
      //   do {
      //     HAL_Delay(100);
      //     auto tc_main = PUTM_CAN::can.get_tc_main();
      //     tc_activated = tc_main.traction_control_enable;
      //   } while (!tc_activated);

      // Check that the brake pressure is still built
      // up correctly
      valve1.deactivate();
      HAL_Delay(config.valve_settle_delay);
      valve_timeout_timer.restart();
      while (true) {
        if (valve_timeout_timer.checkIfTimedOutThenReset())
          ebs_state = ebs_error();
        while (brake_data_can.status == false) {
        }
        brake_data_can.status = false;
        if (brakes.check_holdup(air_pressure.ebs) == HAL_OK)
          break;
      }
      valve1.activate();

      valve2.deactivate();
      HAL_Delay(config.valve_settle_delay);
      valve_timeout_timer.restart();
      while (true) {
        if (valve_timeout_timer.checkIfTimedOutThenReset())
          ebs_state = ebs_error();
        while (brake_data_can.status == false) {
        }
        brake_data_can.status = false;
        if (brakes.check_holdup(air_pressure.ebs) == HAL_OK)
          break;
      }
      valve2.activate();

      ebs_state = EBS_CONTINOUS_MONITORING;
      break;
    }
    case EBS_CONTINOUS_MONITORING: {
      // Monitor the storage of brake energy (air pressure)
      air_pressure.update(adc_dma_buffer);
      if (air_pressure.check() == HAL_ERROR) {
        ebs_state = ebs_error();
        break;
      }
      // Convert brake pressure data from if received from CAN
      if (brake_data_can.status == true) {
        brakes.update(brake_data_can);
        brake_data_can.status = false;
      }

      // Check RES state CAN OK?
      // AS OK?

      // Check SDC
      if (!sdc_ready.isActive()) {
        HAL_Delay(config.valve_settle_delay);
        valve_timeout_timer.restart();
        while (true) {
          if (valve_timeout_timer.checkIfTimedOutThenReset())
            ebs_state = ebs_error();
          if (brakes.check_engaged() == HAL_OK)
            break;
        }
        break;
      }
      break;
    }
    case EBS_STOP_MONITORING: {
    }
      return 1;
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified
   * parameters in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {
  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1022;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    LD_OK_Pin | LD_WARN_Pin | LD_ERR_Pin | LD_CHK_Pin |
                        AS_CLOSE_SDC_Pin | AS_MODE_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, VALVE1_Pin | VALVE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_OK_Pin LD_WARN_Pin LD_ERR_Pin LD_CHK_Pin
                         AS_CLOSE_SDC_Pin AS_MODE_Pin */
  GPIO_InitStruct.Pin = LD_OK_Pin | LD_WARN_Pin | LD_ERR_Pin | LD_CHK_Pin |
                        AS_CLOSE_SDC_Pin | AS_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VALVE1_Pin VALVE2_Pin */
  GPIO_InitStruct.Pin = VALVE1_Pin | VALVE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_RDY_Pin */
  GPIO_InitStruct.Pin = SDC_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDC_RDY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void can_driver_input_cb(const PUTM_CAN_M_driver_input_t &driver_input) {
  brake_data_can.brake_pressure_front = driver_input.brake_pressure_front;
  brake_data_can.rear = driver_input.brake_pressure_rear;
  brake_data_can.status = true;
}

void can_filter_config(CAN_HandleTypeDef *hcan) {
  constexpr static CAN_FilterTypeDef sFilterConfig{
      .FilterIdHigh = 0x0000,
      .FilterIdLow = 0x0000,
      .FilterMaskIdHigh = 0x0000,
      .FilterMaskIdLow = 0x0000,
      .FilterFIFOAssignment = CAN_RX_FIFO0,
      .FilterBank = 0,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = CAN_FILTERSCALE_32BIT,
      .FilterActivation = ENABLE,
      .SlaveStartFilterBank = 14};

  if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
}

Ebs_supervisor_state ebs_error() {
  led_err.activate();
  as_close_sdc.deactivate();
  return EBS_STOP_MONITORING;
}

void startTogglingWatchdog() { HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); }

void stopTogglingWatchdog() { HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
   number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
   file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
