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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PUTM_CAN_M.h"
#include "PUTM_EV_CAN_LIBRARY/include/can_driver.hpp"
#include "gpioElements.hpp"
#include "timer.hpp"

#include "math.h"
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
enum Ebs_state {
  EBS_INITIAL_CHECKUP,
  EBS_CONTINOUS_MONITORING,
  EBS_STOP_MONITORING,
};

namespace config {
constexpr float brake_lower_bound = 6.5f;
constexpr float brake_upper_bound = 8.f;
constexpr float brake_press_deviation = 0.01f;
constexpr float can_brake_offset = -0.01f;
constexpr float press_tf_coef = 1.f;
constexpr float can_brake_lower_bound = 1.f;
constexpr float can_engaged_brakes_upper_bound = 90.f;
constexpr float can_engaged_brakes_lower_bound = 60.f;

constexpr uint32_t sdc_settle_timeout = 100;
constexpr uint32_t valve_settle_delay = 200;
constexpr uint32_t valve_settle_timeout = 200;

constexpr uint16_t can_timeout = 100;

constexpr uint32_t adc_count = 3;
}; // namespace config

struct Brake_data_can {
  uint16_t front;
  uint16_t rear;
  bool status;
} volatile brake_data_can;

struct Air_pressure_data_dma {
  uint16_t ebs;
  uint16_t redundant;
  uint16_t main;
} volatile air_pressure_data_dma;

struct TransferFcn {
  float a;
  float b;
  float solve(float x) { return a * x + b; }
};

class Brakes_data {
private:
  // driver_input.break_pressure* returns in kP and we use Bars, so times 0.01
  float convert_raw_data(uint16_t raw_data) {
    return static_cast<float>(raw_data) * 0.01f + config::can_brake_offset;
  }

public:
  float front;
  float rear;

  void update(volatile Brake_data_can &brake_data) {
    this->front = convert_raw_data(brake_data.front);
    this->rear = convert_raw_data(brake_data.rear);
  }

  bool check_buildup(float air_ebs) {
    if (this->front < config::press_tf_coef * air_ebs)
      return true;
    if (this->rear < config::press_tf_coef * air_ebs)
      return true;
    return false;
  }

  bool check_holdup_front(float air_ebs) {
    if (this->front > config::press_tf_coef * air_ebs &&
        this->rear < config::can_brake_lower_bound)
      return false;
    else
      return true;
  }

  bool check_holdup_rear(float air_ebs) {
    if (this->rear > config::press_tf_coef * air_ebs &&
        this->front < config::can_brake_lower_bound)
      return false;
    else
      return true;
  }

  bool check_engaged() {
    if (this->front > config::can_engaged_brakes_lower_bound &&
        this->front<config::can_engaged_brakes_upper_bound &&this->rear>
            config::can_engaged_brakes_lower_bound &&
        this->rear < config::can_engaged_brakes_upper_bound)
      return false;
    else
      return true;
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

  void update(volatile uint16_t air_pressure_data[config::adc_count]) {
    this->ebs = air_transferFcn.solve(static_cast<float>(air_pressure_data[0]));
    this->redundant =
        air_transferFcn.solve(static_cast<float>(air_pressure_data[1]));
    this->main =
        air_transferFcn.solve(static_cast<float>(air_pressure_data[2]));
  }

  bool check() {
    if (std::abs(this->ebs - this->redundant) > config::brake_press_deviation)
      return true;
    if (config::brake_lower_bound >= this->ebs ||
        this->ebs >= config::brake_upper_bound)
      return true;
    if (config::brake_lower_bound >= this->redundant ||
        this->redundant >= config::brake_upper_bound)
      return true;
    return false;
  }
};

/* USER CODE END PM */

/* Private variables
 * ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t adc_dma_buffer[3];

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

/* USER CODE BEGIN PFP */
void startTogglingWatchdog();
void stopTogglingWatchdog();
void can_filter_config(CAN_HandleTypeDef *hcan);
void can_driver_input_cb(const PUTM_CAN_M_driver_input_t &driver_input);
void ebs_error();

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
  Ebs_state ebs_state = EBS_INITIAL_CHECKUP;
  bool ebs_break = false;

  Brakes_data brakes;
  Air_pressure_data air_pressure;

  // Initialize CAN
  putm_ev_can::CanDriver can_m;
  can_filter_config(&hcan1);
  if (!can_m.Init(&hcan1)) {
    ebs_state = EBS_STOP_MONITORING;
  } else {
    can_m.RegisterCallback<PUTM_CAN_M_driver_input_t>(
        PUTM_CAN_M_DRIVER_INPUT_FRAME_ID, can_driver_input_cb);
  }

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, config::adc_count);

  Timer sdc_timeout_timer(config::sdc_settle_timeout);
  Timer valve_timeout_timer(config::valve_settle_timeout);
  Timer can_timeout_timer(config::can_timeout);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (true) {
    switch (ebs_state) {

    case (EBS_INITIAL_CHECKUP): {
      // Turn on checkup led
      led_warn.activate();

      // Check if SDC is working
      startTogglingWatchdog();
      sdc_timeout_timer.restart();
      while (!sdc_ready.isActive()) {
        if (ebs_break = sdc_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_error();
          break;
        }
      }
      if (ebs_break) {
        ebs_state = EBS_STOP_MONITORING;
        break;
      }
      stopTogglingWatchdog();
      sdc_timeout_timer.restart();
      while (sdc_ready.isActive()) {
        if (ebs_break = sdc_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_error();
          break;
        }
      }
      if (ebs_break) {
        ebs_state = EBS_STOP_MONITORING;
        break;
      }
      startTogglingWatchdog();

      // Check that the EBS energy storage is filled
      air_pressure.update(adc_dma_buffer);
      if (air_pressure.check()) {
        ebs_error();
        ebs_state = EBS_STOP_MONITORING;
        break;
      }

      // Check CAN
      can_timeout_timer.restart();
      while (!brake_data_can.status) {
        if (ebs_break = can_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_error();
          break;
        }
      }
      if (ebs_break) {
        ebs_state = EBS_STOP_MONITORING;
        break;
      }
      brakes.update(brake_data_can);
      brake_data_can.status = false;

      // Check that the brake pressure is built up correctly
      air_pressure.update(adc_dma_buffer);
      if (brakes.check_buildup(air_pressure.ebs)) {
        ebs_error();
        ebs_state = EBS_STOP_MONITORING;
        break;
      }

      // // Enable TS
      as_close_sdc.activate();

      // Wait for TS
      // Add checking TS from external signal/CAN

      // Check that the brake pressure is still built
      // up correctly #1
      valve1.activate();
      valve2.deactivate();
      HAL_Delay(config::valve_settle_delay);
      valve_timeout_timer.restart();
      while (brakes.check_holdup_front(air_pressure.ebs)) {
        if (ebs_break = valve_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_error();
          break;
        }

        can_timeout_timer.restart();
        while (!brake_data_can.status) {
          if (ebs_break = can_timeout_timer.checkIfTimedOutThenReset()) {
            ebs_error();
            break;
          }
        }
        if (ebs_break) {
          break;
        }
        brakes.update(brake_data_can);
        brake_data_can.status = false;
      }
      if (ebs_break) {
        ebs_state = EBS_STOP_MONITORING;
        break;
      }

      // Check that the brake pressure is still built
      // up correctly #2
      valve1.deactivate();
      valve2.activate();
      HAL_Delay(config::valve_settle_delay);
      valve_timeout_timer.restart();
      while (brakes.check_holdup_rear(air_pressure.ebs)) {
        if (ebs_break = valve_timeout_timer.checkIfTimedOutThenReset()) {
          ebs_error();
          break;
        }

        can_timeout_timer.restart();
        while (!brake_data_can.status) {
          if (ebs_break = can_timeout_timer.checkIfTimedOutThenReset()) {
            ebs_error();
            break;
          }
        }
        if (ebs_break) {
          break;
        }
        brakes.update(brake_data_can);
        brake_data_can.status = false;
      }
      if (ebs_break) {
        ebs_state = EBS_STOP_MONITORING;
        break;
      }
      valve1.activate();
      valve2.activate();

      // Start continous monitoring
      led_warn.deactivate();
      led_ok.activate();
      ebs_state = EBS_CONTINOUS_MONITORING;
      break;
    }

    case EBS_CONTINOUS_MONITORING: {

      // Monitor the storage of brake energy (air pressure)
      air_pressure.update(adc_dma_buffer);
      if (air_pressure.check()) {
        ebs_error();
        ebs_state = EBS_STOP_MONITORING;
        break;
      }
      // // Convert brake pressure data from if received from CAN
      // if (brake_data_can.status == true) {
      //   brakes.update(brake_data_can);
      //   brake_data_can.status = false;
      // }

      // Check RES state CAN OK?
      // AS OK?

      // // Check SDC
      // if (!sdc_ready.isActive()) {
      //   HAL_Delay(config::valve_settle_delay);
      //   valve_timeout_timer.restart();
      //   while (brakes.check_engaged()) {
      //     if (ebs_break = valve_timeout_timer.checkIfTimedOutThenReset()) {
      //       ebs_error();
      //       break;
      //     }

      //     can_timeout_timer.restart();
      //     while (!brake_data_can.status) {
      //       if (ebs_break = can_timeout_timer.checkIfTimedOutThenReset()) {
      //         ebs_error();
      //         break;
      //       }
      //     }
      //     if (ebs_break)
      //       break;

      //     brakes.update(brake_data_can);
      //     brake_data_can.status = false;
      //   }
      //   if (ebs_break) {
      //     ebs_state = EBS_STOP_MONITORING;
      //     break;
      //   }
      // }

      break;
    }
    case EBS_STOP_MONITORING: {
      if (sdc_ready.isActive()) {
        led_err.deactivate();
        ebs_break = false;
        ebs_state = EBS_INITIAL_CHECKUP;
        // Add proper ASB reset
      }
      break;
    }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
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

/* USER CODE BEGIN 4 */

void can_driver_input_cb(const PUTM_CAN_M_driver_input_t &driver_input) {
  brake_data_can.front = driver_input.brake_pressure_front;
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

void ebs_error() {
  led_ok.deactivate();
  led_warn.deactivate();
  led_err.activate();
  valve1.deactivate();
  valve2.deactivate();
  as_close_sdc.deactivate();
  return;
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
