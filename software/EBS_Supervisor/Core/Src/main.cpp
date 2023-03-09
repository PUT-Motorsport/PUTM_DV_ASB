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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <functional>
#include <math.h>

#include "timer.hpp"
#include "gpioElements.hpp"
#include "PUTM_EV_CAN_LIBRARY/lib/can_interface.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USE_TEST 1

#define USE_TEST_POINTS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct Fun
{
	float a;
	float b;
	float solve(float x) { return a * x + b; }
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

const int adc_count = 3;

volatile uint16_t adc_dma_buffer[3];

uint16_t can_timeout_counter;

volatile Fun air_fun = { 0.0026308866087872f, -2.00210470928703f };

volatile float air_ebs;
volatile float air_redundant;
volatile float air_main;

/* GPIO Section */
GpioOutElement led_ok(LD_OK_GPIO_Port, LD_OK_Pin);
GpioOutElement led_warn(LD_WARN_GPIO_Port, LD_WARN_Pin);
GpioOutElement led_err(LD_ERR_GPIO_Port, LD_ERR_Pin);
GpioOutElement led_chk(LD_CHK_GPIO_Port, LD_CHK_Pin);

GpioOutElement as_close_sdc(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin);
GpioOutElement as_mode(AS_MODE_GPIO_Port, AS_MODE_Pin);

GpioOutElement spare1(spare1_GPIO_Port, spare1_Pin);
GpioOutElement spare2(spare2_GPIO_Port, spare2_Pin);

GpioOutElement valve1(VALVE1_GPIO_Port, VALVE1_Pin);
GpioOutElement valve2(VALVE2_GPIO_Port, VALVE2_Pin);

GpioInElement sdc_ready(SDC_RDY_GPIO_Port, SDC_RDY_Pin);

static struct Config
{
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
}volatile config;

/* Tests */
#if USE_TEST_POINTS
static struct TestPoints
{
	struct ADC
	{
		volatile const uint16_t &air_ebs_adc = adc_dma_buffer[0];
		volatile const uint16_t &air_redundant_adc = adc_dma_buffer[1];
		volatile const uint16_t &air_main_adc = adc_dma_buffer[2];
	}volatile adc;
	volatile float &air_ebs_ = air_ebs;
	volatile float &air_redundant_adc_ = air_redundant;
	volatile float &air_main_adc_ = air_main;

	volatile bool sdc_ready = false;
} volatile test_points;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void starTogglingWatchdog();
void stopTogglingWatchdog();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, adc_count);

	//--------------------------------------------------------------------------------------------------------------------
	//Initial checkup

	//check if sdc is working
	Timer sdc_timeout_timer(config.sdc_settle_timeout);
	starTogglingWatchdog();
	while(true)
	{
		if(sdc_timeout_timer.checkIfTimedOutThenReset()) Error_Handler();
		if(sdc_ready.isActive()) break;
	}

	sdc_timeout_timer.restart();
	stopTogglingWatchdog();
	while(true)
	{
		if(sdc_timeout_timer.checkIfTimedOutThenReset()) Error_Handler();
		if(!sdc_ready.isActive()) break;
	}

	starTogglingWatchdog();

	//check pressure
	if(std::abs(air_ebs - air_redundant) > config.brake_press_deviation) Error_Handler();
	if(config.brake_lower_bound >= air_ebs || air_ebs >= config.brake_upper_bound) Error_Handler();
	if(config.brake_lower_bound >= air_redundant || air_redundant >= config.brake_upper_bound) Error_Handler();

	auto aq_main = PUTM_CAN::can.get_aq_main();

	//aq_main.break_press returns in kP and we use Bars, so times 0.01
	float brake_press_front = aq_main.brake_pressure_front * 0.01f + config.can_brake_offset;
	float brake_press_rear = aq_main.brake_pressure_rear * 0.01f + config.can_brake_offset;

	if(brake_press_front < config.press_tf_coef * air_ebs) Error_Handler();
	if(brake_press_rear < config.press_tf_coef * air_ebs) Error_Handler();

	//wait for tc enabled
	bool tc_activated = false;
	do
	{
		HAL_Delay(100);
		auto tc_main = PUTM_CAN::can.get_tc_main();
		tc_activated = tc_main.traction_control_enable;
	} while(!tc_activated);

	//check valves
	valve1.activate();
	valve2.deactivate();
	HAL_Delay(config.valve_settle_delay);
	Timer valve_timeout_timer(config.valve_settle_timeout);
	while(true)
	{
		if(valve_timeout_timer.checkIfTimedOutThenReset()) Error_Handler();
		if(PUTM_CAN::can.get_aq_main_new_data())
		{
			aq_main = PUTM_CAN::can.get_aq_main();

			brake_press_front = aq_main.brake_pressure_front * 0.01f + config.can_brake_offset;
			brake_press_rear = aq_main.brake_pressure_rear * 0.01f + config.can_brake_offset;
		}
		if(brake_press_front > config.press_tf_coef * air_ebs && brake_press_rear < config.can_brake_lower_bound) break;
	}
	valve1.activate();
	valve2.deactivate();
	HAL_Delay(config.valve_settle_delay);
	valve_timeout_timer.restart();
	while(true)
	{
		if(valve_timeout_timer.checkIfTimedOutThenReset()) Error_Handler();
		if(PUTM_CAN::can.get_aq_main_new_data())
		{
			aq_main = PUTM_CAN::can.get_aq_main();

			brake_press_front = aq_main.brake_pressure_front * 0.01f + config.can_brake_offset;
			brake_press_rear = aq_main.brake_pressure_rear * 0.01f + config.can_brake_offset;
		}
		if(brake_press_rear > config.press_tf_coef * air_ebs && brake_press_front < config.can_brake_lower_bound) break;
	}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//--------------------------------------------------------------------------------------------------------------------
	//continuous monitoring
	auto apps_main = PUTM_CAN::can.get_apps_main();
	auto prev_apps_counter_value = apps_main.counter;
	while (1)
	{
		//check sdc
		if(!sdc_ready.isActive())
		{
			HAL_Delay(config.valve_settle_delay);
			valve_timeout_timer.restart();
			while(true)
			{
				if(valve_timeout_timer.checkIfTimedOutThenReset()) Error_Handler();
				if(PUTM_CAN::can.get_aq_main_new_data())
				{
					aq_main = PUTM_CAN::can.get_aq_main();

					brake_press_front = aq_main.brake_pressure_front * 0.01f + config.can_brake_offset;
					brake_press_rear = aq_main.brake_pressure_rear * 0.01f + config.can_brake_offset;
				}
				if(config.can_engaged_brakes_lower_bound < brake_press_rear  && config.can_engaged_brakes_upper_bound > brake_press_rear &&
				   config.can_engaged_brakes_lower_bound < brake_press_front && config.can_engaged_brakes_upper_bound > brake_press_front) break;
			}
			break;
		}

		//check if can alive
		auto apps_main = PUTM_CAN::can.get_apps_main();
		counter_value = apps_main.counter();
		if(prev_apps_counter_value = counter_value) can_timeout_counter++;
		else can_timeout_counter = 0;
		if(can_timeout_counter >= config.can_timeout) Error_Handler();

		//check Ass
		//TODO: do ustalenia skąd mam to niby brać

		//check RES
		//TODO: też do ustalenia

		//check pressure
		if(std::abs(air_ebs - air_redundant) > config.brake_press_deviation) Error_Handler();
		if(config.brake_lower_bound >= air_ebs || air_ebs >= config.brake_upper_bound) Error_Handler();
		if(config.brake_lower_bound >= air_redundant || air_redundant >= config.brake_upper_bound) Error_Handler();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}

	//--------------------------------------------------------------------------------------------------------------------
	//Stop monitoring
	stopTogglingWatchdog();

	//TODO: idk if in stop monitoring mode sth should be done

	while(true) {}
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 511;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16524;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD_OK_Pin|LD_WARN_Pin|LD_ERR_Pin|LD_CHK_Pin
                          |AS_CLOSE_SDC_Pin|AS_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, spare1_Pin|spare2_Pin|VALVE1_Pin|VALVE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_OK_Pin LD_WARN_Pin LD_ERR_Pin LD_CHK_Pin
                           AS_CLOSE_SDC_Pin AS_MODE_Pin */
  GPIO_InitStruct.Pin = LD_OK_Pin|LD_WARN_Pin|LD_ERR_Pin|LD_CHK_Pin
                          |AS_CLOSE_SDC_Pin|AS_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : spare1_Pin spare2_Pin VALVE1_Pin VALVE2_Pin */
  GPIO_InitStruct.Pin = spare1_Pin|spare2_Pin|VALVE1_Pin|VALVE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_RDY_Pin */
  GPIO_InitStruct.Pin = SDC_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDC_RDY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConsCpltCallback(ADC_HandleTypeDef *hadc)
{
	air_ebs = air_fun.solve(float(adc_dma_buffer[0]));
	air_redundant_adc = air_fun.solve(float(adc_dma_buffer[1]));
	air_main = air_fun.solve(float(adc_dma_buffer[2]));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, adc_count);
}

void starTogglingWatchdog()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void stopTogglingWatchdog()
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
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

  led_err.activate();
  stopTogglingWatchdog();

  Timer can_send_timer(100);
  while (1)
  {
	  if(can_send_timer.checkIfTimedOutThenReset())
	  {
		  PUTM_CAN::DV_Ass dv_ass
		  {
			  .ass = PUTM_CAN::AutonomousSystemStatus::Emergency,
		  };
		  PUTM_CAN::Can_tx_message<PUTM_CAN::DV_Ass> can_sender(dv_ass, PUTM_CAN::can_tx_header_DV_ASS);
		  if(can_sender.send(hcan1) != HAL_StatusTypeDef::HAL_OK) led_chk.activate();
	  }

	//TODO: wysyłanie ramki do ASSI
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
