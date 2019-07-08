/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "network_data.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_buffer.h"
#include "debug_trace.h"
#include "timer_sched.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "mnist_predict.pb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct tp_resp {
	float digits[10];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 100
#define UART_BUFFER_SIZE 256

#define DEBUG_PORT GPIOG
#define DEBUG_PIN GPIO_PIN_7

#define PROJ_VERSION 100

#define AI_BUFFER_NULL(ptr_)  AI_BUFFER_OBJ_INIT( \
		AI_BUFFER_FORMAT_NONE|AI_BUFFER_FMT_FLAG_CONST, \
		0, 0, 0, 0, \
		AI_HANDLE_PTR(ptr_))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

DECLARE_COMM_BUFFER(dbg_uart, UART_BUFFER_SIZE, UART_BUFFER_SIZE);
uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];

__IO uint32_t glb_tmr_1ms = 0;
__IO uint8_t tmp_rx = 0;
__IO uint32_t rx_timeout = 0;

uint32_t trace_levels;

/* Benchmark timer object */
struct obj_timer_t * benchmark_timer;

/* Create the list head for the timer */
static LIST_HEAD(obj_timer_list);

/* output and input streams for nanopb */
pb_ostream_t ostream;
pb_istream_t istream;

ai_handle network;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void debug_pin_init(void);
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void pb_send_stream(pb_ostream_t * s)
{
	HAL_UART_Transmit(&huart6, tx_buffer, s->bytes_written, 100);
	s->bytes_written = 0;
}

static inline void pb_send_version()
{
	pb_msg_version msg = pb_msg_version_init_zero;

	msg.version = PROJ_VERSION;

	if (pb_encode(&ostream, pb_msg_version_fields, &msg)) {
		pb_send_stream(&ostream);
//		TRACE(("OK\n"));
	}
	else {
		TRACE(("Failed to encode\n"));
	}

}

static inline void pb_recv_digit()
{
	pb_msg_predict_req msg = pb_msg_predict_req_init_zero;

//	memcpy(msg.buffer.bytes, dbg_uart.rx_buffer, dbg_uart.rx_ptr_in);
//	msg.buffer.size = dbg_uart.rx_ptr_in;

	TRACE(("Received: %d bytes\n", dbg_uart.rx_ptr_in));

	bool status = pb_decode(&istream, pb_msg_predict_req_fields, &msg);
	if (!status) {
		TRACE(("Failed to decode.\n"));
	}
	else {
		TRACE(("Decoded...\n"));
	}

    ai_buffer ai_input[1];
    ai_buffer ai_output[1];

    ai_input[0].n_batches  = 1;
    ai_input[0].data = AI_HANDLE_PTR(dbg_uart.rx_buffer);
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(dbg_uart.tx_buffer);

	ai_i32 resp = ai_network_run(network,
			ai_input,
			ai_output);
	if (resp <= 0) {
		TRACE(("Failed: ai_network_run\n"));
	}
	else {
		TRACE(("AI RUN, OK: %lu\n", resp));
		for (int i=0; i<resp; i++) {
			TRACE(("%02X,", tx_buffer[i]));
		}
		TRACE(("\n"));
	}

}

static inline int pb_send_reply()
{
	int ret = 0;

	struct tp_resp resp;

	pb_msg_predict_resp msg = pb_msg_predict_resp_init_zero;
//	for (int i=0; i<10; i++) {
//		resp.digits[i] = 0.1234 + i;
//		TRACE(("Number: %f\n", resp.digits[i]));
//	}
	msg.exec_time = 13000;
	msg.found_digit = 8;
	memcpy((void*) &msg.resp.bytes, &resp, sizeof(struct tp_resp));
	msg.resp.size = sizeof(struct tp_resp);

//	TRACE(("---------------------\n"));
//	float * f = (float*) &msg.resp.bytes;
//	for (int i=0; i<10; i++) {
//		TRACE(("Number: %f\n", *(f + i)));
//	}

	pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t*) tx_buffer, TX_BUFFER_SIZE);
	bool status = pb_encode(&stream, pb_msg_predict_resp_fields, &msg);
	if (!status) {
		TRACE(("Failed to encode\n"));
		return EXIT_FAILURE;
	}

//	TRACE(("Encode succeded:\n")); /* prints !!!Hello World!!! */
//	for (int i=0; i<stream.bytes_written; i++) {
//		TRACE(("%02X,", tx_buffer[i]));
//	}
//	TRACE(("\n"));
//
//	TRACE(("PB data:\n"));
	HAL_UART_Transmit(&huart6, tx_buffer, stream.bytes_written, 100);
	return ret;
}

void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	if (!strncmp((char*) buffer, "PBTEST1", 7)) {
		pb_send_version();
	}
	else if (!strncmp((char*) buffer, "PBTEST2", 7)) {
		pb_recv_digit();
	}
	/* try decode nanopb messages */
	else {
		pb_recv_digit();
	}
}


static inline void toggle_debug_pin(void)
{
	/* First toggle */
	DEBUG_PORT->ODR |= DEBUG_PIN;
	DEBUG_PORT->ODR &= ~DEBUG_PIN;

	/* Second toggle */
	DEBUG_PORT->ODR |= DEBUG_PIN;
	DEBUG_PORT->ODR &= ~DEBUG_PIN;
}


void main_loop()
{
	if (glb_tmr_1ms) {
		glb_tmr_1ms = 0;

		mod_timer_polling(&obj_timer_list);

		if (rx_timeout) rx_timeout--;
		if (rx_timeout == 1) {
			rx_timeout = 0;
			dbg_uart.rx_buffer[dbg_uart.rx_ptr_in] = 0; // terminate string
//			TRACE(("Received: %s\n", dbg_uart.rx_buffer));
			dbg_uart_parser(dbg_uart.rx_buffer, dbg_uart.rx_ptr_in, 0);
			dbg_uart.rx_ptr_in = 0;
		}
	}
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
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_CRC_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

	trace_levels_set(
			0
			| TRACE_LEVEL_DEFAULT
			,1);

	HAL_UART_Receive_IT(&huart6, (uint8_t*) &tmp_rx, 1);

	/* Init debug pin */
	debug_pin_init();

	ostream = pb_ostream_from_buffer((pb_byte_t*) tx_buffer, TX_BUFFER_SIZE);
	istream = pb_istream_from_buffer((pb_byte_t*) rx_buffer, RX_BUFFER_SIZE);

  /* USER CODE END 2 */
  printf("Program started\n");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	// Initialize the network

    AI_ALIGNED(4)
    static ai_u8 activations[AI_MNETWORK_DATA_ACTIVATIONS_SIZE];

//    static ai_float in_data[AI_MNETWORK_IN_1_SIZE];
//    static ai_float out_data[AI_MNETWORK_OUT_1_SIZE];

    ai_error err = ai_mnetwork_create(AI_NETWORK_MODEL_NAME, &network, NULL);
	if (err.type) {
		TRACE(("Error: ai_mnetwork_create\n"));
	}
	else {
		TRACE(("Network '%s' created...\n", AI_NETWORK_MODEL_NAME));
	}

	const ai_network_params params = {
	            AI_BUFFER_NULL(NULL),
	            AI_BUFFER_NULL(activations) };

	if (!ai_mnetwork_init(network, &params)) {
		TRACE(("Error: ai_mnetwork_init\n"));
	}
	else {
		TRACE(("Network initialized...\n"));
	}

  while (1)
  {
    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
  	  main_loop();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

}

/* USER CODE BEGIN 4 */
static void debug_pin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = DEBUG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Receive byte from uart */
	rx_timeout = 100;
	dbg_uart.rx_buffer[0xFF & dbg_uart.rx_ptr_in] = tmp_rx;
	dbg_uart.rx_ptr_in++;
	HAL_UART_Receive_IT(&huart6, (uint8_t*) &tmp_rx, 1);
}


int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t*) &ch, 1, 10);
	return ch;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
