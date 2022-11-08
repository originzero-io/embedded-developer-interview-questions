/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEADER_LENGTH 12
#define MAX_PACKET_SIZE 100
#define MAX_CONTENT_SIZE (MAX_PACKET_SIZE - HEADER_LENGTH)

#define PACKET_BUFFER_DEPTH 4
#define START_BYTE 0xFF
#define STOP_BYTE 0xFE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

// Input stream to simulate communications
uint8_t input_stream[20] = { 0xFF, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0xAA,
		0x8F, 0x00, 0x03, 0xA0/*checksum*/, 0xAA, 0xAA, 0x00, /*CRC*/0x00, 0x00,
		0x00, 0x0F/*CRC*/, 0xFE };
uint16_t input_byte = 0;

// Receive buffer
uint8_t incoming_packet[MAX_PACKET_SIZE] = { };
uint16_t packet_index = 0;
uint8_t byte_receive = 0;

// Storage Buffer
uint8_t data_process[PACKET_BUFFER_DEPTH][MAX_PACKET_SIZE] = { };
// Shows amount of data ready for process
uint8_t data_process_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// CRC calculation
uint32_t crc32(const char *s, size_t n);

// Checksum calculation
static uint8_t checksum_base_calculate(uint32_t checksum_total_value);
uint8_t checksum_calculate(const uint8_t *s, size_t n);

// For debugging only
void interupt_sim();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart2, &byte_receive, 1);

	// For debugging
	const uint8_t hey[] = "HEY!\r\n";
	HAL_UART_Transmit(&huart2, hey, sizeof(hey), HAL_MAX_DELAY);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Act like you received interrupt
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
			interupt_sim();
		}

		// Process received package
		if (data_process_index > 0) {
			uint16_t current_process = data_process_index - 1; // Array index to use

			// Extract data size
			uint16_t data_size = ((data_process[current_process][1] << 8)
					| (data_process[current_process][2]));

			// Calculate CRC
			uint32_t crc_calc = crc32(
					(const char*) (data_process[data_process_index]),
					data_size - 5);

			// Extract CRC
			uint32_t crc_recv = 0;
			crc_recv |=
					(uint32_t) (data_process[current_process][data_size - 5])
							<< 24;
			crc_recv |=
					(uint32_t) (data_process[current_process][data_size - 4])
							<< 16;
			crc_recv |=
					(uint32_t) (data_process[current_process][data_size - 3])
							<< 8;
			crc_recv |=
					(uint32_t) (data_process[current_process][data_size - 2]);

			// Stop byte check

			// Compare CRCs
			if (crc_recv == crc_calc) {
				// Valid Data Do Stuff
				const uint8_t str[] = "Valid data!\r\n";
				HAL_UART_Transmit(&huart2, str, sizeof(str), HAL_MAX_DELAY);
				data_process_index--;
			} else {
				// CRC Error
				const uint8_t str[] = "CRC Error!\r\n";
				HAL_UART_Transmit(&huart2, str, sizeof(str), HAL_MAX_DELAY);
				data_process_index--;
			}

		}
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Consider look up table implementation
uint32_t crc32(const char *s, size_t n) {
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < n; i++) {
		char ch = s[i];
		for (size_t j = 0; j < 8; j++) {
			uint32_t b = (ch ^ crc) & 1;
			crc >>= 1;
			if (b)
				crc = crc ^ 0xEDB88320;
			ch >>= 1;
		}
	}

	return ~crc;
}

static uint8_t checksum_base_calculate(uint32_t checksum_total_value) {
	checksum_total_value = ((checksum_total_value ^ 255) + 1);
	return ((uint8_t*) &checksum_total_value)[3];
}

uint8_t checksum_calculate(const uint8_t *s, size_t n) {
	if (NULL == s) {
		return 0;
	}

	uint32_t checksum = 0;
	for (size_t i = 0; i < n; i++) {
		checksum += s[i];
	}

	return checksum_base_calculate(checksum);
	//return 0xA0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Value already read into buffer

	// Will be used to check end of package later
	static uint16_t data_length = 0;

	// Check if start byte has received
	if (incoming_packet[0] == START_BYTE) {
		packet_index++; // Increase index for next element

		// Checksum check
		if (packet_index == HEADER_LENGTH) {
			uint8_t checksum_calc = checksum_calculate(incoming_packet,
			HEADER_LENGTH - 1);

			if (checksum_calc != incoming_packet[HEADER_LENGTH - 1]) {
				// Invalid Checksum
				packet_index = 0;	// Reset buffer

				// Debugging purposes only. Error report.
				const uint8_t str[] = "Checksum error!";
				HAL_UART_Transmit(&huart2, str, sizeof(str), HAL_MAX_DELAY);
			} else {
				// Valid Checksum
				// Find the end index of the packet
				data_length =
						((incoming_packet[1] << 8) | (incoming_packet[2]));
			}
		} else if ((packet_index > HEADER_LENGTH)
				&& (packet_index > data_length)) {
			// End of packet
			// Maybe check for stop byte?

			// Copy package to process buffer
			memcpy(data_process[data_process_index], incoming_packet,
					packet_index);
			data_process_index++;	// Increase number of data to be processed
			packet_index = 0;	// Reset buffer
		}
	}
	// Debugging purposes only.
	input_byte++;	// Increment receive array
	if (input_byte == sizeof(input_stream)) {
		// Reset receive array
		input_byte = 0;
	}


	// Enable interrupt and read next data coming
	HAL_UART_Receive_IT(&huart2, &byte_receive, 1);
}

void interupt_sim() {
	// Read value into buffer
	// Will be changed when code converted to real interrupt
	incoming_packet[packet_index] = input_stream[input_byte];

	// Will be used to check end of package later
	static uint16_t data_length = 0;

	// Check if start byte has received
	if (incoming_packet[0] == START_BYTE) {
		packet_index++; // Increase index for next element

		// Checksum check
		if (packet_index == HEADER_LENGTH) {
			uint8_t checksum_calc = checksum_calculate(incoming_packet,
			HEADER_LENGTH - 1);

			if (checksum_calc != incoming_packet[HEADER_LENGTH - 1]) {
				// Invalid Checksum
				packet_index = 0;	// Reset buffer

				// Debugging purposes only. Error report.
				const uint8_t str[] = "Checksum error!";
				HAL_UART_Transmit(&huart2, str, sizeof(str), HAL_MAX_DELAY);
			} else {
				// Valid Checksum
				// Find the end index of the packet
				data_length =
						((incoming_packet[1] << 8) | (incoming_packet[2]));
			}
		} else if ((packet_index > HEADER_LENGTH)
				&& (packet_index > data_length)) {
			// End of packet
			// Maybe check for stop byte?

			// Copy package to process buffer
			memcpy(data_process[data_process_index], incoming_packet,
					packet_index);
			data_process_index++;	// Increase number of data to be processed
			packet_index = 0;	// Reset buffer
		}
	}
	// Debugging purposes only.
	input_byte++;	// Increment receive array
	if (input_byte == sizeof(input_stream)) {
		// Reset receive array
		input_byte = 0;
	}
}

/* USER CODE END 4 */

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
