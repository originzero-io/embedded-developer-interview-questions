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
#include "stdbool.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HEADER_LENGTH 12
#define MAX_PACKET_SIZE 100
#define MAX_CONTENT_SIZE (MAX_PACKET_SIZE - HEADER_LENGTH)

typedef struct {
	uint8_t start_byte;
	uint16_t data_length;
	uint8_t target_id;
	uint32_t packet_number;
	uint16_t content_length;
	uint8_t checksum;
	uint8_t content[MAX_CONTENT_SIZE];
} __attribute__((packed, aligned(4))) packet_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_BUFFER_DEPTH 4
#define START_BYTE 0xFF
#define STOP_BYTE 0xFE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t input_stream[20] = { 0xFF, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0xAA, 0x8F, 0x00,
		0x03, 0xA0/*checksum*/, 0xFF, 0xAA, 0x00, /*CRC*/0x00, 0x00, 0x00,
		0x00/*CRC*/, 0xFE };

uint16_t input_byte = 0;
packet_t incoming_packet = { };
uint8_t *p_incoming_packet = (uint8_t*) &incoming_packet;
uint16_t packet_index = 0;

bool packet_started = false;
bool packet_ready = false;

packet_t packet_buffer[PACKET_BUFFER_DEPTH] = { };
uint16_t buffer_index;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint32_t calculate_crc(uint8_t *data, uint16_t data_length);
static uint8_t checksum_base_calculate(uint32_t checksum_total_value);
uint8_t checksum_calculate(const uint8_t *s, size_t n);
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
			interupt_sim();
		}

		if (packet_ready) {
			uint32_t crc_calc = calculate_crc((incoming_packet.content),
					incoming_packet.content_length);
			uint32_t *p_crc_recv; // Maybe use memcpy
			p_crc_recv =
					(uint32_t*) &(incoming_packet.content[incoming_packet.content_length]);
			if (*p_crc_recv == crc_calc) {
				memcpy(&packet_buffer[buffer_index], &incoming_packet,
						sizeof(packet_t));
			}

		}

		/* USER CODE BEGIN 3 */
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

uint32_t calculate_crc(uint8_t *data, uint16_t data_length) {
	return 0x0000000F;
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
}

void interupt_sim() {
	*p_incoming_packet = input_stream[input_byte];
	if (packet_started) {
		packet_index++;
		if (packet_index == HEADER_LENGTH) {
			uint8_t checksum = checksum_calculate(
					(const uint8_t*) &incoming_packet, HEADER_LENGTH - 1);
			if (checksum != incoming_packet.checksum) {
				p_incoming_packet = (uint8_t*) &incoming_packet;
				packet_index = 0;
				packet_started = false;
			}
		} else if (packet_index == incoming_packet.data_length) {
			packet_ready = true;
			p_incoming_packet = (uint8_t*) &incoming_packet;
		}
		p_incoming_packet++;
	} else if (*p_incoming_packet == START_BYTE) {
		packet_started = true;
		p_incoming_packet++;
		packet_index++;
	}
	input_byte++;
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
