/*
 * uart_driver.c
 *
 *  Created on: Apr 6, 2024
 *      Author: vasilaki
 */

/* Includes ------------------------------------------------------------------*/
#include "uart_driver.h"
#include "stm32l4xx_hal.h"

static UART_HandleTypeDef *uart_handle = NULL;

void UART_Init(UART_HandleTypeDef *huart)
{
	uart_handle = huart;
}

/**
  * @brief Implementation of sending string over UART
  * @param str: The string that is sent over UART
  * @retval None
  */
void send_string_over_UART(char *str)
{
	if (uart_handle != NULL)
	{
		HAL_UART_Transmit(uart_handle, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	}
}


