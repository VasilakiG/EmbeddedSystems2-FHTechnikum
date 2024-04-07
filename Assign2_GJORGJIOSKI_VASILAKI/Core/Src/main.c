/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "application.h"


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();

	SystemClock_Config();

	/* Initialization */
	initialize_GPIO__manual(LD3_GPIO_Port, LD3_Pin);
	initialize_ADC_manual(ADC1);
	initialize_UART_manual(USART2, BAUD_RATE);

	ADC_Init(&hadc1);
	UART_Init(&huart2);



	while (1)
	{
	  /* Read ADC value every second */
	  float voltage = read_ADC_value();
	  send_voltage_over_UART(voltage);

	  HAL_Delay(1000);
	}
}
