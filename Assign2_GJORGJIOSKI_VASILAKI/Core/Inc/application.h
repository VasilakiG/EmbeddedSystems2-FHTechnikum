/*
 * application.h
 *
 *  Created on: Mar 18, 2024
 *      Author: vasilaki
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "adc_driver.h"
#include "uart_driver.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

static ADC_HandleTypeDef hadc1;
static UART_HandleTypeDef huart2;

uint32_t BAUD_RATE = 115200;

void initialize_ADC();
void initialize_UART();
float read_ADC_value();
void send_voltage_over_UART(float voltage);

#endif /* INC_APPLICATION_H_ */
