/*
 * adc_driver.c
 *
 *  Created on: Apr 6, 2024
 *      Author: vasilaki
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "adc_driver.h"

static ADC_HandleTypeDef *adc_handle = NULL;

void ADC_Init(ADC_HandleTypeDef *hadc)
{
	adc_handle = hadc;
}


/**
  * @brief Function that reads the value of the ADC
  * @param None
  * @retval The 16bit value of the ADC
  */
uint16_t read_ADC()
{
	if (adc_handle != NULL)
	{
		HAL_ADC_Start(adc_handle);
		HAL_ADC_PollForConversion(adc_handle, HAL_MAX_DELAY);
	}
	else
	{
        // Handle the case where the ADC handle is not initialized
        // Return some default value or throw an error
        return 0;
	}

	return HAL_ADC_GetValue(adc_handle);
}


/**
  * @brief Function that converts the value of the ADC from 16bit value to float
  * @param[in] adc_value: The 16bit value that's read from the ADC
  * @retval The float representation of the ADC value
  */
float convert_to_voltage(uint16_t adc_value)
{
    // Vref 3.3V and 12-bit ADC resolution
    return ((float)adc_value / 4095.0f) * 3.3f;
}

