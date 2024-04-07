/*
 * adc_driver.h
 *
 *  Created on: Apr 6, 2024
 *      Author: vasilaki
 */

#ifndef ADC_ADC_DRIVER_H_
#define ADC_ADC_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

uint16_t read_ADC();
float convert_to_voltage(uint16_t adc_value);

#endif /* ADC_ADC_DRIVER_H_ */
