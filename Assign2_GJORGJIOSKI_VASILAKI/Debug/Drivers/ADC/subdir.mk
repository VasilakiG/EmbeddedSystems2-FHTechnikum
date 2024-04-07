################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ADC/adc_driver.c 

OBJS += \
./Drivers/ADC/adc_driver.o 

C_DEPS += \
./Drivers/ADC/adc_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ADC/%.o Drivers/ADC/%.su Drivers/ADC/%.cyclo: ../Drivers/ADC/%.c Drivers/ADC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/UART -I../Drivers/ADC -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ADC

clean-Drivers-2f-ADC:
	-$(RM) ./Drivers/ADC/adc_driver.cyclo ./Drivers/ADC/adc_driver.d ./Drivers/ADC/adc_driver.o ./Drivers/ADC/adc_driver.su

.PHONY: clean-Drivers-2f-ADC

