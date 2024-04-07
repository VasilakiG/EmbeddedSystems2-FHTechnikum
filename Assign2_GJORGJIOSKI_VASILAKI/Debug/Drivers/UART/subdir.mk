################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UART/uart_driver.c 

OBJS += \
./Drivers/UART/uart_driver.o 

C_DEPS += \
./Drivers/UART/uart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UART/%.o Drivers/UART/%.su Drivers/UART/%.cyclo: ../Drivers/UART/%.c Drivers/UART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/UART -I../Drivers/ADC -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-UART

clean-Drivers-2f-UART:
	-$(RM) ./Drivers/UART/uart_driver.cyclo ./Drivers/UART/uart_driver.d ./Drivers/UART/uart_driver.o ./Drivers/UART/uart_driver.su

.PHONY: clean-Drivers-2f-UART

