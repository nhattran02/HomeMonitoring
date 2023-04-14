################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (7-2018-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DHT11/DHT11.c 

OBJS += \
./DHT11/DHT11.o 

C_DEPS += \
./DHT11/DHT11.d 


# Each subdirectory must supply rules for building sources it contributes
DHT11/%.o DHT11/%.su: ../DHT11/%.c DHT11/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tranm/STM32CubeIDE/workspace_1.11.0/BTL_LTN/DHT11" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DHT11

clean-DHT11:
	-$(RM) ./DHT11/DHT11.d ./DHT11/DHT11.o ./DHT11/DHT11.su

.PHONY: clean-DHT11

