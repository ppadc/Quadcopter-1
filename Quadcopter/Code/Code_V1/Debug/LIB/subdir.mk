################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LIB/MY_NRF24.c \
../LIB/code.c 

OBJS += \
./LIB/MY_NRF24.o \
./LIB/code.o 

C_DEPS += \
./LIB/MY_NRF24.d \
./LIB/code.d 


# Each subdirectory must supply rules for building sources it contributes
LIB/%.o LIB/%.su LIB/%.cyclo: ../LIB/%.c LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tanda/Desktop/Sensor/Code_V1/LIB" -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LIB

clean-LIB:
	-$(RM) ./LIB/MY_NRF24.cyclo ./LIB/MY_NRF24.d ./LIB/MY_NRF24.o ./LIB/MY_NRF24.su ./LIB/code.cyclo ./LIB/code.d ./LIB/code.o ./LIB/code.su

.PHONY: clean-LIB

