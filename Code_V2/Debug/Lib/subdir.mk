################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/TD_Brushless.c \
../Lib/TD_GY86.c \
../Lib/TD_LQR.c \
../Lib/TD_Lowpass.c \
../Lib/TD_Madgiwick.c \
../Lib/TD_RF24L01.c 

OBJS += \
./Lib/TD_Brushless.o \
./Lib/TD_GY86.o \
./Lib/TD_LQR.o \
./Lib/TD_Lowpass.o \
./Lib/TD_Madgiwick.o \
./Lib/TD_RF24L01.o 

C_DEPS += \
./Lib/TD_Brushless.d \
./Lib/TD_GY86.d \
./Lib/TD_LQR.d \
./Lib/TD_Lowpass.d \
./Lib/TD_Madgiwick.d \
./Lib/TD_RF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/%.o Lib/%.su Lib/%.cyclo: ../Lib/%.c Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tanda/Desktop/Sensor/Code_V2/Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib

clean-Lib:
	-$(RM) ./Lib/TD_Brushless.cyclo ./Lib/TD_Brushless.d ./Lib/TD_Brushless.o ./Lib/TD_Brushless.su ./Lib/TD_GY86.cyclo ./Lib/TD_GY86.d ./Lib/TD_GY86.o ./Lib/TD_GY86.su ./Lib/TD_LQR.cyclo ./Lib/TD_LQR.d ./Lib/TD_LQR.o ./Lib/TD_LQR.su ./Lib/TD_Lowpass.cyclo ./Lib/TD_Lowpass.d ./Lib/TD_Lowpass.o ./Lib/TD_Lowpass.su ./Lib/TD_Madgiwick.cyclo ./Lib/TD_Madgiwick.d ./Lib/TD_Madgiwick.o ./Lib/TD_Madgiwick.su ./Lib/TD_RF24L01.cyclo ./Lib/TD_RF24L01.d ./Lib/TD_RF24L01.o ./Lib/TD_RF24L01.su

.PHONY: clean-Lib

