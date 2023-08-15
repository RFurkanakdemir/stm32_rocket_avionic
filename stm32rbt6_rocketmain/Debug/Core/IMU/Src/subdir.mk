################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/IMU/Src/bmi160.c \
../Core/IMU/Src/bmi160_wrapper.c \
../Core/IMU/Src/common_porting.c 

OBJS += \
./Core/IMU/Src/bmi160.o \
./Core/IMU/Src/bmi160_wrapper.o \
./Core/IMU/Src/common_porting.o 

C_DEPS += \
./Core/IMU/Src/bmi160.d \
./Core/IMU/Src/bmi160_wrapper.d \
./Core/IMU/Src/common_porting.d 


# Each subdirectory must supply rules for building sources it contributes
Core/IMU/Src/%.o Core/IMU/Src/%.su Core/IMU/Src/%.cyclo: ../Core/IMU/Src/%.c Core/IMU/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/filter/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/LoRa/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/Presssure/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/IMU/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/GPS/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-IMU-2f-Src

clean-Core-2f-IMU-2f-Src:
	-$(RM) ./Core/IMU/Src/bmi160.cyclo ./Core/IMU/Src/bmi160.d ./Core/IMU/Src/bmi160.o ./Core/IMU/Src/bmi160.su ./Core/IMU/Src/bmi160_wrapper.cyclo ./Core/IMU/Src/bmi160_wrapper.d ./Core/IMU/Src/bmi160_wrapper.o ./Core/IMU/Src/bmi160_wrapper.su ./Core/IMU/Src/common_porting.cyclo ./Core/IMU/Src/common_porting.d ./Core/IMU/Src/common_porting.o ./Core/IMU/Src/common_porting.su

.PHONY: clean-Core-2f-IMU-2f-Src

