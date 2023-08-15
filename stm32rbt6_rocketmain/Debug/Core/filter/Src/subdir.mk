################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/filter/Src/FusionAHRS.c \
../Core/filter/Src/LowPassFilter.c \
../Core/filter/Src/NotchFilter.c \
../Core/filter/Src/quaternion.c 

OBJS += \
./Core/filter/Src/FusionAHRS.o \
./Core/filter/Src/LowPassFilter.o \
./Core/filter/Src/NotchFilter.o \
./Core/filter/Src/quaternion.o 

C_DEPS += \
./Core/filter/Src/FusionAHRS.d \
./Core/filter/Src/LowPassFilter.d \
./Core/filter/Src/NotchFilter.d \
./Core/filter/Src/quaternion.d 


# Each subdirectory must supply rules for building sources it contributes
Core/filter/Src/%.o Core/filter/Src/%.su Core/filter/Src/%.cyclo: ../Core/filter/Src/%.c Core/filter/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/filter/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/LoRa/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/Presssure/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/IMU/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/GPS/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-filter-2f-Src

clean-Core-2f-filter-2f-Src:
	-$(RM) ./Core/filter/Src/FusionAHRS.cyclo ./Core/filter/Src/FusionAHRS.d ./Core/filter/Src/FusionAHRS.o ./Core/filter/Src/FusionAHRS.su ./Core/filter/Src/LowPassFilter.cyclo ./Core/filter/Src/LowPassFilter.d ./Core/filter/Src/LowPassFilter.o ./Core/filter/Src/LowPassFilter.su ./Core/filter/Src/NotchFilter.cyclo ./Core/filter/Src/NotchFilter.d ./Core/filter/Src/NotchFilter.o ./Core/filter/Src/NotchFilter.su ./Core/filter/Src/quaternion.cyclo ./Core/filter/Src/quaternion.d ./Core/filter/Src/quaternion.o ./Core/filter/Src/quaternion.su

.PHONY: clean-Core-2f-filter-2f-Src

