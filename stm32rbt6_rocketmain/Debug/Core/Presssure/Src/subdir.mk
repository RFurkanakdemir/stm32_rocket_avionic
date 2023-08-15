################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Presssure/Src/bmp3.c \
../Core/Presssure/Src/bmp3_common.c \
../Core/Presssure/Src/bmp3_wrapper.c 

OBJS += \
./Core/Presssure/Src/bmp3.o \
./Core/Presssure/Src/bmp3_common.o \
./Core/Presssure/Src/bmp3_wrapper.o 

C_DEPS += \
./Core/Presssure/Src/bmp3.d \
./Core/Presssure/Src/bmp3_common.d \
./Core/Presssure/Src/bmp3_wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Presssure/Src/%.o Core/Presssure/Src/%.su Core/Presssure/Src/%.cyclo: ../Core/Presssure/Src/%.c Core/Presssure/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/filter/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/LoRa/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/Presssure/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/IMU/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/GPS/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Presssure-2f-Src

clean-Core-2f-Presssure-2f-Src:
	-$(RM) ./Core/Presssure/Src/bmp3.cyclo ./Core/Presssure/Src/bmp3.d ./Core/Presssure/Src/bmp3.o ./Core/Presssure/Src/bmp3.su ./Core/Presssure/Src/bmp3_common.cyclo ./Core/Presssure/Src/bmp3_common.d ./Core/Presssure/Src/bmp3_common.o ./Core/Presssure/Src/bmp3_common.su ./Core/Presssure/Src/bmp3_wrapper.cyclo ./Core/Presssure/Src/bmp3_wrapper.d ./Core/Presssure/Src/bmp3_wrapper.o ./Core/Presssure/Src/bmp3_wrapper.su

.PHONY: clean-Core-2f-Presssure-2f-Src

