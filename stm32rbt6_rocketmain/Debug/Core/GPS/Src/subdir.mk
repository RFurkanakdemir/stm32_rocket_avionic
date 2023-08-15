################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/GPS/Src/lwgps.c 

OBJS += \
./Core/GPS/Src/lwgps.o 

C_DEPS += \
./Core/GPS/Src/lwgps.d 


# Each subdirectory must supply rules for building sources it contributes
Core/GPS/Src/%.o Core/GPS/Src/%.su Core/GPS/Src/%.cyclo: ../Core/GPS/Src/%.c Core/GPS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/filter/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/LoRa/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/Presssure/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/IMU/Inc" -I"C:/Users/rfrkn/Desktop/Stm32_examples/stm32rbt6_rocketmain/Core/GPS/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-GPS-2f-Src

clean-Core-2f-GPS-2f-Src:
	-$(RM) ./Core/GPS/Src/lwgps.cyclo ./Core/GPS/Src/lwgps.d ./Core/GPS/Src/lwgps.o ./Core/GPS/Src/lwgps.su

.PHONY: clean-Core-2f-GPS-2f-Src

