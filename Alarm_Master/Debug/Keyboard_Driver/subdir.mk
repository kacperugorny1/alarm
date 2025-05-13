################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Keyboard_Driver/keyboard.c 

OBJS += \
./Keyboard_Driver/keyboard.o 

C_DEPS += \
./Keyboard_Driver/keyboard.d 


# Each subdirectory must supply rules for building sources it contributes
Keyboard_Driver/%.o Keyboard_Driver/%.su Keyboard_Driver/%.cyclo: ../Keyboard_Driver/%.c Keyboard_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Keyboard_Driver

clean-Keyboard_Driver:
	-$(RM) ./Keyboard_Driver/keyboard.cyclo ./Keyboard_Driver/keyboard.d ./Keyboard_Driver/keyboard.o ./Keyboard_Driver/keyboard.su

.PHONY: clean-Keyboard_Driver

