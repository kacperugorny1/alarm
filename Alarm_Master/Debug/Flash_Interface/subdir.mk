################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Flash_Interface/flash_interface.c 

OBJS += \
./Flash_Interface/flash_interface.o 

C_DEPS += \
./Flash_Interface/flash_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Flash_Interface/%.o Flash_Interface/%.su Flash_Interface/%.cyclo: ../Flash_Interface/%.c Flash_Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Expander" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/SIM800L_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Flash_Interface

clean-Flash_Interface:
	-$(RM) ./Flash_Interface/flash_interface.cyclo ./Flash_Interface/flash_interface.d ./Flash_Interface/flash_interface.o ./Flash_Interface/flash_interface.su

.PHONY: clean-Flash_Interface

