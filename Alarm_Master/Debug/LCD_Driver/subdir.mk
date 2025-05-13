################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD_Driver/lcd_driver.c 

OBJS += \
./LCD_Driver/lcd_driver.o 

C_DEPS += \
./LCD_Driver/lcd_driver.d 


# Each subdirectory must supply rules for building sources it contributes
LCD_Driver/%.o LCD_Driver/%.su LCD_Driver/%.cyclo: ../LCD_Driver/%.c LCD_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD_Driver

clean-LCD_Driver:
	-$(RM) ./LCD_Driver/lcd_driver.cyclo ./LCD_Driver/lcd_driver.d ./LCD_Driver/lcd_driver.o ./LCD_Driver/lcd_driver.su

.PHONY: clean-LCD_Driver

