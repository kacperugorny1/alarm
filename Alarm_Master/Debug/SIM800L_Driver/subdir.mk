################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SIM800L_Driver/sim800l_driver.c 

OBJS += \
./SIM800L_Driver/sim800l_driver.o 

C_DEPS += \
./SIM800L_Driver/sim800l_driver.d 


# Each subdirectory must supply rules for building sources it contributes
SIM800L_Driver/%.o SIM800L_Driver/%.su SIM800L_Driver/%.cyclo: ../SIM800L_Driver/%.c SIM800L_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/SIM800L_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SIM800L_Driver

clean-SIM800L_Driver:
	-$(RM) ./SIM800L_Driver/sim800l_driver.cyclo ./SIM800L_Driver/sim800l_driver.d ./SIM800L_Driver/sim800l_driver.o ./SIM800L_Driver/sim800l_driver.su

.PHONY: clean-SIM800L_Driver

