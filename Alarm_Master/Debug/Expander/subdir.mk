################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Expander/expander.c 

OBJS += \
./Expander/expander.o 

C_DEPS += \
./Expander/expander.d 


# Each subdirectory must supply rules for building sources it contributes
Expander/%.o Expander/%.su Expander/%.cyclo: ../Expander/%.c Expander/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Expander" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/SIM800L_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Expander

clean-Expander:
	-$(RM) ./Expander/expander.cyclo ./Expander/expander.d ./Expander/expander.o ./Expander/expander.su

.PHONY: clean-Expander

