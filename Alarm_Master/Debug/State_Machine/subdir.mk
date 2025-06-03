################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../State_Machine/state_machine.c 

OBJS += \
./State_Machine/state_machine.o 

C_DEPS += \
./State_Machine/state_machine.d 


# Each subdirectory must supply rules for building sources it contributes
State_Machine/%.o State_Machine/%.su State_Machine/%.cyclo: ../State_Machine/%.c State_Machine/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Keyboard_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/LCD_Driver" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Display" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/State_Machine" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/Flash_Interface" -I"C:/Users/axeel/Desktop/alarm/Alarm_Master/SIM800L_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-State_Machine

clean-State_Machine:
	-$(RM) ./State_Machine/state_machine.cyclo ./State_Machine/state_machine.d ./State_Machine/state_machine.o ./State_Machine/state_machine.su

.PHONY: clean-State_Machine

