################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GremsyLib/gremsy_button.c \
../GremsyLib/gremsy_ssd1306.c \
../GremsyLib/gremsy_thread.c 

OBJS += \
./GremsyLib/gremsy_button.o \
./GremsyLib/gremsy_ssd1306.o \
./GremsyLib/gremsy_thread.o 

C_DEPS += \
./GremsyLib/gremsy_button.d \
./GremsyLib/gremsy_ssd1306.d \
./GremsyLib/gremsy_thread.d 


# Each subdirectory must supply rules for building sources it contributes
GremsyLib/%.o: ../GremsyLib/%.c GremsyLib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/GremsyLib" -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/Third_Lib" -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/Third_Lib/UGUI-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-GremsyLib

clean-GremsyLib:
	-$(RM) ./GremsyLib/gremsy_button.d ./GremsyLib/gremsy_button.o ./GremsyLib/gremsy_ssd1306.d ./GremsyLib/gremsy_ssd1306.o ./GremsyLib/gremsy_thread.d ./GremsyLib/gremsy_thread.o

.PHONY: clean-GremsyLib

