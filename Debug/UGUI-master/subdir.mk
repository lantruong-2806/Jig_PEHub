################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/01-GREMSY/02-FW/Demo_OLED_Btn/Demo_OLED_Btn/Third_Lib/UGUI-master/ugui.c 

OBJS += \
./UGUI-master/ugui.o 

C_DEPS += \
./UGUI-master/ugui.d 


# Each subdirectory must supply rules for building sources it contributes
UGUI-master/ugui.o: D:/01-GREMSY/02-FW/Demo_OLED_Btn/Demo_OLED_Btn/Third_Lib/UGUI-master/ugui.c UGUI-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/GremsyLib" -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/Third_Lib" -I"D:/01-GREMSY/02-FW/WORKSPACE TEST/Jig_PEHub/Third_Lib/UGUI-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UGUI-2d-master

clean-UGUI-2d-master:
	-$(RM) ./UGUI-master/ugui.d ./UGUI-master/ugui.o

.PHONY: clean-UGUI-2d-master

