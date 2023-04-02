################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ov5640/ov5640.c 

OBJS += \
./Drivers/Components/ov5640/ov5640.o 

C_DEPS += \
./Drivers/Components/ov5640/ov5640.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ov5640/%.o Drivers/Components/ov5640/%.su Drivers/Components/ov5640/%.cyclo: ../Drivers/Components/ov5640/%.c Drivers/Components/ov5640/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ov5640

clean-Drivers-2f-Components-2f-ov5640:
	-$(RM) ./Drivers/Components/ov5640/ov5640.cyclo ./Drivers/Components/ov5640/ov5640.d ./Drivers/Components/ov5640/ov5640.o ./Drivers/Components/ov5640/ov5640.su

.PHONY: clean-Drivers-2f-Components-2f-ov5640

