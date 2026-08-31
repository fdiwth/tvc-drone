################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/Fusion/FusionAhrs.c \
../Middlewares/Third_Party/Fusion/FusionBias.c \
../Middlewares/Third_Party/Fusion/FusionCompass.c 

OBJS += \
./Middlewares/Third_Party/Fusion/FusionAhrs.o \
./Middlewares/Third_Party/Fusion/FusionBias.o \
./Middlewares/Third_Party/Fusion/FusionCompass.o 

C_DEPS += \
./Middlewares/Third_Party/Fusion/FusionAhrs.d \
./Middlewares/Third_Party/Fusion/FusionBias.d \
./Middlewares/Third_Party/Fusion/FusionCompass.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/Fusion/%.o Middlewares/Third_Party/Fusion/%.su Middlewares/Third_Party/Fusion/%.cyclo: ../Middlewares/Third_Party/Fusion/%.c Middlewares/Third_Party/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Middlewares/Third_Party/Fusion -I../Drivers/Inc -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-Fusion

clean-Middlewares-2f-Third_Party-2f-Fusion:
	-$(RM) ./Middlewares/Third_Party/Fusion/FusionAhrs.cyclo ./Middlewares/Third_Party/Fusion/FusionAhrs.d ./Middlewares/Third_Party/Fusion/FusionAhrs.o ./Middlewares/Third_Party/Fusion/FusionAhrs.su ./Middlewares/Third_Party/Fusion/FusionBias.cyclo ./Middlewares/Third_Party/Fusion/FusionBias.d ./Middlewares/Third_Party/Fusion/FusionBias.o ./Middlewares/Third_Party/Fusion/FusionBias.su ./Middlewares/Third_Party/Fusion/FusionCompass.cyclo ./Middlewares/Third_Party/Fusion/FusionCompass.d ./Middlewares/Third_Party/Fusion/FusionCompass.o ./Middlewares/Third_Party/Fusion/FusionCompass.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-Fusion

