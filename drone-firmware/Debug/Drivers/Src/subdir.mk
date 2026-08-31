################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/VL53L1X_api.c \
../Drivers/Src/VL53L1X_calibration.c \
../Drivers/Src/bmm150.c \
../Drivers/Src/bmp388.c \
../Drivers/Src/com.c \
../Drivers/Src/filter.c \
../Drivers/Src/icm42688p.c \
../Drivers/Src/ina226.c \
../Drivers/Src/lqr.c \
../Drivers/Src/motor.c \
../Drivers/Src/pmw3901.c \
../Drivers/Src/servo.c \
../Drivers/Src/signal.c \
../Drivers/Src/vl53l1_platform.c \
../Drivers/Src/w25q128.c 

OBJS += \
./Drivers/Src/VL53L1X_api.o \
./Drivers/Src/VL53L1X_calibration.o \
./Drivers/Src/bmm150.o \
./Drivers/Src/bmp388.o \
./Drivers/Src/com.o \
./Drivers/Src/filter.o \
./Drivers/Src/icm42688p.o \
./Drivers/Src/ina226.o \
./Drivers/Src/lqr.o \
./Drivers/Src/motor.o \
./Drivers/Src/pmw3901.o \
./Drivers/Src/servo.o \
./Drivers/Src/signal.o \
./Drivers/Src/vl53l1_platform.o \
./Drivers/Src/w25q128.o 

C_DEPS += \
./Drivers/Src/VL53L1X_api.d \
./Drivers/Src/VL53L1X_calibration.d \
./Drivers/Src/bmm150.d \
./Drivers/Src/bmp388.d \
./Drivers/Src/com.d \
./Drivers/Src/filter.d \
./Drivers/Src/icm42688p.d \
./Drivers/Src/ina226.d \
./Drivers/Src/lqr.d \
./Drivers/Src/motor.d \
./Drivers/Src/pmw3901.d \
./Drivers/Src/servo.d \
./Drivers/Src/signal.d \
./Drivers/Src/vl53l1_platform.d \
./Drivers/Src/w25q128.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Middlewares/Third_Party/Fusion -I../Drivers/Inc -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/VL53L1X_api.cyclo ./Drivers/Src/VL53L1X_api.d ./Drivers/Src/VL53L1X_api.o ./Drivers/Src/VL53L1X_api.su ./Drivers/Src/VL53L1X_calibration.cyclo ./Drivers/Src/VL53L1X_calibration.d ./Drivers/Src/VL53L1X_calibration.o ./Drivers/Src/VL53L1X_calibration.su ./Drivers/Src/bmm150.cyclo ./Drivers/Src/bmm150.d ./Drivers/Src/bmm150.o ./Drivers/Src/bmm150.su ./Drivers/Src/bmp388.cyclo ./Drivers/Src/bmp388.d ./Drivers/Src/bmp388.o ./Drivers/Src/bmp388.su ./Drivers/Src/com.cyclo ./Drivers/Src/com.d ./Drivers/Src/com.o ./Drivers/Src/com.su ./Drivers/Src/filter.cyclo ./Drivers/Src/filter.d ./Drivers/Src/filter.o ./Drivers/Src/filter.su ./Drivers/Src/icm42688p.cyclo ./Drivers/Src/icm42688p.d ./Drivers/Src/icm42688p.o ./Drivers/Src/icm42688p.su ./Drivers/Src/ina226.cyclo ./Drivers/Src/ina226.d ./Drivers/Src/ina226.o ./Drivers/Src/ina226.su ./Drivers/Src/lqr.cyclo ./Drivers/Src/lqr.d ./Drivers/Src/lqr.o ./Drivers/Src/lqr.su ./Drivers/Src/motor.cyclo ./Drivers/Src/motor.d ./Drivers/Src/motor.o ./Drivers/Src/motor.su ./Drivers/Src/pmw3901.cyclo ./Drivers/Src/pmw3901.d ./Drivers/Src/pmw3901.o ./Drivers/Src/pmw3901.su ./Drivers/Src/servo.cyclo ./Drivers/Src/servo.d ./Drivers/Src/servo.o ./Drivers/Src/servo.su ./Drivers/Src/signal.cyclo ./Drivers/Src/signal.d ./Drivers/Src/signal.o ./Drivers/Src/signal.su ./Drivers/Src/vl53l1_platform.cyclo ./Drivers/Src/vl53l1_platform.d ./Drivers/Src/vl53l1_platform.o ./Drivers/Src/vl53l1_platform.su ./Drivers/Src/w25q128.cyclo ./Drivers/Src/w25q128.d ./Drivers/Src/w25q128.o ./Drivers/Src/w25q128.su

.PHONY: clean-Drivers-2f-Src

