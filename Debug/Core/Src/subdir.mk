################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/commands.c \
../Core/Src/flash.c \
../Core/Src/ground_calibration.c \
../Core/Src/iis2mdc.c \
../Core/Src/iis2mdc_reg.c \
../Core/Src/imu.c \
../Core/Src/logger.c \
../Core/Src/lsm6dso32x_reg.c \
../Core/Src/mag.c \
../Core/Src/main.c \
../Core/Src/ms5611.c \
../Core/Src/s25fl128s.c \
../Core/Src/sensors.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/commands.o \
./Core/Src/flash.o \
./Core/Src/ground_calibration.o \
./Core/Src/iis2mdc.o \
./Core/Src/iis2mdc_reg.o \
./Core/Src/imu.o \
./Core/Src/logger.o \
./Core/Src/lsm6dso32x_reg.o \
./Core/Src/mag.o \
./Core/Src/main.o \
./Core/Src/ms5611.o \
./Core/Src/s25fl128s.o \
./Core/Src/sensors.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/commands.d \
./Core/Src/flash.d \
./Core/Src/ground_calibration.d \
./Core/Src/iis2mdc.d \
./Core/Src/iis2mdc_reg.d \
./Core/Src/imu.d \
./Core/Src/logger.d \
./Core/Src/lsm6dso32x_reg.d \
./Core/Src/mag.d \
./Core/Src/main.d \
./Core/Src/ms5611.d \
./Core/Src/s25fl128s.d \
./Core/Src/sensors.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/commands.cyclo ./Core/Src/commands.d ./Core/Src/commands.o ./Core/Src/commands.su ./Core/Src/flash.cyclo ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/ground_calibration.cyclo ./Core/Src/ground_calibration.d ./Core/Src/ground_calibration.o ./Core/Src/ground_calibration.su ./Core/Src/iis2mdc.cyclo ./Core/Src/iis2mdc.d ./Core/Src/iis2mdc.o ./Core/Src/iis2mdc.su ./Core/Src/iis2mdc_reg.cyclo ./Core/Src/iis2mdc_reg.d ./Core/Src/iis2mdc_reg.o ./Core/Src/iis2mdc_reg.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/lsm6dso32x_reg.cyclo ./Core/Src/lsm6dso32x_reg.d ./Core/Src/lsm6dso32x_reg.o ./Core/Src/lsm6dso32x_reg.su ./Core/Src/mag.cyclo ./Core/Src/mag.d ./Core/Src/mag.o ./Core/Src/mag.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ms5611.cyclo ./Core/Src/ms5611.d ./Core/Src/ms5611.o ./Core/Src/ms5611.su ./Core/Src/s25fl128s.cyclo ./Core/Src/s25fl128s.d ./Core/Src/s25fl128s.o ./Core/Src/s25fl128s.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

