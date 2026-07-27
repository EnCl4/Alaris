################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/bmp280.c \
../Core/Src/dapu_analog.c \
../Core/Src/dapu_boot.c \
../Core/Src/dapu_console.c \
../Core/Src/dapu_log.c \
../Core/Src/dapu_spi.c \
../Core/Src/dapu_state.c \
../Core/Src/dma.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/gps_ubx.c \
../Core/Src/icm20948.c \
../Core/Src/main.c \
../Core/Src/ms5611.c \
../Core/Src/rpm_fft.c \
../Core/Src/sdmmc.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/bmp280.o \
./Core/Src/dapu_analog.o \
./Core/Src/dapu_boot.o \
./Core/Src/dapu_console.o \
./Core/Src/dapu_log.o \
./Core/Src/dapu_spi.o \
./Core/Src/dapu_state.o \
./Core/Src/dma.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/gps_ubx.o \
./Core/Src/icm20948.o \
./Core/Src/main.o \
./Core/Src/ms5611.o \
./Core/Src/rpm_fft.o \
./Core/Src/sdmmc.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/bmp280.d \
./Core/Src/dapu_analog.d \
./Core/Src/dapu_boot.d \
./Core/Src/dapu_console.d \
./Core/Src/dapu_log.d \
./Core/Src/dapu_spi.d \
./Core/Src/dapu_state.d \
./Core/Src/dma.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/gps_ubx.d \
./Core/Src/icm20948.d \
./Core/Src/main.d \
./Core/Src/ms5611.d \
./Core/Src/rpm_fft.d \
./Core/Src/sdmmc.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DARM_MATH_CM7 -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/DSP/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/bmp280.cyclo ./Core/Src/bmp280.d ./Core/Src/bmp280.o ./Core/Src/bmp280.su ./Core/Src/dapu_analog.cyclo ./Core/Src/dapu_analog.d ./Core/Src/dapu_analog.o ./Core/Src/dapu_analog.su ./Core/Src/dapu_boot.cyclo ./Core/Src/dapu_boot.d ./Core/Src/dapu_boot.o ./Core/Src/dapu_boot.su ./Core/Src/dapu_console.cyclo ./Core/Src/dapu_console.d ./Core/Src/dapu_console.o ./Core/Src/dapu_console.su ./Core/Src/dapu_log.cyclo ./Core/Src/dapu_log.d ./Core/Src/dapu_log.o ./Core/Src/dapu_log.su ./Core/Src/dapu_spi.cyclo ./Core/Src/dapu_spi.d ./Core/Src/dapu_spi.o ./Core/Src/dapu_spi.su ./Core/Src/dapu_state.cyclo ./Core/Src/dapu_state.d ./Core/Src/dapu_state.o ./Core/Src/dapu_state.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/gps_ubx.cyclo ./Core/Src/gps_ubx.d ./Core/Src/gps_ubx.o ./Core/Src/gps_ubx.su ./Core/Src/icm20948.cyclo ./Core/Src/icm20948.d ./Core/Src/icm20948.o ./Core/Src/icm20948.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ms5611.cyclo ./Core/Src/ms5611.d ./Core/Src/ms5611.o ./Core/Src/ms5611.su ./Core/Src/rpm_fft.cyclo ./Core/Src/rpm_fft.d ./Core/Src/rpm_fft.o ./Core/Src/rpm_fft.su ./Core/Src/sdmmc.cyclo ./Core/Src/sdmmc.d ./Core/Src/sdmmc.o ./Core/Src/sdmmc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

