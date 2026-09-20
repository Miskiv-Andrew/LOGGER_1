################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/auxil_gps.c \
../Src/auxil_lidar.c \
../Src/fatfs.c \
../Src/main.c \
../Src/radio.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c \
../Src/user_diskio.c \
../Src/user_diskio_spi.c 

OBJS += \
./Src/auxil_gps.o \
./Src/auxil_lidar.o \
./Src/fatfs.o \
./Src/main.o \
./Src/radio.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o \
./Src/user_diskio.o \
./Src/user_diskio_spi.o 

C_DEPS += \
./Src/auxil_gps.d \
./Src/auxil_lidar.d \
./Src/fatfs.d \
./Src/main.d \
./Src/radio.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d \
./Src/user_diskio.d \
./Src/user_diskio_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/auxil_gps.cyclo ./Src/auxil_gps.d ./Src/auxil_gps.o ./Src/auxil_gps.su ./Src/auxil_lidar.cyclo ./Src/auxil_lidar.d ./Src/auxil_lidar.o ./Src/auxil_lidar.su ./Src/fatfs.cyclo ./Src/fatfs.d ./Src/fatfs.o ./Src/fatfs.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/radio.cyclo ./Src/radio.d ./Src/radio.o ./Src/radio.su ./Src/stm32l4xx_hal_msp.cyclo ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_it.cyclo ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.cyclo ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su ./Src/user_diskio.cyclo ./Src/user_diskio.d ./Src/user_diskio.o ./Src/user_diskio.su ./Src/user_diskio_spi.cyclo ./Src/user_diskio_spi.d ./Src/user_diskio_spi.o ./Src/user_diskio_spi.su

.PHONY: clean-Src

