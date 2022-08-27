################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/NMEA-master/NMEA-master/nmea.c 

OBJS += \
./Drivers/NMEA-master/NMEA-master/nmea.o 

C_DEPS += \
./Drivers/NMEA-master/NMEA-master/nmea.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/NMEA-master/NMEA-master/%.o Drivers/NMEA-master/NMEA-master/%.su: ../Drivers/NMEA-master/NMEA-master/%.c Drivers/NMEA-master/NMEA-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-NMEA-2d-master-2f-NMEA-2d-master

clean-Drivers-2f-NMEA-2d-master-2f-NMEA-2d-master:
	-$(RM) ./Drivers/NMEA-master/NMEA-master/nmea.d ./Drivers/NMEA-master/NMEA-master/nmea.o ./Drivers/NMEA-master/NMEA-master/nmea.su

.PHONY: clean-Drivers-2f-NMEA-2d-master-2f-NMEA-2d-master

