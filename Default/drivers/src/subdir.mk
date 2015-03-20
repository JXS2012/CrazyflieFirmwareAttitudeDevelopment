################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/adc.c \
../drivers/src/exti.c \
../drivers/src/hmc5883l.c \
../drivers/src/i2cdev.c \
../drivers/src/i2croutines.c \
../drivers/src/led.c \
../drivers/src/motors.c \
../drivers/src/mpu6050.c \
../drivers/src/ms5611.c \
../drivers/src/nrf24l01.c \
../drivers/src/nvic.c 

OBJS += \
./drivers/src/adc.o \
./drivers/src/exti.o \
./drivers/src/hmc5883l.o \
./drivers/src/i2cdev.o \
./drivers/src/i2croutines.o \
./drivers/src/led.o \
./drivers/src/motors.o \
./drivers/src/mpu6050.o \
./drivers/src/ms5611.o \
./drivers/src/nrf24l01.o \
./drivers/src/nvic.o 

C_DEPS += \
./drivers/src/adc.d \
./drivers/src/exti.d \
./drivers/src/hmc5883l.d \
./drivers/src/i2cdev.d \
./drivers/src/i2croutines.d \
./drivers/src/led.d \
./drivers/src/motors.d \
./drivers/src/mpu6050.d \
./drivers/src/ms5611.d \
./drivers/src/nrf24l01.d \
./drivers/src/nvic.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2  -g -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


