################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/FreeRTOS/portable/GCC/ARM_CM3/port.c 

OBJS += \
./lib/FreeRTOS/portable/GCC/ARM_CM3/port.o 

C_DEPS += \
./lib/FreeRTOS/portable/GCC/ARM_CM3/port.d 


# Each subdirectory must supply rules for building sources it contributes
lib/FreeRTOS/portable/GCC/ARM_CM3/%.o: ../lib/FreeRTOS/portable/GCC/ARM_CM3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2  -g -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


