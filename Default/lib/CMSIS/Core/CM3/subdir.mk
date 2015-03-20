################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/CMSIS/Core/CM3/core_cm3.c \
../lib/CMSIS/Core/CM3/system_stm32f10x.c 

OBJS += \
./lib/CMSIS/Core/CM3/core_cm3.o \
./lib/CMSIS/Core/CM3/system_stm32f10x.o 

C_DEPS += \
./lib/CMSIS/Core/CM3/core_cm3.d \
./lib/CMSIS/Core/CM3/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
lib/CMSIS/Core/CM3/%.o: ../lib/CMSIS/Core/CM3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2  -g -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


