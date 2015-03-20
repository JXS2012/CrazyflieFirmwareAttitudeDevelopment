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
	@echo 'Invoking: GCC C Compiler'
	arm-none-eabi-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


