################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/FreeRTOS/croutine.c \
../lib/FreeRTOS/list.c \
../lib/FreeRTOS/queue.c \
../lib/FreeRTOS/tasks.c \
../lib/FreeRTOS/timers.c 

OBJS += \
./lib/FreeRTOS/croutine.o \
./lib/FreeRTOS/list.o \
./lib/FreeRTOS/queue.o \
./lib/FreeRTOS/tasks.o \
./lib/FreeRTOS/timers.o 

C_DEPS += \
./lib/FreeRTOS/croutine.d \
./lib/FreeRTOS/list.d \
./lib/FreeRTOS/queue.d \
./lib/FreeRTOS/tasks.d \
./lib/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
lib/FreeRTOS/%.o: ../lib/FreeRTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-none-eabi-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


