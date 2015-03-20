################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/FreeRTOS/portable/MemMang/heap_1.c \
../lib/FreeRTOS/portable/MemMang/heap_2.c \
../lib/FreeRTOS/portable/MemMang/heap_3.c \
../lib/FreeRTOS/portable/MemMang/heap_4.c 

OBJS += \
./lib/FreeRTOS/portable/MemMang/heap_1.o \
./lib/FreeRTOS/portable/MemMang/heap_2.o \
./lib/FreeRTOS/portable/MemMang/heap_3.o \
./lib/FreeRTOS/portable/MemMang/heap_4.o 

C_DEPS += \
./lib/FreeRTOS/portable/MemMang/heap_1.d \
./lib/FreeRTOS/portable/MemMang/heap_2.d \
./lib/FreeRTOS/portable/MemMang/heap_3.d \
./lib/FreeRTOS/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
lib/FreeRTOS/portable/MemMang/%.o: ../lib/FreeRTOS/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-none-eabi-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


