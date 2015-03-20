################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utils/src/abort.c \
../utils/src/cfassert.c \
../utils/src/configblock.c \
../utils/src/cpuid.c \
../utils/src/crc.c \
../utils/src/debug.c \
../utils/src/eprintf.c \
../utils/src/filter.c \
../utils/src/fp16.c 

OBJS += \
./utils/src/abort.o \
./utils/src/cfassert.o \
./utils/src/configblock.o \
./utils/src/cpuid.o \
./utils/src/crc.o \
./utils/src/debug.o \
./utils/src/eprintf.o \
./utils/src/filter.o \
./utils/src/fp16.o 

C_DEPS += \
./utils/src/abort.d \
./utils/src/cfassert.d \
./utils/src/configblock.d \
./utils/src/cpuid.d \
./utils/src/crc.d \
./utils/src/debug.d \
./utils/src/eprintf.d \
./utils/src/filter.d \
./utils/src/fp16.d 


# Each subdirectory must supply rules for building sources it contributes
utils/src/%.o: ../utils/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2  -g -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


