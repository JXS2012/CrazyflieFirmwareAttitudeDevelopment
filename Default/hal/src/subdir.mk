################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hal/src/eskylink.c \
../hal/src/freeRTOSdebug.c \
../hal/src/imu.c \
../hal/src/ledseq.c \
../hal/src/pm.c \
../hal/src/radiolink.c \
../hal/src/uart.c \
../hal/src/usec_time.c 

OBJS += \
./hal/src/eskylink.o \
./hal/src/freeRTOSdebug.o \
./hal/src/imu.o \
./hal/src/ledseq.o \
./hal/src/pm.o \
./hal/src/radiolink.o \
./hal/src/uart.o \
./hal/src/usec_time.o 

C_DEPS += \
./hal/src/eskylink.d \
./hal/src/freeRTOSdebug.d \
./hal/src/imu.d \
./hal/src/ledseq.d \
./hal/src/pm.d \
./hal/src/radiolink.d \
./hal/src/uart.d \
./hal/src/usec_time.d 


# Each subdirectory must supply rules for building sources it contributes
hal/src/%.o: ../hal/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-none-eabi-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


