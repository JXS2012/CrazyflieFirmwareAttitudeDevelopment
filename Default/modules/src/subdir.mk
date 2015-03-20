################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modules/src/comm.c \
../modules/src/commander.c \
../modules/src/console.c \
../modules/src/controller.c \
../modules/src/crtp.c \
../modules/src/crtpservice.c \
../modules/src/info.c \
../modules/src/log.c \
../modules/src/mem.c \
../modules/src/param.c \
../modules/src/pid.c \
../modules/src/pidctrl.c \
../modules/src/sensfusion6.c \
../modules/src/stabilizer.c \
../modules/src/system.c \
../modules/src/worker.c 

OBJS += \
./modules/src/comm.o \
./modules/src/commander.o \
./modules/src/console.o \
./modules/src/controller.o \
./modules/src/crtp.o \
./modules/src/crtpservice.o \
./modules/src/info.o \
./modules/src/log.o \
./modules/src/mem.o \
./modules/src/param.o \
./modules/src/pid.o \
./modules/src/pidctrl.o \
./modules/src/sensfusion6.o \
./modules/src/stabilizer.o \
./modules/src/system.o \
./modules/src/worker.o 

C_DEPS += \
./modules/src/comm.d \
./modules/src/commander.d \
./modules/src/console.d \
./modules/src/controller.d \
./modules/src/crtp.d \
./modules/src/crtpservice.d \
./modules/src/info.d \
./modules/src/log.d \
./modules/src/mem.d \
./modules/src/param.d \
./modules/src/pid.d \
./modules/src/pidctrl.d \
./modules/src/sensfusion6.d \
./modules/src/stabilizer.d \
./modules/src/system.d \
./modules/src/worker.d 


# Each subdirectory must supply rules for building sources it contributes
modules/src/%.o: ../modules/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-none-eabi-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


