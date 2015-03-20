################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm

# All of the sources participating in the build are defined here
-include sources.mk
-include utils/src/subdir.mk
-include modules/src/subdir.mk
-include lib/STM32F10x_StdPeriph_Driver/src/subdir.mk
-include lib/FreeRTOS/portable/MemMang/subdir.mk
-include lib/FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
-include lib/FreeRTOS/subdir.mk
-include lib/CMSIS/Core/CM3/subdir.mk
-include init/subdir.mk
-include hal/src/subdir.mk
-include drivers/src/subdir.mk
-include bin/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: CrazyflieFirmware

# Tool invocations
CrazyflieFirmware: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O2  -g -Xlinker --gc-sections -Wl,-Map,"CrazyflieFirmware.map" -o "CrazyflieFirmware" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) CrazyflieFirmware
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
