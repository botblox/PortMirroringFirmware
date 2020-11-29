################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
IP175G_PORTMIRRORING_CONFIG.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
IP175G_PORTMIRRORING_CONFIG.list \

OBJCOPY_BIN += \
IP175G_PORTMIRRORING_CONFIG.bin \


# All Target
all: IP175G_PORTMIRRORING_CONFIG.elf secondary-outputs

# Tool invocations
IP175G_PORTMIRRORING_CONFIG.elf: $(OBJS) $(USER_OBJS) C:\Users\Aaron\STM32CubeIDE\workspace_1.3.0\IP175G_PORTMIRRORING_CONFIG\STM32L011D4PX_FLASH.ld
	arm-none-eabi-gcc -o "IP175G_PORTMIRRORING_CONFIG.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\Aaron\STM32CubeIDE\workspace_1.3.0\IP175G_PORTMIRRORING_CONFIG\STM32L011D4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="IP175G_PORTMIRRORING_CONFIG.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

IP175G_PORTMIRRORING_CONFIG.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "IP175G_PORTMIRRORING_CONFIG.list"
	@echo 'Finished building: $@'
	@echo ' '

IP175G_PORTMIRRORING_CONFIG.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "IP175G_PORTMIRRORING_CONFIG.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
