################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := 10_2_I2C_DAC_MCP4725
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
10_2_I2C_DAC_MCP4725.elf \

MAP_FILES += \
10_2_I2C_DAC_MCP4725.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
10_2_I2C_DAC_MCP4725.list \

OBJCOPY_HEX += \
10_2_I2C_DAC_MCP4725.hex \


# All Target
all: main-build

# Main-build Target
main-build: 10_2_I2C_DAC_MCP4725.elf secondary-outputs

# Tool invocations
10_2_I2C_DAC_MCP4725.elf 10_2_I2C_DAC_MCP4725.map: $(OBJS) $(USER_OBJS) F:\OneDrive\ -\ Universidad\ Tecnologica\ del\ Peru\Estudios\Cursos\Microcontroladores\STM32\STM32\ con\ HAL\codigo\10_2_I2C_DAC_MCP4725\STM32F401RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "10_2_I2C_DAC_MCP4725.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"F:\OneDrive - Universidad Tecnologica del Peru\Estudios\Cursos\Microcontroladores\STM32\STM32 con HAL\codigo\10_2_I2C_DAC_MCP4725\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="10_2_I2C_DAC_MCP4725.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

10_2_I2C_DAC_MCP4725.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "10_2_I2C_DAC_MCP4725.list"
	@echo 'Finished building: $@'
	@echo ' '

10_2_I2C_DAC_MCP4725.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "10_2_I2C_DAC_MCP4725.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 10_2_I2C_DAC_MCP4725.elf 10_2_I2C_DAC_MCP4725.hex 10_2_I2C_DAC_MCP4725.list 10_2_I2C_DAC_MCP4725.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
