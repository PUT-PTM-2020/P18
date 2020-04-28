################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
ptm_dyktafon.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ptm_dyktafon.list \

OBJCOPY_BIN += \
ptm_dyktafon.bin \


# All Target
all: ptm_dyktafon.elf secondary-outputs

# Tool invocations
ptm_dyktafon.elf: $(OBJS) $(USER_OBJS) C:\Users\user\Desktop\Projekt_Dyktafon\ptm_dyktafon\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "ptm_dyktafon.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\user\Desktop\Projekt_Dyktafon\ptm_dyktafon\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ptm_dyktafon.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ptm_dyktafon.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ptm_dyktafon.list"
	@echo 'Finished building: $@'
	@echo ' '

ptm_dyktafon.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ptm_dyktafon.bin"
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
