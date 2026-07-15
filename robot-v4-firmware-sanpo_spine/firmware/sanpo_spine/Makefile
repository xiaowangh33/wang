PROJECT := sanpo_spine
BUILD ?= Release
WD_CAN_BUS_BASE ?= 0
WD_BENCH_REMAP_CAN2_TO_LOGICAL0 ?= 0
WD_ALLOW_LIVE_CAN_CONTROL ?= 0
WD_ENABLE_CALIBRATION_READONLY ?= 1
WD_READONLY_TAG := $(if $(filter 1,$(WD_ENABLE_CALIBRATION_READONLY)),_rawread1,)
WD_BUILD_TAG ?= bus$(WD_CAN_BUS_BASE)_bench$(WD_BENCH_REMAP_CAN2_TO_LOGICAL0)_live$(WD_ALLOW_LIVE_CAN_CONTROL)$(WD_READONLY_TAG)
BUILD_DIR := build/$(BUILD)/$(WD_BUILD_TAG)

LOCAL_TOOLCHAIN_ROOT := $(abspath ../../../.toolchains/arm-none-eabi/usr)
LOCAL_STLINK_ROOT := $(abspath ../../../.toolchains/stlink)
ifneq ($(wildcard $(LOCAL_TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc),)
TOOLCHAIN_ROOT ?= $(LOCAL_TOOLCHAIN_ROOT)
PREFIX ?= $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-
else
TOOLCHAIN_ROOT ?=
PREFIX ?= arm-none-eabi-
endif

NEWLIB_MULTILIB ?= thumb/v7e-m+fp/hard
ifneq ($(TOOLCHAIN_ROOT),)
NEWLIB_DIR := $(TOOLCHAIN_ROOT)/lib/arm-none-eabi/newlib/$(NEWLIB_MULTILIB)
GCC_VERSION ?= 10.3.1
GCC_MULTILIB_DIR := $(TOOLCHAIN_ROOT)/lib/gcc/arm-none-eabi/$(GCC_VERSION)/$(NEWLIB_MULTILIB)
LOCAL_SYS_INCLUDES := -isystem $(TOOLCHAIN_ROOT)/include/newlib -isystem $(TOOLCHAIN_ROOT)/include/newlib/nano
LOCAL_LIB_DIRS := -L$(NEWLIB_DIR)
LOCAL_SPECS := --specs=$(NEWLIB_DIR)/nano.specs --specs=$(NEWLIB_DIR)/nosys.specs
NO_STARTFILES := -nostartfiles
START_FILES := $(GCC_MULTILIB_DIR)/crti.o $(GCC_MULTILIB_DIR)/crtbegin.o
END_FILES := $(GCC_MULTILIB_DIR)/crtend.o $(GCC_MULTILIB_DIR)/crtn.o
else
LOCAL_SYS_INCLUDES :=
LOCAL_LIB_DIRS :=
LOCAL_SPECS := --specs=nano.specs --specs=nosys.specs
NO_STARTFILES :=
START_FILES :=
END_FILES :=
endif

CC := $(PREFIX)gcc
AS := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
OBJDUMP := $(PREFIX)objdump
SIZE := $(PREFIX)size
HOST_CC ?= cc

ifneq ($(wildcard $(LOCAL_STLINK_ROOT)/usr/bin/st-flash),)
ST_FLASH ?= $(LOCAL_STLINK_ROOT)/usr/bin/st-flash
ST_INFO ?= $(LOCAL_STLINK_ROOT)/usr/bin/st-info
STLINK_LIBRARY_PATH ?= $(LOCAL_STLINK_ROOT)/usr/lib/x86_64-linux-gnu
else
ST_FLASH ?= st-flash
ST_INFO ?= st-info
STLINK_LIBRARY_PATH ?=
endif

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS := -DUSE_HAL_DRIVER -DSTM32F407xx \
  -DWD_CAN_BUS_BASE=$(WD_CAN_BUS_BASE) \
  -DWD_BENCH_REMAP_CAN2_TO_LOGICAL0=$(WD_BENCH_REMAP_CAN2_TO_LOGICAL0) \
  -DWD_ALLOW_LIVE_CAN_CONTROL=$(WD_ALLOW_LIVE_CAN_CONTROL) \
  -DWD_ENABLE_CALIBRATION_READONLY=$(WD_ENABLE_CALIBRATION_READONLY)

INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IUSB_DEVICE/App \
  -IUSB_DEVICE/Target \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
  -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
  -IThirdParty/inc

C_SOURCES := \
  $(wildcard Core/Src/*.c) \
  $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
  $(wildcard USB_DEVICE/App/*.c) \
  $(wildcard USB_DEVICE/Target/*.c) \
  $(wildcard Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c) \
  $(wildcard Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c) \
  $(wildcard Middlewares/Third_Party/FreeRTOS/Source/*.c) \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
  ThirdParty/src/mpu6050.c

ASM_SOURCES := Core/Startup/startup_stm32f407vetx.s

ifeq ($(BUILD),Debug)
OPT := -Og -g3
else
OPT := -Os -g0
endif

COMMON_FLAGS := $(MCU_FLAGS) $(DEFS) $(INCLUDES) $(LOCAL_SYS_INCLUDES) $(OPT) \
  -ffunction-sections -fdata-sections -Wall -Wextra -fmessage-length=0

CFLAGS := $(COMMON_FLAGS) -std=gnu11 -MMD -MP
ASFLAGS := $(COMMON_FLAGS) -x assembler-with-cpp -MMD -MP

LDSCRIPT := STM32F407VETX_FLASH.ld
LIB_DIRS := -LLibs $(LOCAL_LIB_DIRS)
LIBS := -lloop_function_v4 -lc -lm -lnosys
LDFLAGS := $(MCU_FLAGS) -T$(LDSCRIPT) $(LIB_DIRS) \
  $(NO_STARTFILES) \
  -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref \
  -Wl,--gc-sections -Wl,--print-memory-usage \
  $(LOCAL_SPECS) -u _printf_float -u _scanf_float

OBJECTS := \
  $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES)) \
  $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_SOURCES))

DEPS := $(OBJECTS:.o=.d)
ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex
LST := $(BUILD_DIR)/$(PROJECT).list

.PHONY: all debug release release-mcu-a release-mcu-b \
  release-calibration-mcu-a release-calibration-mcu-b clean toolchain-check \
  flash-openocd flash-stlink flash-stlink-mcu-a flash-stlink-mcu-b \
  flash-stlink-calibration-mcu-a flash-stlink-calibration-mcu-b \
  probe-stlink size host-test

all: release

debug:
	$(MAKE) BUILD=Debug $(BUILD_DIR)/$(PROJECT).elf post-build

release:
	$(MAKE) BUILD=Release $(BUILD_DIR)/$(PROJECT).elf post-build

release-mcu-a:
	$(MAKE) release WD_CAN_BUS_BASE=0 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

release-mcu-b:
	$(MAKE) release WD_CAN_BUS_BASE=2 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

release-calibration-mcu-a:
	$(MAKE) release WD_CAN_BUS_BASE=0 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

release-calibration-mcu-b:
	$(MAKE) release WD_CAN_BUS_BASE=2 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

post-build: $(BIN) $(HEX) $(LST) size

toolchain-check:
	@command -v $(CC) >/dev/null || { echo "missing $(CC)"; exit 1; }
	@command -v $(OBJCOPY) >/dev/null || { echo "missing $(OBJCOPY)"; exit 1; }
	@command -v $(SIZE) >/dev/null || { echo "missing $(SIZE)"; exit 1; }

$(ELF): $(OBJECTS) $(LDSCRIPT) Libs/libloop_function_v4.a | toolchain-check
	$(CC) $(START_FILES) $(OBJECTS) $(LDFLAGS) $(LIBS) $(END_FILES) -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LST): $(ELF)
	$(OBJDUMP) -h -S $< > $@

size: $(ELF)
	$(SIZE) $<

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

flash-openocd: $(BIN)
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	  -c "program $(BIN) 0x08000000 verify reset exit"

flash-stlink: $(BIN)
	LD_LIBRARY_PATH="$(STLINK_LIBRARY_PATH):$$LD_LIBRARY_PATH" $(ST_FLASH) --reset write $(BIN) 0x08000000

flash-stlink-mcu-a:
	$(MAKE) flash-stlink WD_CAN_BUS_BASE=0 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1

flash-stlink-mcu-b:
	$(MAKE) flash-stlink WD_CAN_BUS_BASE=2 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1

flash-stlink-calibration-mcu-a:
	$(MAKE) flash-stlink WD_CAN_BUS_BASE=0 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

flash-stlink-calibration-mcu-b:
	$(MAKE) flash-stlink WD_CAN_BUS_BASE=2 WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0 WD_ALLOW_LIVE_CAN_CONTROL=1 WD_ENABLE_CALIBRATION_READONLY=1

probe-stlink:
	LD_LIBRARY_PATH="$(STLINK_LIBRARY_PATH):$$LD_LIBRARY_PATH" $(ST_INFO) --probe

host-test:
	@mkdir -p build/host
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -ICore/Inc \
	  Core/Src/wd_actuator_units.c tests/test_wd_actuator_units.c -lm \
	  -o build/host/test_wd_actuator_units
	build/host/test_wd_actuator_units
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -ICore/Inc \
	  Core/Src/wd_can_scheduler.c tests/test_wd_can_scheduler.c \
	  -o build/host/test_wd_can_scheduler
	build/host/test_wd_can_scheduler
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -ICore/Inc \
	  Core/Src/wd_fast_feedback.c tests/test_wd_fast_feedback.c -lm \
	  -o build/host/test_wd_fast_feedback
	build/host/test_wd_fast_feedback
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -ICore/Inc \
	  Core/Src/wd_motor_calibration.c Core/Src/wd_safety.c \
	  tests/test_wd_safety.c -lm \
	  -o build/host/test_wd_safety
	build/host/test_wd_safety
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -ICore/Inc \
	  Core/Src/wd_robstride_motion.c tests/test_wd_robstride_motion.c -lm \
	  -o build/host/test_wd_robstride_motion
	build/host/test_wd_robstride_motion

clean:
	rm -rf build

-include $(DEPS)
