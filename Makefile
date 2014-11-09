# Makefile for Lab06, ELEC424 Fall 2014
# Authors: Jie Liao, Abeer Javed, Steven Arroyo. Rice University 
# Derived from the crazyflie-firmware Makefile

# Path Definitions
STM_LIB ?= ../STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
STARTUP = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7
SYSTEM = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
DRIVER = $(STM_LIB)/STM32F10x_StdPeriph_Driver
DRIVER_INC = $(DRIVER)/inc
DRIVER_SRC = $(DRIVER)/src
CORE_INC = $(STM_LIB)/CMSIS/CM3/CoreSupport
INC = inc
LINKER = linker_script
PROG = scheduling
INIT_PROG = sys_init
SRC = src
BIN = bin
LIB = lib
FREERTOS = $(LIB)/FreeRTOS/Source
OPENOCD_FLAG = -d0 -f interface/busblaster.cfg -f target/stm32f1x.cfg

# Compiler 
CC = arm-none-eabi-gcc

# Particular processor
PROCESSOR = -mcpu=cortex-m3 -mthumb

# Directories of used header files
FREERTOS_INC = $(FREERTOS)/include
FREERTOS_PORT_INC = $(FREERTOS)/portable/GCC/ARM_CM3
INCLUDE = -I$(INC) -I$(SYSTEM) \
-I$(DRIVER_INC) \
-I$(CORE_INC) \
-I$(FREERTOS_INC) \
-I$(FREERTOS_PORT_INC) \

# STM chip specific flags
STFLAGS = -DSTM32F10X_MD -include $(INC)/stm32f10x_conf.h

FREERTOS_OBJS = \
$(FREERTOS)/tasks.c \
$(FREERTOS)/queue.c \
$(FREERTOS)/list.c \
$(FREERTOS)/portable/GCC/ARM_CM3/port.c \
$(FREERTOS)/portable/MemMang/heap_1.c \

# Object files
OBJS = $(SRC)/$(PROG).c \
$(SRC)/$(INIT_PROG).c \
$(DRIVER_SRC)/stm32f10x_gpio.c \
$(DRIVER_SRC)/stm32f10x_rcc.c \
$(DRIVER_SRC)/stm32f10x_tim.c \
$(DRIVER_SRC)/misc.c \
$(LIB)/lab06_task.a \
$(FREERTOS_OBJS)


# Define the compiler flags
CFLAGS = -O0 -g3 $(PROCESSOR) $(INCLUDE) $(STFLAGS) -Wl,--gc-sections -T $(LINKER)/stm32_flash.ld

# Build all relevant files and create .elf
all:
	@$(CC) $(CFLAGS) $(OBJS) $(STARTUP)/startup_stm32f10x_md.s -o $(BIN)/$(PROG).elf

# Program .elf into Crazyflie flash memory via the busblaster
flash:
	@openocd $(OPENOCD_FLAG) -c init -c targets -c "reset halt" -c "flash write_image erase bin/$(PROG).elf" -c "verify_image bin/$(PROG).elf" -c "reset run" -c shutdown


# Runs OpenOCD, opens GDB terminal, and establishes connection with Crazyflie
debug:
	@openocd $(OPENOCD_FLAG) -c init -c targets -c "reset halt" &
	@arm-none-eabi-gdb $(BIN)/$(PROG).elf --eval-command="target remote:3333"
	@ps axf | grep openocd | grep -v grep | awk '{print "kill -9 " $$1}' | sh

# Remove all files generated by target 'all'
clean:
	@rm -f $(BIN)/$(PROG).elf

kill:
	@ps axf | grep openocd | grep -v grep | awk '{print "kill -9 " $$1}' | sh

stop:
	@openocd $(OPENOCD_FLAG) -c init -c targets -c "reset halt" -c shutdown
