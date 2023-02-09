#-------------------------------------------------------------------------------
# @aMike Kim
# @JoeRobotics
#
# INCLUDE THIS FILE IN YOUR MAKEFILE USING THE ...
#
#   include <path/to/>make_due_firmware.mk
#
# ... MAKEFILE DIRECTIVE.
#
# You have then the options:
#
#   make all	  : Everything except uploading
#   make binary	  : Makes the binary that can be uploaded (default firmware.bin)
#   make install  : Uploads to the DUE using BOSSA
#   make clean	  : Cleans up the "$OUTPUT_DIR/release" or "$OUTPUT_DIR/debug"
#   make install TARGET_DIR=zoeRoboticsFirmware 
#   make install TARGET_DIR=testSerial 
# -------------------------------------------------------------------
# check Programming port: ls /dev/cu.usb*
# ==> /dev/cu.usbmodem146401
# Check Serial port: ls /dev/tty.usb*
# ==>  /dev/tty.usbmodem146401
#---------------------------------------------------------
# Kill Serial Port: fuser -c /dev/cu.usbmodem1461401 (get ID:53572)
# kill -TERM 53572
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CONFIG: OVERWRITE as params to modify
#-------------------------------------------------------------------------------

.SUFFIXES: .o .a .c .s

SHELL = /bin/sh
OUTPUT_NAME=firmware
OUTPUT_DIR = .
PROJECT_DIR = $(TARGET_DIR)

# UPLOAD_PORT=/dev/cu.usbmodem143401 
# UPLOAD_PORT=/dev/cu.usbmodem14301 
UPLOAD_PORT=/dev/cu.usbmodem14201 
# UPLOAD_PORT=/dev/cu.usbmodem1461401 
USB_DEFINITIONS=-DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON '-DUSB_MANUFACTURER="Unknown"' '-DUSB_PRODUCT="Arduino Due"'

LIB_DIR = /Users/mikekim/Developer/ZoeRobotics_project/ArduinoCore-sam-master
UPLOAD_BOSSA = /Users/mikekim/Library/Arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac
#-------------------------------------------------------------------------------
# Related directories and files
#-------------------------------------------------------------------------------
ROOT := $(realpath $(shell dirname '$(dir $(lastword $(MAKEFILE_LIST)))'))
vpath %.c $(PROJECT_DIR)
VPATH+=$(PROJECT_DIR)
 

current_dir = $(shell pwd)
INCLUDES = -I${current_dir}/include/

# INCLUDES += -I$(current_dir)/include
INCLUDES += -I$(current_dir)/sam
INCLUDES += -I$(current_dir)/sam/libsam
INCLUDES += -I$(current_dir)/sam/CMSIS/CMSIS/Include
INCLUDES += -I$(current_dir)/sam/CMSIS/Device/ATMEL

INCLUDES =  -I${LIB_DIR}/system/libsam/
INCLUDES += -I${LIB_DIR}/system/CMSIS/CMSIS/Include/ 
INCLUDES += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/ 
INCLUDES += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/
INCLUDES += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/include/
INCLUDES += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/include/component/
# INCLUDES += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3n/

INCLUDES += -I${LIB_DIR}/cores/arduino/
INCLUDES += -I${LIB_DIR}/variants/arduino_due_x/

INCLUDES += -I${LIB_DIR}/system/libsam/include
INCLUDES += -I${LIB_DIR}/system/
#-------------------------------------------------------------------------------
# Target selection
#-------------------------------------------------------------------------------
ifdef DEBUG
OPTIMIZATION = -g -O0 -DDEBUG
#OBJ_DIR=debug
OBJ_DIR=$(addprefix debug, $(TARGET_DIR))
OUTPUT_DIR=$(addprefix debug, $(TARGET_DIR))
else
OPTIMIZATION = -Os
OPTIMIZATION = -O1
#OBJ_DIR=release
OBJ_DIR=$(addprefix release, $(TARGET_DIR))
OUTPUT_DIR=$(addprefix release, $(TARGET_DIR))
endif

#-------------------------------------------------------------------------------
#  Toolchain
#-------------------------------------------------------------------------------
CROSS_COMPILE = /Users/mikekim/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AS = $(CROSS_COMPILE)as
NM = $(CROSS_COMPILE)nm
#LKELF = $(CROSS_COMPILE)gcc
 LKELF = $(CROSS_COMPILE)g++
OBJCP = $(CROSS_COMPILE)objcopy
RM=rm -Rf
MKDIR=mkdir -p
# UPLOAD_BOSSA=$(FLASH)/tools/bossac


#-------------------------------------------------------------------------------
#  Flags
#-------------------------------------------------------------------------------
CFLAGS += -Wall --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls
CFLAGS += -ffunction-sections -fdata-sections -nostdlib -std=c99
CFLAGS += $(OPTIMIZATION) $(INCLUDES)
#CFLAGS += -Dprintf=iprintf
CPPFLAGS += -Wall --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -nostdlib
CPPFLAGS += -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -std=c++98
CPPFLAGS += $(OPTIMIZATION) $(INCLUDES)
#CPPFLAGS += -Dprintf=iprintf
ASFLAGS = -mcpu=cortex-m3 -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES)
ARFLAGS = rcs

LNK_SCRIPT=${LIB_DIR}/flash.ld
LIBSAM_ARCHIVE=${LIB_DIR}/libsam_sam3x8e_gcc_rel.a
# LIBSAM_ARCHIVE=${LIB_DIR}/libvariant_arduino_due_x_gcc_rel.a
UPLOAD_PORT_BASENAME=$(patsubst /dev/%,%,$(UPLOAD_PORT))
# UPLOAD_PORT_BASENAME=cu.usbmodem14301

#-------------------------------------------------------------------------------
# High verbosity flags
#-------------------------------------------------------------------------------
ifdef VERBOSE
CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return -Wstrict-prototypes
CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS += -Wunreachable-code
CFLAGS += -Wcast-align
CFLAGS += -Wmissing-noreturn
CFLAGS += -Wconversion
CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wcast-align -Wunreachable-code
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wpacked -Wredundant-decls -Winline -Wlong-long
CPPFLAGS += -Wmissing-noreturn
CPPFLAGS += -Wconversion
CPPFLAGS += -ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics --param max-inline-insns-single=500 -fno-rtti -fno-exceptions
UPLOAD_VERBOSE_FLAGS += -i -d
endif

#-------------------------------------------------------------------------------
# Source files and objects
#-------------------------------------------------------------------------------
C_SRC=$(wildcard $(PROJECT_DIR)/*.c) $(wildcard ../src/*.c)
C_OBJ=$(patsubst %.c, %.o, $(notdir $(C_SRC)))
CPP_SRC:=$(wildcard $(PROJECT_DIR)/*.cpp)
CPP_SRC:=$(filter-out $(PROJECT_DIR)/main_x_axis.cpp, $(CPP_SRC))
CPP_OBJ=$(patsubst %.cpp, %.o, $(notdir $(CPP_SRC)))
A_SRC=$(wildcard $(PROJECT_DIR)/*.s)
A_OBJ=$(patsubst %.s, %.o, $(notdir $(A_SRC)))

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------

# .PHONY: flash
#flash:

#-------------------------------------------------------------------------------
all: binary install

#-------------------------------------------------------------------------------
.PHONY: clean
clean:
	@echo $(OUTPUT_DIR)/$(OUTPUT_NAME)
	@echo $(OBJ_DIR)
	-@$(RM) $(OBJ_DIR) 1>/dev/null 2>&1
	-@$(RM) $(OUTPUT_DIR)/$(OUTPUT_NAME) 1>/dev/null 2>&1

#-------------------------------------------------------------------------------
.PHONY: prepare
prepare:
	-@$(MKDIR) $(OBJ_DIR) 1>/dev/null 2>&1

#-------------------------------------------------------------------------------
.PHONY: binary
binary: prepare $(OBJ_DIR)/$(OUTPUT_NAME).bin

#-------------------------------------------------------------------------------
# .bin ------> UPLOAD TO CONTROLLER
.PHONY: install
install: 
	
	
	# -@$(RM) $(OBJ_DIR) 1>/dev/null 2>&1
	# -@$(RM) $(OUTPUT_DIR)/$(OUTPUT_NAME) 1>/dev/null 2>&1

	-@echo "MK: Touch programming port ..."
	-@stty -f "/dev/$(UPLOAD_PORT_BASENAME)" raw ispeed 1200 ospeed 1200 cs8 -cstopb ignpar eol 255 eof 255
	-@printf "\x00" > "/dev/$(UPLOAD_PORT_BASENAME)"
	-@echo "Waiting before uploading ..."
	-@sleep 1
	-@echo "Uploading ..."
#	$(UPLOAD_BOSSA) $(UPLOAD_VERBOSE_FLAGS) --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -b "$(OBJ_DIR)/$(OUTPUT_NAME).bin" -R
	 $(UPLOAD_BOSSA) $(UPLOAD_VERBOSE_FLAGS) --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "$(OBJ_DIR)/$(OUTPUT_NAME).bin" -R
# test $(UPLOAD_BOSSA) $(UPLOAD_VERBOSE_FLAGS) --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "/Users/mikekim/Developer/ZoeRobotics_project/IStackerRobot/Arm_sam3x8/test_blink/mkZoeRobotics_main.bin" -R

	@echo "Done." 
#-------------------------------------------------------------------------------
# .c -> .o
$(addprefix $(OBJ_DIR)/,$(C_OBJ)): $(OBJ_DIR)/%.o: %.c
	"$(CC)" -c $(CFLAGS) $< -o $@

# .cpp -> .o
$(addprefix $(OBJ_DIR)/,$(CPP_OBJ)): $(OBJ_DIR)/%.o: %.cpp
	"$(CXX)" -g  $(OPTIMIZATION) -w   -std=gnu++11 -c $(CPPFLAGS) $< -o $@

# .s -> .o
$(addprefix $(OBJ_DIR)/,$(A_OBJ)): $(OBJ_DIR)/%.o: %.s
	"$(AS)" -c $(ASFLAGS) $< -o $@

# .o -> .a
$(OBJ_DIR)/$(OUTPUT_NAME).a: $(addprefix $(OBJ_DIR)/, $(C_OBJ)) $(addprefix $(OBJ_DIR)/, $(CPP_OBJ)) $(addprefix $(OBJ_DIR)/, $(A_OBJ))
	"$(AR)" $(ARFLAGS) $@ $^
	"$(NM)" $@ > $@.txt
#-L${LIB_DIR} -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,
#--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,
#--start-group -u _sbrk -u link -u _close -u _fstat -u _isatty -u _lseek -u _read -u _write -u _exit -u kill -u _getpid
#  -> .elf
$(OBJ_DIR)/$(OUTPUT_NAME).elf: $(OBJ_DIR)/$(OUTPUT_NAME).a
	"$(LKELF)"  $(OPTIMIZATION) -Wl,--gc-sections -mcpu=cortex-m3 \
	  "-T$(LNK_SCRIPT)" "-Wl,-Map,$(OBJ_DIR)/$(OUTPUT_NAME).map" \
	  -o $@ \
	  "-L$(OBJ_DIR)" \
	  -lm -lgcc -mthumb -Wl,--cref \
	  -Wl,--check-sections \
	  -Wl,--gc-sections \
	  -Wl,--entry=Reset_Handler \
	  -Wl,--unresolved-symbols=report-all \
	  -Wl,--warn-common \
	  -Wl,--warn-section-align \
	  -Wl,--warn-unresolved-symbols \
	  -Wl,--start-group   -specs=nosys.specs\
	  $^ $(LIBSAM_ARCHIVE) \
	  -Wl,--end-group

# .elf -> .bin
$(OBJ_DIR)/$(OUTPUT_NAME).bin: $(OBJ_DIR)/$(OUTPUT_NAME).elf
	"$(OBJCP)" -O binary $< $@