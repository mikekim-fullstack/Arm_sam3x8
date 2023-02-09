TARGET=CORTEXM3


ifeq ($(TARGET), CORTEXM3)
## USB PORT
UPLOAD_PORT=/dev/cu.usbmodem14301 
#PORT = /dev/cu.usbmodem141401
UPLOAD_PORT_BASENAME=$(patsubst /dev/%,%,$(UPLOAD_PORT))
## DEFINT WORKING DIRECTORY
current_dir = $(shell pwd)
WORKING_DIR = /Users/mikekim/Developer/ArduinoDue
# MAIN_FILE = test_SAM3X_main
# MAIN_FILE = mkSerial
MAIN_FILE = mkZoeRobotics_main


## DEFAULT SAM3X LIBRARY INFO.
#LIB_DIR = /Users/mikekim/Developer/ArduinoDue 
#LIB_DIR =/Users/mikekim/Developer/ArduinoDue/arduino_build
LIB_DIR = /Users/mikekim/Developer/ZoeRobotics_project/ArduinoCore-sam-master
COMPILE_DIR = /Users/mikekim/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin
CPP = $(COMPILE_DIR)/arm-none-eabi-g++
FLAGS = -c -g -Os -w   -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics --param max-inline-insns-single=500 -fno-rtti -fno-exceptions
OPTS = -MMD -mcpu=cortex-m3 -mthumb -DF_CPU=84000000L -DARDUINO=10813 -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON "-DUSB_MANUFACTURER=\"ZoeRobotics LLC\"" "-DUSB_PRODUCT=\"AI Robot-Cafe\""
INC_LIBS =  -I${LIB_DIR}/system/libsam/
INC_LIBS += -I${LIB_DIR}/system/CMSIS/CMSIS/Include/ 
INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/ 
INC_LIBS += -I${LIB_DIR}/cores/arduino/
INC_LIBS += -I${LIB_DIR}/variants/arduino_due_x/
INC_LIBS += -I${LIB_DIR}/system/libsam/include/

INC_LIBS =  -I${LIB_DIR}/system/libsam/
INC_LIBS += -I${LIB_DIR}/system/CMSIS/CMSIS/Include/ 
INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/ 
INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/
INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/include/
INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3xa/include/component/
# INC_LIBS += -I${LIB_DIR}/system/CMSIS/Device/ATMEL/sam3n/

INC_LIBS += -I${LIB_DIR}/cores/arduino/
INC_LIBS += -I${LIB_DIR}/variants/arduino_due_x/

INC_LIBS += -I${LIB_DIR}/system/libsam/include
INC_LIBS += -I${LIB_DIR}/system/

# INC_LIBS += /Users/mikekim/Developer/ArduinoDue/Atmel-sam3x-libs/libchip_sam3x/include

## LINKING OPTION'S DEFINITION
LINK = $(COMPILE_DIR)/arm-none-eabi-gcc
LINK_COPY = $(COMPILE_DIR)/arm-none-eabi-objcopy
LINK_SIZING = $(COMPILE_DIR)/arm-none-eabi-size

LINK_OPTS1 = -mcpu=cortex-m3 -mthumb -Os -Wl,--gc-sections -T${LIB_DIR}/flash.ld -Wl,-Map,${current_dir}/${MAIN_FILE}.map 
LINK_OPTS2 = -L${LIB_DIR} -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group -u _sbrk -u link -u _close -u _fstat -u _isatty -u _lseek -u _read -u _write -u _exit -u kill -u _getpid
LINK_OPT_END =   ${LIB_DIR}/libsam_sam3x8e_gcc_rel.a ${LIB_DIR}/core.a  -Wl,--end-group -lm -lgcc
#   LINK_OPT_END =   ${LIB_DIR}/libsam_sam3x8e_gcc_rel.a ${LIB_DIR}/core.a -Wl,--end-group -lm -lgcc
#${LIB_DIR}/libvariant_arduino_due_x_gcc_rel.a
#LINK_OPT_END =   ${LIB_DIR}/core.a -Wl,--end-group -lm -lgcc
# INC_MK = -I/mkSerial.h -I-mkZoeRobotics.h -I/mkZoeRobotics_include.h  -I/mkZoeRobotics_config.h -I/mega2569_io.h -I/mkTimer0.h
#${LIB_DIR}/libvariant_arduino_due_x_gcc_rel.a
## FLASHING ELF FILE TO ARDUINO DUE BOARD
FLASH = /Users/mikekim/Library/Arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac
UPLOAD_BOSSA = /Users/mikekim/Library/Arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac
UPLOAD_VERBOSE_FLAGS = -i -d
all: blink flash_blink
blink:
	${CPP} $(FLAGS) $(OPTS) $(INC_LIBS)   timetick.c mkZoeRobotics_serial.cpp ${MAIN_FILE}.c
	${LINK} ${LINK_OPTS1} -o  ${MAIN_FILE}.elf ${LINK_OPTS2} timetick.o mkZoeRobotics_serial.o ${MAIN_FILE}.o ${LINK_OPT_END}

	${LINK_SIZING} -A ${MAIN_FILE}.elf
	${LINK_COPY} -O binary ${MAIN_FILE}.elf ${MAIN_FILE}.bin
	
.PHONY:	flash_blink
flash_blink:
	-@echo "MK: Touch programming port ..."
	-@stty -f "/dev/$(UPLOAD_PORT_BASENAME)" raw ispeed 1200 ospeed 1200 cs8 -cstopb ignpar eol 255 eof 255
	-@printf "\x00" > "/dev/$(UPLOAD_PORT_BASENAME)"
	-@echo "Waiting before uploading ..."
	-@sleep 1
	-@echo "Uploading ..."
	$(UPLOAD_BOSSA)  --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "$(MAIN_FILE).bin" -R

#	# $(FLASH) -i -d --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "$(MAIN_FILE).bin" -R 
#	$(FLASH)  --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "/Users/mikekim/Developer/ZoeRobotics_project/IStackerRobot/Arm_sam3x8/test_blink/mkZoeRobotics_main.bin" -R

#	# ${FLASH} -i -d --port="$(UPLOAD_PORT_BASENAME)" -U false -e -w -v -b "/Users/mikekim/Developer/ZoeRobotics_project/IStackerRobot/Arm_sam3x8/testSerial/firmware.bin" -R 
clean:
	rm -f *.bin *.hex *.elf *.map *.d *.o
endif


