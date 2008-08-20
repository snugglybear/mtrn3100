###############################################################################
# Makefile for the project NEWIMU
###############################################################################

## General Flags
PROJECT = NEWIMU
MCU = atmega88
TARGET = AVR.elf
CC = avr-gcc.exe

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2                                               -DF_CPU=8000000UL -O2 -fsigned-char
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += 


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"D:\My Documents\AVR\Robot Design 08\." 

## Objects that must be built in order to link
OBJECTS = ADC.o MAIN.o serial.o timing.o ultrasonic.o wdt.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) AVR.hex AVR.eep size

## Compile
ADC.o: ../ADC.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

MAIN.o: ../MAIN.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

serial.o: ../serial.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

timing.o: ../timing.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ultrasonic.o: ../ultrasonic.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

wdt.o: ../wdt.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) AVR.elf dep/* AVR.hex AVR.eep

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

