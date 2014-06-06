###############################################################################
# Makefile for the project BE200S
###############################################################################

## General Flags
PROJECT = BE200S

MCU = atmega88

##CLK = 11059200UL
##CLK = 7372800UL

TARGET = BE200S.elf
CC = avr-gcc

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)
#COMMON += -finline

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
##CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=$(CLK) -Os -funsigned-char
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -Os -funsigned-char

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON) 
LDFLAGS += -Wl,--section-start=.flash=0x1f00


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
#INCLUDES = -I".." -I"../../blabla" 

## Objects that must be built in order to link
#OBJECTS = main.o blabla.o blubla.o alabala.o 
OBJECTS = BE200S.o

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) BE200S.hex BE200S.lss size

## Compile

BE200S.o: BE200S.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<


##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

#%.eep: $(TARGET)
#	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
clean:
	-rm $(OBJECTS) BE200S.elf BE200S.hex BE200S.lss
