# Define the microcontroller
MCU = atmega328p

# Define the clock frequency
F_CPU = 16000000

# Define the compiler and flags
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall

# Define the output file
TARGET = UPDI_Programmer

# Define the object files
OBJS = main.o

# Define the rules
all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex $< $@

main.o: main.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET).elf $(TARGET).hex $(OBJS)

