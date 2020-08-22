# AVR-GCC Makefile
PROJECT=qi
SOURCES=main.c fifo.c
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=atmega328p
COM=COM6
CFLAGS=-mmcu=$(MMCU) -Wall -std=c99

$(PROJECT).hex: $(PROJECT).out
	$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(CC) $(CFLAGS) -Os -I./ -o $(PROJECT).out $(SOURCES)

p: $(PROJECT).hex
	avrdude -c arduino -p m328p -P $(COM) -b 57600 -U flash:w:$(PROJECT).hex:i

 
clean: 
	rm qi.hex
	rm qi.out