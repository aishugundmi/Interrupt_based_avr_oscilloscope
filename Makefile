# AVR-GCC Makefile
PROJECT=qi
SOURCES=main.c fifo.c
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=atmega328p
COM=COM4	
CFLAGS=-mmcu=$(MMCU) -Wall -std=c99

$(PROJECT).hex: $(PROJECT).out
	$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(CC) $(CFLAGS) -Os -I./ -o $(PROJECT).out $(SOURCES)

p: $(PROJECT).hex
	avrdude -c arduino -p t13 -P $(COM) -b 19200 -U flash:w:$(PROJECT).hex:i

burn_fuse: $(PROJECT).hex
	avrdude -c arduino -p t13 -P $(COM) -b 19200 -U lfuse:w:0x7A:m	-U hfuse:w:0xF9:m	-U lock:w:0xFF:m
	
clean:
	arm 
	arm qi.hex
	arm qi.out