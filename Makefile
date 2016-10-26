all: program

%.o: %.c
	avr-gcc -std=gnu99 -Os -mmcu=$(MCU) $^ -c -o $@

%.elf: %.o
	avr-gcc -O3 -mmcu=$(MCU) $^ -o $@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@

clean:
	$(RM) *.o *.hex *.elf

fuse:
	avrdude -c usbasp-clone -P USB -U lfuse:w:0xf1:m -p $(DEV)

program: $(TARGET).hex
	avrdude -c usbasp-clone -P USB -U flash:w:$(TARGET).hex -p $(DEV)
