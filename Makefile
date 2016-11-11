all: program

%.o: %.c
	avr-gcc -lm --fast-math -std=c11 -Ofast -mmcu=$(MCU) $^ -c -o $@

%.elf: %.o
	avr-gcc -lm -mmcu=$(MCU) $^ -o $@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@

clean:
	$(RM) *.o *.hex *.elf

fuse:
	sudo avrdude -c usbasp-clone -P USB -U lfuse:w:0xf1:m -p $(DEV)

program: $(TARGET).hex $(TARGET).elf
	sudo avrdude -c usbasp-clone -P USB -U flash:w:$(TARGET).hex -p $(DEV)
