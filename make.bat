avr-gcc -mmcu=atmega644p -DF_CPU=12000000 -Os -Wall -u vfprintf -lprintf_flt -lm embedded_boost.c -o embedded_boost.elf
avr-objcopy -O ihex embedded_boost.elf embedded_boost.hex
avrdude -c usbasp -p m644p -U flash:w:embedded_boost.hex