avr-gcc -mmcu=atmega644p -DF_CPU=12000000 -Wall -Os -Wl,-u,vfprintf -lprintf_flt -lm -c embedded_boost.c
avr-gcc -mmcu=atmega644p -L ./  -o embedded_boost.elf embedded_boost.o -llcd
avr-objcopy -O ihex embedded_boost.elf embedded_boost.hex
avrdude -c usbasp -p m644p -U flash:w:embedded_boost.hex
pause >nul