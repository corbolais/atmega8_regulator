
TGT = regulator
$(TGT).hex: $(TGT).o
	avr-objcopy -j .text -j .data -O ihex regulator.o regulator.hex

$(TGT).o: $(TGT).c
	avr-gcc -mmcu=atmega8 -Os regulator.c -o regulator.o

.PHONY = upload
upload:
	sudo uisp -dprog=dapa -dlpt=0x378 --erase
	sudo uisp -dprog=dapa -dlpt=0x378 --upload if=regulator.hex 
