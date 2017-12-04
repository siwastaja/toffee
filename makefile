# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy


CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m7 -Wall -fstack-usage -Winline -DSTM32F765xx

ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m7 -mthumb -nostartfiles -gc-sections

DEPS =
OBJ = stm32init.o main.o own_std.o
ASMS = stm32init.s main.s own_std.s

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^
	$(OBJCOPY) -Obinary --remove-section=.ARM* main.elf main_full.bin
	$(SIZE) main.elf

flash_full: main.bin
	sudo stm32sprog -b 115200 -vw main_full.bin

flash: flash_full

stack:
	cat *.su

sections:
	arm-none-eabi-objdump -h main.elf

syms:
	arm-none-eabi-objdump -t main.elf

%.s: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e: 
	gedit --new-window stm32init.c main.c &

s:
	sudo screen /dev/ttyUSB0 230400
