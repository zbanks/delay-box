DEVICE          = stm32f446ret6u
OPENCM3_DIR     = libopencm3
OBJS            += $(patsubst %.c,%.o,$(wildcard src/*.c))
OBJS            += openfat/stm32/mmc.o

CFLAGS          += -O0 -ggdb3 -std=c11
CFLAGS          += #-flto
CFLAGS          += -Werror -Wall -Wextra -Wshadow -Wconversion
CFLAGS          += -Isrc/
CFLAGS          += -Iopenfat/include -Iopenfat/stm32
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean format all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -rf binary.* $(OBJS) generated*.ld

format:
	clang-format -i src/*.c src/*.h

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk

DB = $(PREFIX)-gdb

gdb: binary.elf
	$(DB) $< -x init.gdb
load: binary.elf
	$(DB) $< -x init.gdb -ex load
loadquit: binary.elf
	$(DB) $< -batch -x init.gdb -ex load -ex kill -ex quit
loadquitpwr: binary.elf
	$(DB) $< -batch -x init.gdb -ex 'mon tpwr en' -ex load -ex kill -ex quit
stlink:
	st-util -p 4244
stload: binary.bin
	st-flash write $< 0x8000000
sterase:
	st-flash erase
dfuload: binary.bin
	dfu-util -d 0483:df11 -a 0 -s 0x8000000 -D $<

