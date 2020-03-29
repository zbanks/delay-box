#include "hal.h"
#include "prelude.h"
#include "sdcard.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

static size_t error_counts[10] = {0};

int main() {
    hal_init();

    for (size_t i = 0; i < 5; i++) {
        hal_led_set(i & 1);
        hal_delay_ms(50 + i * 5);
    }

    while (true) {
        int rc = -1;
        int init_count = 0;
        while (rc != 0) {
            hal_delay_ms(50);
            rc = sdcard_setup();
            init_count++;
        }

        uint32_t write_pointer = 0;
        // bool led_on = false;
        while (true) {
            uint8_t buf[512];
            // uint32_t value = 100;
            uint32_t value = write_pointer & ~511u; // hal_now_ms();
            memset(buf, (uint8_t)write_pointer, sizeof(buf));
            value = hal_now_ms();
            memcpy(&buf[4], &value, sizeof(value));
            buf[0] = 0xAB;
            buf[1] = 0xCD;
            buf[2] = 0x00;
            buf[3] = 0x00;

            rc = 8;
            // int retries = 3;
            do {
                // rc = sdio_writeblock(card, write_pointer, buf);
                rc = sdcard_write(write_pointer, buf, 512);
                if (rc >= 0 && rc <= 9) {
                    error_counts[rc]++;
                }
                /*
                if (retries-- <= 0) {
                    card = sdio_open();
                    retries = 3;
                    error_counts[9]++;
                }
                */
                //} while (!(rc == SDIO_ESUCCESS || rc == SDIO_EDCRCFAIL));
            } while (rc != 0);

            write_pointer++;
            // hal_led_set(write_pointer & 128);
            hal_led_set((write_pointer & 128) == 0);
            // hal_delay_ms(10);
            // led_on = !led_on;
        }
    }
}
