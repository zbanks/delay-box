#include "hal.h"
#include "prelude.h"
#include "sdio.h"

uint32_t error_counts[10] = {0};

int main() {
    hal_init();
    sdio_init();

    for (size_t i = 0; i < 5; i++) {
        hal_led_set(i & 1);
        hal_delay_ms(50 + i * 5);
    }

    while (true) {
        hal_delay_ms(50);

        struct sdio_card * card = sdio_open();
        if (card == NULL) {
            hal_led_set(true);
            hal_delay_ms(100);
            hal_led_set(false);
            hal_delay_ms(50);
            hal_led_set(true);
            hal_delay_ms(100);
            hal_led_set(false);
            hal_delay_ms(500);
            continue;
        }

        //uint8_t tbuf[512];
        //sdio_readblock(card, 0, tbuf);

        uint32_t write_pointer = 0;
        //bool led_on = false;
        while (true) {
            uint8_t buf[512];
            //uint32_t value = 100;
            uint32_t value = write_pointer & ~511u; //hal_now_ms();
            memset(buf, (uint8_t) write_pointer, sizeof(buf));
            value = hal_now_ms();
            memcpy(&buf[4], &value, sizeof(value));
            //memset(buf, 0x15, sizeof(buf));
            //buf[0] = 0;
            //buf[0] = 0;
            (void) value;
            //memcpy(&buf[0], &value, sizeof(value));
            /*
            for (size_t i = 0; i < 512; i += sizeof(value)) {
                memcpy(&buf[i], &value, sizeof(value));
            }
            */

            int rc = 1;
            int retries = 3;
            do {
                rc = sdio_writeblock(card, write_pointer, buf);
                //hal_delay_ms(100);
                if (rc <= 0 && rc >= -9) {
                    error_counts[-rc]++;
                }
                if (retries-- <= 0) {
                    card = sdio_open();
                    retries = 3;
                    error_counts[9]++;
                }
            //} while (!(rc == SDIO_ESUCCESS || rc == SDIO_EDCRCFAIL));
            } while (rc != SDIO_ESUCCESS);

            write_pointer++;
            hal_led_set(write_pointer & 128);
            //led_on = !led_on;
        }
    }
}
