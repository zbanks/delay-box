#include "debug.h"
#include "hal.h"
#include "prelude.h"
#include "sdcard.h"

static size_t error_counts[10] = {0};
static const uint32_t io_timeout_ms = 1000 + 1;

int main() {
    hal_init();

    for (size_t i = 0; i < 5; i++) {
        hal_led_set(i & 1);
        hal_delay_ms(50 + i * 5);
    }

    uint32_t write_pointer = 0;
    while (true) {
        int rc = -1;
        int init_count = 0;
        while (rc != 0) {
            hal_delay_ms(50);
            rc = sdcard_setup();
            init_count++;
        }

        uint32_t deadline_ms = hal_now_ms() + io_timeout_ms;
        while (hal_now_ms() < deadline_ms) {
            uint8_t buf[512];
            uint32_t value = write_pointer & ~511u; // hal_now_ms();
            memset(buf, (uint8_t)write_pointer, sizeof(buf));

            rc = -1;
            while (rc != 0 && hal_now_ms() < deadline_ms) {
                rc = sdcard_read(write_pointer + 1, buf, 512);
            }
            if (rc == 0) {
                deadline_ms = hal_now_ms() + io_timeout_ms;
            } else {
                continue;
            }

            value = hal_now_ms();
            memcpy(&buf[4], &value, sizeof(value));
            memcpy(&buf[0], &write_pointer, sizeof(write_pointer));

            rc = -1;
            while (rc != 0 && hal_now_ms() < deadline_ms) {
                rc = sdcard_write(write_pointer, buf, 512);
                if (rc >= 0 && rc <= 9) {
                    error_counts[rc]++;
                } else {
                    error_counts[9]++;
                }
            }
            if (rc == 0) {
                deadline_ms = hal_now_ms() + io_timeout_ms;
            } else {
                continue;
            }

            write_pointer++;
        }
    }
}
