#include "debug.h"
#include "hal.h"
#include "prelude.h"
#include "sdcard.h"

static size_t error_counts[10] = {0};
static const uint32_t io_timeout_ms = 1000 + 1;

#define ADC_BUFFER_CHUNKS ((size_t) 8)
#define ADC_BUFFER_SIZE ((size_t) (ADC_BUFFER_CHUNKS * 256))
static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static size_t dropped_buffers = 0;
static size_t dropped_buffer_count = 0;

int main() {
    hal_init();

    /*
    for (size_t i = 0; i < 5; i++) {
        hal_led_set(i & 1);
        hal_delay_ms(50 + i * 5);
    }
    */
    hal_adc_begin(adc_buffer, ADC_BUFFER_SIZE);
    hal_dac_begin(adc_buffer, ADC_BUFFER_SIZE);

    uint32_t write_pointer = 0;
    while (true) {
        int rc = -1;
        int init_count = 0;
        while (rc != 0) {
            hal_delay_ms(50);
            rc = sdcard_setup();
            init_count++;
        }

        //hal_adc_begin(adc_buffer, ADC_BUFFER_SIZE);

        uint32_t deadline_ms = hal_now_ms() + io_timeout_ms;
        while (hal_now_ms() < deadline_ms) {
            /*
            rc = -1;
            while (rc != 0 && hal_now_ms() < deadline_ms) {
                rc = sdcard_read(write_pointer + 1, buf, 512);
            }
            if (rc == 0) {
                deadline_ms = hal_now_ms() + io_timeout_ms;
            } else {
                continue;
            }
            */
            size_t adc_chunk = 0;
            while (adc_chunk <= write_pointer) {
                adc_chunk = hal_adc_count() / 256;
            }
            if (adc_chunk > write_pointer + ADC_BUFFER_CHUNKS - 2) {
                dropped_buffers += adc_chunk - write_pointer;
                write_pointer = adc_chunk;
                dropped_buffer_count++;
            }
            size_t offset = (write_pointer % ADC_BUFFER_CHUNKS) * 256;

            rc = -1;
            while (rc != 0 && hal_now_ms() < deadline_ms) {
                rc = sdcard_write(write_pointer + 1, &adc_buffer[offset], 512);
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
