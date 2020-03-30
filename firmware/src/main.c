#include "debug.h"
#include "hal.h"
#include "prelude.h"
#include "sdcard.h"

#define MEMORY_BARRIER() __asm__ volatile ("": : :"memory")

static const uint32_t io_timeout_ms = 1000 + 1;

#define ADC_BUFFER_CHUNKS ((size_t) 64)
#define ADC_BUFFER_SIZE ((size_t) (ADC_BUFFER_CHUNKS * 256))
static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static uint16_t dac_buffer[ADC_BUFFER_SIZE];
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
    hal_dac_begin(dac_buffer, ADC_BUFFER_SIZE);

    uint32_t write_pointer = 0;
    uint32_t read_pointer = 0;
    uint32_t delay = 256;
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
            */
            size_t adc_chunk = hal_adc_count() / 256;
            if (adc_chunk > write_pointer + 4) {
                if (adc_chunk > write_pointer + ADC_BUFFER_CHUNKS - 2) {
                    DEBUG(write_pointer);
                    DEBUG(read_pointer);
                    dropped_buffers += DEBUG(adc_chunk - write_pointer);
                    write_pointer = adc_chunk;
                    dropped_buffer_count++;
                }
                size_t write_chunks = 0;
                while (adc_chunk > write_pointer && write_chunks < 8) {
                    size_t offset = (write_pointer % ADC_BUFFER_CHUNKS) * 256;

                    rc = -1;
                    while (rc != 0 && hal_now_ms() < deadline_ms) {
                        rc = sdcard_write(write_pointer + 256 + 1, &adc_buffer[offset], 512);
                        DEBUG(rc);
                    }
                    if (rc == 0) {
                        deadline_ms = hal_now_ms() + io_timeout_ms;
                    } else {
                        continue;
                    }
                    MEMORY_BARRIER();
                    write_pointer++;
                    write_chunks++;
                    adc_chunk = hal_adc_count() / 256;
                }
                DEBUG(write_chunks);
            }

            size_t dac_chunk = hal_dac_count() / 256;
            if (dac_chunk + ADC_BUFFER_CHUNKS - 5 > read_pointer) {
                size_t read_chunks = 0;
                while (dac_chunk + ADC_BUFFER_CHUNKS - 1 > read_pointer && read_chunks < 8) {
                    size_t offset = (read_pointer % ADC_BUFFER_CHUNKS) * 256;

                    rc = -1;
                    deadline_ms = hal_now_ms() + io_timeout_ms;
                    while (rc != 0 && hal_now_ms() < deadline_ms) {
                        rc = sdcard_read(read_pointer + 256 + 1 - delay, &dac_buffer[offset], 512);
                        DEBUG(rc);
                    }
                    if (rc == 0) {
                        deadline_ms = hal_now_ms() + io_timeout_ms;
                    } else {
                        continue;
                    }
                    MEMORY_BARRIER();
                    read_pointer++;
                    read_chunks++;
                    dac_chunk = hal_dac_count() / 256;
                }
                DEBUG(read_chunks);
            }

            if (hal_button_pushed()) {
                if (read_pointer > delay) {
                    delay += 256;
                }
            }
        }
    }
}
