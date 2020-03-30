#pragma once

#include "prelude.h"

void hal_init(void);

uint32_t hal_now_ms(void);
void hal_delay_ms(uint32_t m);

void hal_led_set(bool on);
bool hal_button_get(void);
bool hal_button_pushed(void);

void hal_adc_begin(uint16_t * sample_buffer, size_t sample_count);
size_t hal_adc_count(void);
void hal_dac_begin(uint16_t * sample_buffer, size_t sample_size);
size_t hal_dac_count(void);

void hal_sdcard_init(void);
void hal_sdcard_speed(bool fast);
uint8_t hal_sdcard_xfer(uint8_t tx_byte);
int hal_sdcard_select(bool active, uint32_t deadline_ms);
void hal_sdcard_bulk_write(const void * buffer, size_t len);
void hal_sdcard_bulk_read(void * buffer, size_t len);
