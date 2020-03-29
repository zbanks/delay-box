#pragma once

#include "prelude.h"

void hal_init(void);

uint32_t hal_now_ms(void);
void hal_delay_ms(uint32_t m);

void hal_led_set(bool on);

void hal_sdcard_init(void);
void hal_sdcard_speed(bool fast);
uint8_t hal_sdcard_xfer(uint8_t tx_byte);
void hal_sdcard_select(bool active);
void hal_sdcard_bulk_write(const void * buffer, size_t len);
void hal_sdcard_bulk_read(void * buffer, size_t len);
