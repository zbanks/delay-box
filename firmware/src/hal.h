#pragma once

#include "prelude.h"

void hal_init(void);

uint32_t hal_now_ms(void);
void hal_delay_ms(uint32_t m);

void hal_led_set(bool on);
