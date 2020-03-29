#include "hal.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void hal_init() {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOAEN);

    // Configure systick to 1kHz
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    //systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    // Configure LED output
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO5);
    hal_led_set(false);
}

// SysTick / timer
volatile uint32_t clock_ms = 0;

void sys_tick_handler() { clock_ms++; }

uint32_t hal_now_ms() { return clock_ms; }

void hal_delay_ms(uint32_t m) {
    uint32_t end_time_ms = m + hal_now_ms();
    while (hal_now_ms() < end_time_ms)
        ;
}

// LED
void hal_led_set(bool on) {
    if (on) {
        gpio_set(GPIOA, GPIO5);
    } else {
        gpio_clear(GPIOA, GPIO5);
    }
}
