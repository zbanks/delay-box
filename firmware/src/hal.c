#include "hal.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

void hal_init() {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOAEN);

    // Configure systick to 1kHz
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    // systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    // Configure LED output
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO5);
    hal_led_set(false);

    // SPI SD Card
    hal_sdcard_init();
}

// SysTick / timer
volatile uint32_t clock_ms = 0;

void sys_tick_handler() { clock_ms++; }

uint32_t hal_now_ms() { return clock_ms; }

void hal_delay_ms(uint32_t m) {
    uint32_t end_time_ms = hal_now_ms() + m + 1;
    while (hal_now_ms() < end_time_ms)
        ;
}

// LED
void hal_led_set(bool on) {
    (void)on;
    // GPIO A5 is used by SPI1 CLK
    /*
    if (on) {
        gpio_set(GPIOA, GPIO5);
    } else {
        gpio_clear(GPIOA, GPIO5);
    }
    */
}

// SPI SD Card
void hal_sdcard_init() {
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_set(GPIOA, GPIO4);

    spi_disable(SPI1);
    spi_reset(SPI1);
    spi_init_master(SPI1, SPI_CR1_BR_FPCLK_DIV_128, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2,
                    SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);

    hal_sdcard_speed(false);
}

void hal_sdcard_speed(bool fast) {
    (void)fast;
    // spi_set_baudrate_prescaler(SPI1, fast ? SPI_CR1_BR_FPCLK_DIV_32 : SPI_CR1_BR_FPCLK_DIV_128);
}

uint8_t hal_sdcard_xfer(uint8_t tx_byte) {
    while (SPI1_SR & SPI_SR_BSY)
        ;
    return (uint8_t)spi_xfer(SPI1, tx_byte);
}

void hal_sdcard_select(bool active) {
    /*
    if (active == !gpio_get(GPIOA, GPIO4)) {
        // Already in the correct state
        return;
    }
    */

    if (active) {
        gpio_clear(GPIOA, GPIO4);
        while (hal_sdcard_xfer(0xFF) == 0)
            ;
    } else {
        gpio_set(GPIOA, GPIO4);
        // Send 8 clocks after releasing card
        hal_sdcard_xfer(0xFF);
    }
}

void hal_sdcard_bulk_write(const void * buffer, size_t len) {
    const uint8_t * buffer8 = buffer;
    while (len--) {
        hal_sdcard_xfer(*buffer8++);
    }
}

void hal_sdcard_bulk_read(void * buffer, size_t len) {
    uint8_t * buffer8 = buffer;
    while (len--) {
        *buffer8++ = hal_sdcard_xfer(0xFF);
    }
}
