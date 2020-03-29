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

// ADC
void hal_adc_begin(uint16_t * sample_buffer, size_t sample_count) {
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA1EN);
    (void)sample_buffer;
    (void)sample_count;
}

size_t hal_adc_count(void) { return 0; }

// SPI SD Card
void hal_sdcard_init() {
    // Configure the SD card to use SPI1 on GPIO PA5 PA6 PA7 (Alt Fn 5)
    // PA5 CLK; PA6 MISO; PA7 MOSI
    // PA9 is used as software CS

    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO9 | GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_set(GPIOA, GPIO9);

    spi_disable(SPI1);
    spi_reset(SPI1);
    spi_init_master(SPI1, SPI_CR1_BR_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2,
                    SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);

    hal_sdcard_speed(false);
}

void hal_sdcard_speed(bool fast) {
    // CPU clock is 168MHz
    // Slow SPI clock is 168MHz / 256 = 650 kHz
    // Fast SPI clock is 168MHz / 8 = 21 MHz
    spi_set_baudrate_prescaler(SPI1, fast ? SPI_CR1_BR_FPCLK_DIV_8 : SPI_CR1_BR_FPCLK_DIV_256);
}

uint8_t hal_sdcard_xfer(uint8_t tx_byte) { return (uint8_t)spi_xfer(SPI1, tx_byte); }

int hal_sdcard_select(bool active, uint32_t deadline_ms) {
    if (active) {
        gpio_clear(GPIOA, GPIO9);
        int rc = -1;
        while (rc != 0 && hal_now_ms() < deadline_ms) {
            if (hal_sdcard_xfer(0xFF) != 0) {
                rc = 0;
            }
        }
        return rc;
    } else {
        gpio_set(GPIOA, GPIO9);
        // Send 8 clocks after releasing card
        hal_sdcard_xfer(0xFF);
        return 0;
    }
}

void hal_sdcard_bulk_write(const void * buffer, size_t len) {
    // This could be replaced with DMA (but should remain a blocking call)
    const uint8_t * buffer8 = buffer;
    while (len--) {
        hal_sdcard_xfer(*buffer8++);
    }
}

void hal_sdcard_bulk_read(void * buffer, size_t len) {
    // This could be replaced with DMA (but should remain a blocking call)
    uint8_t * buffer8 = buffer;
    while (len--) {
        *buffer8++ = hal_sdcard_xfer(0xFF);
    }
}
