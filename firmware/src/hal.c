#include "hal.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>

#define MEMORY_BARRIER() __asm__ volatile ("": : :"memory")

void hal_init() {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_DAC);

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

    // Blue USER button, has both pullup/pulldown & 100nF cap
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);

    // SPI SD Card
    hal_sdcard_init();

    // TIM2, a 44100Hz interrupt
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO9);

    nvic_enable_irq(NVIC_TIM2_IRQ);
    rcc_periph_reset_pulse(RST_TIM2);
    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2, rcc_apb1_frequency / 44100);
    //timer_set_period(TIM2, rcc_apb1_frequency / 8000);
    timer_enable_counter(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
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

// Button
bool hal_button_get() {
    return !gpio_get(GPIOC, GPIO13);
}

bool hal_button_pushed() {
    static bool state = false;
    bool new_state = hal_button_get();
    if (new_state && !state) {
        state = new_state;
        return true;
    }
    state = new_state;
    return false;
}

// ADC
static uint16_t * volatile adc_sample_buffer = NULL;
static volatile size_t adc_sample_size = 0;
static size_t adc_sample_count = 0;

void hal_adc_begin(uint16_t * sample_buffer, size_t sample_size) {

    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

    adc_power_off(ADC1);
    //adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    //adc_calibrate(ADC1);
    //adc_disable_external_trigger_regular(ADC1);
    //adc_set_left_aligned(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    //adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_144CYC);
    uint8_t adc_channels[1] = {8};
    adc_set_regular_sequence(ADC1, 1, adc_channels);
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    //adc_disable_analog_watchdog(ADC1);
    adc_power_on(ADC1);
    adc_start_conversion_regular(ADC1);

    adc_sample_buffer = sample_buffer;
    adc_sample_size = sample_size;
    adc_sample_count = 0;
    MEMORY_BARRIER();
}


size_t hal_adc_count(void) { 
    MEMORY_BARRIER();
    return adc_sample_count; 
}

static uint16_t * dac_sample_buffer = NULL;
static volatile size_t dac_sample_size = 0;
static size_t dac_sample_count = 0;
void hal_dac_begin(uint16_t * sample_buffer, size_t sample_size) {
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    dac_disable(CHANNEL_1);
    dac_enable(CHANNEL_1);
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
    dac_load_data_buffer_single(0, LEFT12, CHANNEL_1);

    dac_sample_buffer = sample_buffer;
    dac_sample_size = sample_size;
    dac_sample_count = 0;
    MEMORY_BARRIER();
}

size_t hal_dac_count() {
    MEMORY_BARRIER();
    return dac_sample_count;
}

void tim2_isr(void) {
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    gpio_toggle(GPIOB, GPIO9);

    if (dac_sample_buffer != NULL && dac_sample_size != 0) {
        dac_software_trigger(CHANNEL_1);
        MEMORY_BARRIER();
        int v = dac_sample_buffer[dac_sample_count++ % dac_sample_size];

#if 0
        // A basic 1-pole LPF to remove DC bias before applying a constant gain
        // With a denominator of 256, the time constant is 
        static int avg_v = 0;
        avg_v += (v - avg_v) / 256;
        static const int gain = 1;
        v = (v - avg_v) * gain + 2048;

        // 12-bit saturation
        if (v < 0) {
            v = 0;
        } else if (v > 4095) {
            v = 4095;
        }
#endif

        dac_load_data_buffer_single((uint16_t) v, LEFT12, CHANNEL_1);
    }

    if (adc_sample_buffer != NULL && adc_sample_size != 0) {
        adc_sample_buffer[adc_sample_count++ % adc_sample_size] = (uint16_t) adc_read_regular(ADC1);
        MEMORY_BARRIER();

        adc_start_conversion_regular(ADC1);
    }

}

// SPI SD Card
void hal_sdcard_init() {
    // Configure the SD card to use SPI1 on GPIO PA5 PA6 PA7 (Alt Fn 5)
    // PA5 CLK; PA6 MISO; PA7 MOSI
    // PA9 is used as software CS

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
