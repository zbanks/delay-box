// Original source:
// https://github.com/ChuckM/stm32f4-sdio-driver

/*
 * sdio.c
 *
 * SDIO Bus Driver layer. This code sends commands and drives
 * the SDIO peripheral on the STM32F4xx, there is a layer above
 * this, the SD Card driver, which uses this driver to talk to
 * SD Cards. The SPI driver can also talk to SD Cards, hence the
 * split at this layer.
 *
 * Note that the simple implementation for the SDIO driver runs
 * in a 'polled' mode. This is easier to explain and debug and
 * sufficient for the first few projects. A more sophisticated
 * version with DMA and interrupts will follow.
 *
 * Could be part of the libopencm3 project if they wanted it.
 */

#include "sdio.h"
#include "hal.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/sdio.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Helper defines to pull out various bit fields of the CSD for
 * the size calculation.
 */
#define SDIO_CSD_VERSION(x) sdio_bit_slice(x->csd, 128, 127, 126)
#define SDIO_CSD1_CSIZE_MULT(x) sdio_bit_slice(x->csd, 128, 49, 47)
#define SDIO_CSD1_RBLKLEN(x) sdio_bit_slice(x->csd, 128, 83, 80)
#define SDIO_CSD1_CSIZE(x) sdio_bit_slice(x->csd, 128, 73, 62)
#define SDIO_CSD2_CSIZE(x) sdio_bit_slice(x->csd, 128, 69, 48)

/*
 * Conveniently swaps the bytes in a long around
 * used by the SCR code.
 */
#define byte_swap(val) __asm__("rev %[swap], %[swap]" : [swap] "=r"(val) : "0"(val));

static int sdio_bus(int bits, enum sdio_clock_div freq);
static void sdio_reset(void);
static int sdio_command(uint32_t cmd, uint32_t arg);
static uint32_t sdio_bit_slice(uint32_t a[], int bits, int msb, int lsb);

/*
 * sdio_bus
 *
 * Set the bus width and the clock speed for the
 * SDIO bus.
 *
 * Returns 0 on success
 *      -1 illegal bit specification
 *      -2 illegal clock specification
 */
static int sdio_bus(int bits, enum sdio_clock_div freq) {
    uint32_t clkreg = 0;

    switch (bits) {
    case 1:
        clkreg |= SDIO_CLKCR_WIDBUS_1;
        break;
    case 4:
        clkreg |= SDIO_CLKCR_WIDBUS_4;
        break;
    default:
        return -1;
    }
    switch (freq) {
    case SDIO_24MHZ:
        break;
    case SDIO_16MHZ:
        clkreg |= 1;
        break;
    case SDIO_12MHZ:
        clkreg |= 2;
        break;
    case SDIO_8MHZ:
        clkreg |= 8;
        break;
    case SDIO_4MHZ:
        clkreg |= 10;
        break;
    case SDIO_1MHZ:
        clkreg |= 46;
        break;
    case SDIO_400KHZ:
        clkreg |= 118;
        break;
    default:
        return -2;
    }
    clkreg |= SDIO_CLKCR_CLKEN;
    SDIO_CLKCR = clkreg;
    return 0;
}

/*
 * Set up the GPIO pins and peripheral clocks for the SDIO
 * system. The code should probably take an option card detect
 * pin, at the moment it uses the one used by the Embest board.
 */
void sdio_init(void) {
    /* Enable clocks for SDIO and DMA2 */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SDIOEN);

#ifdef WITH_DMA2
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
#endif
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIODEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_CRCEN);

    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO15);

    /* Setup GPIO Pins for SDIO:
        PC8 - PC11 - DAT0 thru DAT3
              PC12 - CLK
               PD2 - CMD
    */
    // All SDIO lines are push-pull, 25Mhz
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8 | GPIO9 | GPIO10 | GPIO11);
    // All SDIO lines are push-pull, 25Mhz
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO12);
    // D0 - D3 enable pullups (bi-directional)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8 | GPIO9 | GPIO10 | GPIO11);
    // CLK line no pullup
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);

    gpio_set_af(GPIOC, GPIO_AF12, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12);

    /* GPIOD setup */
    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);
    gpio_set_af(GPIOD, GPIO_AF12, GPIO2);
}

/*
 * Reset the state of the SDIO bus and peripheral. This code tries
 * to reset the bus *AND* the card if one is plugged in. The bus
 * can be reset by software but the card is reset by powering it down.
 *
 * The SDIO_POWER_STATE tells the code which state to leave the bus in,
 * powered up or powered down.
 *
 * If the state is POWER_ON, then the bus is reset to 400Khz, 1 bit wide
 * which is what he spec requires. Once the type and capabilities of the
 * card have been determined, it can be upgraded.
 */
static void sdio_reset() {

    SDIO_POWER = SDIO_POWER_PWRCTRL_PWROFF;
    /* reset the SDIO peripheral interface */
    rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_SDIORST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_SDIORST);

    SDIO_POWER = SDIO_POWER_PWRCTRL_PWRON;
    sdio_bus(1, SDIO_400KHZ); // required by the spec
}

/*
 * The error message catalog.
 */
/*
static const char *__sdio_error_msgs[] = {
    "Success",
    "Command Timeout",              // -1
    "Command CRC Failure",          // -2
    "Soft Timeout (No Response)",   // -3
    "Data CRC Failure",             // -4
    "RX FIFO Overrun",              // -5
    "TX FIFO Underrun",             // -6
    "Unsupported Card"              // -7
};
*/

#define SDIO_ESUCCESS 0
#define SDIO_ECTIMEOUT -1
#define SDIO_ECCRCFAIL -2
#define SDIO_ENORESP -3
#define SDIO_EDCRCFAIL -4
#define SDIO_ERXOVERR -5
#define SDIO_ETXUNDER -6
#define SDIO_EBADCARD -7
#define SDIO_EUNKNOWN -8

/*
 * sdio_bit_slice - helper function
 *
 * A number of the things the SDIO returns are in bit
 * fields. This code is designed to slice out a range
 * of bits and return them as a value (up to 32 bits
 * worth).
 */
static uint32_t sdio_bit_slice(uint32_t a[], int bits, int msb, int lsb) {
    uint32_t t;
    int i;

    if (((msb >= bits) || (msb < 0)) || (lsb > msb) || ((lsb < 0) || (lsb >= bits))) {
        return 0;
    }
    t = 0;
    for (i = msb; i > lsb; i--) {
        t |= (a[((bits - 1) - i) / 32] >> (i % 32)) & 0x1;
        t <<= 1;
    }
    t |= (a[((bits - 1) - lsb) / 32] >> (lsb % 32)) & 0x1;
    return t;
}

/*
 * A convienence define. These are the flags we care about when
 * sending a command. During command processing SDIO_STA_CMDACT
 * will be set.
 */
#define COMMAND_FLAGS (SDIO_STA_CMDSENT | SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)

/*
 * Send a command over the SDIO bus.
 * Passed a command (8 bit value) and an argument (32 bit value)
 * This command figures out if the command will return a short (32 bit)
 * or long (64 bit) response. It is up to the calling program to pull
 * data from the long response commands.
 * Passed:
 *          cmd - Command to execute
 *          arg - Argument to pass to the command
 *          buf - pointer to a long aligned buffer if data
 *          len - expected length of buffer (in bytes)
 */
static int sdio_command(uint32_t cmd, uint32_t arg) {
    uint32_t tmp_val;
    int error = 0;

    tmp_val = SDIO_CMD & ~0x7ffu;              // Read pre-existing state
    tmp_val |= (cmd & SDIO_CMD_CMDINDEX_MASK); // Put the Command in
    tmp_val |= SDIO_CMD_CPSMEN;                // We'll be running CPSM

    switch (cmd) {
    case 0:
        tmp_val |= SDIO_CMD_WAITRESP_NO_0;
        break;
    case 2:
    case 9:
        tmp_val |= SDIO_CMD_WAITRESP_LONG;
        break;
    default:
        tmp_val |= SDIO_CMD_WAITRESP_SHORT; // the common case
        break;
    }
/* If a data transaction is in progress, wait for it to finish */
#if 0
    if ((cmd != 12) && (SDIO_STA & (SDIO_STA_RXACT | SDIO_STA_TXACT))) {
        // XXX: This should be an error, we don't have multithread
        tmp_val |= SDIO_CMD_WAITPEND;
    }
#endif

    /*
     * EXECUTE:
     *    o Reset all status bits
     *    o Put ARG into SDIO ARG
     *    o reset the error indicator
     *    o Enable all interrupts.
     *    o Do the command
     */
    SDIO_ICR = 0x7ff; // Reset everything that isn't bolted down.
    SDIO_ARG = arg;
    SDIO_CMD = tmp_val;
    /*
     * In a polled mode we should be able to just read the status bits
     * directly.
     */
    tmp_val = 0;
    do {
        tmp_val |= (SDIO_STA & 0x7ff);
    } while ((SDIO_STA & SDIO_STA_CMDACT) || (!tmp_val));
    ;
    SDIO_ICR = tmp_val;

    /*
     * Compute the error here. Which can be one of
     * -- Success (either CMDSENT or CMDREND depending on response)
     * -- Timeout (based on CTIMEOUT)
     * -- No Response (based on no response in the time alloted)
     * -- CRC Error (based on CCRCFAIL)
     */
    if (!tmp_val) {
        error = SDIO_ENORESP;
    } else if (tmp_val & SDIO_STA_CCRCFAIL) {
        error = SDIO_ECCRCFAIL;
    } else if (tmp_val & (SDIO_STA_CMDREND | SDIO_STA_CMDSENT)) {
        error = SDIO_ESUCCESS;
    } else if (tmp_val & SDIO_STA_CTIMEOUT) {
        error = SDIO_ECTIMEOUT;
    } else {
        error = SDIO_EUNKNOWN;
    }

#ifdef SDIO_LOGGING
    // Note the result in our short log
    sdio_log(cmd, arg, error);
#endif
    return error;
}

/* our static data buffer we use for data movement commands */
static uint32_t data_buf[129];
static int data_len;

/*
 * Helper function - sdio_select
 *
 * This function "selects" a card using CMD7, note that if
 * you select card 0 that deselects the card (RCA is not allowed
 * to be 0)
 */
static int sdio_select(uint32_t rca) {
    int err;

    err = sdio_command(7, rca << 16u);
    if ((rca == 0) && (err == SDIO_ECTIMEOUT)) {
        return 0; // "cheat" a timeout selecting 0 is a successful deselect
    }
    return err;
}

/*
 * Helper function - sdio_scr
 *
 * Unlike the CID and CSD functions this function transfers data
 * so it needs to use the DPSM.
 *
 * Note that data over the wire is byte swapped so we swap it back
 * to "fix" it.
 *
 * Note when this return 0 the first two longs in the data_buf are
 * the SCR register.
 */

static int sdio_scr(struct sdio_card * c) {
    int err;
    uint32_t tmp_reg;
    int ndx;

    /* Select the card */
    err = sdio_select(c->rca);
    if (!err) {
        /* Set the Block Size */
        err = sdio_command(16, 8);
        if (!err) {
            /* APPCMD (our RCA) */
            err = sdio_command(55, ((uint32_t)c->rca) << 16u);
            if (!err) {
                SDIO_DTIMER = 0xffffffff;
                SDIO_DLEN = 8;
                SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTEN;
                /* ACMD51 - Send SCR */
                err = sdio_command(51, 0);
                if (!err) {
                    data_len = 0;
                    do {
                        tmp_reg = SDIO_STA;
                        if (tmp_reg & SDIO_STA_RXDAVL) {
                            data_buf[data_len++] = SDIO_FIFO;
                        }
                    } while (tmp_reg & SDIO_STA_RXACT);
                    if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                        if (tmp_reg & SDIO_STA_DCRCFAIL) {
                            err = SDIO_EDCRCFAIL;
                        } else if (tmp_reg & SDIO_STA_RXOVERR) {
                            err = SDIO_ERXOVERR;
                        } else {
                            err = SDIO_EUNKNOWN; // XXX: unknown error
                        }
                    }
                    if (!err) {
                        for (ndx = 0; ndx < 2; ndx++) {
                            byte_swap(data_buf[ndx]);
                            c->scr[ndx] = data_buf[ndx];
                        }
                    }
                }
            }
        }
    }
    (void)sdio_select(0);
    return err;
}

/*
 * Read a Block from our Card
 *
 * NB: There is a possibly useless test in this code, during the read
 * phase it allows that the SDIO card might try to send more than 512
 * bytes (128 32 bit longs) and allows it to do so, constantly over
 * writing the last long in the just-in-case-over-long-by-1 data buffer.
 * To compromise the system you would need a borked or custom crafted
 * sdio card which did that.
 */
int sdio_readblock(struct sdio_card * c, uint32_t lba, uint8_t * buf) {
    int err;
    uint32_t tmp_reg;
    uint32_t addr = lba;
    uint8_t * t;
    int ndx;

    if (!SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }
    err = sdio_select(c->rca);
    if (!err) {
        err = sdio_command(16, 512);
        if (!err) {
            SDIO_DTIMER = 0xffffffff;
            SDIO_DLEN = 512;
            SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTEN;
            err = sdio_command(17, addr);
            if (!err) {
                data_len = 0;
                do {
                    tmp_reg = SDIO_STA;
                    if (tmp_reg & SDIO_STA_RXDAVL) {
                        data_buf[data_len] = SDIO_FIFO;
                        if (data_len < 128) {
                            ++data_len;
                        }
                    }
                } while (tmp_reg & SDIO_STA_RXACT);
                if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                    if (tmp_reg & SDIO_STA_DCRCFAIL) {
                        err = SDIO_EDCRCFAIL;
                    } else if (tmp_reg & SDIO_STA_RXOVERR) {
                        err = SDIO_ERXOVERR;
                    } else {
                        err = SDIO_EUNKNOWN; // Unknown Error!
                    }
                } else {
/* Data received, byte swap and put in user
 * supplied buffer.
 */
#if 0
                    for (ndx = 0; ndx < data_len; ndx++) {
                        byte_swap(data_buf[ndx]);
                    }
#endif
                    t = (uint8_t *)(data_buf);
                    /* copy out to the user buffer */
                    for (ndx = 0; ndx < 512; ndx++) {
                        *buf = *t;
                        buf++;
                        t++;
                    }
                }
            }
        }
    }
    // deselect the card
    (void)sdio_select(0);
    return err;
}

/*
 * Write a Block from our Card
 */
int sdio_writeblock(struct sdio_card * c, uint32_t lba, uint8_t * buf) {
    int err;
    uint32_t tmp_reg;
    uint32_t addr = lba;
    uint8_t * t;
    int ndx;

    if (!SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }

    /*
     * Copy buffer to our word aligned buffer. Nominally you
     * can just use the passed in buffer and cast it to a
     * uint32_t * but that can cause issues if it isn't
     * aligned.
     */
    t = (uint8_t *)(data_buf);
    for (ndx = 0; ndx < 512; ndx++) {
        *t = *buf;
        buf++;
        t++;
    }
    err = sdio_select(c->rca);
    if (!err) {
        /* Set Block Size to 512 */
        err = sdio_command(16, 512);
        if (!err) {
            SDIO_DTIMER = 0xffffffff;
            SDIO_DLEN = 512;
            SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 | SDIO_DCTRL_DTEN;
            err = sdio_command(24, addr);
            if (!err) {
                data_len = 0;
                do {
                    tmp_reg = SDIO_STA;
                    if (tmp_reg & SDIO_STA_TXFIFOHE) {
                        SDIO_FIFO = data_buf[data_len];
                        if (data_len < 128) {
                            ++data_len;
                        }
                    }
                } while (tmp_reg & SDIO_STA_TXACT);
                if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                    if (tmp_reg & SDIO_STA_DCRCFAIL) {
                        err = SDIO_EDCRCFAIL;
                    } else if (tmp_reg & SDIO_STA_TXUNDERR) {
                        err = SDIO_ETXUNDER;
                    } else {
                        err = SDIO_EUNKNOWN; // Unknown Error!
                    }
                }
            }
        }
    }
    // deselect the card
    (void)sdio_select(0);
    return err;
}

/*
 * sdio-status - Get Card Status page
 *
 * This function fetches the SD Card Status page and
 * copies it into the CARD structure.
 */
int sdio_status(struct sdio_card * c) {
    uint32_t tmp_reg;
    int ndx;
    int err;

    err = sdio_select(c->rca);
    if (!err) {
        err = sdio_command(16, 64);
        if (!err) {
            err = sdio_command(55, ((uint32_t)c->rca) << 16u);
            if (!err) {
                SDIO_DTIMER = 0xffffffff;
                SDIO_DLEN = 64;
                SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_6 | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTEN;
                /* ACMD13 - Send Status Reg */
                err = sdio_command(13, 0);
                if (!err) {
                    data_len = 0;
                    do {
                        tmp_reg = SDIO_STA;
                        if (tmp_reg & SDIO_STA_RXDAVL) {
                            data_buf[data_len] = SDIO_FIFO;
                            if (data_len < 128) {
                                ++data_len;
                            }
                        }
                    } while (tmp_reg & SDIO_STA_RXACT);
                    if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                        if (tmp_reg & SDIO_STA_DCRCFAIL) {
                            err = SDIO_EDCRCFAIL;
                        } else if (tmp_reg & SDIO_STA_RXOVERR) {
                            err = SDIO_ERXOVERR;
                        } else {
                            err = SDIO_EUNKNOWN; // Unknown Error!
                        }
                    } else {
                        for (ndx = 0; ndx < 16; ndx++) {
                            byte_swap(data_buf[ndx]);
                            c->status[ndx] = data_buf[ndx];
                        }
                    }
                    (void)sdio_select(0);
                }
            }
        }
    }
    return err;
}

static struct sdio_card __sdio_card_data;
#define MAX_RETRIES 5

/*
 * sdio_open - Prepare to use SDIO card
 *
 * This function resets the SDIO bus and identifies the
 * card (if any) that is plugged in. If there is no card
 * present, or an error in figuring out what the card is
 * (for example its an old MMC card) the function returns
 * NULL. If it fails and you have logging enabled you can
 * look at the last few commands sent.
 */
struct sdio_card * sdio_open(void) {
    int err;
    int i;
    uint8_t * t;
    uint32_t tmp_reg;
    struct sdio_card * res = &__sdio_card_data;

    // basically bset(0, __sdio_card_data)
    t = (uint8_t *)&__sdio_card_data;
    for (i = 0; i < (int)sizeof(__sdio_card_data); i++) {
        *t++ = 0;
    }
    sdio_reset();
    hal_delay_ms(10);
    err = sdio_command(0, 0);
    if (!err) {
        err = sdio_command(8, 0x1aa);
        if (!err) {
            // Woot! We support CMD8 so we're a v2 card at least */
            tmp_reg = SDIO_RESP1;
            __sdio_card_data.props = 1;
            i = 0;
            err = sdio_command(5, 0);
            if (!err) {
                // It is an SDIO card which is unsupported!
                err = SDIO_EBADCARD;
                return NULL;
            }
            do {
                err = sdio_command(55, 0); // broadcast ACMD
                if (err) {
                    break; // try again
                }
                // Testing Card Busy, Voltage match, and capacity
                err = sdio_command(41, 0xc0100000);
                if (err != -2) { // Expect CCRCFAIL here
                    break;       // try again
                }
                tmp_reg = SDIO_RESP1; // what did the card send?
                if ((tmp_reg & 0x80000000) == 0) {
                    hal_delay_ms(1u << ((uint32_t)i * 2u + 1u));
                    continue; // still powering up
                }
                res->ocr = tmp_reg; // Ok OCR is valid
                break;
            } while (++i < MAX_RETRIES);
            if (res->ocr) {
                err = sdio_command(2, 0);
                if (!err) {
                    res->cid[0] = SDIO_RESP1;
                    res->cid[1] = SDIO_RESP2;
                    res->cid[2] = SDIO_RESP3;
                    res->cid[3] = SDIO_RESP4;
                    err = sdio_command(3, 0); // get the RCA
                    if (!err) {
                        tmp_reg = SDIO_RESP1;
                        res->rca = (uint16_t)((tmp_reg >> 16u) & 0xffffu);
                        if (!res->rca) {
                            /*
                             * If the card says '0' tell it to pick
                             * we assume this will work because the
                             * previous send RCA worked and the card
                             * should be in the ident state if it is
                             * functioning correctly.
                             */
                            (void)sdio_command(3, 0); // try again
                            tmp_reg = SDIO_RESP1;
                            res->rca = (uint16_t)((tmp_reg >> 16u) & 0xffffu);
                        }
                        err = sdio_command(9, ((uint32_t)res->rca) << 16u);
                        if (!err) {
                            res->csd[0] = SDIO_RESP1;
                            res->csd[1] = SDIO_RESP2;
                            res->csd[2] = SDIO_RESP3;
                            res->csd[3] = SDIO_RESP4;
                            err = sdio_scr(res); // Capture the SCR
                            if (!err) {
                                /* All SD Cards support 4 bit bus and 24Mhz */
                                err = sdio_select(res->rca);
                                if (!err) {
                                    err = sdio_command(55, ((uint32_t)res->rca) << 16u);
                                    if (!err) {
                                        err = sdio_command(6, 2);
                                        if (!err) {
                                            sdio_bus(4, SDIO_24MHZ);
                                            (void)sdio_select(0);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    /* Compute the size of the card based on fields in the CSD
     * block. There are two kinds, V1 or V2.
     * In the V1 Case :
     *     Size = 1<<BLOCK_LEN * 1<<(MULT+2) * (C_SIZE+1) bytes.
     * In the V2 Case :
     *     Size = (C_SIZE + 1) * 512K bytes.
     * But for our structure we want the size in 512 byte "blocks"
     * since that is the addressing unit we're going to export so
     * we compute the size / 512 as the "size" for the structure.
     */

    if (!err) {
        res->size = 0;
        switch (SDIO_CSD_VERSION(res)) {
        case 0:
            tmp_reg = ((1u << (SDIO_CSD1_CSIZE_MULT(res) + 2u)) * (1u << SDIO_CSD1_RBLKLEN(res))) >> 9u;
            res->size = tmp_reg * (SDIO_CSD1_CSIZE(res) + 1);
            break;
        case 1:
            res->size = (SDIO_CSD2_CSIZE(res) + 1) << 10;
            break;
        default:
            res->size = 0; // Bug if its not CSD V1 or V2
        }
    }

    sdio_bus(4, SDIO_24MHZ);
    return (err == 0) ? res : NULL;
}
