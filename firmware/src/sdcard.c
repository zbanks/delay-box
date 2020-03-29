#include "sdcard.h"
#include "debug.h"
#include "hal.h"

static int sdcard_command(uint8_t cmd, uint32_t arg) {
    // CRC is only required for CMD0 & CMD8; use hard-coded CRC values for those
    uint8_t crc = 0x55;
    if (cmd == 0) {
        EXPECT(arg == 0);
        crc = 0x95;
    } else if (cmd == 8) {
        EXPECT(arg == 0x1AA);
        crc = 0x87;
    }

    // `arg` is sent in big-endian
    const uint8_t * arg8 = (const uint8_t *)&arg;

    hal_sdcard_select(true);
    hal_sdcard_xfer(cmd | 0x40);
    hal_sdcard_xfer(arg8[3]);
    hal_sdcard_xfer(arg8[2]);
    hal_sdcard_xfer(arg8[1]);
    hal_sdcard_xfer(arg8[0]);
    hal_sdcard_xfer(crc);

    // TODO: timeout
    while (true) {
        uint8_t rc = hal_sdcard_xfer(0xFF);
        if ((rc & 0x80) == 0) {
            return rc;
        }
    }
}

int sdcard_setup(void) {
    hal_sdcard_init();

    // Initialization sequence from:
    // https://electronics.stackexchange.com/a/238217
    for (size_t i = 0; i < 80; i++) {
        hal_sdcard_xfer(0xFF);
    }

    int rc = -1;
    if (DEBUG(sdcard_command(0, 0)) != 1) {
        goto done;
    }

    if (DEBUG(sdcard_command(8, 0x1AA)) != 1) {
        goto done;
    }
    uint32_t response = 0;
    hal_sdcard_bulk_read(&response, sizeof(response));
    if (DEBUG(response) != 0xAA010000) {
        goto done;
    }

    // TODO: timeout
    for (size_t retries = 1000;; retries--) {
        if (retries == 0) {
            goto done;
        }

        if (DEBUG(sdcard_command(55, 0)) != 1) {
            goto done;
        }

        if (DEBUG(sdcard_command(41, 0x40000000)) == 0) {
            break;
        }

        hal_delay_ms(20);
    }

    hal_sdcard_speed(true);

    // TODO: timeout
    while (true) {
        rc = sdcard_command(1, 0);
        if (rc == 0) {
            break;
        }
        DEBUG((uint32_t) rc);
        hal_delay_ms(20);
    }

done:
    hal_sdcard_select(false);
    return rc;
}

int sdcard_write(uint32_t block_id, const void * buf, size_t len) {
    // In the future, this function may support multiples of 512
    if (len != 512) {
        DEBUG(len);
        return -1;
    }

    DEBUG(sdcard_command(24, block_id));

    // TODO: timeout
    while (true) {
        if (hal_sdcard_xfer(0xFF) == 0xFF) {
            break;
        }
    }

    hal_sdcard_xfer(0xFE); // Start single write
    hal_sdcard_bulk_write(buf, len);

    // TODO: Timeout
    int rc = -1;
    while (true) {
        uint8_t response = hal_sdcard_xfer(0xFF);
        if (response == 0xFF) {
            continue;
        }

        // 0x05: Data accepted
        // 0x0B: Data rejected, CRC error
        // 0x0D: Data rejected, write error
        if ((response & 0x1F) == 0x05) {
            rc = 0;
        } else {
            DEBUG(response);
        }
        break;
    }

    hal_sdcard_select(false);
    return rc;
}

int sdcard_read(uint32_t block_id, void * buf, size_t len) {
    // In the future, this function may support multiples of 512
    if (len != 512) {
        DEBUG(len);
        return -1;
    }

    DEBUG(sdcard_command(17, block_id));

    while (true) {
        uint8_t ready = hal_sdcard_xfer(0xFF);
        if (ready == 0xFF) {
            continue;
        }
        if (ready != 0xFE) {
            return -1;
        }
        break;
    }

    hal_sdcard_bulk_read(buf, len);

    // Read 16-bit CRC (but throw it out)
    hal_sdcard_xfer(0xFF);
    hal_sdcard_xfer(0xFF);

    hal_sdcard_select(false);
    return 0;
}
