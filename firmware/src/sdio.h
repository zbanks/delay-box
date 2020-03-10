// Original source:
// https://github.com/ChuckM/stm32f4-sdio-driver

#include "prelude.h"

enum sdio_clock_div { SDIO_24MHZ = 0, SDIO_16MHZ, SDIO_12MHZ, SDIO_8MHZ, SDIO_4MHZ, SDIO_1MHZ, SDIO_400KHZ };

#define SDIO_CARD_CCS(c) (((c)->ocr & 0x40000000) != 0)
#define SDIO_CARD_UHS2(c) (((c)->ocr & 0x40000000) != 0)
#define SDIO_CARD_LVOK(c) (((c)->ocr & 0x01000000) != 0)

struct sdio_card {
    uint32_t props;
    uint32_t ocr;
    uint32_t cid[4];
    uint32_t csd[4];
    uint32_t scr[2];
    uint32_t status[16];
    uint32_t size;
    uint16_t rca;
};

void sdio_init(void);
struct sdio_card * sdio_open(void);
int sdio_readblock(struct sdio_card * card, uint32_t lba, uint8_t * buf);
int sdio_writeblock(struct sdio_card * card, uint32_t lba, uint8_t * buf);
int sdio_status(struct sdio_card * card);
