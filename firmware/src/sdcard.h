#pragma once

#include "prelude.h"

int sdcard_setup(void);
int sdcard_write(uint32_t block_id, const void * buf, size_t len);
int sdcard_read(uint32_t block_id, void * buf, size_t len);
