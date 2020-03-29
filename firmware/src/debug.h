#pragma once
#include "prelude.h"

#define DEBUG_ENTRIES_SIZE ((size_t)16)

#define DEBUG(v)                                                                                                       \
    ({                                                                                                                 \
        __typeof__(v) value = (v);                                                                                     \
        debug_log((uint32_t)value, __FILE__, __LINE__);                                                                \
        value;                                                                                                         \
    })
#define EXPECT(t)                                                                                                      \
    ({                                                                                                                 \
        __typeof__(t) test = (t);                                                                                      \
        if (!test) {                                                                                                   \
            DEBUG(0);                                                                                                  \
        };                                                                                                             \
        test;                                                                                                          \
    })

void debug_log(uint32_t value, const char * file, int line);
