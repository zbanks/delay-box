#pragma once
#include "prelude.h"

#define DEBUG_LOG_SIZE ((size_t)32)

#define DEBUG(v)                                                                                                       \
    ({                                                                                                                 \
        __typeof__(v) _value = (v);                                                                                    \
        debug_log((uint32_t)_value, __FILE__, __LINE__);                                                               \
        _value;                                                                                                        \
    })
#define EXPECT(t)                                                                                                      \
    ({                                                                                                                 \
        __typeof__(t) _test = (t);                                                                                     \
        if (!_test) {                                                                                                  \
            DEBUG(0);                                                                                                  \
        };                                                                                                             \
        _test;                                                                                                         \
    })

void debug_log(uint32_t value, const char * file, int line);
