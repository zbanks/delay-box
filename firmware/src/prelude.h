#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define typeof __typeof__
#define asm __asm__

#define MIN(a, b)                                                                                                      \
    ({                                                                                                                 \
        typeof(a) _a = (a);                                                                                            \
        typeof(b) _b = (b);                                                                                            \
        (_a < _b) ? _a : _b;                                                                                           \
    })
#define MAX(a, b)                                                                                                      \
    ({                                                                                                                 \
        typeof(a) _a = (a);                                                                                            \
        typeof(b) _b = (b);                                                                                            \
        (_a > _b) ? _a : _b;                                                                                           \
    })

#define STRINGIFY(x) STRINGIFY2(x)
#define STRINGIFY2(x) #x

#define CONCAT(x, y) CONCAT2(x, y)
#define CONCAT2(x, y) x##y
