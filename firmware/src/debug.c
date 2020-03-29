#include "debug.h"
#include "hal.h"

static struct debug_entry {
    uint32_t value;
    const char * file;
    int line;
    uint32_t time_ms;
} _debug_log[DEBUG_LOG_SIZE];

static size_t _debug_log_count = 0;

void debug_log(uint32_t value, const char * file, int line) {
    memmove(&_debug_log[1], &_debug_log[0], sizeof(_debug_log) - sizeof(_debug_log[0]));
    _debug_log[0] = (struct debug_entry){
        .value = value, .file = file, .line = line, .time_ms = hal_now_ms(),
    };
    _debug_log_count++;
}
