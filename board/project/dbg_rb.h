#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Ring buffer init. capacity must be >= 64 and power-of-two recommended.
bool dbg_rb_init(uint8_t *storage, size_t capacity);

// Write raw bytes into RB. Returns number of bytes written (may be < len if full).
size_t dbg_rb_push(const uint8_t *data, size_t len);

// Pop up to len bytes. Returns number of bytes popped.
size_t dbg_rb_pop(uint8_t *out, size_t len);

// How many bytes currently stored.
size_t dbg_rb_available(void);

// Free space remaining.
size_t dbg_rb_free(void);

#ifdef __cplusplus
}
#endif