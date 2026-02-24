#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Must be called once at boot, after scheduler init but before using debug macros is ok too.
void dbg_task_init(void);

// Start the debug task. Call once after scheduler start context is ready.
// stack_words: FreeRTOS words, not bytes. priority: low (e.g. 1).
void dbg_task_start(unsigned stack_words, unsigned priority);

// printf-like write into ring buffer + notify debug task.
int dbg_write(const char *fmt, ...);

// Write an array into ring buffer with per-byte formatting (no prefix).
// Example format: "%02X " or "%u," etc. You control separators.
void dbg_write_buffer(const uint8_t *buffer, size_t length, const char *format);

#ifdef __cplusplus
}
#endif