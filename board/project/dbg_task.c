// dbg_task.c

#include "dbg_task.h"
#include "dbg_rb.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

// -------- config --------
#ifndef DBG_RB_CAPACITY
#define DBG_RB_CAPACITY 2048u
#endif

#ifndef DBG_PRINTF_TMP
#define DBG_PRINTF_TMP 256u
#endif

#ifndef DBG_FLUSH_CHUNK
#define DBG_FLUSH_CHUNK 64u
#endif
// ------------------------

static TaskHandle_t s_dbg_task = NULL;

// Static RB storage
static uint8_t s_rb_storage[DBG_RB_CAPACITY];

static void dbg_notify(void) {
    if (s_dbg_task) {
        (void)xTaskNotifyGive(s_dbg_task);
    }
}

void dbg_task_init(void) {
    // Safe to call multiple times
    static bool inited = false;
    if (inited) return;
    inited = true;
    (void)dbg_rb_init(s_rb_storage, sizeof(s_rb_storage));
}

static void dbg_task_fn(void *arg) {
    (void)arg;
    uint8_t chunk[DBG_FLUSH_CHUNK];

    for (;;) {
        // Wait until something is written, but also wake periodically.
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        // Drain RB
        while (dbg_rb_available() > 0) {
            size_t n = dbg_rb_pop(chunk, sizeof(chunk));
            for (size_t i = 0; i < n; i++) {
                // Use putchar to avoid heavy printf parsing.
                putchar((int)chunk[i]);
            }
            // Optional: flush stdio if your backend needs it
            // fflush(stdout);
            taskYIELD();
        }
    }
}

void dbg_task_start(unsigned stack_words, unsigned priority) {
    dbg_task_init();

    if (s_dbg_task) return; // already started

    (void)xTaskCreate(
        dbg_task_fn,
        "dbg",
        (configSTACK_DEPTH_TYPE)stack_words,
        NULL,
        (UBaseType_t)priority,
        &s_dbg_task
    );
}

int dbg_write(const char *fmt, ...) {
    if (!fmt) return 0;

    char tmp[DBG_PRINTF_TMP];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (n <= 0) return n;

    // If truncated, n is the number that *would* have been written.
    size_t to_write = (size_t)n;
    if (to_write >= sizeof(tmp)) {
        to_write = sizeof(tmp) - 1; // write truncated content
    }

    (void)dbg_rb_push((const uint8_t *)tmp, to_write);
    dbg_notify();
    return n;
}

void dbg_write_buffer(const uint8_t *buffer, size_t length, const char *format) {
    if (!buffer || length == 0 || !format) return;

    // Format each byte into a small temp then push.
    // Keep this small to avoid big stack usage.
    char tmp[32];

    for (size_t i = 0; i < length; i++) {
        int n = snprintf(tmp, sizeof(tmp), format, buffer[i]);
        if (n > 0) {
            size_t to_write = (size_t)n;
            if (to_write >= sizeof(tmp)) to_write = sizeof(tmp) - 1;
            (void)dbg_rb_push((const uint8_t *)tmp, to_write);
        }
    }

    dbg_notify();
}