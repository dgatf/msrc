#include "dbg_rb.h"

static uint8_t *s_buf = NULL;
static size_t   s_cap = 0;
static volatile size_t s_head = 0; // write index
static volatile size_t s_tail = 0; // read index

static inline size_t rb_next(size_t idx) {
    return (idx + 1) % s_cap;
}

bool dbg_rb_init(uint8_t *storage, size_t capacity) {
    if (!storage || capacity < 64) return false;
    s_buf = storage;
    s_cap = capacity;
    s_head = 0;
    s_tail = 0;
    return true;
}

size_t dbg_rb_available(void) {
    size_t head = s_head;
    size_t tail = s_tail;
    if (head >= tail) return head - tail;
    return (s_cap - tail) + head;
}

size_t dbg_rb_free(void) {
    // One byte is left unused to distinguish full vs empty.
    return (s_cap - 1) - dbg_rb_available();
}

size_t dbg_rb_push(const uint8_t *data, size_t len) {
    if (!s_buf || !data || len == 0) return 0;

    size_t written = 0;
    while (written < len) {
        // If next head equals tail, buffer is full.
        size_t next = rb_next(s_head);
        if (next == s_tail) break;
        s_buf[s_head] = data[written++];
        s_head = next;
    }
    return written;
}

size_t dbg_rb_pop(uint8_t *out, size_t len) {
    if (!s_buf || !out || len == 0) return 0;

    size_t read = 0;
    while (read < len) {
        if (s_tail == s_head) break; // empty
        out[read++] = s_buf[s_tail];
        s_tail = rb_next(s_tail);
    }
    return read;
}