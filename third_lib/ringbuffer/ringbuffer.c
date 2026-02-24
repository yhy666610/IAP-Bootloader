#include "ringbuffer.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "ringbuffer.h"

struct ringbuffer
{
    uint16_t head;
    uint16_t tail;
    uint16_t size;

    uint8_t buffer[];
};

rb_t rb_new(uint8_t *buffer, uint32_t length)
{
    if (length < sizeof(struct ringbuffer) + 1)
        return NULL;

    rb_t rb = (rb_t)buffer;
    rb->head = 0;
    rb->tail = 0;
    rb->size = length - sizeof(struct ringbuffer);

    return rb;
}

static inline uint16_t next_head(rb_t rb)
{
    return rb->head + 1 < rb->size ? rb->head + 1 : 0;
}

static inline uint16_t next_tail(rb_t rb)
{
    return rb->tail + 1 < rb->size ? rb->tail + 1 : 0;
}

bool rb_empty(rb_t rb)
{
    return rb->head == rb->tail;
}

bool rb_full(rb_t rb)
{
    return next_head(rb) == rb->tail;
}

// 只有在rb_full为假时才允许写入数据,确保不会覆盖未读取的数据, 保证了缓冲区最多只能存储size - 1个字节的数据
bool rb_put(rb_t rb, uint8_t data)
{
    if (rb_full(rb))
        return false;

    rb->buffer[rb->head] = data;
    rb->head = next_head(rb);

    return true;
}

bool rb_get(rb_t rb, uint8_t *data)
{
    if (rb_empty(rb))
        return false;

    *data = rb->buffer[rb->tail];
    rb->tail = next_tail(rb);

    return true;
}

bool rb_puts(rb_t rb, const uint8_t *data, uint32_t length)
{
    while (length--)
    {
        if (!rb_put(rb, *data++))
            return false;
    }
    return true;
}

uint32_t rb_gets(rb_t rb, uint8_t *data, uint32_t length)
{
    uint32_t count = 0;
    while (length--)
    {
        if (!rb_get(rb, data++))
            break;
        count++;
    }
    return count;
}
