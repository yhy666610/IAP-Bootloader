#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdbool.h>
#include <stdint.h>

struct ringbuffer;
typedef struct ringbuffer *rb_t;

rb_t rb_new(uint8_t *buffer, uint32_t length);
bool rb_empty(rb_t rb);
bool rb_full(rb_t rb);
bool rb_put(rb_t rb, uint8_t data);
bool rb_get(rb_t rb, uint8_t *data);
bool rb_puts(rb_t rb, const uint8_t *data, uint32_t length);
uint32_t rb_gets(rb_t rb, uint8_t *data, uint32_t length);

#endif /* __RINGBUFFER_H__ */
